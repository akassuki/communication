/**
 * main.c  –  LoRa Gateway Tool (C99, POSIX)
 * ══════════════════════════════════════════════════════════════
 *
 *  Cú pháp:
 *    ./master <ADDL>
 *    ./master <ADDL> OTA
 *
 *  stdout  : JSON cảm biến thuần (chỉ 1 dòng) – caller parse
 *  stderr  : log/debug – caller có thể bỏ qua hoặc ghi file
 *
 *  Exit code:
 *    EXIT_OK      (0)  thành công, stdout có JSON
 *    EXIT_ERR     (1)  lỗi chung (serial, arg, ...)
 *    EXIT_TIMEOUT (2)  node không phản hồi trong thời gian cho phép
 *    EXIT_BADFRAM (3)  frame lỗi (byte sai, payload không hợp lệ)
 *
 *  Biến môi trường:
 *    LORA_PORT   serial port  (mặc định: /dev/ttyUSB0)
 *    LORA_BAUD   baud rate    (mặc định: 9600)
 *
 * ── PROTOCOL ───────────────────────────────────────────────────
 *  POLL  GW→Node  [01][GW_ADDH][GW_ADDL][XOR3]              4B
 *  DATA  Node→GW  [02][ADDH][ADDL][idx][total][len][payload]
 *  ACK   GW→Node  [03][GW_ADDH][GW_ADDL][frag_idx]          4B
 *                  ACK chỉ gửi cho frag KHÔNG phải cuối
 *  OTA   GW→Node  [10][NODE_ADDH][NODE_ADDL][OTA_CH]
 *                  [OTA_ADDH][OTA_ADDL][XOR6]                7B
 *
 *  E32 Fixed-TX prefix prepend bởi Gateway:
 *    [DST_ADDH][DST_ADDL][CH]  3 bytes
 * ══════════════════════════════════════════════════════════════
 */

#define _POSIX_C_SOURCE 200809L
#define _DEFAULT_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/select.h>

/* ════════════════════════════════════════════════════════════
 * EXIT CODES
 * ════════════════════════════════════════════════════════════ */
#define EXIT_OK       0
#define EXIT_ERR      1
#define EXIT_TIMEOUT  2
#define EXIT_BADFRAM  3

/* ════════════════════════════════════════════════════════════
 * CẤU HÌNH CỨNG – chỉnh tại đây
 * ════════════════════════════════════════════════════════════ */
#define DEFAULT_PORT        "/dev/ttyUSB0"
#define DEFAULT_BAUD        9600

#define GW_ADDH             0x00
#define GW_ADDL             0x00
#define LORA_CH             20

#define OTA_ADDH            0x00
#define OTA_ADDL            0x01
#define OTA_CH              21

/* Timeout cho từng fragment (ms).
 * Tool sẽ thoát EXIT_TIMEOUT nếu không nhận được trong khoảng này. */
#define FRAG_TIMEOUT_MS     10000

/* Delay nhỏ trước khi gửi ACK, tránh node chưa sẵn sàng nhận */
#define ACK_DELAY_MS        100

/* ════════════════════════════════════════════════════════════
 * PROTOCOL CONSTANTS
 * ════════════════════════════════════════════════════════════ */
#define CMD_POLL            0x01
#define CMD_DATA            0x02
#define CMD_ACK             0x03
#define CMD_OTA             0x10

#define POLL_FRAME_LEN      4
#define ACK_FRAME_LEN       4
#define OTA_FRAME_LEN       7
#define DATA_HDR_LEN        6
#define FRAG_PAYLOAD_MAX    49
#define MAX_JSON_LEN        512
#define E32_PFX_LEN         3   /* [DST_ADDH][DST_ADDL][CH] */

/* ════════════════════════════════════════════════════════════
 * LOGGING  →  stderr (KHÔNG bao giờ ra stdout)
 * ════════════════════════════════════════════════════════════ */
/* __VA_OPT__ không có trong C99, dùng do-while trick với format string rỗng */
#define LOG(...)  fprintf(stderr, "[INFO] " __VA_ARGS__), fputc('\n', stderr)
#define WARN(...) fprintf(stderr, "[WARN] " __VA_ARGS__), fputc('\n', stderr)
#define ERR(...)  fprintf(stderr, "[ERR]  " __VA_ARGS__), fputc('\n', stderr)

static void log_hex(const char *tag, const uint8_t *b, size_t n)
{
    fprintf(stderr, "[DBG]  %s: ", tag);
    for (size_t i = 0; i < n; i++) fprintf(stderr, "%02X ", b[i]);
    fputc('\n', stderr);
}

/* ════════════════════════════════════════════════════════════
 * THỜI GIAN
 * ════════════════════════════════════════════════════════════ */
static long now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000L + ts.tv_nsec / 1000000L;
}

static void sleep_ms(int ms)
{
    struct timespec ts;
    ts.tv_sec  = ms / 1000;
    ts.tv_nsec = (ms % 1000) * 1000000L;
    nanosleep(&ts, NULL);
}

/* ════════════════════════════════════════════════════════════
 * SERIAL
 * ════════════════════════════════════════════════════════════ */
static int serial_open(const char *port, int baud)
{
    int fd = open(port, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        ERR("Mở serial %s thất bại: %s", port, strerror(errno));
        return -1;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fd, &tty) != 0) {
        ERR("tcgetattr: %s", strerror(errno));
        close(fd); return -1;
    }

    speed_t spd;
    switch (baud) {
        case 9600:   spd = B9600;   break;
        case 19200:  spd = B19200;  break;
        case 38400:  spd = B38400;  break;
        case 57600:  spd = B57600;  break;
        case 115200: spd = B115200; break;
        default:
            WARN("Baud %d không được hỗ trợ, dùng 9600", baud);
            spd = B9600;
    }
    cfsetispeed(&tty, spd);
    cfsetospeed(&tty, spd);

    tty.c_cflag  = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
    tty.c_cflag |=  (CLOCAL | CREAD);
    tty.c_lflag  = 0;
    tty.c_oflag  = 0;
    tty.c_iflag  = 0;
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;

    tcflush(fd, TCIOFLUSH);
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        ERR("tcsetattr: %s", strerror(errno));
        close(fd); return -1;
    }
    return fd;
}

/*
 * Đọc đúng `need` bytes, trả về số bytes thực sự đọc được.
 * Trả về < need khi hết timeout_ms → caller tự phán đoán là TIMEOUT.
 */
static int serial_read(int fd, uint8_t *buf, size_t need, int timeout_ms)
{
    size_t got      = 0;
    long   deadline = now_ms() + timeout_ms;

    while (got < need) {
        long rem = deadline - now_ms();
        if (rem <= 0) break;

        struct timeval tv;
        tv.tv_sec  = rem / 1000;
        tv.tv_usec = (rem % 1000) * 1000;

        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(fd, &fds);

        if (select(fd + 1, &fds, NULL, NULL, &tv) <= 0) break;

        ssize_t n = read(fd, buf + got, need - got);
        if (n > 0) got += (size_t)n;
    }
    return (int)got;
}

static int serial_write(int fd, const uint8_t *buf, size_t len)
{
    return (int)write(fd, buf, len);
}

/* ════════════════════════════════════════════════════════════
 * CHECKSUM
 * ════════════════════════════════════════════════════════════ */
static uint8_t xor_chk(const uint8_t *b, size_t n)
{
    uint8_t c = 0;
    for (size_t i = 0; i < n; i++) c ^= b[i];
    return c;
}

/* ════════════════════════════════════════════════════════════
 * BUILD TX FRAME  –  prepend E32 fixed-TX header
 *   out[0..2] = [DST_ADDH][DST_ADDL][CH]
 *   out[3..]  = payload
 *   trả về tổng độ dài, -1 nếu buffer không đủ
 * ════════════════════════════════════════════════════════════ */
static int build_frame(uint8_t *out, size_t out_sz,
                       uint8_t dst_addh, uint8_t dst_addl, uint8_t ch,
                       const uint8_t *body, size_t body_len)
{
    if (E32_PFX_LEN + body_len > out_sz) return -1;
    out[0] = dst_addh;
    out[1] = dst_addl;
    out[2] = ch;
    memcpy(out + E32_PFX_LEN, body, body_len);
    return (int)(E32_PFX_LEN + body_len);
}

/* ════════════════════════════════════════════════════════════
 * SEND POLL
 * ════════════════════════════════════════════════════════════ */
static int send_poll(int fd, uint8_t node_addh, uint8_t node_addl)
{
    uint8_t body[POLL_FRAME_LEN];
    body[0] = CMD_POLL;
    body[1] = GW_ADDH;
    body[2] = GW_ADDL;
    body[3] = xor_chk(body, 3);

    uint8_t frame[E32_PFX_LEN + POLL_FRAME_LEN];
    int len = build_frame(frame, sizeof(frame),
                          node_addh, node_addl, LORA_CH,
                          body, POLL_FRAME_LEN);
    if (len < 0) return EXIT_ERR;

    log_hex("TX POLL", frame, (size_t)len);
    return (serial_write(fd, frame, (size_t)len) == len) ? EXIT_OK : EXIT_ERR;
}

/* ════════════════════════════════════════════════════════════
 * SEND ACK  –  chỉ gọi cho fragment KHÔNG phải cuối
 * ════════════════════════════════════════════════════════════ */
static int send_ack(int fd, uint8_t node_addh, uint8_t node_addl,
                    uint8_t frag_idx)
{
    uint8_t body[ACK_FRAME_LEN] = { CMD_ACK, GW_ADDH, GW_ADDL, frag_idx };

    uint8_t frame[E32_PFX_LEN + ACK_FRAME_LEN];
    int len = build_frame(frame, sizeof(frame),
                          node_addh, node_addl, LORA_CH,
                          body, ACK_FRAME_LEN);
    if (len < 0) return EXIT_ERR;

    log_hex("TX ACK ", frame, (size_t)len);
    return (serial_write(fd, frame, (size_t)len) == len) ? EXIT_OK : EXIT_ERR;
}

/* ════════════════════════════════════════════════════════════
 * SEND OTA REDIRECT
 *
 *  Frame body 7 bytes:
 *    [CMD_OTA][NODE_ADDH][NODE_ADDL][OTA_CH][OTA_ADDH][OTA_ADDL][XOR6]
 *
 *  Node nhận → reconfigure LoRa → kết nối OTA server để tải firmware
 * ════════════════════════════════════════════════════════════ */
static int send_ota(int fd, uint8_t node_addh, uint8_t node_addl)
{
    uint8_t body[OTA_FRAME_LEN];
    body[0] = CMD_OTA;
    body[1] = node_addh;
    body[2] = node_addl;
    body[3] = OTA_CH;
    body[4] = OTA_ADDH;
    body[5] = OTA_ADDL;
    body[6] = xor_chk(body, 6);

    uint8_t frame[E32_PFX_LEN + OTA_FRAME_LEN];
    int len = build_frame(frame, sizeof(frame),
                          node_addh, node_addl, LORA_CH,
                          body, OTA_FRAME_LEN);
    if (len < 0) return EXIT_ERR;

    log_hex("TX OTA ", frame, (size_t)len);
    return (serial_write(fd, frame, (size_t)len) == len) ? EXIT_OK : EXIT_ERR;
}

/* ════════════════════════════════════════════════════════════
 * RECV ONE FRAGMENT
 *
 *  DATA frame (Node → GW):
 *    [02][ADDH][ADDL][frag_idx][frag_total][pay_len][payload...]
 *
 *  Trả về:
 *    EXIT_OK       frame hợp lệ, *f đã điền
 *    EXIT_TIMEOUT  không nhận đủ bytes trong timeout
 *    EXIT_BADFRAM  bytes nhận được nhưng nội dung không hợp lệ
 * ════════════════════════════════════════════════════════════ */
typedef struct {
    uint8_t addh, addl;
    uint8_t frag_idx;
    uint8_t frag_total;
    uint8_t pay_len;
    uint8_t payload[FRAG_PAYLOAD_MAX];
} DataFrag;

static int recv_frag(int fd, int timeout_ms, DataFrag *f)
{
    uint8_t hdr[DATA_HDR_LEN];
    int n = serial_read(fd, hdr, DATA_HDR_LEN, timeout_ms);

    if (n < DATA_HDR_LEN) {
        WARN("Timeout chờ header (%d/%d bytes)", n, DATA_HDR_LEN);
        return EXIT_TIMEOUT;
    }

    log_hex("RX HDR ", hdr, DATA_HDR_LEN);

    if (hdr[0] != CMD_DATA) {
        WARN("Byte[0]=0x%02X, mong CMD_DATA(0x02)", hdr[0]);
        tcflush(fd, TCIFLUSH);
        return EXIT_BADFRAM;
    }

    f->addh       = hdr[1];
    f->addl       = hdr[2];
    f->frag_idx   = hdr[3];
    f->frag_total = hdr[4];
    f->pay_len    = hdr[5];

    if (f->pay_len == 0 || f->pay_len > FRAG_PAYLOAD_MAX) {
        WARN("pay_len=%d ngoài khoảng hợp lệ [1..%d]",
             f->pay_len, FRAG_PAYLOAD_MAX);
        return EXIT_BADFRAM;
    }

    n = serial_read(fd, f->payload, f->pay_len, 2000);
    if (n < (int)f->pay_len) {
        WARN("Payload thiếu: cần %d được %d bytes", f->pay_len, n);
        return EXIT_TIMEOUT;
    }

    LOG("Frag %d/%d  node=[%02X:%02X]  payload=%dB",
        f->frag_idx, f->frag_total - 1, f->addh, f->addl, f->pay_len);
    return EXIT_OK;
}

/* ════════════════════════════════════════════════════════════
 * POLL NODE
 *
 *  Gửi POLL → nhận N fragments → gửi ACK sau mỗi frag trừ cuối
 *  → reassemble → ghi vào json_out
 *
 *  Trả về exit code (EXIT_OK / EXIT_TIMEOUT / EXIT_BADFRAM / EXIT_ERR)
 * ════════════════════════════════════════════════════════════ */
static int poll_node(int fd,
                     uint8_t node_addh, uint8_t node_addl,
                     char *json_out, size_t json_size)
{
    /* 1. Gửi POLL */
    int rc = send_poll(fd, node_addh, node_addl);
    if (rc != EXIT_OK) {
        ERR("Gửi POLL thất bại");
        return EXIT_ERR;
    }
    LOG("POLL → [%02X:%02X] đã gửi", node_addh, node_addl);

    /* 2. Nhận fragments và reassemble */
    char   buf[MAX_JSON_LEN + 1];
    int    buf_len     = 0;
    int    total_frags = -1;
    int    expect      = 0;

    while (1) {
        DataFrag frag;
        rc = recv_frag(fd, FRAG_TIMEOUT_MS, &frag);

        if (rc == EXIT_TIMEOUT) {
            ERR("Timeout chờ fragment %d", expect);
            return EXIT_TIMEOUT;
        }
        if (rc == EXIT_BADFRAM) {
            ERR("Frame lỗi tại fragment %d", expect);
            return EXIT_BADFRAM;
        }

        /* Bỏ qua fragment từ node khác (có thể có nhiễu trên kênh) */
        if (frag.addh != node_addh || frag.addl != node_addl) {
            WARN("Fragment từ node [%02X:%02X] khác, bỏ qua",
                 frag.addh, frag.addl);
            continue;   /* không tăng expect, tiếp tục đọc */
        }

        /* Lần đầu: chốt tổng số fragment */
        if (total_frags < 0)
            total_frags = (int)frag.frag_total;

        /* Kiểm tra thứ tự */
        if ((int)frag.frag_idx != expect) {
            ERR("Thứ tự fragment sai: nhận %d, mong %d",
                frag.frag_idx, expect);
            return EXIT_BADFRAM;
        }

        /* Append payload vào buffer */
        if (buf_len + (int)frag.pay_len > MAX_JSON_LEN) {
            ERR("JSON vượt giới hạn %d bytes", MAX_JSON_LEN);
            return EXIT_ERR;
        }
        memcpy(buf + buf_len, frag.payload, frag.pay_len);
        buf_len += (int)frag.pay_len;

        int is_last = (expect == total_frags - 1);

        if (!is_last) {
            /* 3. Gửi ACK cho fragment này (trừ fragment cuối) */
            sleep_ms(ACK_DELAY_MS);
            if (send_ack(fd, node_addh, node_addl, frag.frag_idx) != EXIT_OK) {
                ERR("Gửi ACK(%d) thất bại", frag.frag_idx);
                return EXIT_ERR;
            }
            LOG("ACK(%d) đã gửi", frag.frag_idx);
        } else {
            LOG("Fragment cuối nhận xong, không gửi ACK");
            break;
        }

        expect++;
    }

    /* 4. Ghi kết quả */
    buf[buf_len] = '\0';
    if ((size_t)(buf_len + 1) > json_size) {
        ERR("json_out buffer quá nhỏ (%zu cần %d)", json_size, buf_len + 1);
        return EXIT_ERR;
    }
    memcpy(json_out, buf, (size_t)(buf_len + 1));
    return EXIT_OK;
}

/* ════════════════════════════════════════════════════════════
 * USAGE
 * ════════════════════════════════════════════════════════════ */
static void usage(const char *prog)
{
    fprintf(stderr,
        "Dùng:\n"
        "  %s <ADDL>        poll node, in JSON cảm biến ra stdout\n"
        "  %s <ADDL> OTA    gửi lệnh OTA redirect tới node\n"
        "\n"
        "  ADDL  : địa chỉ node, hex (0x02) hoặc decimal (2)\n"
        "\n"
        "Biến môi trường:\n"
        "  LORA_PORT   cổng serial  (mặc định: %s)\n"
        "  LORA_BAUD   baud rate    (mặc định: %d)\n"
        "\n"
        "Exit code:\n"
        "  0  OK        stdout chứa JSON cảm biến\n"
        "  1  ERR       lỗi serial, tham số, ...\n"
        "  2  TIMEOUT   node không phản hồi\n"
        "  3  BADFRAM   frame không hợp lệ\n"
        "\n"
        "stdout  : chỉ JSON (ví dụ: {\"temp\":25.1,\"hum\":60.0})\n"
        "stderr  : toàn bộ log debug\n"
        "\n"
        "Ví dụ Python:\n"
        "  import subprocess, json\n"
        "  r = subprocess.run(['./gateway.elf','0x02'],\n"
        "                     capture_output=True, timeout=15)\n"
        "  if r.returncode == 0:\n"
        "      data = json.loads(r.stdout)\n",
        prog, prog, DEFAULT_PORT, DEFAULT_BAUD);
}

/* ════════════════════════════════════════════════════════════
 * MAIN
 * ════════════════════════════════════════════════════════════ */
int main(int argc, char *argv[])
{
    if (argc < 2) { usage(argv[0]); return EXIT_ERR; }

    /* ── Parse ADDL ─────────────────────────────────────────── */
    char *endptr;
    unsigned long addl_val = strtoul(argv[1], &endptr, 0);
    if (*endptr != '\0' || addl_val > 0xFF) {
        ERR("ADDL không hợp lệ: '%s'  (vd: 0x02 hoặc 2)", argv[1]);
        return EXIT_ERR;
    }
    uint8_t node_addh = 0x00;
    uint8_t node_addl = (uint8_t)addl_val;

    /* ── Mode ───────────────────────────────────────────────── */
    int ota_mode = 0;
    if (argc >= 3) {
        if (strcmp(argv[2], "OTA") == 0 || strcmp(argv[2], "ota") == 0) {
            ota_mode = 1;
        } else {
            ERR("Tham số không hợp lệ: '%s'", argv[2]);
            usage(argv[0]);
            return EXIT_ERR;
        }
    }

    /* ── Đọc cấu hình từ environment ───────────────────────── */
    const char *port = getenv("LORA_PORT");
    if (!port || port[0] == '\0') port = DEFAULT_PORT;

    int baud = DEFAULT_BAUD;
    const char *baud_env = getenv("LORA_BAUD");
    if (baud_env && baud_env[0] != '\0') baud = atoi(baud_env);

    LOG("Port=%-15s  Baud=%-6d  Node=[%02X:%02X]  Mode=%s",
        port, baud, node_addh, node_addl, ota_mode ? "OTA" : "POLL");

    /* ── Mở serial ──────────────────────────────────────────── */
    int fd = serial_open(port, baud);
    if (fd < 0) return EXIT_ERR;

    int exit_code;

    if (ota_mode) {
        /* ════════════════════════════════════════════════════
         * CHẾ ĐỘ OTA
         *  stdout: JSON mô tả kết quả để Python parse đồng nhất
         * ════════════════════════════════════════════════════ */
        if (send_ota(fd, node_addh, node_addl) == EXIT_OK) {
            /* stdout JSON – Python dùng json.loads() bình thường */
            printf("{\"ota\":true,\"node\":\"%02X%02X\","
                   "\"ota_ch\":%d,\"ota_server\":\"%02X%02X\"}\n",
                   node_addh, node_addl, OTA_CH, OTA_ADDH, OTA_ADDL);
            LOG("OTA redirect gửi thành công → node [%02X:%02X]"
                " chuyển sang CH=%d, server [%02X:%02X]",
                node_addh, node_addl, OTA_CH, OTA_ADDH, OTA_ADDL);
            exit_code = EXIT_OK;
        } else {
            ERR("Gửi OTA thất bại");
            exit_code = EXIT_ERR;
        }

    } else {
        /* ════════════════════════════════════════════════════
         * CHẾ ĐỘ POLL
         *  stdout: JSON cảm biến thô từ node, ví dụ:
         *          {"temp":25.1,"hum":60.2,"volt":3.72,"uptime":123}
         * ════════════════════════════════════════════════════ */
        char json[MAX_JSON_LEN + 1];
        exit_code = poll_node(fd, node_addh, node_addl, json, sizeof(json));

        if (exit_code == EXIT_OK) {
            /* Chỉ một dòng JSON ra stdout – không có gì khác */
            puts(json);
            LOG("Done. JSON=%d bytes", (int)strlen(json));
        } else {
            ERR("Poll thất bại (exit=%d)", exit_code);
        }
    }

    close(fd);
    return exit_code;
}
