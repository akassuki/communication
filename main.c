/**
 * main.c  –  LoRa Gateway Tool (C99, POSIX)
 * ══════════════════════════════════════════════════════════════
 *
 *  Cú pháp:
 *    ./master <PORT> <ADDL_GW> <ADDL_NODE> <ADDL_OTA> <CH> <BAUD>
 *
 *  Dùng "xxx" cho bất kỳ trường nào để giữ giá trị mặc định.
 *  ADDL_NODE bắt buộc (không có mặc định).
 *
 *  Ví dụ:
 *    ./master /dev/ttyUSB0 0x00 0x02 xxx  20 9600   → POLL node 0x02
 *    ./master /dev/ttyUSB0 0x00 0x02 0x01 20 9600   → OTA  node 0x02 → server 0x01
 *    ./master xxx          xxx  0x02 xxx  xxx xxx   → POLL node 0x02, mọi thứ mặc định
 *
 *  Mặc định:
 *    PORT    = /dev/ttyUSB0
 *    ADDL_GW = 0x00
 *    CH      = 20
 *    BAUD    = 9600
 *
 *  stdout  : JSON 1 dòng (chỉ khi exit=0) – caller parse
 *  stderr  : log có màu + timestamp
 *  log/    : file log gateway_YYYYMMDD_HHMMSS.log
 *
 *  Exit code:
 *    0  OK       stdout chứa JSON
 *    1  ERR      lỗi serial / tham số
 *    2  TIMEOUT  node không phản hồi
 *    3  BADFRAM  frame không hợp lệ
 *
 * ── PROTOCOL ───────────────────────────────────────────────────
 *  POLL  GW→Node  [01][GW_ADDH][GW_ADDL][XOR3]                  4B
 *  DATA  Node→GW  [02][ADDH][ADDL][idx][total][len][payload...]
 *  ACK   GW→Node  [03][GW_ADDH][GW_ADDL][frag_idx]              4B
 *                  → chỉ gửi cho frag KHÔNG phải cuối
 *  OTA   GW→Node  [10][NODE_ADDH][NODE_ADDL][CH]
 *                     [OTA_ADDH][OTA_ADDL][XOR6]                 7B
 *                  (CH dùng chung cho POLL và OTA)
 *
 *  E32 Fixed-TX prefix (GW prepend khi gửi):
 *    [DST_ADDH][DST_ADDL][CH]  3 bytes
 * ══════════════════════════════════════════════════════════════
 */

#define _POSIX_C_SOURCE 200809L
#define _DEFAULT_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdarg.h>
#include <errno.h>
#include <time.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/stat.h>
#include <sys/select.h>

/* ════════════════════════════════════════════════════════════
 * EXIT CODES
 * ════════════════════════════════════════════════════════════ */
#define EXIT_OK       0
#define EXIT_ERR      1
#define EXIT_TIMEOUT  2
#define EXIT_BADFRAM  3

/* ════════════════════════════════════════════════════════════
 * GIÁ TRỊ MẶC ĐỊNH
 * ════════════════════════════════════════════════════════════ */
#define DEF_PORT      "/dev/ttyUSB0"
#define DEF_ADDH      0x00          /* ADDH luôn = 0x00 */
#define DEF_GW_ADDL   0x00
#define DEF_CH        20
#define DEF_BAUD      9600

/* ════════════════════════════════════════════════════════════
 * PROTOCOL
 * ════════════════════════════════════════════════════════════ */
#define CMD_POLL          0x01
#define CMD_DATA          0x02
#define CMD_ACK           0x03
#define CMD_OTA           0x10

#define POLL_FRAME_LEN    4
#define ACK_FRAME_LEN     4
#define OTA_FRAME_LEN     7
#define DATA_HDR_LEN      6
#define FRAG_PAYLOAD_MAX  49
#define MAX_JSON_LEN      512
#define E32_PFX_LEN       3     /* [DST_ADDH][DST_ADDL][CH] */

#define FRAG_TIMEOUT_MS   12000
#define FRAG_RETRY_MAX    3
#define PAYLOAD_TIMEOUT_MS   50000
#define ACK_DELAY_MS      100

/* ════════════════════════════════════════════════════════════
 * CẤU HÌNH RUNTIME
 * ════════════════════════════════════════════════════════════ */
typedef struct {
    char    port[64];
    uint8_t gw_addl;
    uint8_t node_addl;
    uint8_t ota_addl;   /* 0xFF = không OTA (POLL mode) */
    uint8_t ch;
    int     baud;
    int     ota_mode;
} Config;

/* ════════════════════════════════════════════════════════════
 * LOGGER
 * ════════════════════════════════════════════════════════════ */
static FILE *g_log_fp = NULL;

typedef enum { LV_INFO, LV_WARN, LV_ERR, LV_DBG } LogLevel;

static const char *lv_color[] = {
    "\033[0;36m",   /* INFO – cyan */
    "\033[0;33m",   /* WARN – vàng */
    "\033[0;31m",   /* ERR  – đỏ   */
    "\033[0;90m",   /* DBG  – xám  */
};
static const char *lv_tag[] = { "INFO", "WARN", "ERR ", "DBG " };
#define RESET "\033[0m"

static void ts_str(char *buf, size_t sz)
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    struct tm *t = localtime(&ts.tv_sec);
    snprintf(buf, sz, "%02d:%02d:%02d.%03d",
             t->tm_hour, t->tm_min, t->tm_sec,
             (int)(ts.tv_nsec / 1000000));
}

static void gw_log(LogLevel lv, const char *fmt, ...)
{
    char ts[32];
    ts_str(ts, sizeof(ts));

    fprintf(stderr, "%s%s [%s] ", lv_color[lv], ts, lv_tag[lv]);
    va_list ap;
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
    fprintf(stderr, "%s\n", RESET);

    if (g_log_fp) {
        fprintf(g_log_fp, "%s [%s] ", ts, lv_tag[lv]);
        va_list ap2;
        va_start(ap2, fmt);
        vfprintf(g_log_fp, fmt, ap2);
        va_end(ap2);
        fputc('\n', g_log_fp);
        fflush(g_log_fp);
    }
}

static void log_hex(const char *tag, const uint8_t *b, size_t n)
{
    char hex[256] = {0};
    size_t pos = 0;
    for (size_t i = 0; i < n && pos + 3 < sizeof(hex); i++)
        pos += (size_t)snprintf(hex + pos, sizeof(hex) - pos, "%02X ", b[i]);
    if (pos > 0) hex[pos - 1] = '\0';
    gw_log(LV_DBG, "%-8s [%s]", tag, hex);
}

#define LOG(...)  gw_log(LV_INFO, __VA_ARGS__)
#define WARN(...) gw_log(LV_WARN, __VA_ARGS__)
#define ERR(...)  gw_log(LV_ERR,  __VA_ARGS__)
#define DBG(...)  gw_log(LV_DBG,  __VA_ARGS__)

static void log_open(void)
{
    mkdir("log", 0755);
    time_t now = time(NULL);
    struct tm *t = localtime(&now);
    char path[256];
    snprintf(path, sizeof(path),
             "log/gateway_%04d%02d%02d_%02d%02d%02d.log",
             t->tm_year + 1900, t->tm_mon + 1, t->tm_mday,
             t->tm_hour, t->tm_min, t->tm_sec);
    g_log_fp = fopen(path, "w");
    if (g_log_fp)
        fprintf(stderr, "\033[0;32mLog → %s\033[0m\n", path);
    else
        fprintf(stderr, "[WARN] Không mở được log: %s\n", path);
}

static void log_close(void)
{
    if (g_log_fp) { fclose(g_log_fp); g_log_fp = NULL; }
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
        ERR("Mở serial '%s' thất bại: %s", port, strerror(errno));
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
            WARN("Baud %d không hỗ trợ, dùng 9600", baud);
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
 * CHECKSUM & FRAME
 * ════════════════════════════════════════════════════════════ */
static uint8_t xor_chk(const uint8_t *b, size_t n)
{
    uint8_t c = 0;
    for (size_t i = 0; i < n; i++) c ^= b[i];
    return c;
}

/*
 * Module E32 USB tự xử lý định tuyến ở tầng hardware.
 * Gateway CHỈ gửi body thuần – KHÔNG prepend [DST_ADDH][DST_ADDL][CH].
 * Các tham số dst_* và ch chỉ dùng để log, không đưa vào frame.
 */
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
static int send_poll(int fd, const Config *cfg)
{
    uint8_t body[POLL_FRAME_LEN];
    body[0] = CMD_POLL;
    body[1] = DEF_ADDH;
    body[2] = cfg->gw_addl;
    body[3] = xor_chk(body, 3);

    uint8_t frame[E32_PFX_LEN + POLL_FRAME_LEN];
    int len = build_frame(frame, sizeof(frame),
                          DEF_ADDH, cfg->node_addl, cfg->ch,
                          body, POLL_FRAME_LEN);
    if (len < 0) return EXIT_ERR;

    log_hex("TX POLL", frame, (size_t)len);
    LOG("POLL → node[00:%02X]  gw=[00:%02X]  ch=%d",
        cfg->node_addl, cfg->gw_addl, cfg->ch);

    return (serial_write(fd, frame, (size_t)len) == len) ? EXIT_OK : EXIT_ERR;
}

/* ════════════════════════════════════════════════════════════
 * SEND ACK  (không gửi cho fragment cuối)
 * ════════════════════════════════════════════════════════════ */
static int send_ack(int fd, const Config *cfg, uint8_t frag_idx)
{
    uint8_t body[ACK_FRAME_LEN] = {
        CMD_ACK, DEF_ADDH, cfg->gw_addl, frag_idx
    };

    uint8_t frame[E32_PFX_LEN + ACK_FRAME_LEN];
    int len = build_frame(frame, sizeof(frame),
                          DEF_ADDH, cfg->node_addl, cfg->ch,
                          body, ACK_FRAME_LEN);
    if (len < 0) return EXIT_ERR;

    log_hex("TX ACK ", frame, (size_t)len);
    LOG("ACK(%d) → node[00:%02X]", frag_idx, cfg->node_addl);

    return (serial_write(fd, frame, (size_t)len) == len) ? EXIT_OK : EXIT_ERR;
}

/* ════════════════════════════════════════════════════════════
 * SEND OTA REDIRECT
 *  body: [CMD_OTA][NODE_ADDH][NODE_ADDL][CH][OTA_ADDH][OTA_ADDL][XOR6]
 *  CH dùng chung → node biết kênh nào để kết nối OTA server
 * ════════════════════════════════════════════════════════════ */
static int send_ota(int fd, const Config *cfg)
{
    uint8_t body[OTA_FRAME_LEN];
    body[0] = CMD_OTA;
    body[1] = DEF_ADDH;
    body[2] = cfg->node_addl;
    body[3] = cfg->ch;
    body[4] = DEF_ADDH;
    body[5] = cfg->ota_addl;
    body[6] = xor_chk(body, 6);

    uint8_t frame[E32_PFX_LEN + OTA_FRAME_LEN];
    int len = build_frame(frame, sizeof(frame),
                          DEF_ADDH, cfg->node_addl, cfg->ch,
                          body, OTA_FRAME_LEN);
    if (len < 0) return EXIT_ERR;

    log_hex("TX OTA ", frame, (size_t)len);
    LOG("OTA → node[00:%02X]  server=[00:%02X]  ch=%d",
        cfg->node_addl, cfg->ota_addl, cfg->ch);

    return (serial_write(fd, frame, (size_t)len) == len) ? EXIT_OK : EXIT_ERR;
}

/* ════════════════════════════════════════════════════════════
 * RECV ONE DATA FRAGMENT
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
        WARN("Byte[0]=0x%02X không phải CMD_DATA(0x02) – flush & bỏ", hdr[0]);
        tcflush(fd, TCIFLUSH);
        return EXIT_BADFRAM;
    }

    f->addh       = hdr[1];
    f->addl       = hdr[2];
    f->frag_idx   = hdr[3];
    f->frag_total = hdr[4];
    f->pay_len    = hdr[5];

    if (f->pay_len == 0 || f->pay_len > FRAG_PAYLOAD_MAX) {
        WARN("pay_len=%d ngoài khoảng [1..%d]", f->pay_len, FRAG_PAYLOAD_MAX);
        return EXIT_BADFRAM;
    }

    n = serial_read(fd, f->payload, f->pay_len, PAYLOAD_TIMEOUT_MS);
    if (n < (int)f->pay_len) {
        WARN("Payload thiếu: cần %d, nhận %d bytes", f->pay_len, n);
        return EXIT_TIMEOUT;
    }

    log_hex("RX PAY ", f->payload, f->pay_len);
    LOG("Fragment %d/%d  node=[%02X:%02X]  payload=%dB",
        f->frag_idx, f->frag_total - 1, f->addh, f->addl, f->pay_len);

    return EXIT_OK;
}

/* ════════════════════════════════════════════════════════════
 * POLL NODE
 * ════════════════════════════════════════════════════════════ */
static int poll_node(int fd, const Config *cfg,
                     char *json_out, size_t json_size)
{
    tcflush(fd, TCIOFLUSH);

    int rc = EXIT_ERR;
    for (int i = 0; i < FRAG_RETRY_MAX; i++) {
        rc = send_poll(fd, cfg);
        if (rc == EXIT_OK) break;
        WARN("Gửi POLL thất bại lần %d/%d", i + 1, FRAG_RETRY_MAX);
        sleep_ms(500);
    }
    if (rc != EXIT_OK) {
        ERR("Gửi POLL thất bại sau %d lần", FRAG_RETRY_MAX);
        return EXIT_ERR;
    }

    char buf[MAX_JSON_LEN + 1];
    int  buf_len     = 0;
    int  total_frags = -1;
    int  expect      = 0;

    while (1) {
        DataFrag frag;
        int frag_ok = 0;

        for (int retry = 0; retry < FRAG_RETRY_MAX; retry++) {
            rc = recv_frag(fd, FRAG_TIMEOUT_MS, &frag);

            if (rc == EXIT_OK) {
                if (frag.addh != DEF_ADDH || frag.addl != cfg->node_addl) {
                    WARN("Fragment từ node [%02X:%02X] khác, bỏ qua",
                         frag.addh, frag.addl);
                    retry--;
                    continue;
                }
                frag_ok = 1;
                break;
            }

            if (rc == EXIT_TIMEOUT) {
                WARN("Timeout fragment %d, retry %d/%d",
                     expect, retry + 1, FRAG_RETRY_MAX);
                if (expect > 0) {
                    sleep_ms(ACK_DELAY_MS);
                    send_ack(fd, cfg, (uint8_t)(expect - 1));
                    LOG("Re-ACK(%d) để kích node gửi lại fragment %d",
                        expect - 1, expect);
                } else {
                    sleep_ms(500);
                    tcflush(fd, TCIOFLUSH);
                    send_poll(fd, cfg);
                    LOG("Re-POLL vì fragment 0 timeout");
                }
                continue;
            }

            /* EXIT_BADFRAM */
            WARN("BADFRAM fragment %d, retry %d/%d",
                 expect, retry + 1, FRAG_RETRY_MAX);
            tcflush(fd, TCIFLUSH);
            sleep_ms(200);
        }

        if (!frag_ok) {
            ERR("Fragment %d thất bại sau %d lần retry", expect, FRAG_RETRY_MAX);
            return EXIT_TIMEOUT;
        }

        if (total_frags < 0)
            total_frags = (int)frag.frag_total;

        if ((int)frag.frag_idx != expect) {
            ERR("Thứ tự fragment sai: nhận %d, mong %d",
                frag.frag_idx, expect);
            return EXIT_BADFRAM;
        }

        if (buf_len + (int)frag.pay_len > MAX_JSON_LEN) {
            ERR("JSON vượt giới hạn %d bytes", MAX_JSON_LEN);
            return EXIT_ERR;
        }
        memcpy(buf + buf_len, frag.payload, frag.pay_len);
        buf_len += (int)frag.pay_len;

        int is_last = (expect == total_frags - 1);
        if (!is_last) {
            sleep_ms(ACK_DELAY_MS);
            if (send_ack(fd, cfg, frag.frag_idx) != EXIT_OK) {
                ERR("Gửi ACK(%d) thất bại", frag.frag_idx);
                return EXIT_ERR;
            }
            expect++;
        } else {
            LOG("Fragment cuối nhận xong");
            break;
        }
    }

    buf[buf_len] = '\0';
    if ((size_t)(buf_len + 1) > json_size) {
        ERR("json_out buffer quá nhỏ (%zu cần %d)", json_size, buf_len + 1);
        return EXIT_ERR;
    }
    memcpy(json_out, buf, (size_t)(buf_len + 1));
    return EXIT_OK;
}

/* ════════════════════════════════════════════════════════════
 * PARSE CLI
 *
 *  argv[1] PORT
 *  argv[2] ADDL_GW
 *  argv[3] ADDL_NODE  (bắt buộc)
 *  argv[4] ADDL_OTA   (xxx = POLL, hex = OTA)
 *  argv[5] CH
 *  argv[6] BAUD
 * ════════════════════════════════════════════════════════════ */
static int is_xxx(const char *s) { return strcmp(s, "xxx") == 0; }

static int parse_byte(const char *s, uint8_t *out)
{
    char *end;
    unsigned long v = strtoul(s, &end, 0);
    if (*end != '\0' || v > 0xFF) return -1;
    *out = (uint8_t)v;
    return 0;
}

static void usage(const char *prog)
{
    fprintf(stderr,
        "Dùng:\n"
        "  %s <PORT> <ADDL_GW> <ADDL_NODE> <ADDL_OTA> <CH> <BAUD>\n"
        "\n"
        "  \"xxx\" = dùng giá trị mặc định cho trường đó\n"
        "\n"
        "  PORT      serial port        (mặc định: %s)\n"
        "  ADDL_GW   địa chỉ gateway    (mặc định: 0x%02X)\n"
        "  ADDL_NODE địa chỉ node       (bắt buộc, không được xxx)\n"
        "  ADDL_OTA  địa chỉ OTA server (xxx → POLL mode | hex → OTA mode)\n"
        "  CH        kênh LoRa          (mặc định: %d)\n"
        "  BAUD      baud rate          (mặc định: %d)\n"
        "\n"
        "Ví dụ:\n"
        "  %s /dev/ttyUSB0 0x00 0x02 xxx  20   9600  → POLL node 0x02\n"
        "  %s /dev/ttyUSB0 0x00 0x02 0x01 20   9600  → OTA  node 0x02 → server 0x01\n"
        "  %s xxx          xxx  0x03 xxx  xxx  xxx   → POLL node 0x03, mọi thứ mặc định\n"
        "\n"
        "Exit code:\n"
        "  0  OK       stdout chứa JSON\n"
        "  1  ERR      lỗi serial / tham số\n"
        "  2  TIMEOUT  node không phản hồi\n"
        "  3  BADFRAM  frame lỗi\n",
        prog, DEF_PORT, DEF_GW_ADDL, DEF_CH, DEF_BAUD,
        prog, prog, prog);
}

static int parse_args(int argc, char *argv[], Config *cfg)
{
    if (argc != 7) {
        fprintf(stderr, "[ERR] Cần đúng 6 tham số (nhận %d)\n", argc - 1);
        usage(argv[0]);
        return -1;
    }

    /* PORT */
    if (is_xxx(argv[1]))
        strncpy(cfg->port, DEF_PORT, sizeof(cfg->port) - 1);
    else
        strncpy(cfg->port, argv[1], sizeof(cfg->port) - 1);
    cfg->port[sizeof(cfg->port) - 1] = '\0';

    /* ADDL_GW */
    if (is_xxx(argv[2])) {
        cfg->gw_addl = DEF_GW_ADDL;
    } else if (parse_byte(argv[2], &cfg->gw_addl) < 0) {
        fprintf(stderr, "[ERR] ADDL_GW không hợp lệ: '%s'\n", argv[2]);
        return -1;
    }

    /* ADDL_NODE – bắt buộc */
    if (is_xxx(argv[3])) {
        fprintf(stderr, "[ERR] ADDL_NODE bắt buộc, không được để xxx\n");
        return -1;
    }
    if (parse_byte(argv[3], &cfg->node_addl) < 0) {
        fprintf(stderr, "[ERR] ADDL_NODE không hợp lệ: '%s'\n", argv[3]);
        return -1;
    }

    /* ADDL_OTA */
    if (is_xxx(argv[4])) {
        cfg->ota_addl = 0xFF;
        cfg->ota_mode = 0;
    } else {
        if (parse_byte(argv[4], &cfg->ota_addl) < 0) {
            fprintf(stderr, "[ERR] ADDL_OTA không hợp lệ: '%s'\n", argv[4]);
            return -1;
        }
        cfg->ota_mode = 1;
    }

    /* CH */
    if (is_xxx(argv[5])) {
        cfg->ch = DEF_CH;
    } else {
        char *end;
        unsigned long v = strtoul(argv[5], &end, 0);
        if (*end != '\0' || v > 0xFF) {
            fprintf(stderr, "[ERR] CH không hợp lệ: '%s'\n", argv[5]);
            return -1;
        }
        cfg->ch = (uint8_t)v;
    }

    /* BAUD */
    if (is_xxx(argv[6])) {
        cfg->baud = DEF_BAUD;
    } else {
        char *end;
        long v = strtol(argv[6], &end, 10);
        if (*end != '\0' || v <= 0) {
            fprintf(stderr, "[ERR] BAUD không hợp lệ: '%s'\n", argv[6]);
            return -1;
        }
        cfg->baud = (int)v;
    }

    return 0;
}

/* ════════════════════════════════════════════════════════════
 * MAIN
 * ════════════════════════════════════════════════════════════ */
int main(int argc, char *argv[])
{
    Config cfg;
    memset(&cfg, 0, sizeof(cfg));

    if (parse_args(argc, argv, &cfg) < 0)
        return EXIT_ERR;

    log_open();

    LOG("════════════════════════════════════════");
    LOG("PORT=%-15s  BAUD=%-6d  CH=%d", cfg.port, cfg.baud, cfg.ch);
    LOG("GW=[00:%02X]  NODE=[00:%02X]  MODE=%s",
        cfg.gw_addl, cfg.node_addl, cfg.ota_mode ? "OTA" : "POLL");
    if (cfg.ota_mode)
        LOG("OTA_SERVER=[00:%02X]", cfg.ota_addl);
    LOG("════════════════════════════════════════");

    int fd = serial_open(cfg.port, cfg.baud);
    if (fd < 0) { log_close(); return EXIT_ERR; }
    LOG("Serial OK");

    int exit_code;

    if (cfg.ota_mode) {
        /* ── OTA ── */
        if (send_ota(fd, &cfg) == EXIT_OK) {
            LOG("OTA redirect gửi thành công");
            printf("{\"mode\":\"ota\",\"node\":\"00%02X\","
                   "\"ota_server\":\"00%02X\",\"ch\":%d}\n",
                   cfg.node_addl, cfg.ota_addl, cfg.ch);
            exit_code = EXIT_OK;
        } else {
            ERR("Gửi OTA thất bại");
            exit_code = EXIT_ERR;
        }
    } else {
        /* ── POLL ── */
        char json[MAX_JSON_LEN + 1];
        exit_code = poll_node(fd, &cfg, json, sizeof(json));
        if (exit_code == EXIT_OK) {
            LOG("JSON: %s", json);
            LOG("────────────────────────────────────────");
            puts(json);
        } else {
            ERR("Poll kết thúc lỗi (exit=%d)", exit_code);
        }
    }

    close(fd);
    LOG("Kết thúc (exit=%d)", exit_code);
    log_close();
    return exit_code;
}
