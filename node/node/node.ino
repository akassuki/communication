/**
 * node_test.ino
 * ══════════════════════════════════════════════════════════════
 * 1 task polling duy nhất: chờ POLL → gửi DATA fragments → chờ ACK
 * Core 0: PollingTask
 * Core 1: dành cho OTA sau này
 *
 * ── CHỈ CẦN SỬA PHẦN NÀY TRƯỚC KHI FLASH ──────────────────
 *   #define NODE_ADDL  0x02   → node_01
 *   #define NODE_ADDL  0x03   → node_02
 * ──────────────────────────────────────────────────────────────
 */

#include <Arduino.h>
#include <LoRa_E32.h>

// ════════════════════════════════════════════════════════════
// ĐỊA CHỈ
// ════════════════════════════════════════════════════════════
#define NODE_ADDH         0x00
#define NODE_ADDL         0x02    // ← ĐỔI cho từng node

#define GW_ADDH           0x00
#define GW_ADDL           0x00

// ════════════════════════════════════════════════════════════
// LORA E32
// ════════════════════════════════════════════════════════════
#define LORA_CH           20
#define LORA_BAUD         9600
#define LORA_TX_PIN       34
#define LORA_RX_PIN       25

// ════════════════════════════════════════════════════════════
// GIAO THỨC
// ════════════════════════════════════════════════════════════
#define CMD_POLL          0x01
#define CMD_DATA          0x02
#define CMD_ACK           0x03
#define CMD_OTA           0x10
#define CMD_HEARTBEAT     0x20
#define CMD_ERROR         0xFF

#define POLL_FRAME_LEN    4     // [CMD_POLL][GW_ADDH][GW_ADDL][XOR]
#define ACK_FRAME_LEN     4     // [CMD_ACK][GW_ADDH][GW_ADDL][frag_idx]
#define FRAG_PAYLOAD_MAX  49    // 55 - 6 byte header
#define ACK_TIMEOUT_MS    8000
#define POLL_WAIT_MS      10

#define FRAG_TX_RETRY  5  

// ════════════════════════════════════════════════════════════
// TEST CASES
// ════════════════════════════════════════════════════════════
#define CASE_SMALL    0
#define CASE_MEDIUM   1
#define CASE_LARGE    2

// ════════════════════════════════════════════════════════════
// GLOBALS
// ════════════════════════════════════════════════════════════
HardwareSerial loraSerial(1);
LoRa_E32 e32(LORA_TX_PIN, LORA_RX_PIN, &loraSerial, UART_BPS_RATE_9600);

// ════════════════════════════════════════════════════════════
// TIỆN ÍCH
// ════════════════════════════════════════════════════════════
static uint8_t xor_chk(const uint8_t* buf, size_t len) {
    uint8_t c = 0;
    for (size_t i = 0; i < len; i++) c ^= buf[i];
    return c;
}

static float sim_val(float base, float range) {
    return base + (float)(millis() % (uint32_t)(range * 1000)) / 1000.0f;
}

// ════════════════════════════════════════════════════════════
// BUILD JSON
// ════════════════════════════════════════════════════════════
static int build_json(int test_case, char* buf, size_t buf_size) {
    if (test_case == CASE_SMALL) {
        return snprintf(buf, buf_size,
            "{\"temp\":%.1f,\"hum\":%.1f,\"volt\":%.2f,\"uptime\":%lu}",
            sim_val(20.0f,5.0f), sim_val(55.0f,5.0f),
            sim_val(3.60f,0.4f), millis()/1000UL);

    } else if (test_case == CASE_MEDIUM) {
        return snprintf(buf, buf_size,
            "{\"t1\":%.1f,\"t2\":%.1f,\"t3\":%.1f,\"t4\":%.1f,"
            "\"t5\":%.1f,\"t6\":%.1f,\"t7\":%.1f,\"t8\":%.1f,"
            "\"h1\":%.1f,\"h2\":%.1f,\"h3\":%.1f,\"h4\":%.1f,"
            "\"h5\":%.1f,\"h6\":%.1f,\"h7\":%.1f,\"h8\":%.1f,"
            "\"v1\":%.2f,\"v2\":%.2f,\"v3\":%.2f,\"v4\":%.2f,"
            "\"v5\":%.2f,\"v6\":%.2f,\"v7\":%.2f,\"v8\":%.2f}",
            sim_val(20.0f,5.0f),sim_val(21.0f,5.0f),
            sim_val(22.0f,5.0f),sim_val(23.0f,5.0f),
            sim_val(24.0f,5.0f),sim_val(25.0f,5.0f),
            sim_val(26.0f,5.0f),sim_val(27.0f,5.0f),
            sim_val(60.0f,5.0f),sim_val(61.0f,5.0f),
            sim_val(62.0f,5.0f),sim_val(63.0f,5.0f),
            sim_val(64.0f,5.0f),sim_val(65.0f,5.0f),
            sim_val(66.0f,5.0f),sim_val(67.0f,5.0f),
            sim_val(3.60f,0.4f),sim_val(3.61f,0.4f),
            sim_val(3.62f,0.4f),sim_val(3.63f,0.4f),
            sim_val(3.64f,0.4f),sim_val(3.65f,0.4f),
            sim_val(3.66f,0.4f),sim_val(3.67f,0.4f));

    } else {
        return snprintf(buf, buf_size,
            "{\"t1\":%.1f,\"t2\":%.1f,\"t3\":%.1f,\"t4\":%.1f,"
            "\"t5\":%.1f,\"t6\":%.1f,\"t7\":%.1f,\"t8\":%.1f,"
            "\"t9\":%.1f,\"t10\":%.1f,"
            "\"h1\":%.1f,\"h2\":%.1f,\"h3\":%.1f,\"h4\":%.1f,"
            "\"h5\":%.1f,\"h6\":%.1f,\"h7\":%.1f,\"h8\":%.1f,"
            "\"h9\":%.1f,\"h10\":%.1f,"
            "\"v1\":%.2f,\"v2\":%.2f,\"v3\":%.2f,\"v4\":%.2f,"
            "\"v5\":%.2f,\"v6\":%.2f,\"v7\":%.2f,\"v8\":%.2f,"
            "\"v9\":%.2f,\"v10\":%.2f}",
            sim_val(20.0f,5.0f),sim_val(21.0f,5.0f),
            sim_val(22.0f,5.0f),sim_val(23.0f,5.0f),
            sim_val(24.0f,5.0f),sim_val(25.0f,5.0f),
            sim_val(26.0f,5.0f),sim_val(27.0f,5.0f),
            sim_val(28.0f,5.0f),sim_val(29.0f,5.0f),
            sim_val(60.0f,5.0f),sim_val(61.0f,5.0f),
            sim_val(62.0f,5.0f),sim_val(63.0f,5.0f),
            sim_val(64.0f,5.0f),sim_val(65.0f,5.0f),
            sim_val(66.0f,5.0f),sim_val(67.0f,5.0f),
            sim_val(68.0f,5.0f),sim_val(69.0f,5.0f),
            sim_val(3.60f,0.4f),sim_val(3.61f,0.4f),
            sim_val(3.62f,0.4f),sim_val(3.63f,0.4f),
            sim_val(3.64f,0.4f),sim_val(3.65f,0.4f),
            sim_val(3.66f,0.4f),sim_val(3.67f,0.4f),
            sim_val(3.68f,0.4f),sim_val(3.69f,0.4f));
    }
}

// ════════════════════════════════════════════════════════════
// CHỜ ACK
// ════════════════════════════════════════════════════════════
static bool wait_ack(uint8_t expected_idx) {
    uint32_t start = millis();
    while (millis() - start < ACK_TIMEOUT_MS) {
        if (e32.available() >= ACK_FRAME_LEN) {
            ResponseStructContainer rsc = e32.receiveMessage(ACK_FRAME_LEN);
            if (rsc.status.code != SUCCESS) {
                rsc.close();
                vTaskDelay(pdMS_TO_TICKS(POLL_WAIT_MS));
                continue;
            }
            uint8_t* p = (uint8_t*)rsc.data;
            Serial.printf("[0x%02X] ACK bytes: %02X %02X %02X %02X \n",
                          NODE_ADDL, p[0],p[1],p[2],p[3]);
            if (p[0] == CMD_ACK && p[3] == expected_idx) {
                Serial.printf("[0x%02X] ACK(%d) OK\n", NODE_ADDL, expected_idx);
                rsc.close();
                return true;
            }
            else if (p[0] == CMD_POLL) {
                // Bỏ qua POLL khi đang chờ ACK
                Serial.printf("[0x%02X] Bỏ qua POLL khi đang chờ ACK\n", NODE_ADDL);
                rsc.close();
                continue;
            }
            rsc.close();
        }
        vTaskDelay(pdMS_TO_TICKS(POLL_WAIT_MS));
    }
    Serial.printf("[0x%02X] ACK(%d) timeout!\n", NODE_ADDL, expected_idx);
    return false;
}

// ════════════════════════════════════════════════════════════
// GỬI DATA FRAGMENTS
// ════════════════════════════════════════════════════════════
static void send_data(int test_case) {
    static char json[512];
    int jlen = build_json(test_case, json, sizeof(json));
    if (jlen <= 0 || jlen >= (int)sizeof(json)) {
        Serial.println("[ERR] JSON lỗi");
        return;
    }

    const char* cases[] = {"SMALL(4)", "MEDIUM(24)", "LARGE(30)"};
    uint8_t frag_total = (jlen + FRAG_PAYLOAD_MAX - 1) / FRAG_PAYLOAD_MAX;
    Serial.printf("[0x%02X] Case=%s  JSON=%dB  frags=%d\n",
                  NODE_ADDL, cases[test_case], jlen, frag_total);

    for (uint8_t idx = 0; idx < frag_total; idx++) {
        uint16_t offset  = idx * FRAG_PAYLOAD_MAX;
        uint8_t  pay_len = (uint8_t)min((int)FRAG_PAYLOAD_MAX, jlen - (int)offset);

        uint8_t frame[55];
        frame[0] = CMD_DATA;
        frame[1] = NODE_ADDH;
        frame[2] = NODE_ADDL;
        frame[3] = idx;
        frame[4] = frag_total;
        frame[5] = pay_len;
        memcpy(&frame[6], json + offset, pay_len);
        uint8_t frame_len = 6 + pay_len;

        bool ack_ok = false;

        for (uint8_t tx_try = 0; tx_try < FRAG_TX_RETRY; tx_try++) {

            ResponseStatus rs = e32.sendFixedMessage(
                GW_ADDH, GW_ADDL, LORA_CH, frame, frame_len);

            Serial.printf("[0x%02X] Chuẩn bị gửi frag %d  offset=%d  pay_len=%d\n",
              NODE_ADDL, idx, offset, pay_len);

            if (rs.code != SUCCESS) {
                Serial.printf("[0x%02X] TX frag %d FAIL code=%d\n",
                              NODE_ADDL, idx, rs.code);
                break;
            }
            //vTaskDelay(pdMS_TO_TICKS(200));
            Serial.printf("[0x%02X] TX frag %d/%d  len=%d  try=%d\n",
                          NODE_ADDL, idx, frag_total-1, pay_len, tx_try+1);

            if (idx == frag_total - 1) {
                ack_ok = true;
                break;
            }

            if (wait_ack(idx)) {
                ack_ok = true;
                break;
            }

            Serial.printf("[0x%02X] Retry frag %d (lần %d/%d)\n",
                          NODE_ADDL, idx, tx_try+1, FRAG_TX_RETRY);
        }

        if (!ack_ok) {
            Serial.printf("[0x%02X] Huỷ tại frag %d sau %d lần thử\n",
                          NODE_ADDL, idx, FRAG_TX_RETRY);
            return;
        }
    }

    Serial.printf("[0x%02X] Gửi xong\n", NODE_ADDL);
}

// ════════════════════════════════════════════════════════════
// POLLING TASK – task duy nhất xử lý LoRa
// Core 0: PollingTask
// Core 1: dành cho OTA
// ════════════════════════════════════════════════════════════
static void PollingTask(void* pv) {
    Serial.printf("[PollingTask] Bắt đầu, chờ POLL...\n");

    while (true) {
        // Chờ đủ bytes
        if (e32.available() < POLL_FRAME_LEN) {
            vTaskDelay(pdMS_TO_TICKS(POLL_WAIT_MS));
            continue;
        }

        ResponseStructContainer rsc = e32.receiveMessage(POLL_FRAME_LEN);
        if (rsc.status.code != SUCCESS) {
            rsc.close();
            continue;
        }

        uint8_t* p = (uint8_t*)rsc.data;
        Serial.printf("[0x%02X] RX: %02X %02X %02X %02X\n",
                      NODE_ADDL, p[0],p[1],p[2],p[3]);

        if (p[0] != CMD_POLL)                    { rsc.close(); continue; }
        if (p[1] != GW_ADDH || p[2] != GW_ADDL) { rsc.close(); continue; }
        if (xor_chk(p, 3) != p[3])              { rsc.close(); continue; }

        rsc.close();
        Serial.printf("[0x%02X] POLL hợp lệ\n", NODE_ADDL);

        // Chọn random case và gửi
        int test_case = random(0, 3);
        const char* cases[] = {"SMALL(4)", "MEDIUM(24)", "LARGE(30)"};
        Serial.printf("[0x%02X] Case = %d (%s)\n", NODE_ADDL, test_case, cases[test_case]);
        send_data(test_case);
    }
}

// ════════════════════════════════════════════════════════════
// SETUP
// ════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    delay(500);
    randomSeed(esp_random());

    Serial.printf("\n[Node 0x%02X] Khởi động...\n", NODE_ADDL);

    loraSerial.begin(LORA_BAUD, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN);
    e32.begin();

    ResponseStructContainer csc = e32.getConfiguration();
    if (csc.status.code == SUCCESS) {
        Configuration cfg = *(Configuration*)csc.data;
        cfg.ADDH = NODE_ADDH;
        cfg.ADDL = NODE_ADDL;
        cfg.CHAN  = LORA_CH;
        cfg.OPTION.fixedTransmission = FT_FIXED_TRANSMISSION;
        e32.setConfiguration(cfg, WRITE_CFG_PWR_DWN_SAVE);
        Serial.printf("[Node 0x%02X] LoRa OK\n", NODE_ADDL);
    }
    csc.close();

    // Core 0: PollingTask | Core 1: dành cho OTA
    xTaskCreatePinnedToCore(PollingTask, "PollingTask", 8192, NULL, 2, NULL, 0);

    Serial.printf("[Node 0x%02X] Sẵn sàng\n", NODE_ADDL);
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}
