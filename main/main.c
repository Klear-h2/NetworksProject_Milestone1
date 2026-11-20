/* MCT431 Production station, classic ESP32 DevKit UART0 via USB-UART bridge.
 * Uses SLIP + ATR header + CRC16/CCITT-FALSE and runtime CONFIG/START commands.
 */
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdint.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "driver/uart.h"
#include "cJSON.h"
#include "wire.h"

#define CMD_CACHE 48
#define SERIAL_BAUD 115200

static uint32_t cycle_ms = 500;
static uint32_t target = 0;
static uint32_t batch_id = 0;
static uint32_t produced = 0;
static bool running = false, paused = false, pending = false, blocked = false;
static uint64_t last_produced_ms = 0, last_part_send_ms = 0, last_heartbeat_ms = 0;
static int retries = 0;
static char pending_id[64] = "";
static char cache[CMD_CACHE][64];
static uint8_t cache_ok[CMD_CACHE];
static unsigned cache_pos = 0;

static uint64_t now_ms(void) { return (uint64_t) esp_timer_get_time()/1000ULL; }
static const char *getstr(const cJSON *o, const char *field) {
    const cJSON *v = cJSON_GetObjectItemCaseSensitive(o, field);
    return cJSON_IsString(v) ? v->valuestring : NULL;
}
static bool get_u32(const cJSON *o, const char *field, uint32_t min, uint32_t max, uint32_t *out) {
    const cJSON *v = cJSON_GetObjectItemCaseSensitive(o, field);
    if (!cJSON_IsNumber(v) || v->valuedouble < min || v->valuedouble > max) return false;
    uint32_t u = (uint32_t)v->valuedouble;
    if ((double)u != v->valuedouble) return false;
    *out = u; return true;
}
static void send_ack(const char *ref, const char *phase, const char *status) {
    cJSON *a = cJSON_CreateObject();
    cJSON_AddStringToObject(a, "type", "ACK");
    cJSON_AddStringToObject(a, "ref_id", ref);
    cJSON_AddStringToObject(a, "phase", phase);
    cJSON_AddStringToObject(a, "status", status);
    mct_send(a); cJSON_Delete(a);
}
static void send_part(void) {
    cJSON *p = cJSON_CreateObject();
    cJSON_AddStringToObject(p, "type", "PART");
    cJSON_AddStringToObject(p, "job_id", pending_id);
    cJSON_AddNumberToObject(p, "batch_id", batch_id);
    cJSON_AddNumberToObject(p, "part_id", produced);
    mct_send(p); cJSON_Delete(p);
    last_part_send_ms = now_ms();
}
static void process(cJSON *o) {
    const char *type = getstr(o, "type");
    if (!type) return;
    if (!strcmp(type, "ACK")) {
        const char *ref = getstr(o, "ref_id");
        const char *phase = getstr(o, "phase");
        if (pending && ref && phase && !strcmp(ref, pending_id)) {
            if (!strcmp(phase, "ACCEPTANCE")) {
                pending = false; blocked = false; retries = 0;
            } else if (!strcmp(phase, "BLOCKED")) {
                blocked = true;
            } else if (!strcmp(phase, "REJECTED")) {
                pending = false; running = false;
            }
        }
        return;
    }
    if (strcmp(type, "COMMAND")) return;
    const char *ref = getstr(o, "cmd_id");
    const char *cmd = getstr(o, "command");
    if (!ref || !cmd || !ref[0] || strlen(ref) >= 64) return;
    send_ack(ref, "DELIVERY", "OK");
    for (int i = 0; i < CMD_CACHE; ++i) {
        if (!strcmp(cache[i], ref)) {
            if (cache_ok[i]) { send_ack(ref, "ACCEPTANCE", "OK"); send_ack(ref, "COMPLETION", "OK"); }
            else send_ack(ref, "REJECTED", "INVALID_COMMAND");
            return;
        }
    }
    bool valid = true;
    if (!strcmp(cmd, "CONFIG")) {
        uint32_t value;
        valid = get_u32(o, "production_ms", 100, 30000, &value);
        if (valid) cycle_ms = value;
    } else if (!strcmp(cmd, "START")) {
        uint32_t new_target, new_batch;
        valid = get_u32(o, "target", 1, 1000, &new_target) && get_u32(o, "batch_id", 1, 2147483647, &new_batch);
        if (valid) {
            target = new_target; batch_id = new_batch; produced = 0;
            pending = false; blocked = false; paused = false; running = true;
            last_produced_ms = now_ms();
        }
    } else if (!strcmp(cmd, "PAUSE")) {
        paused = true;
    } else if (!strcmp(cmd, "RESUME")) {
        paused = false;
        last_produced_ms = now_ms();
    } else if (!strcmp(cmd, "STOP")) {
        running = false; paused = false; pending = false; blocked = false;
    } else valid = false;
    snprintf(cache[cache_pos], sizeof(cache[cache_pos]), "%s", ref);
    cache_ok[cache_pos] = valid ? 1 : 0;
    cache_pos = (cache_pos + 1) % CMD_CACHE;
    if (valid) { send_ack(ref, "ACCEPTANCE", "OK"); send_ack(ref, "COMPLETION", "OK"); }
    else send_ack(ref, "REJECTED", "INVALID_COMMAND");
}

void app_main(void) {
    uart_config_t cfg = { .baud_rate = SERIAL_BAUD, .data_bits = UART_DATA_8_BITS,
                          .parity = UART_PARITY_DISABLE, .stop_bits = UART_STOP_BITS_1,
                          .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_DEFAULT };
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 4096, 4096, 0, NULL, 0));

    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &cfg));

    ESP_ERROR_CHECK(uart_set_pin(
        UART_NUM_0,
        1,
        3,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE
    ));
    slip_parser_t parser = {0};
    uint8_t rx[256];
    for (;;) {
        int n = uart_read_bytes(UART_NUM_0, rx, sizeof(rx), pdMS_TO_TICKS(12));
        for (int i = 0; i < n; ++i) {
            cJSON *o = mct_feed(&parser, rx[i]);
            if (o) { process(o); cJSON_Delete(o); }
        }
        uint64_t now = now_ms();
        if (now - last_heartbeat_ms >= 2000) {
            last_heartbeat_ms = now;
            cJSON *h = cJSON_CreateObject();
            cJSON_AddStringToObject(h, "type", "HEARTBEAT");
            cJSON_AddStringToObject(h, "device", "ESP32_A");
            cJSON_AddNumberToObject(h, "heartbeat_id", now);
            if (pending) cJSON_AddStringToObject(h, "pending_job_id", pending_id);
            mct_send(h); cJSON_Delete(h);
        }
        if (!running || paused) continue;
        if (pending) {
            if (!blocked && now - last_part_send_ms > 8000) {
                if (retries >= 3) {
                    cJSON *err = cJSON_CreateObject();
                    cJSON_AddStringToObject(err, "type", "FAULT");
                    cJSON_AddStringToObject(err, "reason", "PART acknowledgement timeout");
                    mct_send(err); cJSON_Delete(err);
                    running = false;
                } else { send_part(); ++retries; }
            }
        } else if (produced < target && now - last_produced_ms >= cycle_ms) {
            ++produced;
            snprintf(pending_id, sizeof(pending_id), "B%" PRIu32 "-P%" PRIu32, batch_id, produced);
            pending = true; blocked = false; retries = 0;
            last_produced_ms = now;
            send_part();
        }
    }
}
