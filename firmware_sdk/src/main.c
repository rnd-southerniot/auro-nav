/*
 * Airborne Classic — PicoSDK firmware v2
 * Adds: closed-loop PID, ICM-20948 IMU, UART transport option
 *
 * Hardware: Cytron Maker Pi RP2040
 * Encoders: PIO-based quadrature (GP2/3 left, GP4/5 right)
 * Motors: H-bridge GP8-11 @ 20kHz PWM
 * IMU: ICM-20948 on I2C1 (GP6 SDA, GP7 SCL)
 * UART: TX=GP0, RX=GP1 (uart0) for Pi 5 bridge
 *
 * Build: see firmware_sdk/CMakeLists.txt
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"
#include "hardware/pwm.h"
#include "hardware/uart.h"
#include "pico/stdlib.h"
#include "quadrature_sample.pio.h"

/* ── Pin definitions (from HARDWARE_MAP.md) ────────────────────────── */
#define ENC_L_PIN_BASE   2u
#define ENC_R_PIN_BASE   4u
#define IMU_SDA_PIN      6u
#define IMU_SCL_PIN      7u
#define MOTOR_L_A_PIN    8u
#define MOTOR_L_B_PIN    9u
#define MOTOR_R_A_PIN   10u
#define MOTOR_R_B_PIN   11u

/* ── Motor/encoder constants ───────────────────────────────────────── */
#define ENCODER_SAMPLE_HZ      2000.0f
#define MOTOR_PWM_HZ          20000u
#define MOTOR_PWM_WRAP         6249u
#define ENC_L_SIGN                1
#define ENC_R_SIGN               -1
#define COUNTS_PER_WHEEL_REV_L  582.0f
#define COUNTS_PER_WHEEL_REV_R  583.0f

/* ── Kinematics (from HARDWARE_MAP.md / config.json) ───────────────── */
#define WHEEL_DIAMETER_M   0.054875f
#define WHEEL_RADIUS_M     (WHEEL_DIAMETER_M / 2.0f)
#define TRACK_WIDTH_M      0.155f

/* ── PID defaults (tunable via command) ────────────────────────────── */
#define PID_KP_DEFAULT     2.0f
#define PID_KI_DEFAULT     8.0f
#define PID_KD_DEFAULT     0.05f
#define PID_INTEGRAL_MAX   1.0f    /* anti-windup clamp */
#define PID_DT_S           0.02f   /* 50 Hz control loop */
#define CONTROL_LOOP_MS    20u     /* = 1/PID_DT_S in ms */

/* ── Telemetry rate ────────────────────────────────────────────────── */
#define TELE_PERIOD_MS     50u     /* 20 Hz telemetry */

/* ── ICM-20948 registers ───────────────────────────────────────────── */
#define ICM20948_ADDR       0x68
#define ICM20948_ADDR_ALT   0x69
#define ICM20948_WHO_AM_I   0x00
#define ICM20948_WHO_AM_I_VAL 0xEA
#define ICM20948_PWR_MGMT_1 0x06
#define ICM20948_PWR_MGMT_2 0x07
#define ICM20948_GYRO_XOUT_H 0x33
#define ICM20948_REG_BANK_SEL 0x7F
#define ICM20948_GYRO_CONFIG_1 0x01  /* Bank 2 */

/* Gyro full-scale: 500 dps -> 65.5 LSB/dps */
#define GYRO_FS_SEL        1       /* 0=250, 1=500, 2=1000, 3=2000 dps */
#define GYRO_SENSITIVITY   65.5f   /* LSB per dps for 500 dps */

/* ── Line buffer ───────────────────────────────────────────────────── */
#define LINE_BUF_SIZE 256

/* ──────────────────────────────────────────────────────────────────── */
/*                          DATA STRUCTURES                            */
/* ──────────────────────────────────────────────────────────────────── */

typedef struct {
    float kp, ki, kd;
    float integral;
    float prev_error;
    float output;       /* clamped [-1, 1] PWM output */
} pid_state_t;

typedef struct {
    PIO pio;
    uint sm;
    uint pin_base;
    uint8_t prev_state;
    int32_t count;
} encoder_reader_t;

typedef struct {
    float gyro_z_dps;       /* raw gyro Z in degrees/sec */
    float yaw_deg;          /* integrated yaw in degrees */
    float gyro_z_bias;      /* calibrated bias (dps) */
    bool  calibrated;
    uint8_t i2c_addr;       /* detected address */
    bool  present;
} imu_state_t;

typedef struct {
    bool    armed;
    float   pwm_l, pwm_r;          /* actual PWM output [-1,1] */
    float   target_rpm_l, target_rpm_r;  /* PID setpoints */
    int32_t enc_l, enc_r;           /* cumulative encoder counts */
    float   rpm_l, rpm_r;           /* measured RPM */
    float   yaw_deg;                /* from IMU */
    bool    pid_enabled;            /* false = open-loop */
    pid_state_t pid_l, pid_r;
    imu_state_t imu;
} robot_state_t;

/* ──────────────────────────────────────────────────────────────────── */
/*                           UTILITY                                   */
/* ──────────────────────────────────────────────────────────────────── */

static uint32_t now_ms(void) {
    return to_ms_since_boot(get_absolute_time());
}

static float clamp_unit(float v) {
    return (v > 1.0f) ? 1.0f : (v < -1.0f) ? -1.0f : v;
}

static float clampf(float v, float lo, float hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

/* ──────────────────────────────────────────────────────────────────── */
/*                        PID CONTROLLER                               */
/* ──────────────────────────────────────────────────────────────────── */

static void pid_init(pid_state_t *p, float kp, float ki, float kd) {
    p->kp = kp;
    p->ki = ki;
    p->kd = kd;
    p->integral = 0.0f;
    p->prev_error = 0.0f;
    p->output = 0.0f;
}

static void pid_reset(pid_state_t *p) {
    p->integral = 0.0f;
    p->prev_error = 0.0f;
    p->output = 0.0f;
}

static float pid_update(pid_state_t *p, float setpoint, float measured, float dt) {
    float error = setpoint - measured;

    /* Proportional */
    float p_term = p->kp * error;

    /* Integral with anti-windup */
    p->integral += error * dt;
    p->integral = clampf(p->integral, -PID_INTEGRAL_MAX, PID_INTEGRAL_MAX);
    float i_term = p->ki * p->integral;

    /* Derivative (on error, with simple filtering) */
    float d_term = 0.0f;
    if (dt > 0.0f) {
        d_term = p->kd * (error - p->prev_error) / dt;
    }
    p->prev_error = error;

    /* Output clamped to [-1, 1] normalized PWM */
    p->output = clamp_unit(p_term + i_term + d_term);
    return p->output;
}

/* ──────────────────────────────────────────────────────────────────── */
/*                      ICM-20948 IMU DRIVER                           */
/* ──────────────────────────────────────────────────────────────────── */

static i2c_inst_t *imu_i2c = i2c1;

static bool imu_write_reg(uint8_t addr, uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    return i2c_write_blocking(imu_i2c, addr, buf, 2, false) == 2;
}

static bool imu_read_reg(uint8_t addr, uint8_t reg, uint8_t *val) {
    if (i2c_write_blocking(imu_i2c, addr, &reg, 1, true) != 1) return false;
    return i2c_read_blocking(imu_i2c, addr, val, 1, false) == 1;
}

static bool imu_read_bytes(uint8_t addr, uint8_t reg, uint8_t *buf, size_t len) {
    if (i2c_write_blocking(imu_i2c, addr, &reg, 1, true) != 1) return false;
    return i2c_read_blocking(imu_i2c, addr, buf, (uint)len, false) == (int)len;
}

static void imu_select_bank(uint8_t addr, uint8_t bank) {
    imu_write_reg(addr, ICM20948_REG_BANK_SEL, (uint8_t)(bank << 4));
}

static bool imu_init(imu_state_t *imu) {
    uint8_t who = 0;

    /* Try both I2C addresses */
    imu->present = false;
    imu->calibrated = false;
    imu->gyro_z_bias = 0.0f;
    imu->yaw_deg = 0.0f;
    imu->gyro_z_dps = 0.0f;

    for (uint8_t try_addr = ICM20948_ADDR; try_addr <= ICM20948_ADDR_ALT; try_addr++) {
        if (imu_read_reg(try_addr, ICM20948_WHO_AM_I, &who) && who == ICM20948_WHO_AM_I_VAL) {
            imu->i2c_addr = try_addr;
            imu->present = true;
            break;
        }
    }
    if (!imu->present) return false;

    uint8_t addr = imu->i2c_addr;

    /* Bank 0: wake up, auto-select clock */
    imu_select_bank(addr, 0);
    imu_write_reg(addr, ICM20948_PWR_MGMT_1, 0x01);  /* auto clock, no sleep */
    sleep_ms(30);
    imu_write_reg(addr, ICM20948_PWR_MGMT_2, 0x07);  /* disable accel, enable gyro */

    /* Bank 2: configure gyro 500 dps */
    imu_select_bank(addr, 2);
    imu_write_reg(addr, ICM20948_GYRO_CONFIG_1,
                  (uint8_t)((GYRO_FS_SEL << 1) | 0x01)); /* FS=500dps, DLPF enable */

    /* Back to bank 0 for reading */
    imu_select_bank(addr, 0);

    return true;
}

static void imu_read_gyro_z(imu_state_t *imu) {
    if (!imu->present) return;

    uint8_t buf[2];
    /* Gyro Z is at offset +4 from GYRO_XOUT_H (bytes 4,5 of 6-byte block) */
    if (imu_read_bytes(imu->i2c_addr, (uint8_t)(ICM20948_GYRO_XOUT_H + 4), buf, 2)) {
        int16_t raw = (int16_t)((buf[0] << 8) | buf[1]);
        imu->gyro_z_dps = (float)raw / GYRO_SENSITIVITY - imu->gyro_z_bias;
    }
}

static void imu_integrate_yaw(imu_state_t *imu, float dt) {
    if (!imu->present || !imu->calibrated) return;
    /* Dead-zone: ignore noise below 0.5 dps */
    if (fabsf(imu->gyro_z_dps) > 0.5f) {
        imu->yaw_deg += imu->gyro_z_dps * dt;
    }
    /* Wrap to [-180, 180] */
    while (imu->yaw_deg > 180.0f) imu->yaw_deg -= 360.0f;
    while (imu->yaw_deg < -180.0f) imu->yaw_deg += 360.0f;
}

static bool imu_calibrate(imu_state_t *imu, uint16_t samples) {
    if (!imu->present) return false;

    float sum = 0.0f;
    for (uint16_t i = 0; i < samples; i++) {
        uint8_t buf[2];
        if (imu_read_bytes(imu->i2c_addr, (uint8_t)(ICM20948_GYRO_XOUT_H + 4), buf, 2)) {
            int16_t raw = (int16_t)((buf[0] << 8) | buf[1]);
            sum += (float)raw / GYRO_SENSITIVITY;
        }
        sleep_ms(2);
    }
    imu->gyro_z_bias = sum / (float)samples;
    imu->yaw_deg = 0.0f;
    imu->calibrated = true;
    return true;
}

/* ──────────────────────────────────────────────────────────────────── */
/*                      ENCODER (PIO) READER                           */
/* ──────────────────────────────────────────────────────────────────── */

static int quad_delta(uint8_t prev, uint8_t cur) {
    if (prev == cur) return 0;
    if ((prev==0&&cur==1)||(prev==1&&cur==3)||(prev==3&&cur==2)||(prev==2&&cur==0)) return 1;
    if ((prev==0&&cur==2)||(prev==2&&cur==3)||(prev==3&&cur==1)||(prev==1&&cur==0)) return -1;
    return 0;
}

static uint8_t read_encoder_state_from_gpio(uint pin_base) {
    return (uint8_t)((gpio_get(pin_base) << 1u) | gpio_get(pin_base + 1u));
}

static void encoder_reader_init(encoder_reader_t *enc, PIO pio, uint sm, uint pin_base, uint offset) {
    pio_sm_config cfg = quadrature_sample_program_get_default_config(offset);
    enc->pio = pio; enc->sm = sm; enc->pin_base = pin_base; enc->count = 0;
    pio_gpio_init(pio, pin_base);
    pio_gpio_init(pio, pin_base + 1u);
    gpio_pull_up(pin_base);
    gpio_pull_up(pin_base + 1u);
    pio_sm_set_consecutive_pindirs(pio, sm, pin_base, 2, false);
    sm_config_set_in_pins(&cfg, pin_base);
    sm_config_set_clkdiv(&cfg, 125000000.0f / ENCODER_SAMPLE_HZ);
    pio_sm_init(pio, sm, offset, &cfg);
    enc->prev_state = read_encoder_state_from_gpio(pin_base);
    pio_sm_set_enabled(pio, sm, true);
}

static void encoder_reader_poll(encoder_reader_t *enc) {
    while (!pio_sm_is_rx_fifo_empty(enc->pio, enc->sm)) {
        uint32_t raw = pio_sm_get(enc->pio, enc->sm);
        uint8_t cur = (uint8_t)((raw >> 30u) & 0x3u);
        enc->count += quad_delta(enc->prev_state, cur);
        enc->prev_state = cur;
    }
}

static float counts_to_rpm(int32_t delta_counts, uint32_t delta_ms, float counts_per_rev) {
    if (delta_ms == 0u || counts_per_rev <= 0.0f) return 0.0f;
    return ((float)delta_counts * 60000.0f) / (counts_per_rev * (float)delta_ms);
}

/* ──────────────────────────────────────────────────────────────────── */
/*                        MOTOR DRIVER                                 */
/* ──────────────────────────────────────────────────────────────────── */

static uint16_t unit_to_pwm_level(float v) {
    return (uint16_t)(fabsf(clamp_unit(v)) * (float)MOTOR_PWM_WRAP);
}

static void drive_hbridge(uint pin_a, uint pin_b, float v) {
    uint16_t level = unit_to_pwm_level(v);
    if (v > 0.0f) {
        pwm_set_gpio_level(pin_a, level);
        pwm_set_gpio_level(pin_b, 0u);
    } else if (v < 0.0f) {
        pwm_set_gpio_level(pin_a, 0u);
        pwm_set_gpio_level(pin_b, level);
    } else {
        pwm_set_gpio_level(pin_a, 0u);
        pwm_set_gpio_level(pin_b, 0u);
    }
}

static void apply_motor_outputs(const robot_state_t *s) {
    float left  = s->armed ? clamp_unit(s->pwm_l) : 0.0f;
    float right = s->armed ? clamp_unit(s->pwm_r) : 0.0f;
    drive_hbridge(MOTOR_L_A_PIN, MOTOR_L_B_PIN, left);
    drive_hbridge(MOTOR_R_A_PIN, MOTOR_R_B_PIN, right);
}

static void motor_pwm_init(void) {
    uint pins[] = {MOTOR_L_A_PIN, MOTOR_L_B_PIN, MOTOR_R_A_PIN, MOTOR_R_B_PIN};
    for (size_t i = 0; i < sizeof(pins)/sizeof(pins[0]); i++)
        gpio_set_function(pins[i], GPIO_FUNC_PWM);

    uint slice_l = pwm_gpio_to_slice_num(MOTOR_L_A_PIN);
    uint slice_r = pwm_gpio_to_slice_num(MOTOR_R_A_PIN);
    pwm_set_wrap(slice_l, MOTOR_PWM_WRAP); pwm_set_clkdiv(slice_l, 1.0f); pwm_set_enabled(slice_l, true);
    pwm_set_wrap(slice_r, MOTOR_PWM_WRAP); pwm_set_clkdiv(slice_r, 1.0f); pwm_set_enabled(slice_r, true);
    for (size_t i = 0; i < sizeof(pins)/sizeof(pins[0]); i++)
        pwm_set_gpio_level(pins[i], 0u);
}

/* ──────────────────────────────────────────────────────────────────── */
/*                     NDJSON EMIT HELPERS                             */
/* ──────────────────────────────────────────────────────────────────── */

static void emit_tele(const robot_state_t *s) {
    printf(
        "{\"type\":\"tele\",\"ts\":%lu,\"mode\":\"%s\",\"armed\":%s,"
        "\"pwm_l\":%.3f,\"pwm_r\":%.3f,\"enc_l\":%ld,\"enc_r\":%ld,"
        "\"rpm_l\":%.1f,\"rpm_r\":%.1f,\"yaw\":%.2f,"
        "\"pid\":%s,\"fault\":null}\n",
        (unsigned long)now_ms(),
        s->armed ? "ARMED" : "DISARMED",
        s->armed ? "true" : "false",
        s->pwm_l, s->pwm_r,
        (long)s->enc_l, (long)s->enc_r,
        s->rpm_l, s->rpm_r,
        s->yaw_deg,
        s->pid_enabled ? "true" : "false"
    );
}

static void emit_ack_ok(const char *id, const char *data) {
    if (data)
        printf("{\"type\":\"ack\",\"id\":\"%s\",\"ok\":true,\"data\":%s}\n", id, data);
    else
        printf("{\"type\":\"ack\",\"id\":\"%s\",\"ok\":true}\n", id);
}

static void emit_ack_err(const char *id, const char *code, const char *msg) {
    printf("{\"type\":\"ack\",\"id\":\"%s\",\"ok\":false,\"error\":{\"code\":\"%s\",\"message\":\"%s\"}}\n",
           id, code, msg);
}

/* ──────────────────────────────────────────────────────────────────── */
/*                    JSON FIELD EXTRACTION                            */
/* ──────────────────────────────────────────────────────────────────── */

static bool extract_string_field(const char *line, const char *field, char *out, size_t out_size) {
    const char *p = strstr(line, field);
    if (!p || out_size == 0) return false;
    p = strchr(p, ':'); if (!p) return false;
    const char *start = strchr(p, '"'); if (!start) return false;
    start++;
    const char *end = strchr(start, '"'); if (!end) return false;
    size_t n = (size_t)(end - start);
    if (n >= out_size) n = out_size - 1;
    memcpy(out, start, n); out[n] = '\0';
    return true;
}

static bool extract_number_field(const char *line, const char *field, float *val) {
    const char *p = strstr(line, field); if (!p) return false;
    p = strchr(p, ':'); if (!p) return false;
    p++;
    char *endptr; float v = strtof(p, &endptr);
    if (endptr == p) return false;
    *val = v; return true;
}

/* ──────────────────────────────────────────────────────────────────── */
/*                     COMMAND HANDLER                                 */
/* ──────────────────────────────────────────────────────────────────── */

static void handle_line(
    const char *line, robot_state_t *s,
    encoder_reader_t *enc_l, encoder_reader_t *enc_r,
    int32_t *last_enc_l, int32_t *last_enc_r, uint32_t *last_tele_ts
) {
    char id[64];
    float left = 0.0f, right = 0.0f;

    if (!extract_string_field(line, "\"id\"", id, sizeof(id)))
        { strncpy(id, "?", sizeof(id)); id[sizeof(id)-1] = '\0'; }

    if (!strstr(line, "\"type\":\"cmd\""))
        { emit_ack_err(id, "bad_type", "expected cmd"); return; }

    if (strstr(line, "\"cmd\":\"ping\"")) {
        emit_ack_ok(id, NULL); return;
    }
    if (strstr(line, "\"cmd\":\"version\"")) {
        emit_ack_ok(id, "{\"fw\":\"sdk-v2-pid\",\"robot\":\"airborne-classic\"}"); return;
    }
    if (strstr(line, "\"cmd\":\"arm\"")) {
        s->armed = true; emit_ack_ok(id, NULL); return;
    }
    if (strstr(line, "\"cmd\":\"disarm\"")) {
        s->armed = false; s->pwm_l = 0; s->pwm_r = 0;
        s->target_rpm_l = 0; s->target_rpm_r = 0;
        pid_reset(&s->pid_l); pid_reset(&s->pid_r);
        emit_ack_ok(id, NULL); return;
    }
    if (strstr(line, "\"cmd\":\"stop\"")) {
        s->pwm_l = 0; s->pwm_r = 0;
        s->target_rpm_l = 0; s->target_rpm_r = 0;
        pid_reset(&s->pid_l); pid_reset(&s->pid_r);
        emit_ack_ok(id, NULL); return;
    }
    if (strstr(line, "\"cmd\":\"set_pwm\"")) {
        if (!extract_number_field(line, "\"left\"", &left) ||
            !extract_number_field(line, "\"right\"", &right))
            { emit_ack_err(id, "bad_args", "need left/right"); return; }
        s->pid_enabled = false;
        s->pwm_l = s->armed ? clamp_unit(left) : 0.0f;
        s->pwm_r = s->armed ? clamp_unit(right) : 0.0f;
        emit_ack_ok(id, NULL); return;
    }
    if (strstr(line, "\"cmd\":\"set_rpm\"")) {
        if (!extract_number_field(line, "\"left\"", &left) ||
            !extract_number_field(line, "\"right\"", &right))
            { emit_ack_err(id, "bad_args", "need left/right"); return; }
        s->pid_enabled = true;
        s->target_rpm_l = left;
        s->target_rpm_r = right;
        /* If targets are zero, also zero PID state to avoid wind-up */
        if (fabsf(left) < 0.1f && fabsf(right) < 0.1f) {
            pid_reset(&s->pid_l); pid_reset(&s->pid_r);
            s->pwm_l = 0; s->pwm_r = 0;
        }
        emit_ack_ok(id, NULL); return;
    }
    if (strstr(line, "\"cmd\":\"set_pid\"")) {
        float kp, ki, kd;
        if (!extract_number_field(line, "\"kp\"", &kp) ||
            !extract_number_field(line, "\"ki\"", &ki) ||
            !extract_number_field(line, "\"kd\"", &kd))
            { emit_ack_err(id, "bad_args", "need kp/ki/kd"); return; }
        s->pid_l.kp = kp; s->pid_l.ki = ki; s->pid_l.kd = kd;
        s->pid_r.kp = kp; s->pid_r.ki = ki; s->pid_r.kd = kd;
        pid_reset(&s->pid_l); pid_reset(&s->pid_r);
        char buf[128];
        snprintf(buf, sizeof(buf), "{\"kp\":%.3f,\"ki\":%.3f,\"kd\":%.3f}", kp, ki, kd);
        emit_ack_ok(id, buf); return;
    }
    if (strstr(line, "\"cmd\":\"cal_encoders\"")) {
        enc_l->count = 0; enc_r->count = 0;
        s->enc_l = 0; s->enc_r = 0;
        s->rpm_l = 0; s->rpm_r = 0;
        *last_enc_l = 0; *last_enc_r = 0;
        *last_tele_ts = now_ms();
        emit_ack_ok(id, NULL); return;
    }
    if (strstr(line, "\"cmd\":\"cal_imu\"")) {
        if (!s->imu.present) {
            emit_ack_err(id, "no_imu", "ICM-20948 not detected"); return;
        }
        bool ok = imu_calibrate(&s->imu, 500);  /* ~1 second at 2ms/sample */
        if (ok) {
            char buf[128];
            snprintf(buf, sizeof(buf), "{\"bias_z\":%.4f,\"samples\":500}", s->imu.gyro_z_bias);
            emit_ack_ok(id, buf);
        } else {
            emit_ack_err(id, "cal_fail", "IMU calibration failed");
        }
        return;
    }
    emit_ack_err(id, "bad_cmd", "unknown cmd");
}

/* ──────────────────────────────────────────────────────────────────── */
/*                             MAIN                                    */
/* ──────────────────────────────────────────────────────────────────── */

int main(void) {
    stdio_init_all();
    sleep_ms(500);

    /* ── Motor PWM init ── */
    motor_pwm_init();

    /* ── I2C init for IMU ── */
    i2c_init(imu_i2c, 400000);
    gpio_set_function(IMU_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(IMU_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(IMU_SDA_PIN);
    gpio_pull_up(IMU_SCL_PIN);

    /* ── Encoder PIO init ── */
    uint offset = pio_add_program(pio0, &quadrature_sample_program);
    uint sm_l = pio_claim_unused_sm(pio0, true);
    uint sm_r = pio_claim_unused_sm(pio0, true);
    encoder_reader_t enc_l_reader, enc_r_reader;
    encoder_reader_init(&enc_l_reader, pio0, sm_l, ENC_L_PIN_BASE, offset);
    encoder_reader_init(&enc_r_reader, pio0, sm_r, ENC_R_PIN_BASE, offset);

    /* ── Robot state ── */
    robot_state_t state = {0};
    state.pid_enabled = true;  /* default to PID mode */
    pid_init(&state.pid_l, PID_KP_DEFAULT, PID_KI_DEFAULT, PID_KD_DEFAULT);
    pid_init(&state.pid_r, PID_KP_DEFAULT, PID_KI_DEFAULT, PID_KD_DEFAULT);

    /* ── IMU init ── */
    if (imu_init(&state.imu)) {
        /* Auto-calibrate at boot (robot must be stationary) */
        imu_calibrate(&state.imu, 250);  /* ~0.5s */
    }

    /* ── Timing ── */
    uint32_t last_tele_ts = now_ms();
    int32_t last_enc_l = 0, last_enc_r = 0;
    uint32_t last_control_ts = now_ms();
    int32_t ctrl_prev_enc_l = 0, ctrl_prev_enc_r = 0;

    /* ── Boot telemetry ── */
    emit_tele(&state);
    absolute_time_t next_tele = make_timeout_time_ms(TELE_PERIOD_MS);
    absolute_time_t next_control = make_timeout_time_ms(CONTROL_LOOP_MS);

    /* ── Line buffer ── */
    char line[LINE_BUF_SIZE];
    size_t line_len = 0;

    while (true) {
        /* ── Read serial input ── */
        int c = getchar_timeout_us(0);
        if (c != PICO_ERROR_TIMEOUT) {
            if (c == '\r' || c == '\n') {
                line[line_len] = '\0';
                if (line_len > 0)
                    handle_line(line, &state, &enc_l_reader, &enc_r_reader,
                                &last_enc_l, &last_enc_r, &last_tele_ts);
                line_len = 0;
            } else if (line_len + 1 < LINE_BUF_SIZE) {
                line[line_len++] = (char)c;
            } else {
                line_len = 0;
            }
        }

        /* ── Poll encoders ── */
        encoder_reader_poll(&enc_l_reader);
        encoder_reader_poll(&enc_r_reader);
        state.enc_l = ENC_L_SIGN * enc_l_reader.count;
        state.enc_r = ENC_R_SIGN * enc_r_reader.count;

        /* ── Read IMU ── */
        imu_read_gyro_z(&state.imu);

        /* ── 50 Hz PID control loop ── */
        if (absolute_time_diff_us(get_absolute_time(), next_control) <= 0) {
            uint32_t ctrl_now = now_ms();
            uint32_t ctrl_dt_ms = ctrl_now - last_control_ts;
            float dt = (float)ctrl_dt_ms / 1000.0f;

            /* Compute measured RPM from encoder deltas */
            int32_t dl = state.enc_l - ctrl_prev_enc_l;
            int32_t dr = state.enc_r - ctrl_prev_enc_r;
            state.rpm_l = counts_to_rpm(dl, ctrl_dt_ms, COUNTS_PER_WHEEL_REV_L);
            state.rpm_r = counts_to_rpm(dr, ctrl_dt_ms, COUNTS_PER_WHEEL_REV_R);
            ctrl_prev_enc_l = state.enc_l;
            ctrl_prev_enc_r = state.enc_r;
            last_control_ts = ctrl_now;

            /* Run PID if enabled and armed */
            if (state.pid_enabled && state.armed) {
                state.pwm_l = pid_update(&state.pid_l, state.target_rpm_l, state.rpm_l, dt);
                state.pwm_r = pid_update(&state.pid_r, state.target_rpm_r, state.rpm_r, dt);
            }

            /* Integrate IMU yaw */
            imu_integrate_yaw(&state.imu, dt);
            state.yaw_deg = state.imu.yaw_deg;

            next_control = make_timeout_time_ms(CONTROL_LOOP_MS);
        }

        /* ── Apply motor outputs ── */
        apply_motor_outputs(&state);

        /* ── 20 Hz telemetry ── */
        if (absolute_time_diff_us(get_absolute_time(), next_tele) <= 0) {
            /* Update tele RPM from most recent control loop values */
            uint32_t tele_ts = now_ms();
            uint32_t delta_ms = tele_ts - last_tele_ts;
            int32_t delta_l = state.enc_l - last_enc_l;
            int32_t delta_r = state.enc_r - last_enc_r;
            /* Use control-loop RPM (more accurate, already computed) */
            last_tele_ts = tele_ts;
            last_enc_l = state.enc_l;
            last_enc_r = state.enc_r;

            emit_tele(&state);
            next_tele = make_timeout_time_ms(TELE_PERIOD_MS);
        }

        sleep_us(200);
    }
}
