#include "chassis.h"
#include "pid.h"
#include "cmsis_os.h"
#include "tim.h"
#include <stdlib.h>
#include <string.h>

/* ══════════════════════════════════════════════════════════════════════
   Global Variables
   ══════════════════════════════════════════════════════════════════════ */
volatile uint8_t   g_emergency_stop  = 0;
ChassisState_t     g_chassis_state   = STATE_DEBUG;
uint32_t           g_state_ticks     = 0;
uint16_t           g_red_count       = 0;
uint16_t           g_green_count     = 0;
int32_t            g_enc_total       = 0;
int32_t            g_turn_enc        = 0;
float              g_avg_speed_mps   = 0.0f;
uint32_t           g_sys_tick        = 0;     // freertos.c 每周期 +1

/* ── 防重触发：摘完一株后短暂抑制 ── */
static uint32_t    g_suppress_until_tick = 0;

/* ══════════════════════════════════════════════════════════════════════
   Phase 2 文件级持久状态（保持巡线+避障不变）
   ══════════════════════════════════════════════════════════════════════ */

// LINE_FOLLOW 持久状态
static float    last_line_err   = 0.0f;
static uint8_t  lf_trig_cnt    = 0;       // 触发确认计数器
static uint8_t  lf_trig_side   = 0;       // 当前触发侧 0=none 1=left 2=right

// OBSTACLE 持久状态
static ObsSubStage_t ob_stage      = OB_STOP;
static int32_t       ob_start_enc  = 0;
static uint32_t      ob_ticks      = 0;
static int32_t       ob_mem_dist   = 0;
static int32_t       ob_turn_start = 0;

/* ══════════════════════════════════════════════════════════════════════
   Phase 3 文件级持久状态（草莓检测链 v5）
   ══════════════════════════════════════════════════════════════════════ */

// VISUAL_APPROACH 持久状态
static float   va_y_filt        = 120.0f;   // Y 一阶低通滤波输出
static float   va_last_x_center = 160.0f;   // 丢目标时的上帧 X_center
static int     va_center_cnt    = 0;        // X 居中连续帧计数
static int     va_no_target_ms  = 0;        // 丢目标累计 ms

// SCANNING 持久状态
static int     scan_tick        = 0;
static int     scan_max_red     = 0;
static int     scan_max_green   = 0;

// BREAKAWAY 转向持久状态
static int     ba_no_target_ms  = 0;

/* ══════════════════════════════════════════════════════════════════════
   Chassis_Move API — 只设 pid_X.Target，CalcOneWheel 统一处理
   ══════════════════════════════════════════════════════════════════════ */

void Chassis_MoveForward(float speed_mps) {
    pid_FL.Target = speed_mps;
    pid_FR.Target = speed_mps;
    pid_BL.Target = speed_mps;
    pid_BR.Target = speed_mps;
}

void Chassis_MoveBackward(float speed_mps) {
    pid_FL.Target = -speed_mps;
    pid_FR.Target = -speed_mps;
    pid_BL.Target = -speed_mps;
    pid_BR.Target = -speed_mps;
}

void Chassis_Stop(void) {
    pid_FL.Target = 0.0f;
    pid_FR.Target = 0.0f;
    pid_BL.Target = 0.0f;
    pid_BR.Target = 0.0f;
}

void Chassis_DiffTurn(float base_speed, float turn_diff) {
    pid_FL.Target = base_speed + turn_diff;
    pid_BL.Target = base_speed + turn_diff;
    pid_FR.Target = base_speed - turn_diff;
    pid_BR.Target = base_speed - turn_diff;
}

void Chassis_TurnInPlace(float speed_mps, int8_t dir) {
    pid_FL.Target =  speed_mps * dir;
    pid_BL.Target =  speed_mps * dir;
    pid_FR.Target = -speed_mps * dir;
    pid_BR.Target = -speed_mps * dir;
}

void Chassis_SetTargets(float fl, float fr, float bl, float br) {
    pid_FL.Target = fl;
    pid_FR.Target = fr;
    pid_BL.Target = bl;
    pid_BR.Target = br;
}

/* ══════════════════════════════════════════════════════════════════════
   Helper: wheel pointer lookup
   ══════════════════════════════════════════════════════════════════════ */

static PID_TypeDef *get_wheel_pid(uint8_t idx) {
    switch (idx) {
        case 0: return &pid_FL;
        case 1: return &pid_FR;
        case 2: return &pid_BL;
        case 3: return &pid_BR;
        default: return &pid_FL;
    }
}

/* ══════════════════════════════════════════════════════════════════════
   Servo Y-axis servo (TIM9 CH1, PA2, 50Hz)
   deviation > 0 = target below center → tilt down
   deviation < 0 = target above center → tilt up
   ══════════════════════════════════════════════════════════════════════ */

void Servo_Y_Servo(float y_deviation_px)
{
    float pulse_us = 1500.0f + y_deviation_px * 4.0f;
    if (pulse_us > 2100.0f) pulse_us = 2100.0f;
    if (pulse_us <  900.0f) pulse_us =  900.0f;
    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, (uint16_t)pulse_us);
}

/* ══════════════════════════════════════════════════════════════════════
   VOFA+ 命令解析
   MD→DEBUG  M0→IDLE  M1→LINE_FOLLOW ... M9→FINISH
   ══════════════════════════════════════════════════════════════════════ */

static uint8_t cur_wheel = 4; // 0=FL,1=FR,2=BL,3=BR,4=ALL

void Chassis_VOFA_Parse(const uint8_t *cmd_buf) {
    if (cmd_buf[0] == '\0') return;

    if (cmd_buf[0] == 'M') {
        if (cmd_buf[1] == 'D') {
            g_chassis_state = STATE_DEBUG;
        } else if (cmd_buf[1] >= '0' && cmd_buf[1] <= '9') {
            // M0=IDLE(1), M1=LINE_FOLLOW(2), ..., M9=FINISH(10)
            g_chassis_state = (ChassisState_t)(cmd_buf[1] - '0' + 1);
        }
        return;
    }

    if (cmd_buf[0] == 'W') {
        cur_wheel = (cmd_buf[1] == 'A') ? 4 : (cmd_buf[1] - '0');
        if (g_chassis_state == STATE_DEBUG) {
            for (int i = 0; i < 4; i++) {
                if (cur_wheel < 4 && i != (int)cur_wheel) {
                    get_wheel_pid(i)->Target = 0.0f;
                }
            }
        }
        return;
    }

    if (cmd_buf[0] == 'T' && g_chassis_state != STATE_DEBUG) return;

    float val = atof((const char *)&cmd_buf[1]);

    if (cur_wheel < 4) {
        PID_TypeDef *pid = get_wheel_pid(cur_wheel);
        switch (cmd_buf[0]) {
            case 'P': pid->Kp = val; break;
            case 'I': pid->Ki = val; break;
            case 'D': pid->Kd = val; break;
            case 'T': pid->Target = val; break;
        }
    } else {
        switch (cmd_buf[0]) {
            case 'P': pid_FL.Kp = pid_FR.Kp = pid_BL.Kp = pid_BR.Kp = val; break;
            case 'I': pid_FL.Ki = pid_FR.Ki = pid_BL.Ki = pid_BR.Ki = val; break;
            case 'D': pid_FL.Kd = pid_FR.Kd = pid_BL.Kd = pid_BR.Kd = val; break;
            case 'T': pid_FL.Target = pid_FR.Target = pid_BL.Target = pid_BR.Target = val; break;
        }
    }
}

/* ══════════════════════════════════════════════════════════════════════
   Entry 函数 — 进入状态时调用一次
   ══════════════════════════════════════════════════════════════════════ */

static void entry_debug(void)           { /* 保持 PID 状态，不操作 */ }

static void entry_idle(void)            { Chassis_Stop(); }

static void entry_line_follow(void) {
    Chassis_MoveForward(LINE_BASE_SPEED);
    // 排空 K210 队列（避障/倒车期间囤的旧帧）
    extern QueueHandle_t xQueue_VisionData;
    uint8_t dummy[24];
    while (xQueueReceive(xQueue_VisionData, dummy, 0) == pdTRUE) {}
    last_line_err = 0.0f;
    lf_trig_cnt   = 0;       // 重置触发计数器
    lf_trig_side  = 0;
}

static void entry_obstacle(void) {
    Chassis_Stop();
    ob_stage      = OB_STOP;
    ob_start_enc  = 0;
    ob_ticks      = 0;
    ob_mem_dist   = 0;
    ob_turn_start = 0;
}

static void entry_green_filter(void)    { Chassis_Stop(); }

/* ── BREAKAWAY: 刹车 + 准备转向 ── */
static void entry_breakaway(void) {
    Chassis_Stop();
    ba_no_target_ms = 0;
}

/* ── VISUAL_APPROACH: 初始化双轴伺服 ── */
static void entry_visual_approach(void) {
    va_y_filt        = 120.0f;
    va_last_x_center = 160.0f;
    va_center_cnt    = 0;
    va_no_target_ms  = 0;
}

/* ── SCANNING: 初始化扫描窗口 ── */
static void entry_scanning(void) {
    scan_tick      = 0;
    scan_max_red   = 0;
    scan_max_green = 0;
}

/* ── DOCKING: 汇总上报 ── */
static void entry_docking(void) {
    g_red_count   += scan_max_red;
    g_green_count += scan_max_green;
    // 设防重触发窗口
    g_suppress_until_tick = g_sys_tick + POST_DOCK_SUPPRESS_MS / 10;
}

static void entry_blind_return(void)    { Chassis_MoveBackward(RETURN_SPEED); }
static void entry_finish(void)          { Chassis_Stop(); }

/* ══════════════════════════════════════════════════════════════════════
   Run 函数 — 每周期调用，返回下一状态（或同状态继续）
   ══════════════════════════════════════════════════════════════════════ */

/* ── 0. DEBUG ── */
static ChassisState_t run_debug(const SensorData_t *s,
                                uint8_t hv, const VisionFrame_t *v) {
    (void)s; (void)hv; (void)v;
    return STATE_DEBUG;
}

/* ── 1. IDLE ── */
static ChassisState_t run_idle(const SensorData_t *s,
                                uint8_t hv, const VisionFrame_t *v) {
    (void)s; (void)hv; (void)v;
    return STATE_IDLE;
}

/* ── 2. LINE_FOLLOW — 红外巡线 + v5 侧边触发 ── */
static ChassisState_t run_line_follow(const SensorData_t *s,
                                       uint8_t hv, const VisionFrame_t *v) {
    /* ── 1. 红外位置计算（加权平均） ── */
    float ir_sum   = 0.0f;
    int   black_cnt = 0;
    for (int i = 0; i < 8; i++) {
        if (s->ir[i] == 0) { ir_sum += (float)i; black_cnt++; }
    }

    float line_err;
    if (black_cnt > 0) {
        line_err = ir_sum / (float)black_cnt - 3.5f;
        last_line_err = line_err;
    } else {
        line_err = last_line_err;
    }

    /* ── 2. P 控制器 → 差速转向 ── */
#if FORCE_STRAIGHT
    Chassis_MoveForward(LINE_BASE_SPEED);
#else
    float turn_corr = line_err * LINE_KP;
    float speed = (black_cnt == 0) ? 0.15f : LINE_BASE_SPEED;
    Chassis_DiffTurn(speed, turn_corr);
#endif

    /* ── 3. v5 侧边触发检测 ── */
    // 防重触发检查
    if (g_sys_tick < g_suppress_until_tick) {
        return STATE_LINE_FOLLOW;
    }

    if (hv && v != NULL && v->count > 0) {
        uint8_t cur_side = 0;
        for (int i = 0; i < v->count; i++) {
            if (v->t[i].cls == 0 || v->t[i].cls == 1) {  // 任何草莓
                if (v->t[i].x < PLANT_X_LEFT)  cur_side = 1;  // 左
                if (v->t[i].x > PLANT_X_RIGHT) cur_side = 2;  // 右
            }
        }

        if (cur_side != 0 && cur_side == lf_trig_side) {
            lf_trig_cnt++;
            if (lf_trig_cnt >= TRIGGER_CONFIRM) {
                lf_trig_cnt  = 0;
                lf_trig_side = 0;
                return STATE_BREAKAWAY;
            }
        } else {
            lf_trig_cnt  = (cur_side != 0) ? 1 : 0;
            lf_trig_side = cur_side;
        }
    }

    return STATE_LINE_FOLLOW;
}

/* ── 3. OBSTACLE — 10 阶段避障绕行（保持不变） ── */
static ChassisState_t run_obstacle(const SensorData_t *s,
                                    uint8_t hv, const VisionFrame_t *v) {
    (void)hv; (void)v;

    int32_t dist_traveled;
    int     black_cnt;

    static ObsSubStage_t prev_ob_stage = OB_DONE;
    static uint8_t       brake_ticks   = 0;

    if (ob_stage != prev_ob_stage) {
        brake_ticks   = 0;
        prev_ob_stage = ob_stage;
    }

    if (brake_ticks < OB_BRAKE_TIMEOUT) {
        Chassis_Stop();
        brake_ticks++;
        float spd = pid_FL.Actual; if (spd < 0) spd = -spd;
        if (spd < 0.03f) {
            spd = pid_FR.Actual; if (spd < 0) spd = -spd;
            if (spd < 0.03f) {
                spd = pid_BL.Actual; if (spd < 0) spd = -spd;
                if (spd < 0.03f) {
                    spd = pid_BR.Actual; if (spd < 0) spd = -spd;
                    if (spd < 0.03f) brake_ticks = OB_BRAKE_TIMEOUT;
                }
            }
        }
        return STATE_OBSTACLE;
    }

    switch (ob_stage) {

    case OB_STOP:
        ob_ticks++;
        if (ob_ticks >= 30) {
            ob_stage = OB_BACK;
            ob_start_enc = g_enc_total;
            ob_ticks = 0;
        }
        break;

    case OB_BACK:
        Chassis_MoveBackward(OB_BACK_SPEED);
        dist_traveled = ob_start_enc - g_enc_total;
        if (dist_traveled >= (int32_t)(OB_BACK_CM * ENC_PER_CM)) {
            ob_stage = OB_TURN_RIGHT; ob_ticks = 0;
        }
        break;

    case OB_TURN_RIGHT:
        if (ob_ticks == 0) ob_turn_start = g_turn_enc;
        Chassis_TurnInPlace(OB_TURN_SPEED, 1);
        ob_ticks++;
        if (g_turn_enc - ob_turn_start >= TURN90_TOTAL_ENC || ob_ticks >= OB_TURN_MS / 10) {
            ob_stage = OB_CROSS; ob_start_enc = g_enc_total; ob_ticks = 0;
        }
        break;

    case OB_CROSS:
        Chassis_MoveForward(OB_FWD_SPEED);
        dist_traveled = g_enc_total - ob_start_enc;
        ob_mem_dist = dist_traveled;
        if (dist_traveled >= (int32_t)(OB_CROSS_CM * ENC_PER_CM)) {
            ob_stage = OB_TURN_LEFT1; ob_ticks = 0;
        }
        break;

    case OB_TURN_LEFT1:
        if (ob_ticks == 0) ob_turn_start = g_turn_enc;
        Chassis_TurnInPlace(OB_TURN_SPEED, -1);
        ob_ticks++;
        if (g_turn_enc - ob_turn_start >= TURN90_TOTAL_ENC || ob_ticks >= OB_TURN_MS / 10) {
            ob_stage = OB_FORWARD; ob_start_enc = g_enc_total; ob_ticks = 0;
        }
        break;

    case OB_FORWARD:
        Chassis_MoveForward(OB_FWD_SPEED);
        dist_traveled = g_enc_total - ob_start_enc;
        if (dist_traveled >= (int32_t)(OB_FORWARD_CM * ENC_PER_CM)) {
            ob_stage = OB_TURN_LEFT2; ob_ticks = 0;
        }
        break;

    case OB_TURN_LEFT2:
        if (ob_ticks == 0) ob_turn_start = g_turn_enc;
        Chassis_TurnInPlace(OB_TURN_SPEED, -1);
        ob_ticks++;
        if (g_turn_enc - ob_turn_start >= TURN90_TOTAL_ENC || ob_ticks >= OB_TURN_MS / 10) {
            ob_stage = OB_RETURN; ob_start_enc = g_enc_total; ob_ticks = 0;
        }
        break;

    case OB_RETURN:
        Chassis_MoveForward(OB_FWD_SPEED);
        dist_traveled = g_enc_total - ob_start_enc;
        if (dist_traveled >= ob_mem_dist) {
            ob_stage = OB_TURN_45; ob_ticks = 0;
        }
        break;

    case OB_TURN_45:
        if (ob_ticks == 0) ob_turn_start = g_turn_enc;
        Chassis_TurnInPlace(OB_TURN_SPEED, 1);
        ob_ticks++;
        if (g_turn_enc - ob_turn_start >= TURN45_TOTAL_ENC || ob_ticks >= OB_TURN_MS / 10) {
            ob_stage = OB_FORWARD_TO_LINE; ob_start_enc = g_enc_total; ob_ticks = 0;
        }
        break;

    case OB_FORWARD_TO_LINE:
        Chassis_MoveForward(OB_FWD_SPEED);
        ob_ticks++;
        black_cnt = 0;
        for (int i = 0; i < 8; i++) if (s->ir[i] == 0) black_cnt++;
        dist_traveled = g_enc_total - ob_start_enc;
        if (black_cnt >= 2 || dist_traveled >= (int32_t)(OB_FWD_TO_LINE_CM * ENC_PER_CM)) {
            ob_stage = OB_DONE;
        }
        break;

    case OB_DONE:
    default:
        ob_stage = OB_STOP;
        return STATE_LINE_FOLLOW;
    }

    return STATE_OBSTACLE;
}

/* ── 4. GREEN_FILTER [STUB] ── */
static ChassisState_t run_green_filter(const SensorData_t *s,
                                        uint8_t hv, const VisionFrame_t *v) {
    (void)s; (void)hv; (void)v;
    return STATE_GREEN_FILTER;
}

/* ── 5. BREAKAWAY — X 轴视觉伺服原地转向 ── */
static ChassisState_t run_breakaway(const SensorData_t *s,
                                     uint8_t hv, const VisionFrame_t *v) {
    (void)s;

    if (hv && v != NULL && v->count > 0) {
        ba_no_target_ms = 0;

        // 计算植株水平中心
        uint8_t min_x = 255, max_x = 0;
        for (int i = 0; i < v->count; i++) {
            if (v->t[i].cls <= 1) {  // 有效目标
                if (v->t[i].x < min_x) min_x = v->t[i].x;
                if (v->t[i].x > max_x) max_x = v->t[i].x;
            }
        }
        float x_center = (min_x + max_x) / 2.0f;
        float dev = x_center - X_CENTER_PX;

        if (dev < -X_DEADBAND_PX) {
            Chassis_TurnInPlace(TURN_SERVO_SPEED, -1);  // CCW (植株偏左)
        } else if (dev > X_DEADBAND_PX) {
            Chassis_TurnInPlace(TURN_SERVO_SPEED, 1);   // CW  (植株偏右)
        } else {
            Chassis_Stop();
            return STATE_VISUAL_APPROACH;  // 已对中 → 前进
        }
    } else {
        // 丢目标：累计超时
        ba_no_target_ms += 10;
        if (ba_no_target_ms >= 1000) {
            Chassis_Stop();
            return STATE_BLIND_RETURN;  // 1s 无目标 → 退回
        }
        // 保持慢转，期望重新捕获
        Chassis_TurnInPlace(TURN_SERVO_SPEED, 1);
    }

    return STATE_BREAKAWAY;
}

/* ── 6. VISUAL_APPROACH — 双轴伺服前进 + 全景停车 ── */
static ChassisState_t run_visual_approach(const SensorData_t *s,
                                           uint8_t hv, const VisionFrame_t *v) {
    (void)s;

    if (hv && v != NULL && v->count > 0) {
        va_no_target_ms = 0;

        // 计算植株边界
        uint8_t min_x = 255, max_x = 0;
        uint8_t min_y = 240, max_y = 0;
        uint8_t max_area = 0;
        for (int i = 0; i < v->count; i++) {
            if (v->t[i].cls <= 1) {
                if (v->t[i].x < min_x) min_x = v->t[i].x;
                if (v->t[i].x > max_x) max_x = v->t[i].x;
                if (v->t[i].y < min_y) min_y = v->t[i].y;
                if (v->t[i].y > max_y) max_y = v->t[i].y;
                if (v->t[i].area > max_area) max_area = v->t[i].area;
            }
        }

        /* ── X 轴伺服（差速转向） ── */
        float x_center = (min_x + max_x) / 2.0f;
        va_last_x_center = x_center;
        float x_dev = x_center - X_CENTER_PX;

        /* ── Y 轴伺服（低通滤波 + 死区） ── */
        float y_center_raw = (min_y + max_y) / 2.0f;
        va_y_filt = va_y_filt * (1.0f - Y_FILT_NEW) + y_center_raw * Y_FILT_NEW;
        float y_dev = va_y_filt - Y_CENTER_PX;

        if (y_dev < -Y_DEADBAND_PX || y_dev > Y_DEADBAND_PX) {
            Servo_Y_Servo(y_dev);  // Outside deadband → adjust
        }
        // Inside deadband → servo stays

        /* ── 速度递减 ── */
        float range = (float)(APPROACH_AREA_STOP);
        float frac  = (float)max_area / range;
        if (frac > 1.0f) frac = 1.0f;
        float speed = APPROACH_BASE_SPEED - (APPROACH_BASE_SPEED - APPROACH_MIN_SPEED) * frac;

        /* ── X 轴 P 控制器 → DiffTurn ── */
        float turn_kp = 0.02f;
        Chassis_DiffTurn(speed, x_dev * turn_kp);

        /* ── 全景停车判定 ── */
        if (max_area >= APPROACH_AREA_STOP) {
            float abs_dev = (x_dev > 0) ? x_dev : -x_dev;
            if (abs_dev < X_DEADBAND_PX) {
                va_center_cnt++;
            } else {
                va_center_cnt = 0;
            }
            if (va_center_cnt >= APPROACH_CENTER_OK) {
                Chassis_Stop();
                return STATE_SCANNING;
            }
        } else {
            va_center_cnt = 0;
        }

    } else {
        // 丢目标
        va_no_target_ms += 10;
        if (va_no_target_ms > 200) {
            Chassis_Stop();
            return STATE_BLIND_RETURN;  // >200ms → 退回
        }
        // ≤200ms: 保持上帧方向继续
        float turn_kp = 0.02f;
        Chassis_DiffTurn(APPROACH_MIN_SPEED, (va_last_x_center - X_CENTER_PX) * turn_kp);
    }

    return STATE_VISUAL_APPROACH;
}

/* ── 7. SCANNING — 固定窗口取单帧最高纪录 ── */
static ChassisState_t run_scanning(const SensorData_t *s,
                                    uint8_t hv, const VisionFrame_t *v) {
    (void)s;

    if (hv && v != NULL && v->count > 0) {
        int cur_red = 0, cur_green = 0;
        for (int i = 0; i < v->count; i++) {
            if (v->t[i].cls == 1) cur_red++;
            if (v->t[i].cls == 0) cur_green++;
        }
        if (cur_red   > scan_max_red)   scan_max_red   = cur_red;
        if (cur_green > scan_max_green) scan_max_green = cur_green;
    }

    scan_tick++;
    if (scan_tick >= SCAN_FRAMES) {
        return STATE_DOCKING;
    }

    return STATE_SCANNING;
}

/* ── 8. DOCKING — 蜂鸣上报 ── */
static ChassisState_t run_docking(const SensorData_t *s,
                                   uint8_t hv, const VisionFrame_t *v) {
    (void)s; (void)hv; (void)v;

    // 蜂鸣：0~2000ms 长响，2000~2400ms 两短声
    uint32_t t = g_state_ticks * 10;  // ms
    if (t < 2000) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);  // 长响
    } else if (t < 2080) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET); // 第一短声间隔
    } else if (t < 2160) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);   // 第一短声
    } else if (t < 2240) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET); // 间隔
    } else if (t < 2320) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);   // 第二短声
    } else {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
    }

    if (g_state_ticks >= 250) {  // 2.5s @ 100Hz
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
        return STATE_BLIND_RETURN;
    }
    return STATE_DOCKING;
}

/* ── 9. BLIND_RETURN — 倒车回线 ── */
static ChassisState_t run_blind_return(const SensorData_t *s,
                                        uint8_t hv, const VisionFrame_t *v) {
    (void)hv; (void)v;

    /* ── 阶段 0：倒车 ── */
    static uint8_t br_phase = 0;  // 0=倒车, 1=自旋找线
    static int32_t  br_start_enc = 0;
    static uint32_t br_spin_ticks = 0;

    if (g_state_ticks == 0) {
        br_phase = 0;
        br_start_enc = 0;
        br_spin_ticks = 0;
    }

    if (br_phase == 0) {
        if (br_start_enc == 0) br_start_enc = g_enc_total;
        Chassis_MoveBackward(RETURN_SPEED);

        // 倒车过程中也检查红外：如果退到线上了直接开始找线
        int bc = 0;
        for (int i = 0; i < 8; i++) if (s->ir[i] == 0) bc++;

        // 退了 40cm 或发现黑线 → 进入自旋找线阶段
        int32_t backed = br_start_enc - g_enc_total;
        if (backed >= (int32_t)(40.0f * ENC_PER_CM) || bc >= 2) {
            Chassis_Stop();
            br_phase = 1;
            br_spin_ticks = 0;
        }
    } else {
        // 阶段 1：自旋找线
        Chassis_TurnInPlace(RETURN_SPIN_SPEED, 1);  // CW 自旋
        br_spin_ticks++;

        int bc = 0;
        for (int i = 0; i < 8; i++) if (s->ir[i] == 0) bc++;

        if (bc >= 2) {
            Chassis_Stop();
            br_phase = 0;
            return STATE_LINE_FOLLOW;
        }

        // 兜底：自旋超过 3s 还没找到线 → 放弃，直接切 LINE_FOLLOW
        if (br_spin_ticks >= 300) {
            Chassis_Stop();
            br_phase = 0;
            return STATE_LINE_FOLLOW;
        }
    }

    return STATE_BLIND_RETURN;
}

/* ── 10. FINISH [STUB] ── */
static ChassisState_t run_finish(const SensorData_t *s,
                                  uint8_t hv, const VisionFrame_t *v) {
    (void)s; (void)hv; (void)v;
    return STATE_FINISH;
}

/* ══════════════════════════════════════════════════════════════════════
   状态描述符表（STATE_SCANNING 已插入）
   ══════════════════════════════════════════════════════════════════════ */

static const StateDescriptor_t state_table[STATE_COUNT] = {
    [STATE_DEBUG]           = { "DEBUG",           entry_debug,           run_debug,           1 },
    [STATE_IDLE]            = { "IDLE",            entry_idle,            run_idle,            0 },
    [STATE_LINE_FOLLOW]     = { "LINE_FOLLOW",     entry_line_follow,     run_line_follow,     0 },
    [STATE_OBSTACLE]        = { "OBSTACLE",        entry_obstacle,        run_obstacle,        0 },
    [STATE_GREEN_FILTER]    = { "GREEN_FILTER",    entry_green_filter,    run_green_filter,    0 },
    [STATE_BREAKAWAY]       = { "BREAKAWAY",       entry_breakaway,       run_breakaway,       0 },
    [STATE_VISUAL_APPROACH] = { "VISUAL_APPROACH", entry_visual_approach, run_visual_approach, 0 },
    [STATE_SCANNING]        = { "SCANNING",        entry_scanning,        run_scanning,        0 },
    [STATE_DOCKING]         = { "DOCKING",         entry_docking,         run_docking,         0 },
    [STATE_BLIND_RETURN]    = { "BLIND_RETURN",    entry_blind_return,    run_blind_return,    0 },
    [STATE_FINISH]          = { "FINISH",          entry_finish,          run_finish,          0 },
};

/* ══════════════════════════════════════════════════════════════════════
   Public helpers
   ══════════════════════════════════════════════════════════════════════ */

const char *Chassis_StateName(ChassisState_t s) {
    if (s >= STATE_COUNT) return "UNKNOWN";
    return state_table[s].name;
}

/* ══════════════════════════════════════════════════════════════════════
   Chassis_Init — 上电时调用一次
   ══════════════════════════════════════════════════════════════════════ */

void Chassis_Init(void) {
    g_chassis_state       = STATE_DEBUG;
    g_state_ticks         = 0;
    g_emergency_stop      = 0;
    g_red_count           = 0;
    g_green_count         = 0;
    g_enc_total           = 0;
    g_suppress_until_tick = 0;
}

/* ══════════════════════════════════════════════════════════════════════
   Chassis_StateMachine_Run — 每 10ms 主循环调用一次
   ══════════════════════════════════════════════════════════════════════ */

void Chassis_StateMachine_Run(const SensorData_t *sensor,
                              uint8_t has_vision, const VisionFrame_t *vis,
                              int32_t enc_fl, int32_t enc_fr,
                              int32_t enc_bl, int32_t enc_br)
{
    (void)enc_fl; (void)enc_fr; (void)enc_bl; (void)enc_br;

    static ChassisState_t prev_state = STATE_COUNT;

    if (g_chassis_state != prev_state) {
        if (g_chassis_state < STATE_COUNT) {
            state_table[g_chassis_state].entry();
        }
        g_state_ticks = 0;
        prev_state    = g_chassis_state;
    }

    const StateDescriptor_t *desc = &state_table[g_chassis_state];
    ChassisState_t next = desc->run(sensor, has_vision, vis);
    g_state_ticks++;

    if (next != g_chassis_state && next < STATE_COUNT) {
        g_chassis_state = next;
    }
}
