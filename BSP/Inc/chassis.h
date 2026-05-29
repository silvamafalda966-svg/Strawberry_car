#ifndef __CHASSIS_H
#define __CHASSIS_H

#include "main.h"

/* ────────── 10 状态枚举 ────────── */
typedef enum {
    STATE_DEBUG = 0,        // VOFA+ 手动调参
    STATE_IDLE,             // 空闲等待
    STATE_LINE_FOLLOW,      // 红外巡线行驶
    STATE_OBSTACLE,         // 超声波避障绕行
    STATE_GREEN_FILTER,     // 绿草莓抗干扰
    STATE_BREAKAWAY,        // 脱离巡线，原地对准红草莓
    STATE_VISUAL_APPROACH,  // 视觉伺服接近
    STATE_DOCKING,          // 对接完成，采摘确认
    STATE_BLIND_RETURN,     // 盲倒回赛道 + 回正
    STATE_FINISH,           // 任务完成
    STATE_COUNT             // 哨兵值 = 10
} ChassisState_t;

/* ────────── 传感器数据结构体（对齐 1 字节） ────────── */
#pragma pack(1)
typedef struct {
    uint8_t ir[8];          // PE0(最左) ~ PE7(最右), 0=黑线
    float   distance_cm;    // HC-SR04 超声波测距
} SensorData_t;

typedef struct {
    uint8_t header;         // 0xAA
    uint8_t class_id;       // 0=绿草莓, 1=红草莓
    uint8_t x;              // X 坐标 (0~320)
    uint8_t y;              // Y 坐标 (0~240)
    uint8_t area;           // 面积百分比 (0~100)
    uint8_t reserved;
    uint8_t checksum;       // XOR(Class, X, Y, Area, Reserved)
    uint8_t tail;           // 0x55
} VisionData_t;
#pragma pack()

/* ────────── 状态描述符表项 ────────── */
typedef struct {
    const char    *name;
    void         (*entry)(void);
    ChassisState_t (*run)(const SensorData_t *sensor,
                          uint8_t has_vision, const VisionData_t *vis);
    uint8_t        allow_voafa_t;   // 1=DEBUG之外也允许T命令
} StateDescriptor_t;

/* ────────── 全局变量声明 ────────── */
extern volatile uint8_t   g_emergency_stop;
extern ChassisState_t     g_chassis_state;
extern uint32_t           g_state_ticks;
extern int32_t            g_last_picked_enc;
extern int32_t            g_skip_until_enc;
extern uint8_t            g_skip_strawberry;

/* ────────── Chassis_Move API（只设 Target，不碰 PWM） ────────── */
void Chassis_MoveForward(float speed_mps);
void Chassis_MoveBackward(float speed_mps);
void Chassis_Stop(void);
void Chassis_DiffTurn(float base_speed, float turn_diff);
void Chassis_TurnInPlace(float speed_mps, int8_t dir);
void Chassis_SetTargets(float fl, float fr, float bl, float br);

/* ────────── 状态机调度 ────────── */
void Chassis_Init(void);
void Chassis_StateMachine_Run(const SensorData_t *sensor,
                              uint8_t has_vision, const VisionData_t *vis,
                              int32_t enc_fl, int32_t enc_fr,
                              int32_t enc_bl, int32_t enc_br);
void Chassis_VOFA_Parse(const uint8_t *cmd_buf);

#endif /* __CHASSIS_H */
