/**
 * @file  vision_uart.c
 * @brief 视觉通信 UART DMA 驱动 + 9字节协议解码器
 *
 * 帧格式 (9 字节):
 *   [0]    0xAA        帧头1
 *   [1]    0xBB        帧头2
 *   [2]    X_LE        X 坐标低字节 (int16 LE, mm)
 *   [3]    X_BE        X 坐标高字节
 *   [4]    Y_LE
 *   [5]    Y_BE
 *   [6]    Z_LE
 *   [7]    Z_BE
 *   [8]    CS          XOR(byte[0]..byte[7])
 *
 * 实现:
 *   - UART1 DMA 自动收 → 内部 2048 字节环形缓冲
 *   - vision_uart_poll() 批量读出, 逐字节过状态机 (零阻塞, 无额外任务)
 *   - 校验通过后原子更新 vision_target_t
 *   - (-1, -1, -1) 标记 valid=false
 */

#include "vision_uart.h"
#include "driver/uart.h"
#include "esp_log.h"
#include <string.h>
#include <math.h>

static const char *TAG = "vis_uart";

/*------ 硬件常量 ------*/
#define VIS_UART_PORT       UART_NUM_1
#define VIS_UART_RX_PIN     21
#define VIS_UART_TX_PIN     20
#define VIS_UART_RX_BUF     2048
#define VIS_UART_TX_BUF     512

/*------ 协议常量 ------*/
#define FRAME_LEN           9
#define FRAME_SYNC1         0xAA
#define FRAME_SYNC2         0xBB
#define NO_TARGET_MARKER    ((int16_t)(-1))

/*------ 解码状态机 ------*/
typedef enum {
    STATE_WAIT_AA = 0,
    STATE_WAIT_BB,
    STATE_RECV_DATA,
} dec_state_t;

/*------ 模块内部状态 ------*/
static dec_state_t     s_state = STATE_WAIT_AA;
static uint8_t         s_buf[FRAME_LEN];  /* 当前帧缓冲 */
static uint8_t         s_idx = 0;

/* 最新解析结果 (原子读写 — 32 位对齐, RISC-V 无需锁) */
static vision_target_t s_latest;
static volatile bool   s_updated = false;

/* 相机安装位姿 — 位置 (mm) + 预计算旋转 sin/cos */
static int16_t  s_cam_pos_x = 0;
static int16_t  s_cam_pos_y = 0;
static int16_t  s_cam_pos_z = 0;
static float    s_cam_sin   = 0.0f;   /* sin(tilt) */
static float    s_cam_cos   = 1.0f;   /* cos(tilt), 默认 0° */

/* 统计 */
static uint32_t        s_total_bytes = 0;
static uint32_t        s_good_frames = 0;
static uint32_t        s_bad_frames  = 0;

/*===============================================================
 *  vision_uart_init
 *===============================================================*/
esp_err_t vision_uart_init(void)
{
    esp_err_t ret;

    /* 1. 安装 UART 驱动 (内部 DMA) */
    ret = uart_driver_install(VIS_UART_PORT, VIS_UART_RX_BUF,
                              VIS_UART_TX_BUF, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "driver_install err: %s", esp_err_to_name(ret));
        return ret;
    }

    /* 2. 参数: 115200 8N1 */
    uart_config_t cfg = {
        .baud_rate  = 115200,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ret = uart_param_config(VIS_UART_PORT, &cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "param_config err: %s", esp_err_to_name(ret));
        uart_driver_delete(VIS_UART_PORT);
        return ret;
    }

    /* 3. 绑定引脚 */
    ret = uart_set_pin(VIS_UART_PORT,
                       VIS_UART_TX_PIN,
                       VIS_UART_RX_PIN,
                       UART_PIN_NO_CHANGE,
                       UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "set_pin err: %s", esp_err_to_name(ret));
        uart_driver_delete(VIS_UART_PORT);
        return ret;
    }

    /* 4. 清空 RX FIFO + 初始化状态 */
    uint8_t dummy[64];
    uart_read_bytes(VIS_UART_PORT, dummy, sizeof(dummy), 0);
    s_state = STATE_WAIT_AA;
    s_idx   = 0;
    memset(&s_latest, 0, sizeof(s_latest));
    s_updated = false;

    ESP_LOGI(TAG, "Init OK  UART%d  RX=IO%d  TX=IO%d  115200 8N1  DMA  proto=9B",
             VIS_UART_PORT, VIS_UART_RX_PIN, VIS_UART_TX_PIN);
    return ESP_OK;
}

/*===============================================================
 *  process_byte — 状态机核心: 每字节调用一次
 *===============================================================*/
static inline void process_byte(uint8_t byte)
{
    switch (s_state) {

    case STATE_WAIT_AA:
        if (byte == FRAME_SYNC1) {
            s_state = STATE_WAIT_BB;
        }
        /* else: 丢弃, 继续等 */
        break;

    case STATE_WAIT_BB:
        if (byte == FRAME_SYNC2) {
            s_state = STATE_RECV_DATA;
            s_idx   = 0;
        } else if (byte != FRAME_SYNC1) {
            s_state = STATE_WAIT_AA;  /* 不是 BB 也不是 AA → 从头来 */
        }
        /* byte==AA 时保持 WAIT_BB (允许 AA AA BB 这种畸形帧恢复) */
        break;

    case STATE_RECV_DATA:
        s_buf[s_idx++] = byte;
        if (s_idx == FRAME_LEN) {
            /* 收满 9 字节 → 校验 */
            s_state = STATE_WAIT_AA;

            uint8_t cs = 0;
            for (int i = 0; i < FRAME_LEN - 1; i++) {
                cs ^= s_buf[i];
            }
            if (cs != s_buf[FRAME_LEN - 1]) {
                s_bad_frames++;
                return;  /* 校验失败, 丢弃 */
            }

            /* 解包 int16 LE */
            int16_t x = (int16_t)(s_buf[0] | ((uint16_t)s_buf[1] << 8));
            int16_t y = (int16_t)(s_buf[2] | ((uint16_t)s_buf[3] << 8));
            int16_t z = (int16_t)(s_buf[4] | ((uint16_t)s_buf[5] << 8));

            /* 更新全局结果 */
            s_latest.x_mm  = x;
            s_latest.y_mm  = y;
            s_latest.z_mm  = z;
            s_latest.valid = !(x == NO_TARGET_MARKER &&
                               y == NO_TARGET_MARKER &&
                               z == NO_TARGET_MARKER);
            s_updated = true;
            s_good_frames++;
        }
        break;
    }
}

/*===============================================================
 *  vision_uart_poll — 批量读出并解码
 *===============================================================*/
int vision_uart_poll(void)
{
    uint32_t prev_good = s_good_frames;

    /* 一次最多处理 256 字节, 防止单次调用过长 */
    for (int round = 0; round < 8; round++) {
        uint8_t chunk[64];
        int len = uart_read_bytes(VIS_UART_PORT, chunk, sizeof(chunk), 0);
        if (len <= 0) break;

        s_total_bytes += len;

        for (int i = 0; i < len; i++) {
            process_byte(chunk[i]);
        }
    }

    return (int)(s_good_frames - prev_good);
}

/*===============================================================
 *  vision_camera_to_robot — 纯函数: 相机系 → 底座系
 *
 *  相机安装: X_cam=机器人Y轴, Y_cam=指向地面, Z_cam=视线方向
 *  绕相机 X 轴 (=机器人 Y 轴) 倾斜。tilt_deg 负值=下俯。
 *
 *  变换公式 (以 θ = tilt_deg):
 *    X_r = pos_x + sin(θ)·cy + cos(θ)·cz
 *    Y_r = pos_y + cx
 *    Z_r = pos_z - cos(θ)·cy + sin(θ)·cz
 *
 *  验证 (tilt=-31°, sin≈-0.515, cos≈0.857, pos=(-100,0,490)):
 *    目标在视线正前方 cz>0, cy=0:
 *      X_r = -100 + 0.857·cz  → 前方 ✓
 *      Z_r =  490 - 0.515·cz  → 低于相机 ✓
 *===============================================================*/
void vision_camera_to_robot(int16_t cam_x, int16_t cam_y, int16_t cam_z,
                            float *rob_x, float *rob_y, float *rob_z,
                            int16_t pos_x, int16_t pos_y, int16_t pos_z,
                            float tilt_deg)
{
    float rad = tilt_deg * (float)M_PI / 180.0f;
    float s   = sinf(rad);
    float c   = cosf(rad);

    float cx = (float)cam_x;
    float cy = (float)cam_y;
    float cz = (float)cam_z;

    *rob_x = (float)pos_x  +  s * cy  +  c * cz;
    *rob_y = (float)pos_y  +  cx;
    *rob_z = (float)pos_z  -  c * cy  +  s * cz;
}

/*===============================================================
 *  vision_uart_set_camera_pose — 设置相机安装位姿
 *===============================================================*/
void vision_uart_set_camera_pose(int16_t pos_x_mm, int16_t pos_y_mm, int16_t pos_z_mm,
                                  float tilt_deg)
{
    s_cam_pos_x = pos_x_mm;
    s_cam_pos_y = pos_y_mm;
    s_cam_pos_z = pos_z_mm;

    float rad = tilt_deg * (float)M_PI / 180.0f;
    s_cam_sin  = sinf(rad);
    s_cam_cos  = cosf(rad);

    ESP_LOGI(TAG, "Camera pose set: pos=(%+d,%+d,%+d)mm  tilt=%.1f°  "
             "(sin=%.4f cos=%.4f)",
             pos_x_mm, pos_y_mm, pos_z_mm, (double)tilt_deg,
             (double)s_cam_sin, (double)s_cam_cos);
}

/*===============================================================
 *  vision_uart_get_target  (变换 + X取反 + 30mm 径向补偿)
 *
 *  后处理:
 *    1. X 轴取反 (修正相机坐标方向)
 *    2. 径向 +50mm 补偿 (测量点到夹取点的系统偏差)
 *       正前方:  X += 30, Y += 0
 *       右前30°: X += 30·cos30°, Y += 30·sin30°
 *===============================================================*/
bool vision_uart_get_target(vision_target_t *target)
{
    if (!target) return false;

    *target = s_latest;

    if (target->valid) {
        float cx = (float)target->x_mm;
        float cy = (float)target->y_mm;
        float cz = (float)target->z_mm;

        /* 相机 → 底座旋转变换 */
        float rx = (float)s_cam_pos_x  +  s_cam_sin * cy  +  s_cam_cos * cz;
        float ry = (float)s_cam_pos_y  +  cx;
        float rz = (float)s_cam_pos_z  -  s_cam_cos * cy  +  s_cam_sin * cz;

        /* 2. 径向 +30mm 补偿 (沿底座→目标方向, 先补偿再处理符号) */
        float r = sqrtf(rx * rx + ry * ry);
        if (r > 1.0f) {
            float scale = (r + 50.0f) / r;
            rx *= scale;
            ry *= scale;
        } else {
            rx += 50.0f;  /* 目标在原点, 默认向前加 */
        }

        /* 四舍五入回 int16 */
        target->x_mm = (int16_t)(rx + (rx >= 0 ? 0.5f : -0.5f));
        target->y_mm = (int16_t)(ry + (ry >= 0 ? 0.5f : -0.5f));
        target->z_mm = (int16_t)(rz + (rz >= 0 ? 0.5f : -0.5f));
    }

    bool was   = s_updated;
    s_updated  = false;
    return was;
}

/*===============================================================
 *  vision_uart_stats
 *===============================================================*/
void vision_uart_stats(uint32_t *total_bytes,
                       uint32_t *good_frames,
                       uint32_t *bad_frames)
{
    if (total_bytes) *total_bytes = s_total_bytes;
    if (good_frames) *good_frames = s_good_frames;
    if (bad_frames)  *bad_frames  = s_bad_frames;
}

/*===============================================================
 *  vision_uart_send
 *===============================================================*/
int vision_uart_send(const uint8_t *data, int len)
{
    if (!data || len <= 0) return 0;
    int sent = uart_write_bytes(VIS_UART_PORT, data, len);
    if (sent < 0) {
        ESP_LOGE(TAG, "send err: %s", esp_err_to_name((esp_err_t)sent));
        return -1;
    }
    return sent;
}

/*===============================================================
 *  vision_uart_reset — 清空缓冲+复位解码器
 *===============================================================*/
void vision_uart_reset(void)
{
    /* 清空 DMA RX 环形缓冲 */
    uint8_t dummy[64];
    while (uart_read_bytes(VIS_UART_PORT, dummy, sizeof(dummy), 0) > 0) {}

    /* 复位解码状态机 */
    s_state = STATE_WAIT_AA;
    s_idx   = 0;
    memset(&s_latest, 0, sizeof(s_latest));
    s_updated = false;

    ESP_LOGI(TAG, "UART RX buffer + decoder reset");
}
