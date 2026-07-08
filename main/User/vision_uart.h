/**
 * @file  vision_uart.h
 * @brief 视觉上位机通信层 — UART DMA 接收 + 9字节协议解码
 *
 * 硬件:
 *   UART1  RX=IO21  TX=IO20  115200 8N1  DMA
 *
 * 协议 (9 字节/帧):
 *   [0xAA] [0xBB] [X_LE] [X_BE] [Y_LE] [Y_BE] [Z_LE] [Z_BE] [CS]
 *   X/Y/Z = int16 LE (mm)
 *   CS = XOR(byte0..7)
 *   (-1, -1, -1) = 无目标
 */

#ifndef VISION_UART_H
#define VISION_UART_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*------ 解析后的目标坐标 ------*/
typedef struct {
    int16_t x_mm;    /* X: 左右 (正=右) */
    int16_t y_mm;    /* Y: 上下 (正=上) */
    int16_t z_mm;    /* Z: 深度 (正=前方) */
    bool    valid;   /* true=有效目标, false=无目标/未收到/超时 */
} vision_target_t;

/**
 * @brief 初始化 UART1 DMA + 内部状态
 * @return ESP_OK 成功
 */
esp_err_t vision_uart_init(void);

/**
 * @brief 轮询处理 — 每帧调用一次
 *
 *        从 DMA RX 缓冲中读出所有待处理字节, 经状态机解码,
 *        校验通过后更新内部目标坐标。
 *        无阻塞, 无额外任务栈, 适合在主循环或定时器中调用。
 *
 * @return 本轮成功解码的帧数 (0 = 无完整帧或全被校验丢弃)
 */
int vision_uart_poll(void);

/**
 * @brief 获取最新解析目标 (非阻塞, 线程安全)
 * @param target [out] 拷贝最新目标坐标
 * @return true 若上次 poll 以来有新帧更新
 */
bool vision_uart_get_target(vision_target_t *target);

/**
 * @brief 获取累计统计 (调试用)
 * @param total_bytes  [out] 累计接收字节数
 * @param good_frames  [out] 校验通过的帧数
 * @param bad_frames   [out] 校验失败的帧数
 */
void vision_uart_stats(uint32_t *total_bytes, uint32_t *good_frames, uint32_t *bad_frames);

/**
 * @brief 设置相机安装位姿 (位置 + 俯仰角)
 *
 *        相机固定在机器人上, 相对底座中心有固定偏移和俯仰。
 *        调用后, get_target() 自动将相机系坐标旋转+平移为机器人底座系。
 *
 *        几何约定:
 *          相机 X 轴 → 机器人 Y 轴 (左右)
 *          相机 Y 轴 → 指向地面 (正=下)
 *          相机 Z 轴 → 视线方向
 *          俯仰角 tilt_deg: 视线与水平面夹角 (下俯为负, 如 -31°)
 *
 * @param pos_x_mm  相机光心在底座系的 X 坐标 (前方)
 * @param pos_y_mm  相机光心在底座系的 Y 坐标 (右方)
 * @param pos_z_mm  相机光心在底座系的 Z 坐标 (高度)
 * @param tilt_deg  相机俯仰角 (度), 下俯为负 (典型 -31°)
 */
void vision_uart_set_camera_pose(int16_t pos_x_mm, int16_t pos_y_mm, int16_t pos_z_mm,
                                  float tilt_deg);

/**
 * @brief 相机坐标系→机器人底座坐标系 (纯函数, 可用于外部验证)
 *
 * @param cam_x, cam_y, cam_z  相机系目标坐标 (mm, PC 发来的原始值)
 * @param rob_x, rob_y, rob_z  [out] 底座系坐标 (mm)
 * @param pos_x, pos_y, pos_z  相机在底座系的位置 (mm)
 * @param tilt_deg             相机俯仰角 (度)
 */
void vision_camera_to_robot(int16_t cam_x, int16_t cam_y, int16_t cam_z,
                            float *rob_x, float *rob_y, float *rob_z,
                            int16_t pos_x, int16_t pos_y, int16_t pos_z,
                            float tilt_deg);

/**
 * @brief 重置内部解码器状态 + 清空 DMA 缓冲
 */
void vision_uart_reset(void);

/**
 * @brief 发送原始字节
 */
int vision_uart_send(const uint8_t *data, int len);

#ifdef __cplusplus
}
#endif

#endif /* VISION_UART_H */
