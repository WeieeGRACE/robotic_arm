/**
 * @file  cartesian.h
 * @brief 笛卡尔空间直线/圆弧插补 — IK采样 + 轨迹执行
 */

#ifndef CARTESIAN_H
#define CARTESIAN_H

#include "trajectory.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 笛卡尔直线运动 (阻塞)
 *
 *        在起点→终点之间等距采样, IK求解各点关节角,
 *        通过轨迹引擎执行多段平滑运动。
 *
 * @param x1,y1,z1  起点世界坐标 (mm)
 * @param x2,y2,z2  终点世界坐标 (mm)
 * @param speed_mms 末端移动速度 (mm/s), 推荐 50~150
 * @return true 成功, false 路径上有不可达点
 */
bool cartesian_move_line(float x1, float y1, float z1,
                         float x2, float y2, float z2,
                         float speed_mms);

/**
 * @brief 笛卡尔直线运动 (从当前末端位置出发)
 * @param x,y,z  目标世界坐标
 * @param speed_mms  末端速度
 */
bool cartesian_move_to(float x, float y, float z, float speed_mms);

#ifdef __cplusplus
}
#endif

#endif /* CARTESIAN_H */
