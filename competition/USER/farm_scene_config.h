#ifndef FARM_SCENE_CONFIG_H
#define FARM_SCENE_CONFIG_H

/*
 * 农场采摘任务的机械臂与传感器几何参数。
 * TOF200F 数据帧中的 distance 单位为毫米。
 * 测距和颜色传感器位于车体右侧，与机械臂底座处在同一前后（X）平面。
 */
/* 车体坐标：+X 前方，+Y 右侧，+Z 向上。 */
#define FARM_SENSOR_RIGHT_OFFSET_M   0.04795f /* 测距传感器相对底座的右向偏移：47.95 mm */
#define FARM_SENSOR_FORWARD_OFFSET_M 0.00000f
#define FARM_SENSOR_HEIGHT_OFFSET_M  0.00525f /* 测距点相对底座中心的高度：5.25 mm */
#define FARM_SURFACE_TO_GRIP_M       0.00050f /* 测距命中面到夹口目标的偏移：0.5 mm */
#define FARM_GRIP_ABOVE_SENSOR_M    0.00000f /* 不额外抬高夹取目标，保持测距射线高度 */
#define FARM_BIN_DROP_X_M          (-0.12000)
#define FARM_BIN_DROP_Y_M            0.001083331 /* 车筐中心相对底座侧向偏移 */
#define FARM_BIN_DROP_Z_M            0.09000 /* 高于筐沿56.35mm固件尺寸 */
#define FARM_NOMINAL_PEPPER_DISTANCE_MM 77.6f /* 通信帧直接读到的毫米值 */

/* 辣椒参考高度为 8.65 mm；当前抓取目标使用上方传感器高度参数。 */
#define FARM_PEPPER_HEIGHT_M         0.00865f
#define FARM_ARM_RADIAL_MIN_M        0.100f
#define FARM_ARM_RADIAL_MAX_M        0.160f

#endif /* FARM_SCENE_CONFIG_H */
