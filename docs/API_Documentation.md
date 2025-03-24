# **简介**

本接口文档面向**软件调用方**，主要说明：

- 接口头文件（ball_tracker_interface.h）的使用方式；
- 公共头文件（ball_tracker_common.h）中定义的两个关键数据结构：
    - BallStatus（小球状态结构）
    - RobotTarget（机械臂目标结构）

---

# **接口头文件说明：ball_tracker_interface.h**

BallTrackerInterface 类提供了整个跟踪系统的核心接口，包含初始化轨道、启动和停止跟踪、小球状态获取、机械臂目标获取等功能。

---

## **构造函数**

```cpp
explicit BallTrackerInterface(const std::string& balls_config_file_path, const std::pair<double, double>& init_pos);
```

**作用**：

- 从配置文件中加载多个小球的配置（包括id、颜色、HSV信息等）。
- 同时以指定的起点坐标初始化所有小球跟踪器。

**参数**：

- balls_config_file_path：小球配置文件路径（JSON格式保存的结构化数据，记录每个小球的属性）
- init_pos：小球跟踪器统一的初始位置（通常为轨道起点位置）。

**调用示例**：

```cpp
BallTrackerInterface tracker_interface("../config/balls_config.json", std::make_pair(100.0, 200.0));
```

---

## **初始化轨道轨迹**

```cpp
int InitTrack(const std::string& out_trajectory_file_path);
```

**作用**：

- 初始化轨道轨迹，通常是用单个小球跑一圈，算法记录轨迹数据与鸟瞰图，并保存至指定文件路径。

**参数**：

- out_trajectory_file_path：轨道轨迹与鸟瞰图的输出文件路径。

**返回**：

- 返回清晰的错误码，提示初始化过程中可能出现的问题。

**调用示例**：

```cpp
auto status = tracker_interface.InitTrack("../config/trajectory");

if (status != InitTrackErrorCode::SUCCESS) {
    // 根据错误码执行相应的处理
}
```

> **错误码说明：**
>
>
>
> | **错误码枚举值** | **错误说明** |
> | --- | --- |
> | 0 (SUCCESS) | 初始化成功，无错误 |
> | 1 (CAMERA_NOT_CONNECTED) | 俯瞰位相机未连接或无法正常打开 |
> | 2 (CAMERA_CAPTURE_ERROR) | 相机图像采集异常（如图像质量不合格） |
> | 3(BALL_NOT_DETECTED_AT_START) | 初始位置未检测到小球 |
> | 4(BALL_LOST_DURING_TRACKING) | 未能全程跟踪到小球（中途丢失） |
> | 5 (TRACKING_TIMEOUT) | 跟踪过程超时 |

---

## **启动跟踪算法**

```cpp
void StartTracking();
```

**作用**：

- 启动彩球跟踪算法，内部开始采集图像，并实时跟踪小球位置。

**调用示例**：

```cpp
tracker_interface.StartTracking();
```

---

## **停止跟踪算法**

```cpp
void StopTracking();
```

**作用**：

- 停止彩球跟踪算法的运行，释放相关资源。

**调用示例**：

```cpp
tracker_interface.StopTracking();
```

---

## **获取所有小球状态**

```cpp
std::vector<BallStatus> GetBallStatus();
```

**作用**：

- 实时获取当前所有小球的状态信息，包括位置、速度、是否检测到等。

**调用示例**：

```cpp
auto ball_statuses = tracker_interface.GetBallStatus();
for (const auto& status : ball_statuses) {
    std::cout << "Ball ID: " << status.id << ", X: " << status.x << ", Y: " << status.y << std::endl;
}
```

---

## **获取机械臂目标信息**

```cpp
RobotTarget GetRobotTarget();
```

**作用**：

- 获取当前排名第一的小球对应的机械臂目标位置及速度。

**调用示例**：

```cpp
auto robot_target = tracker_interface.GetRobotTarget();
std::cout << "Robot Target: X=" << robot_target.X_arm << ", Y=" << robot_target.Y_arm << std::endl;
```

---

# **公共头文件关键结构体说明：ball_tracker_common.h**

## **BallStatus：小球状态结构体**

用于存储单个小球的实时状态数据。

```cpp
struct BallStatus {
    int id;                ///< 小球唯一标识符
    std::string color;     ///< 小球颜色描述
    double x, y;           ///< 小球当前位置（二维坐标）
    double vx, vy;         ///< 小球当前速度（X方向和Y方向）
    double progress;       ///< 小球在轨道上的进度百分比（0~1）
    bool detected;         ///< 当前位置是否为真实检测值（true），或卡尔曼预测值（false）
};
```

**字段含义说明**：

- id：用于区分不同小球的唯一整数。
- color：小球的颜色描述（例如：“red”、“blue”）。
- x, y：当前小球的位置（图像坐标）。
- vx, vy：当前小球在X和Y方向上的速度（图像坐标）。
- progress：当前小球沿轨道运动的进度（0表示起点，1表示终点）。
- detected：表示当前状态是真实检测到的，还是通过卡尔曼滤波器预测得到的。

---

## **（待定）RobotTarget ：机械臂目标结构体**

用于存储机械臂的目标位置和速度信息

```cpp
struct RobotTarget {
    double X_arm, Y_arm;     ///< 机械臂目标位置坐标
    double V_arm_x, V_arm_y; ///< 机械臂目标速度（X和Y方向速度）
};
```

**字段含义说明**：

- X_arm, Y_arm：机械臂应移动到的目标位置坐标。
- V_arm_x, V_arm_y：机械臂移动到目标位置时所需的速度分量。

---

# **调用示例参考**

```cpp
#include <iostream>

#include "ball_tracker_interface.h"

int main() {
    // 创建接口实例，加载配置并初始化所有小球
    BallTrackerInterface tracker_interface("../config/balls_config.json", std::make_pair(100.0, 200.0)));

    // 初始化轨道轨迹（只需一次）
    tracker_interface.InitTrack("../trajectory/track.json");

    // 启动跟踪算法
    tracker_interface.StartTracking();

    // 获取小球状态并打印
    auto ball_statuses = tracker_interface.GetBallStatus();
    for (const auto& status : ball_statuses) {
        std::cout << "Ball ID: " << status.id
                  << ", Position: (" << status.x << ", " << status.y << ")"
                  << ", Progress: " << status.progress
                  << ", Detected: " << (status.detected ? "True" : "False") << std::endl;
    }

    // 获取机械臂目标位置并打印
    auto robot_target = tracker_interface.GetRobotTarget();
    std::cout << "Robot Target Position: (" << robot_target.X_arm << ", " << robot_target.Y_arm << ")"
              << ", Velocity: (" << robot_target.V_arm_x << ", " << robot_target.V_arm_y << ")" << std::endl;

    // 停止跟踪
    tracker_interface.StopTracking();

    return 0;
}
```

---

# **其他说明**

- 接口内部实现了图像采集、ROI内目标检测、颜色分类和卡尔曼滤波预测等功能，调用方无需了解细节。
- 若需修改小球配置（如添加或删除小球、修改颜色阈值），仅需修改balls_config.json文件即可，无需修改源代码。