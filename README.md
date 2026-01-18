# Local-Segment-AStar-Replanner

[![ROS2](https://img.shields.io/badge/ROS2-Humble/Foxy-blue)](https://docs.ros.org/en/humble/index.html) [![Language](https://img.shields.io/badge/Language-C++-red)](https://isocpp.org/) [![License](https://img.shields.io/badge/License-Apache%202.0-green)](https://opensource.org/licenses/Apache-2.0)

**Local-Segment-AStar-Replanner** 是一款专门针对移动机器人设计的轻量级局部路径重规划器。

### 核心功能
*   **障碍物区间识别**：自动检测全局参考路径与障碍物的交段。
*   **碎片区间合并**：智能聚合相邻的碰撞区域，减少不必要的重规划开销。
*   **局部绕障搜索**：利用 A* 算法在障碍物区间生成最优绕障路径。
*   **轨迹无缝缝合**：将生成的局部绕障路径完美拼接回原始全局规划中，确保机器人避障后能迅速回归参考轨道。

---

## 🚀 算法效果对比

相较于传统的全局 A* 算法，本算法在**保持全局导向性**和**回归参考线**方面具有显著优势。

### 1. 传统全局 A* 算法
传统 A* 往往会规划出一条全新的路径，可能会大幅偏离预设的参考线。
![A-star](https://github.com/JackJu-HIT/Local-Segment-AStar-Replanner/blob/master/files/A-star.png)

### 2. Local-Segment-AStar-Replanner (本项目)
本算法在绕过障碍物后，能够**精准回归到原始全局轨道**上，适用于有预设参考轨迹的场景（如巡检、固定路线运输）。
![Local-Segment-AStar-Replanner](https://github.com/JackJu-HIT/Local-Segment-AStar-Replanner/blob/master/files/Local-Segment-AStar-Replanner.png)

---

## 📖 理论背景

本算法的核心思想源于以下学术论文：

> **C. Ju**, Q. Luo and X. Yan, "**Path Planning Using an Improved A-star Algorithm**," *2020 11th International Conference on Prognostics and System Health Management (PHM-2020 Jinan)*, Jinan, China, 2020, pp. 23-26, doi: [10.1109/PHM-Jinan48558.2020.00012](https://doi.org/10.1109/PHM-Jinan48558.2020.00012).

📄 [点击此处阅读原文 PDF](https://github.com/JackJu-HIT/Local-Segment-AStar-Replanner/blob/master/files/Path%20Planning%20Using%20an%20Improved%20A-star%20Algorithm%EF%BC%89IEEE%EF%BC%89.pdf)

---

## 🛠 如何运行

### 环境准备
*   ROS2 (Humble, Foxy 或兼容版本)
*   C++ 17

### 编译项目
在您的 ROS2 工作空间下执行：
```bash
# 进入工作空间
cd ~/ros2_ws/src
# 克隆仓库 (请确保路径正确)
git clone https://github.com/JackJu-HIT/Local-Segment-AStar-Replanner.git
# 编译
cd ..
colcon build --symlink-install
```

### 运行规划器
```bash
source install/setup.bash
ros2 run local_segment_astar_replanner local_segment_astar_node
```

---

## 📺 可视化接口 (RViz2)

启动 RViz2 后，添加以下话题即可观测实时规划逻辑：

| 话题名称 | 消息类型 | 功能说明 |
| :--- | :--- | :--- |
| `/visual_global_path` | `nav_msgs/Path` | **参考线**：起始点到目标点的原始参考直线 |
| `/visual_local_obstacles` | `sensor_msgs/PointCloud2` | **感知点云**：规划器当前考虑的局部障碍物分布 |
| `/visual_original_astar_path` | `nav_msgs/Path` | **对比线**：传统 A* 算法生成的原始路径 |
| `/visual_local_trajectory` | `nav_msgs/Path` | **最终轨迹**：避障并“缝合”后的平滑最终路径 |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | **交互设置**：通过 RViz 的 *2D Pose Estimate* 设定起点 |

---

## 📝 引用说明

如果此算法对您的研究或工程项目有所帮助，请引用以下文献：

```bibtex
@INPROCEEDINGS{9335882,
  author={Ju, Chunyu and Luo, Qingxian and Yan, Xuefeng},
  booktitle={2020 11th International Conference on Prognostics and System Health Management (PHM-2020 Jinan)}, 
  title={Path Planning Using an Improved A-star Algorithm}, 
  year={2020},
  pages={23-26},
  doi={10.1109/PHM-Jinan48558.2020.00012}
}
```

---

## 📚 教程与技术支持

想要深入了解算法实现细节或观看视频教程？欢迎关注我们的社区：

*   **微信公众号**：`机器人规划与控制研究所` (深度解析文章)
*   **B 站 (Bilibili)**：[机器人算法研究所](https://space.bilibili.com/您的UID) (视频演示)
*   **技术博客**：[点击阅读详细教程文章](https://mp.weixin.qq.com/s/您的文章链接)

---
*Developed by **Jack Ju** @ HIT*
