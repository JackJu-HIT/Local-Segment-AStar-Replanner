# Local-Segment-AStar-Replanner

[![ROS2](https://img.shields.io/badge/ROS2-Humble/Foxy-blue)](https://docs.ros.org/en/humble/index.html) [![Paper](https://img.shields.io/badge/Paper-PHM--2020-orange)](https://doi.org/10.1109/PHM-Jinan48558.2020.00012) [![Author](https://img.shields.io/badge/Author-Jack--Ju-red)](#)

**本项目由作者根据其 2020 年发表的原创学术论文实现。** 

这是一种基于轨迹的局部重规划算法，旨在解决传统全局路径规划算法在避障时偏离参考线过远的问题。

---

## 📢 理论背景

本算法的核心理论支柱为作者 **Jack Ju** 于 2020 年发表的学术论文。该算法提出了一种改进的 A* 逻辑，专注于在复杂环境下通过分段重规划实现高效避障。

> **C. Ju**, Q. Luo and X. Yan, "**Path Planning Using an Improved A-star Algorithm**," *2020 11th International Conference on Prognostics and System Health Management (PHM-2020 Jinan)*, Jinan, China, 2020.

*   **论文原件**：[点击此处在仓库内查看 PDF](https://github.com/JackJu-HIT/Local-Segment-AStar-Replanner/blob/master/files/Path%20Planning%20Using%20an%20Improved%20A-star%20Algorithm.pdf) 
*   **IEEE 官方链接**：[DOI: 10.1109/PHM-Jinan48558.2020.00012](https://doi.org/10.1109/PHM-Jinan48558.2020.00012)

---

## 💡 算法核心优势

本算法（Local-Segment-AStar-Replanner）与传统 A* 算法的最大区别在于：**“局部缝合，快速回归”**。

| 特性 | 传统全局 A* 算法 | Local-Segment-AStar-Replanner |
| :--- | :--- | :--- |
| **规划逻辑** | 重新计算整条路径 | 仅针对障碍物交段进行局部绕障 |
| **参考线保持** | 容易大幅度偏离预设轨道 | **强制在避障后迅速回归原始轨道** |
| **计算开销** | 随地图规模增大而显著增加 | 计算量集中在局部碰撞段，效率极高 |

### 效果对比
*   **传统 A* 效果**：路径虽然无碰撞，但完全丢失了原始参考线的约束。
    ![A-star](https://github.com/JackJu-HIT/Local-Segment-AStar-Replanner/blob/master/files/A-star.png)
*   **本项目算法效果**：在绕过障碍物后，路径精准地“缝合”回原参考线。
    ![Local-Segment-AStar-Replanner](https://github.com/JackJu-HIT/Local-Segment-AStar-Replanner/blob/master/files/Local-Segment-AStar-Replanner.png)

---

## 🛠 快速上手

### 1. 编译环境
*   Ubuntu 20.04/22.04
*   ROS2 Humble/Foxy
*   C++ 17 & Eigen3

### 2. 构建项目
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/JackJu-HIT/Local-Segment-AStar-Replanner.git
cd ..
colcon build --symlink-install
```

### 3. 运行演示
```bash
source install/setup.bash
ros2 run local_segment_astar_replanner local_segment_astar_replanner
```

---

## 📊 可视化与交互 (RViz2)

| 话题名称 | 功能说明 |
| :--- | :--- |
| `/visual_global_path` | 原始参考直线（起始点 -> 目标点） |
| `/visual_local_obstacles` | 当前感知到的局部障碍物点云 |
| `/visual_local_trajectory` | **最终输出轨迹**：避障后缝合回参考线的路径 |
| `/initialpose` | 在 RViz 中使用 *2D Pose Estimate* 工具设置起点 |
| `/visual_original_astar_path` |  原始路线(目前仅考虑起点和终点的直线)

---

## 📜 引用要求 (Citation)

如果您在科研、教学或工程项目中使用此算法，请务必引用作者 2020 年的原始论文：

```bibtex
@INPROCEEDINGS{9335882,
  author={Ju, Chunyu and Luo, Qinghua and Yan, Xiaozhen},
  booktitle={2020 11th International Conference on Prognostics and System Health Management (PHM-2020 Jinan)}, 
  title={Path Planning Using an Improved A-star Algorithm}, 
  year={2020},
  pages={23-26},
  doi={10.1109/PHM-Jinan48558.2020.00012}
}
```

---

## 🤝 交流与支持

如果您对算法的实现细节感兴趣，或者在部署过程中遇到问题，欢迎通过以下渠道与我交流：

*   **微信公众号**：[机器人规划与控制研究所](https://mp.weixin.qq.com/s/xjxazkdFrHZYgzjGiUUJAw) （深度解析本项目核心逻辑）
*   **Bilibili**：[机器人算法研究所]
*   **联系作者**：Jack Ju HIT

---

### 💡 技术解析文章推荐
为了帮助您更好地理解本项目，建议阅读以下专题文章：
*   [【机器人轨迹优化基础】从原理到实现：支持动态膨胀的机器人A*局部路径规划开源算法【Github开源】【ROS2】【C++】](https://mp.weixin.qq.com/s/xjxazkdFrHZYgzjGiUUJAw)
