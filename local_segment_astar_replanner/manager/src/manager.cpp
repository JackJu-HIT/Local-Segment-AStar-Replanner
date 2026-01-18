#include "manager.h"

manager::manager()
{
}
manager::~manager()
{
}

void manager::init(std::shared_ptr<GridMap2D> map ,float merge_threshold)
{
    map_ = map;
    merge_threshold_ = merge_threshold;
    planner_ = std::make_shared<AStar>(); 
    Eigen::Vector2i map_size(200,200);
    planner_->initGridMap(map_,map_size); 
}

/**
 * 生成路径规划
 * @param start 起点
 * @param goal 终点
 * @param planned_traj 输出的规划路径
 * @param success 规划是否成功
 */
void manager::makePlan(PathPoint start, PathPoint goal, std::vector<PathPoint> &planned_traj, bool &success)
{
    start_ = start;
    goal_  = goal;
    std::cout << "[manager] start "  << start_.x << ", " << start_.y << std::endl;
    std::cout << "[manager] goal "  << goal_.x << ", " << goal_.y << std::endl;
    //1.连接起点和终点，并离散成路径点
    std::vector<PathPoint> global_plan_traj_temp;
    global_plan_traj_temp.push_back(start_);
    global_plan_traj_temp.push_back(goal_);
    discretize_trajectory(global_plan_traj_temp, global_plan_traj_, 0.1);

    std::cout << "[manager] global_plan_traj_ size: " << global_plan_traj_.size() << std::endl;

    //2.确定交接点
    calculate_intersection_points();

    //3.检测交接点是否有合并的必要行性
    merge_adjacent_segments();

    //4.开始针对于每段进行A*搜索并组合路径
    replan_and_combine_path();
    planned_traj = std::move(final_path_);

    std::cout << "[manager] planned_traj size: " << planned_traj.size() << std::endl;
     std::cout << "[manager] Merged intersection segments count: " << intersection_points_.size() << std::endl;
    // for(int i = 0 ; i < planned_traj.size(); i++)
    // {
    //     std::cout << "[manager] planned_traj[" << i << "]: " << planned_traj[i].x << ", " << planned_traj[i].y << std::endl;
    // }   
    if(planned_traj.size() > 0)
        success = true;
    else
        success = false;
}
/**
 * 获取原始A*路径
 */
void manager::getOrginalAstarPath(std::vector<PathPoint> &path)
{
    path = global_plan_traj_;
}

/**
 * 计算路径与障碍物的交点
 * 将交点存储在 intersection_points_ 成员变量中
 */
void manager::calculate_intersection_points()
{
    intersection_points_.clear();
    if (global_plan_traj_.empty()) return;

    bool is_inside_obstacle = false;
    intersectionPoints current_segment;
    
    if (global_plan_traj_.size() < 7) 
    {
        std::cout << "[calculate_intersection_points] 轨迹点过少，无法计算交点。" << std::endl;
        return;
    }
    for (int i = 3; i < (int)global_plan_traj_.size() - 3; ++i)
    {
        const auto& current_point = global_plan_traj_[i];
        bool collision = check_collision(current_point); // 你的碰撞检测逻辑

        if (!is_inside_obstacle && collision)
        {
            // 进入障碍物：记录起点坐标和索引
            is_inside_obstacle = true;
            current_segment.start = global_plan_traj_[i - 3];
            current_segment.startIndex = i - 3;
        }
        else if (is_inside_obstacle && !collision)
        {
            // 离开障碍物：记录上一个点为终点坐标和索引
            is_inside_obstacle = false;
            current_segment.goal = global_plan_traj_[i + 3];
            current_segment.goalIndex = i + 3;
            intersection_points_.push_back(current_segment);
        }
    }

    // 处理轨迹末尾仍在障碍物内的情况
    if (is_inside_obstacle)
    {
        return;
        current_segment.goal = global_plan_traj_.back();
        current_segment.goalIndex = (int)global_plan_traj_.size() - 1;
        intersection_points_.push_back(current_segment);
    }
}

/**
 * @brief 合并相邻障碍物段
 * @param merge_dist_threshold— 距离阈值（两段之间的缝隙距离）
 */
void manager::merge_adjacent_segments()
{
    if (intersection_points_.size() < 2) return;

    std::vector<intersectionPoints> merged_results;
    
    // 初始化：取第一段作为当前待处理段
    intersectionPoints current_proc = intersection_points_[0];

    for (size_t i = 1; i < intersection_points_.size(); ++i)
    {
        const auto& next_seg = intersection_points_[i];

        // 计算当前段终点与下一段起点之间的欧氏距离
        float dx = current_proc.goal.x - next_seg.start.x;
        float dy = current_proc.goal.y - next_seg.start.y;
        float dz = current_proc.goal.z - next_seg.start.z;
        float gap_dist = std::sqrt(dx*dx + dy*dy + dz*dz);

        if (gap_dist <= merge_threshold_)
        {
            // 满足合并条件：
            // 1. 起点(start)和起点索引(startIndex)保持不变
            // 2. 终点(goal)和终点索引(goalIndex)更新为下一段的值
            current_proc.goal = next_seg.goal;
            current_proc.goalIndex = next_seg.goalIndex;
            
            // 注：此时不 push_back，因为下一段可能还能和再下一段合并
        }
        else
        {
            // 不满足合并条件：
            // 保存当前已完成合并的段
            merged_results.push_back(current_proc);
            // 将下一段作为新的待处理段
            current_proc = next_seg;
        }
    }

    // 压入最后处理的一段
    merged_results.push_back(current_proc);

    // 更新成员变量
    intersection_points_ = std::move(merged_results);
   
}

/**
 * @brief 执行避障路径重规划并组合最终路径
 */
void manager::replan_and_combine_path()
{
    // 最终生成的完整路径
    final_path_.clear();
   
    // 记录原始轨迹中上一次处理到的索引位置
    int last_processed_idx = 0;

    // 遍历所有识别出的障碍物段
    for (const auto& segment : intersection_points_)
    {
        // 第一步：添加原始轨迹中 [上一个终点, 当前障碍物起点) 之间的安全点
        for (int i = last_processed_idx; i < segment.startIndex; ++i)
        {
            final_path_.push_back(global_plan_traj_[i]);
        }

        // 第二步：针对当前障碍物段，调用 A* 算法进行绕障搜索
        std::vector<PathPoint> astar_segment;
        bool success = search_astar(segment.start, segment.goal, astar_segment);

        if (success)
        {
            // 如果 A* 寻路成功，将绕障路径点加入最终路径
            // 注意：astar_segment 的起点和终点通常就是 segment.start 和 segment.goal
            final_path_.insert(final_path_.end(), astar_segment.begin(), astar_segment.end());
        }
        else
        {
            // 如果 A* 失败，说明该障碍物无法绕过
            // 这里可以采取策略：比如停止、或者仍然保留原路径（会撞）并报警
            std::cout << "Warning: A* failed to find path for segment: " 
                      << segment.startIndex << " to " << segment.goalIndex << std::endl;
        }

        // 更新索引：下一次从障碍物的 goalIndex 之后开始接原始点
        last_processed_idx = segment.goalIndex + 1;
    }

    // 第三步：添加最后一个障碍物之后到轨迹末尾的所有点
    for (int i = last_processed_idx; i < (int)global_plan_traj_.size(); ++i)
    {
        final_path_.push_back(global_plan_traj_[i]);
    }
}

/**
 * @brief A* 搜索实现
 * @param start 起点
 * @param goal 终点
 * @param[out] result 避障路径结果
 * @return 是否成功找到路径
 */
bool manager::search_astar(const PathPoint& start, const PathPoint& goal, std::vector<PathPoint>& result)
{
    result.clear();

    Eigen::Vector2d start_pt(start.x, start.y);  
    Eigen::Vector2d end_pt(goal.x,goal.y);
    bool success = planner_->AstarSearch(0.1, start_pt, end_pt);  // 步长=分辨率

    if (success) 
    {
        result.clear();
        std::vector<Eigen::Vector2d> path = planner_->getPath();  // 注意：getPath需改为返回Vector2d
        std::cout << "\n找到路径（世界坐标）:\n";
        for (const auto& pt : path) 
        {
            PathPoint temp;
            temp.x = pt.x() ;
            temp.y = pt.y();
            temp.z = 0;
            result.push_back(temp);
            // std::cout << "(" << pt.x() << ", " << pt.y() << ")\n";
        }
        return true;
    } 
    else
    {
        std::cout << "\n未找到路径！\n";
        return false;
    }
}

/**
 * 检查路径点是否与障碍物碰撞
 * @param p 路径点
 * @return 是否碰撞
 */
bool manager::check_collision(const PathPoint& p)
{
    return map_->getInflateOccupancy(Eigen::Vector2d(p.x, p.y));
}

// 计算两点之间的欧氏距离（单位：米）
double manager::distance(const PathPoint& p1, const PathPoint& p2) {
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    return std::sqrt(dx*dx + dy*dy);
}

/**
 * 将轨迹离散为均匀间隔的点（间隔10cm）
 * @param original_trajectory 原始轨迹（由多个顶点组成的折线）
 * @param discrete_trajectory 输出的离散轨迹
 * @param interval 间隔距离（单位：米，默认0.1米即10cm）
 */
void manager::discretize_trajectory(const std::vector<PathPoint>& original_trajectory,
                                                            std::vector<PathPoint>& discrete_trajectory,
                                                            double interval) {
    if (original_trajectory.size() < 2) {
        std::cerr << "原始轨迹至少需要2个点！" << std::endl;
        return;
    }

    discrete_trajectory.clear();
    // 添加轨迹起点
    // discrete_trajectory.push_back(cur_pose_);
    discrete_trajectory.push_back(original_trajectory[0]);

    // 遍历原始轨迹的每一段线段
    for (size_t i = 0; i < original_trajectory.size() - 1; ++i) {
        const PathPoint& start = original_trajectory[i];
        const PathPoint& end = original_trajectory[i+1];
        double seg_length = distance(start, end);  // 线段总长度

        if (seg_length < 1e-6) {  // 跳过长度接近0的线段（避免除零）
            continue;
        }

        // 计算当前线段需要插入的点数（不含起点，含终点）
        int num_points = static_cast<int>(seg_length / interval);
        // 最后一个点到终点的距离（避免累积误差）
        double last_interval = seg_length - num_points * interval;

        // 生成线段上的离散点
        for (int j = 1; j <= num_points; ++j) {
            double ratio;
            if (j < num_points) {
                // 前num_points-1个点：按均匀间隔计算
                ratio = (j * interval) / seg_length;
            } else {
                // 最后一个点：直接对齐到线段终点（避免累积误差）
                ratio = 1.0;
            }

            // 线性插值计算点坐标
            PathPoint p;
            p.x = start.x + ratio * (end.x - start.x);
            p.y = start.y + ratio * (end.y - start.y);
            discrete_trajectory.push_back(p);
        }
    }
}