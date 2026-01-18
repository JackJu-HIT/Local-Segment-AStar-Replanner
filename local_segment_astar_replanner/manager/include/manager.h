/**
 * @brief     Local-Segment-AStar-Replanner manager class 
 * @author    juchunyu@qq.com
 * @date      2026-01-18 13:31:01
 * @copyright Copyright (c) 2026, Institute of Robotics Planning and Control (irpc). All rights reserved.
 */
#pragma once

#include <iostream>
#include "plan_env/grid_map.h"
#include "dyn_a_star.h"

struct PathPoint
{
    float x;
    float y;
    float z;
    float v;
};

struct ObstacleInfo
{
    float x;
    float y;
    float z;
};

struct intersectionPoints
{
    PathPoint start;
    PathPoint goal;
    int startIndex;
    int goalIndex;
};


class manager
{
    private:
        std::shared_ptr<GridMap2D> map_;
        PathPoint start_;
        PathPoint goal_;

        std::vector<PathPoint> global_plan_traj_;
        std::vector<PathPoint> final_path_;
        std::vector<intersectionPoints> intersection_points_;
        std::shared_ptr<AStar> planner_;

        float merge_threshold_ = 2.0; // 合并阈值，单位米

    private:
        void discretize_trajectory(const std::vector<PathPoint>& original_trajectory,
                                    std::vector<PathPoint>& discrete_trajectory,
                                    double interval);
        void calculate_intersection_points();
        bool check_collision(const PathPoint& p);
        void merge_adjacent_segments();
        void replan_and_combine_path();
        bool search_astar(const PathPoint& start, const PathPoint& goal, std::vector<PathPoint>& result);
        double distance(const PathPoint& p1, const PathPoint& p2);

    public:
        manager();
        ~manager();
        void init(std::shared_ptr<GridMap2D> map ,float merge_threshold);
        void makePlan(PathPoint start, PathPoint goal, std::vector<PathPoint> &planned_traj, bool &success);
        void getOrginalAstarPath(std::vector<PathPoint> &path);
       

};

