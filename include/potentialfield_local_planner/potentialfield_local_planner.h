#ifndef POTENTIALFIELD_LOCAL_PLANNER_H_
#define POTENTIALFIELD_LOCAL_PLANNER_H_

#include <angles/angles.h>
#include <nav_core/base_local_planner.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Bool.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include "tf2/LinearMath/Quaternion.h"
#include <costmap_2d/costmap_2d_ros.h>

#include <potentialfield_local_planner/PotentialFieldLocalPlannerConfig.h>

#include <algorithm>

namespace potentialfield_local_planner
{
    class PotentialFieldLocalPlanner
    {
        public:
           PotentialFieldLocalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
           ~PotentialFieldLocalPlanner();

           std::vector<geometry_msgs::PoseStamped> PotentialFieldLocal_Planner(geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped goal);
           void reconfigureCB(PotentialFieldLocalPlannerConfig &config);

           unsigned int width_;
           unsigned int height_;

           double resolution_;

        private:
           void dynamic_window_obstacle(unsigned int width, unsigned int height);
           float sumPointPotential(unsigned int width, unsigned int height);
           bool isCellInsideMap(double x, double y);
           double get_distance(double current_x, double current_y, double goal_x, double goal_y);
           void calculateGoalPotential(unsigned int goal_cell_x, unsigned int goal_cell_y);
           bool isGoalAcomplished(unsigned int current_cell, unsigned int goal_cell);
           int findBestCell(unsigned int cell);
           std::vector<int> potentialPlanner(unsigned int current_cell_x, unsigned int current_cell_y, unsigned int goal_cell_x, unsigned int goal_cell_y);
           std::vector<int> findPath(unsigned int current_cell_x, unsigned int current_cell_y, unsigned int goal_cell_x, unsigned int goal_cell_y);

           costmap_2d::Costmap2D* costmap_;

           float* posPotMap_; //position potential map;
           float* obsPotMap_; // obstacle potential map;
           bool* OGM_;

           unsigned int map_size_;

           double ratio_;
           double jump_;
           double walk_;
           double constant_;
           double potential_multiplier_;
           double free_cost_;
           double xy_goal_tolerance_;
    };
}

#endif