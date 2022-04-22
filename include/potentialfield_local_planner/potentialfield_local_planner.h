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
           void calcutate_potentialfield_localMap(unsigned int cell_x, unsigned int cell_y);
           int calculate_bestvalue(int cell);
           double get_distance(unsigned int current_x, unsigned int current_y, unsigned int goal_x, unsigned int goal_y);

           costmap_2d::Costmap2D* costmap_;

           float* attract_force_; //position potential map;
           float* repulsive_force_; // obstacle potential map;
           float* potential_;

           unsigned int map_size_;

           double alpha_;
           double beta_;
           double walk_;
           double free_cost_;
           double xy_goal_tolerance;
    };
}

#endif