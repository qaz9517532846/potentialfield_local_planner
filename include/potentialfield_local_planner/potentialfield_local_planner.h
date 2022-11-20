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
#include <tf/transform_broadcaster.h>
#include <costmap_2d/costmap_2d_ros.h>

#include <potentialfield_local_planner/PotentialFieldLocalPlannerConfig.h>

#include <algorithm>

#define PATH_POINT_NUM 10
#define PI 3.1415926543

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
           unsigned int mapSize_;

        private:

           void potentialFieldForce(geometry_msgs::PoseStamped goal);
           bool isCenterOutMap(int cost_x, int cost_y, int direct_x, int direct_y);
                      
           costmap_2d::Costmap2D* costmap_;

           double att_gain_;
           double rep_gain_;

           int* attCost_;
           int* repCost_;
           int* potentialCost_;
    };
}

#endif