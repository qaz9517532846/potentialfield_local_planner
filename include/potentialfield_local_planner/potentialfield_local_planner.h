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

        private:
           void reconfigureCB(PotentialFieldLocalPlannerConfig &config);
           void dynamic_Map_Obstacle(unsigned int width_, unsigned int height_);
           void sumPointPotential(unsigned int size);
           bool isCellInsideMap(float x, float y);
           bool isStartAndGoalCellsValid(int startCell_x, int startCell_y, int goalCell_x, int goalCell_y);
           float getDistance(float x1, float y1, float x2, float y2);
           std::vector<geometry_msgs::PoseStamped> PotentialFieldLocal_Planner(geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped goal);
           std::vector<int> potentialPlanner(int startCell_x, int startCell_y, int goalCell_x, int goalCell_y);
           std::vector<int> findPath(int startCell_x, int startCell_y, int goalCell_x, int goalCell_y);
           void calculateGoalPotential(int goalCell_x, int goalCell_y);
           bool isGoalAcomplished(int current, int goal);
           int findBestCell(int cellID);

           costmap_2d::Costmap2D* costmap_;

           unsigned int width_;
           unsigned int height_;
           unsigned int map_size_;

           double resolution_;

           float* posPotMap_; //position potential map;
           float* obsPotMap_; // obstacle potential map;
           bool* OGM_;

           double ratio_;
           double jump_;
           double constant_;
           double potential_multiplier_;
           double walk_;
           double xy_tolerance_;
           double free_cost_;
    };
}

#endif