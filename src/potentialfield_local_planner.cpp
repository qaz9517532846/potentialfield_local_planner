#include <potentialfield_local_planner/potentialfield_local_planner.h>

#include <iostream>
#include <fstream>
using namespace std;

namespace potentialfield_local_planner
{
    PotentialFieldLocalPlanner::PotentialFieldLocalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    {
        costmap_ = costmap_ros->getCostmap();
        width_ = costmap_->getSizeInCellsX();
        height_ = costmap_->getSizeInCellsY();

        mapSize_ = width_ * height_;

        attCost_ = new int [mapSize_];
        repCost_ = new int [mapSize_];
        potentialCost_ = new int [mapSize_];
    }

    PotentialFieldLocalPlanner::~PotentialFieldLocalPlanner()
    {

    }

    void PotentialFieldLocalPlanner::reconfigureCB(PotentialFieldLocalPlannerConfig &config)
    {
        att_gain_ = config.att_gain;
        rep_gain_ = config.rep_gain;
    }

    void PotentialFieldLocalPlanner::potentialFieldForce(geometry_msgs::PoseStamped goal)
    {
        int goal_cost_x, goal_cost_y;
        costmap_->worldToMapEnforceBounds(goal.pose.position.x, goal.pose.position.y, goal_cost_x, goal_cost_y);

        for(int iy = 0; iy < height_; iy++)
        {
            for(int ix = 0; ix < width_; ix++)
            {
                attCost_[iy * width_ + ix] = (int)(10 * att_gain_ * sqrt(pow(goal_cost_x - ix, 2) +  pow(goal_cost_y - iy, 2)));
                repCost_[iy * width_ + ix] = (int)costmap_->getCost(ix, iy) * rep_gain_;
                potentialCost_[iy * width_ + ix] = attCost_[iy * width_ + ix] + repCost_[iy * width_ + ix];
            }
        }

    }

    std::vector<geometry_msgs::PoseStamped> PotentialFieldLocalPlanner::PotentialFieldLocal_Planner(geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped goal)
    {
        std::vector<geometry_msgs::PoseStamped> PotentialFieldLocal_Plan_;
        geometry_msgs::PoseStamped lastPose_;
        int start_cost_x, start_cost_y;
        int current_x_, current_y_;
        int last_current_x_, last_current_y_;
        int tempPotentialCost_, tempCostX, tempCostY;
        double findPointX_, findPointY_;

        PotentialFieldLocal_Plan_.push_back(start);
        lastPose_ = start;

        costmap_->worldToMapEnforceBounds(start.pose.position.x, start.pose.position.y, start_cost_x, start_cost_y);
        potentialFieldForce(goal);

        current_x_ = start_cost_x;
        current_y_ = start_cost_y;
        
        while(potentialCost_[current_y_ * width_ + current_x_] != 0)
        {
            geometry_msgs::PoseStamped findPose_;
            double angle_;
            tempPotentialCost_ = std::numeric_limits<int>::max();
            for(int i = -1; i <= 1; i++)
            {
                for(int j = -1; j <= 1; j++)
                {
                    if(!isCenterOutMap(current_x_, current_y_, i, j))
                    {
                        if(potentialCost_[(current_y_ + j) * width_ + (current_x_ + i)] < tempPotentialCost_)
                        {
                            tempPotentialCost_ = potentialCost_[(current_y_ + j) * width_ + (current_x_ + i)];
                            current_x_ = current_x_ + i;
                            current_y_ = current_y_ + j;
                        }
                    }
                }
            }

            costmap_->mapToWorld(current_x_, current_y_, findPointX_, findPointY_);

            findPose_.pose.position.x = findPointX_;
            findPose_.pose.position.y = findPointY_;

            angle_ = atan2(findPose_.pose.position.y - start.pose.position.y, findPose_.pose.position.x - start.pose.position.x);
            findPose_.pose.orientation = tf::createQuaternionMsgFromYaw(angle_);
            
            PotentialFieldLocal_Plan_.push_back(findPose_);
            if(current_x_ == last_current_x_ && current_y_ == last_current_y_) break;

            lastPose_ = findPose_;
            last_current_x_ = current_x_;
            last_current_y_ = current_y_;
            break;
        }

        PotentialFieldLocal_Plan_.push_back(goal);

        return PotentialFieldLocal_Plan_;
    }

    bool PotentialFieldLocalPlanner::isCenterOutMap(int cost_x, int cost_y, int direct_x, int direct_y)
    {
        bool result = false;
        int costPos_;

        if(direct_x == 0 && direct_y == 0) result = true;
        if(cost_x + direct_x >= width_ || cost_x + direct_x < 0) result = true;
        if(cost_y + direct_y >= height_ || cost_y + direct_y < 0) result = true;

        return result;
    }
}