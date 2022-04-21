#include <potentialfield_local_planner/potentialfield_local_planner.h>

namespace potentialfield_local_planner
{
    PotentialFieldLocalPlanner::PotentialFieldLocalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    {
        costmap_ = costmap_ros->getCostmap();
        width_ = costmap_->getSizeInCellsX();
        height_ = costmap_->getSizeInCellsY();
        resolution_ = costmap_->getResolution();

        map_size_ = width_ * height_;

        attract_force_ = new float [map_size_];
        repulsive_force_ = new float [map_size_];
        potential_ = new float [map_size_];
    }

    PotentialFieldLocalPlanner::~PotentialFieldLocalPlanner()
    {

    }

    void PotentialFieldLocalPlanner::reconfigureCB(PotentialFieldLocalPlannerConfig &config)
    {
        alpha_ = config.alpha;
        beta_ = config.beta;
        walk_ = config.walk;
        free_cost_ = config.potential_free_cost;
    }


    std::vector<geometry_msgs::PoseStamped> PotentialFieldLocalPlanner::PotentialFieldLocal_Planner(geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped goal)
    {
        std::vector<geometry_msgs::PoseStamped> PotentialFieldLocal_Plan_;

        int start_cell[2], goal_cell[2];

        costmap_->worldToMapEnforceBounds(start.pose.position.x, start.pose.position.y, start_cell[0], start_cell[1]);
        costmap_->worldToMapEnforceBounds(goal.pose.position.x, goal.pose.position.y, goal_cell[0], goal_cell[1]);

        calculate_repulsive_localMap();
        calculate_attract_localMap(goal_cell[0], goal_cell[1]);

        return PotentialFieldLocal_Plan_;
    }

    void PotentialFieldLocalPlanner::calculate_attract_localMap(unsigned int cell_x, unsigned int cell_y)
    {
        for(int ix = 0; ix < width_; ix++)
        {
            for(int iy = 0; iy < height_; iy++)
            {
                attract_force_[ix + iy * width_] = alpha_ * get_distance(ix, iy, cell_x, cell_y) * get_distance(ix, iy, cell_x, cell_y) / 2;
            }
        }
    }

    double PotentialFieldLocalPlanner::get_distance(unsigned int current_x, unsigned int current_y, unsigned int goal_x, unsigned int goal_y)
    {
        return sqrt(pow(goal_x - current_x, 2) + pow(goal_y - current_y, 2));
    }

    void PotentialFieldLocalPlanner::calculate_repulsive_localMap()
    {
        for(int ix = 0; ix < width_; ix++)
        {
            for(int iy = 0; iy < height_; iy++)
            {
                repulsive_force_[ix + iy * width_] = costmap_->getCost(ix, iy) < free_cost_ ? 0 : beta_ * costmap_->getCost(ix, iy) * costmap_->getCost(ix, iy) / 2;
            }
        }
    }
}