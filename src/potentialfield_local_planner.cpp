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
        xy_goal_tolerance = config.xy_goal_tolerance;
    }


    std::vector<geometry_msgs::PoseStamped> PotentialFieldLocalPlanner::PotentialFieldLocal_Planner(geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped goal)
    {
        std::vector<geometry_msgs::PoseStamped> PotentialFieldLocal_Plan_;

        int start_cell[2], goal_cell[2];
        int current_cell[2];

        costmap_->worldToMapEnforceBounds(start.pose.position.x, start.pose.position.y, start_cell[0], start_cell[1]);
        costmap_->worldToMapEnforceBounds(goal.pose.position.x, goal.pose.position.y, goal_cell[0], goal_cell[1]);

        current_cell[0] = start_cell[0];
        current_cell[1] = start_cell[1];
        int cutrrent_index = current_cell[0] + current_cell[1] * width_;
        calcutate_potentialfield_localMap(goal_cell[0], goal_cell[1]);

        PotentialFieldLocal_Plan_.push_back(start);
        while(get_distance(current_cell[0], current_cell[1], goal_cell[0], goal_cell[1]) >= xy_goal_tolerance)
        {
            double local_world[2];
            geometry_msgs::PoseStamped local_pose;
            cutrrent_index = calculate_bestvalue(cutrrent_index);
            current_cell[0] = cutrrent_index % width_;
            current_cell[1] = cutrrent_index / width_;

            costmap_->mapToWorld(current_cell[0], current_cell[1], local_world[0], local_world[1]);
            local_pose.pose.position.x = local_world[0];
            local_pose.pose.position.y = local_world[1];
            local_pose.pose.orientation = start.pose.orientation;
            PotentialFieldLocal_Plan_.push_back(local_pose);
        }

        PotentialFieldLocal_Plan_.push_back(goal);
        return PotentialFieldLocal_Plan_;
    }

    double PotentialFieldLocalPlanner::get_distance(unsigned int current_x, unsigned int current_y, unsigned int goal_x, unsigned int goal_y)
    {
        return sqrt(pow(goal_x - current_x, 2) + pow(goal_y - current_y, 2));
    }

    void PotentialFieldLocalPlanner::calcutate_potentialfield_localMap(unsigned int cell_x, unsigned int cell_y)
    {
        for(int ix = 0; ix < width_; ix++)
        {
            for(int iy = 0; iy < height_; iy++)
            {
                double attract_force_ = alpha_ * get_distance(ix, iy, cell_x, cell_y) * resolution_;
                double repulsive_force_ = costmap_->getCost(ix, iy) < free_cost_ ? 0 : beta_ * costmap_->getCost(ix, iy);
                potential_[ix + iy * width_] = attract_force_ + repulsive_force_;
            }
        }
    }

    int PotentialFieldLocalPlanner::calculate_bestvalue(int cell)
    {
        int best_index;
        int center_x = cell % width_;
        int center_y = cell / width_;

        int start_x = center_x - walk_ <           0 ?           0 : center_x - walk_;
        int end_x   = center_x + walk_ >  width_ - 1 ?  width_ - 1 : center_x + walk_;
        int start_y = center_y - walk_ <           0 ?           0 : center_y - walk_;
        int end_y   = center_y + walk_ > height_ - 1 ? height_ - 1 : center_y + walk_;

        std::vector<int> dynamic_window_index_;
        std::vector<double> dynamic_window_potential_;

        for(int iy = start_y; iy <= end_y; iy++)
        {
            for(int ix = start_x; ix <= end_x; ix++)
            {
                dynamic_window_index_.push_back(ix + iy * width_);
                dynamic_window_potential_.push_back(potential_[ix + iy * width_]);
            }
        }

        best_index = std::min_element(dynamic_window_potential_.begin(), dynamic_window_potential_.end()) - dynamic_window_potential_.begin();

        return dynamic_window_index_[best_index];
    }
}