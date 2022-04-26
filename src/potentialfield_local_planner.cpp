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

        posPotMap_ = new float [map_size_];
        obsPotMap_ = new float [map_size_];
        OGM_ = new bool [map_size_];
    }

    PotentialFieldLocalPlanner::~PotentialFieldLocalPlanner()
    {

    }

    void PotentialFieldLocalPlanner::reconfigureCB(PotentialFieldLocalPlannerConfig &config)
    {
        ratio_ = config.ratio;
        jump_ = config.jump;
        constant_ = config.constant;
        potential_multiplier_ = config.potential_multiplier;
        walk_ = config.walk;
        free_cost_ = config.potential_free_cost;
        xy_goal_tolerance_ = config.xy_goal_tolerance;
    }

    void PotentialFieldLocalPlanner::dynamic_window_obstacle(unsigned int width, unsigned int height)
    {
        for(unsigned int iy = 0; iy < height; iy++)
        {
            for(unsigned int ix = 0; ix < width; ix++)
            {
                unsigned int cost = costmap_->getCost(ix, iy);
                OGM_[iy * width_ + ix] = cost < free_cost_ ? true : false;
                obsPotMap_[iy * width_ + ix] = !OGM_[iy * width_ + ix] ? sumPointPotential(ix, iy) : 0;
            }
        }
    }

    float PotentialFieldLocalPlanner::sumPointPotential(unsigned int width, unsigned int height)
    {
        double x, y;
        float cost;
        int index = width + height * width_;

        costmap_->mapToWorld(width, height, x, y);

        for(register float i = -ratio_; i < ratio_; i += jump_ * resolution_)
        {
            for(register float j = -(ratio_ - fabs(i)); j < (ratio_ - fabs(i)); j += jump_ * resolution_)
            {
                if(isCellInsideMap(x + i, y + j))
                {
                    cost = (i != 0 || j != 0) ? constant_ / get_distance(x + i, y + j, x, y) : 300000;
                }
            }
        }

        return cost;
    }

    bool PotentialFieldLocalPlanner::isCellInsideMap(double x, double y)
    {
        int x_prime, y_prime;
        costmap_->worldToMapEnforceBounds(x, y, x_prime, y_prime);
        return x_prime < width_ && x_prime >= 0 && y_prime < height_ && y_prime >= 0;
    }

    std::vector<geometry_msgs::PoseStamped> PotentialFieldLocalPlanner::PotentialFieldLocal_Planner(geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped goal)
    {
        std::vector<geometry_msgs::PoseStamped> PotentialFieldLocal_Plan_;
        std::vector<int> bestPath;
        bestPath.clear();

        int start_cell[2], goal_cell[2];

        costmap_->worldToMapEnforceBounds(start.pose.position.x, start.pose.position.y, start_cell[0], start_cell[1]);
        costmap_->worldToMapEnforceBounds(goal.pose.position.x, goal.pose.position.y, goal_cell[0], goal_cell[1]);

        bestPath = potentialPlanner(start_cell[0], start_cell[1], goal_cell[0], goal_cell[1]); 

        for(const auto& index : bestPath)
        {
            double x, y;
            costmap_->mapToWorld(index % width_, index / width_, x, y);

            geometry_msgs::PoseStamped pose = goal;
            pose.pose.position.x = x;
            pose.pose.position.y = y;
            pose.pose.position.z = 0.0;
            pose.pose.orientation = start.pose.orientation;
            PotentialFieldLocal_Plan_.push_back(pose);
        }

        //PotentialFieldLocal_Plan_.push_back(goal);
        return PotentialFieldLocal_Plan_;
    }

    std::vector<int> PotentialFieldLocalPlanner::potentialPlanner(unsigned int current_cell_x, unsigned int current_cell_y, unsigned int goal_cell_x, unsigned int goal_cell_y)
    {
        std::vector<int> bestPath;
        calculateGoalPotential(goal_cell_x, goal_cell_y);
        bestPath = findPath(current_cell_x, current_cell_y, goal_cell_x, goal_cell_y);
        return bestPath;
    }

    void PotentialFieldLocalPlanner::calculateGoalPotential(unsigned int goal_cell_x, unsigned int goal_cell_y)
    {
        double goal_x, goal_y;
        costmap_->mapToWorld(goal_cell_x, goal_cell_y, goal_x, goal_y);

        double x, y;

        for(int iy = 0; iy < height_; iy++)
        {
            for(int ix = 0; ix < width_; ix++)
            {
                costmap_->mapToWorld(ix, iy, x, y);
                posPotMap_[ix + iy * width_] = potential_multiplier_ * get_distance(ix, iy, goal_x, goal_y);
            }
        }
    }

    std::vector<int> PotentialFieldLocalPlanner::findPath(unsigned int current_cell_x, unsigned int current_cell_y, unsigned int goal_cell_x, unsigned int goal_cell_y)
    {
        std::vector<int> bestPath;
        int currentCell = current_cell_x + current_cell_y * width_;
        int goalCell = goal_cell_x + goal_cell_y * width_;

        bestPath.push_back(currentCell);

        while(!isGoalAcomplished(currentCell, goalCell))
        {
            currentCell = findBestCell(currentCell);

            if(currentCell == - 1)
            {
                return bestPath;
            }

            bestPath.push_back(currentCell);
        }

        return bestPath;
    }

    bool PotentialFieldLocalPlanner::isGoalAcomplished(unsigned int current_cell, unsigned int goal_cell)
    {
        double current_pose[2];
        double goal_pose[2];

        costmap_->mapToWorld(current_cell % width_, current_cell / width_, current_pose[0], current_pose[1]);
        costmap_->mapToWorld(goal_cell % width_, goal_cell / width_, goal_pose[0], goal_pose[1]);

        return get_distance(current_pose[0], current_pose[1], goal_pose[0], goal_pose[1]) < xy_goal_tolerance_ ? true : false;
    }

    unsigned int PotentialFieldLocalPlanner::findBestCell(unsigned int cellID)
    {
        double pose[2];
        unsigned int index, best_index;
        int cell_prime[2];
        float best_value= 999999;

        costmap_->mapToWorld(cellID % width_, cellID / width_, pose[0], pose[1]);

        std::cout << " DADO: (" << pose[0] << "," << pose[1] << ") INDEX: " << index << std::endl;

        for(float i = -walk_ * resolution_ ; i <=  walk_ * resolution_ ; i+= resolution_)
        {
            for(float j = -walk_ * resolution_ ; j <= walk_ * resolution_ ; j+= resolution_)
            {
                if(isCellInsideMap(pose[0] + i, pose[1] + j))
                {
                    costmap_->worldToMapEnforceBounds(pose[0] + i, pose[1] + j, cell_prime[0], cell_prime[1]); 
                    index = cell_prime[0] + cell_prime[1] * width_;
                    if(best_value > obsPotMap_[index] + posPotMap_[index])
                    {
                        best_value = obsPotMap_[index] + posPotMap_[index];
                        best_index = index;
                    }
                }
            }
        }

        if(best_index == cellID)
        {
            ROS_ERROR("MINIMO LOCAL");
            return -1;
        }

        return best_index;
    }

    double PotentialFieldLocalPlanner::get_distance(double current_x, double current_y, double goal_x, double goal_y)
    {
        return sqrt(pow(goal_x - current_x, 2) + pow(goal_y - current_y, 2));
    }
}