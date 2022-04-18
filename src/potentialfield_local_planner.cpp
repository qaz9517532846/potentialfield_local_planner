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
        xy_tolerance_ = config.potentialfield_xy_tolerance;
        free_cost_ = config.potential_free_cost;
    }

    void PotentialFieldLocalPlanner::dynamic_Map_Obstacle()
    {
        for(unsigned int iy = 0; iy < height_; iy++)
        {
            for(unsigned int ix = 0; ix < width_; ix++)
            {
                unsigned int cost = costmap_->getCost(ix, iy);
                OGM_[iy * width_ + ix] = cost < free_cost_ ? true : false;
                if(!OGM_[iy * width_ + ix])
                {
                    sumPointPotential(iy * width_ + ix);
                }
            }
        }
    }

    void PotentialFieldLocalPlanner::sumPointPotential(unsigned int size)
    {
        float x_, y_, cost_;
        int x_index = size % width_;
        int y_index = size / width_;
        x_ = x_index * resolution_;
        y_ = y_index * resolution_;
        for(register int i = -ratio_; i < ratio_; i += jump_ * resolution_)
        {
            for(register int j = -(ratio_ -fabs(i)); j < (ratio_ - fabs(i)); j += jump_ * resolution_)
            {
                if(isCellInsideMap(x_ + i, y_ + j))
                {
                    cost_ = (i != 0 || j != 0) ? constant_ / getDistance(x_ + i, y_ + j, x_, y_) : 300000;
                    if(obsPotMap_[size] < cost_)
                    {
                        obsPotMap_[size] = cost_;
                    }
                }
            }
        }
    }

    bool PotentialFieldLocalPlanner::isCellInsideMap(float x, float y)
    {
        bool valid = !(x >= (width_ * resolution_) || y >= (height_ * resolution_) || (x < 0) || (y < 0));
        return valid;
    }

    bool PotentialFieldLocalPlanner::isStartAndGoalCellsValid(int startCell_x, int startCell_y, int goalCell_x, int goalCell_y)
    {
        bool isFreeStartCell = OGM_[startCell_y * width_ + startCell_x];
        bool isFreeGoalCell = OGM_[goalCell_y * width_ + goalCell_x];

        return (isFreeStartCell && isFreeGoalCell) ? true : false;
    }

    float PotentialFieldLocalPlanner::getDistance(float x1, float y1, float x2, float y2)
    {
        return std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
    }

    std::vector<geometry_msgs::PoseStamped> PotentialFieldLocalPlanner::PotentialFieldLocal_Planner(geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped goal)
    {
        std::vector<geometry_msgs::PoseStamped> PotentialFieldLocal_Plan_;
        int start_cell[2], goal_cell[2];

        dynamic_Map_Obstacle();
        
        costmap_->worldToMapEnforceBounds(start.pose.position.x, start.pose.position.y, start_cell[0], start_cell[1]);
        costmap_->worldToMapEnforceBounds(goal.pose.position.x, goal.pose.position.y, goal_cell[0], goal_cell[1]);

        if(isStartAndGoalCellsValid(start_cell[0], start_cell[1], goal_cell[0], goal_cell[1]))
        {
            std::vector<int> potential_field_Planner_;
            potential_field_Planner_ = potentialPlanner(start_cell[0], start_cell[1], goal_cell[0], goal_cell[1]);
        }

        return PotentialFieldLocal_Plan_;
    }

    std::vector<int> PotentialFieldLocalPlanner::potentialPlanner(int startCell_x, int startCell_y, int goalCell_x, int goalCell_y)
    {
        std::vector<int> bestPath;
        calculateGoalPotential(goalCell_x, goalCell_y);
        bestPath = findPath(startCell_x, startCell_y, goalCell_x, goalCell_y);
        return bestPath;
    }

    void PotentialFieldLocalPlanner::calculateGoalPotential(int goalCell_x, int goalCell_y)
    {
        double goal_pose[2];
        goal_pose[0] = goalCell_x * resolution_;
        goal_pose[1] = goalCell_y * resolution_;

        for(unsigned int iy = 0; iy < height_; iy++)
        {
            for(unsigned int ix = 0; ix < width_; ix++)
            {
                double costmapToWorld[2];
                costmapToWorld[0] = goalCell_x * resolution_;
                costmapToWorld[1] = goalCell_y * resolution_;
                posPotMap_[iy * width_ + ix] = potential_multiplier_ * getDistance(costmapToWorld[0], costmapToWorld[1], goal_pose[0], goal_pose[1]);
            }
        }
    }

    std::vector<int> PotentialFieldLocalPlanner::findPath(int startCell_x, int startCell_y, int goalCell_x, int goalCell_y)
    {
        std::vector<int> bestPath;
        int currentCell = startCell_y * width_ + startCell_x;
        int goalCell = goalCell_y * width_ + goalCell_x;
        bestPath.push_back(currentCell);
        while(!isGoalAcomplished(currentCell, goalCell))
        {
            currentCell = findBestCell(currentCell);
            if(currentCell == -1)
               return bestPath;
            bestPath.push_back(currentCell);
        }

        return bestPath; 
    }

    bool PotentialFieldLocalPlanner::isGoalAcomplished(int current, int goal)
    {
        double current_pose[2];
        double goal_pose[2];
        int currentCell_x = current % width_;
        int currentCell_y = current / width_;
        int goalCell_x = goal % width_;
        int goalCell_y = goal / width_;

        current_pose[0] = currentCell_x * resolution_;
        current_pose[1] = currentCell_y * resolution_;

        goal_pose[0] = goalCell_x * resolution_;
        goal_pose[1] = goalCell_y * resolution_;

        return getDistance(current_pose[0], current_pose[1], goal_pose[0], goal_pose[1]) < xy_tolerance_ ? true : false;
    }

    int PotentialFieldLocalPlanner::findBestCell(int cellID)
    {
        int best_index;
        float best_value = 999999;
        double cell_pose[2];
        int cell_x = cellID % width_;
        int cell_y = cellID / width_;
        cell_pose[0] = cell_x * resolution_;
        cell_pose[1] = cell_y * resolution_;
        for(float i = -walk_ * resolution_; i <= walk_ * resolution_; i += resolution_)
        {
            for(float j = -walk_ * resolution_; j <= walk_ * resolution_; j += resolution_)
            {
                if(isCellInsideMap(cell_pose[0] + i, cell_pose[1] + j))
                {
                    int index = (cell_pose[0] + i) / resolution_ + (cell_pose[1] + j) / resolution_ * width_;
                    if(best_value > obsPotMap_[index] + posPotMap_[index])
                    {
                        best_value = obsPotMap_[index] + posPotMap_[index];
                        best_index = index;
                    }
                }
            }
        }

        if(best_index = cellID)
        {
            return -1;
        }

        return best_index;
    }
}