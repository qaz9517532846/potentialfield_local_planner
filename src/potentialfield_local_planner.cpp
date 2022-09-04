#include <potentialfield_local_planner/potentialfield_local_planner.h>

namespace potentialfield_local_planner
{
    PotentialFieldLocalPlanner::PotentialFieldLocalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    {
        costmap_ = costmap_ros->getCostmap();
        width_ = costmap_->getSizeInCellsX();
        height_ = costmap_->getSizeInCellsY();
        resolution_ = costmap_->getResolution();
    }

    PotentialFieldLocalPlanner::~PotentialFieldLocalPlanner()
    {

    }

    void PotentialFieldLocalPlanner::reconfigureCB(PotentialFieldLocalPlannerConfig &config)
    {
        att_gain_ = config.att_gain;
        rep_gain_ = config.rep_gain;
        obs_cost_ = config.potential_obs_cost;
        min_obs_dis_ = config.min_obs_distance;
        lookahead_ = config.lookahead;
    }

    void PotentialFieldLocalPlanner::calculate_att_force(geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped goal, VectorForce &force)
    {
        force.x = att_gain_ * (goal.pose.position.x - start.pose.position.x);
		force.y = att_gain_ * (goal.pose.position.y - start.pose.position.y);
    }

    void PotentialFieldLocalPlanner::calculate_rep_force(geometry_msgs::PoseStamped start, VectorForce &force)
    {
        double obs_x, obs_y;
        double diff_x, diff_y;
        double min_x, min_y;
        double min_dis = min_obs_dis_;

        min_x = 0; min_y = 0;
        for(unsigned int iy = 0; iy < height_; iy++)
        {
            for(unsigned int ix = 0; ix < width_; ix++)
            {
                unsigned int cost = costmap_->getCost(ix, iy);
                if(cost >= obs_cost_)
                {
                    costmap_->mapToWorld(ix, iy, obs_x, obs_y);

                    diff_x = obs_x - start.pose.position.x;
                    diff_y = obs_y - start.pose.position.y;
                    if(callinearDistance(diff_x, diff_y) < min_dis)
                    {
                        min_dis = callinearDistance(diff_x, diff_y);
                        min_x = diff_x * 100;
                        min_y = diff_y * 100;
                    }
                }
            }
        }

        double gain = pow(rep_gain_, 3) / (rep_gain_ - min_obs_dis_);
        force.x = min_x != 0 ? gain * (1 / min_obs_dis_ - 1 / min_x) * pow(1 / min_x, 2) : 0;
        force.y = min_y != 0 ? gain * (1 / min_obs_dis_ - 1 / min_y) * pow(1 / min_y, 2) : 0;
        if(pow(force.x, 2) + pow(force.y, 2) > lookahead_)
        {
            force.x = 0;
            force.y = 0;
        }
    }

    std::vector<geometry_msgs::PoseStamped> PotentialFieldLocalPlanner::PotentialFieldLocal_Planner(geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped goal)
    {
        std::vector<geometry_msgs::PoseStamped> PotentialFieldLocal_Plan_;

        geometry_msgs::PoseStamped potential_pose = start;

        VectorForce att_force_, rep_force_, sum_force_;

        calculate_att_force(start, goal, att_force_);
        calculate_rep_force(start, rep_force_);

        sum_force_.x = att_force_.x + rep_force_.x;
        sum_force_.y = att_force_.y + rep_force_.y;

        //ROS_INFO("att_force_x = %f", att_force_.x);
        //ROS_INFO("att_force_y = %f", att_force_.y);

        //ROS_INFO("rep_force_x = %f", rep_force_.x);
        //ROS_INFO("rep_force_y = %f", rep_force_.y);

        calPotentialFieldLocal_Path(start, sum_force_, PotentialFieldLocal_Plan_);
        return PotentialFieldLocal_Plan_;
    }

    void PotentialFieldLocalPlanner::calPotentialFieldLocal_Path(geometry_msgs::PoseStamped start, VectorForce force, std::vector<geometry_msgs::PoseStamped> &path)
    {
        int backForward = 1;
        double pose_angle = tf2::getYaw(start.pose.orientation);
        double force_angle = calIdealAngle(atan2(force.y, force.x) - pose_angle, backForward);
        PotentialFieldLocal_Planning(start, force, force_angle, backForward, path);

        for(const auto& pose : path)
        {
            if(get_cost(pose) > obs_cost_)
            {
                path.clear();
                break;
            }
        }

        if(path.size() > 0)
        {
            return;
        }

        if(force_angle > 0)
            force_angle -= PI;
        else
            force_angle += PI;

        if(backForward == 1)
            backForward = -1;
        else if(backForward == -1)
            backForward = 1;

        PotentialFieldLocal_Planning(start, force, force_angle, backForward, path);
    }

    double PotentialFieldLocalPlanner::callinearDistance(double diff_x, double diff_y)
	{
		return sqrt(pow(diff_x, 2) + pow(diff_y, 2));
	}

    double PotentialFieldLocalPlanner::calIdealAngle(double angle, int &use_backDrive)
    {
        if(angle > PI)
			angle = angle - 2 * PI;
		else if(angle < -PI)
			angle = angle + 2 * PI;

        if(angle > PI / 2)
        {
            use_backDrive = -1;
            return angle - PI;
        }
        else if(angle < -PI / 2)
        {
            use_backDrive = -1;
            return angle + PI;
        }
        else
        {
            use_backDrive = 1;
            return angle;
        }
    }

    int PotentialFieldLocalPlanner::get_cost(geometry_msgs::PoseStamped pose)
    {
        int local_costmap_pos[2];
		costmap_->worldToMapEnforceBounds(pose.pose.position.x, pose.pose.position.y, local_costmap_pos[0], local_costmap_pos[1]);
		return costmap_->getCost(local_costmap_pos[0], local_costmap_pos[1]);
    }

    void PotentialFieldLocalPlanner::PotentialFieldLocal_Planning(geometry_msgs::PoseStamped start, VectorForce force, double angle, int direction, std::vector<geometry_msgs::PoseStamped> &path)
    {

        double delta_angle = angle / PATH_POINT_NUM;
        double delta_dis = callinearDistance(force.x, force.y) / PATH_POINT_NUM;
        double pose_angle = tf2::getYaw(start.pose.orientation);

        path.push_back(start);

        for(int i = 0; i < PATH_POINT_NUM; i++)
        {
            tf2::Quaternion quat;
            geometry_msgs::Quaternion quat_tf;
            geometry_msgs::PoseStamped force_pose = start;

            quat.setRPY(0, 0, pose_angle + i * delta_angle);
            quat_tf = tf2::toMsg(quat);
            force_pose.pose.orientation = quat_tf;

            tf2::Vector3 force_point = tf2::Matrix3x3(quat) * tf2::Vector3(direction * delta_dis * i, 0, 0);
            force_pose.pose.position.x += force_point[0];
			force_pose.pose.position.y += force_point[1];

			path.push_back(force_pose);
        }
    }
}