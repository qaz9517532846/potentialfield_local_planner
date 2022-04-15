#include <potentialfield_local_planner/potentialfield_local_planner_ros.h>
#include <pluginlib/class_list_macros.h>

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(potentialfield_local_planner::PotentialFieldLocalPlannerROS, nav_core::BaseLocalPlanner)

namespace potentialfield_local_planner
{
    PotentialFieldLocalPlannerROS::PotentialFieldLocalPlannerROS()
    {

    }

    PotentialFieldLocalPlannerROS::~PotentialFieldLocalPlannerROS()
    {

    }

    void PotentialFieldLocalPlannerROS::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
    {
        ros::NodeHandle private_nh("~/" + name);

        global_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
		local_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);

        tf_ = tf;
		
		//create the actual planner that we'll use.. it'll configure itself from the parameter server
		dp_ = boost::shared_ptr<PotentialFieldLocalPlanner>(new PotentialFieldLocalPlanner(name, costmap_ros));
    }

    bool PotentialFieldLocalPlannerROS::isGoalReached()
    {
        return (state_ == Finished);
    }

	bool PotentialFieldLocalPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan)
	{
        global_plan_.clear();
		local_plan_.clear();

		// Make our copy of the global plan
		global_plan_ = global_plan;

		//ROS_INFO_STREAM("Global plan size: " << global_plan_.size());
        ROS_INFO("Got Plan.");

        if(!costmap_ros_->getRobotPose(robot_pose_))
		{
			ROS_ERROR("path_executer: cannot get robot pose");
			return false;
		}

        nav_msgs::Path global_path_ = path_publisher("map", global_plan_);
        global_plan_pub_.publish(global_path_);
        return true;
	}

	bool PotentialFieldLocalPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
	{
        // Set all values of cmd_vel to zero
		cmd_vel.linear.x = 0.0;
		cmd_vel.linear.y = 0.0;
		cmd_vel.linear.z = 0.0;

		cmd_vel.angular.x = 0.0; 
		cmd_vel.angular.y = 0.0;
		cmd_vel.angular.z = 0.0;

		// Set the default return value as false
		bool ret = false;

		if(!costmap_ros_->getRobotPose(robot_pose_))
		{
			ROS_ERROR("path_executer: cannot get robot pose");
			return false;
		}

		// We need to compute the next heading point from the global plan
		//computeNextHeadingIndex();

		switch(state_)
		{
			case RotatingToStart:
			   ret = rotateToStart(cmd_vel);
			   break;
		    case Moving:
			   ret = move(cmd_vel);
			   break;
		    case RotatingToGoal:
			   ret = rotateToGoal(cmd_vel);
			   break;
		    default:
			   return true;
		}

		return ret;
	}

    nav_msgs::Path PotentialFieldLocalPlannerROS::path_publisher(std::string frame, std::vector<geometry_msgs::PoseStamped> plan)
    {
        nav_msgs::Path pub_path_;
		pub_path_.header.stamp = ros::Time::now();
        pub_path_.header.frame_id = frame;
        pub_path_.poses = plan;
		return pub_path_;
    }
}