#include <potentialfield_local_planner/potentialfield_local_planner_ros.h>
#include <pluginlib/class_list_macros.h>

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(potentialfield_local_planner::PotentialFieldLocalPlannerROS, nav_core::BaseLocalPlanner)

namespace potentialfield_local_planner
{
    PotentialFieldLocalPlannerROS::PotentialFieldLocalPlannerROS() : tf_(NULL),
                                                                     state_(Finished),
                                                                     curr_heading_index_(0),
								                            	     next_heading_index_(0),
									                                 use_BackForward(false)
    {

    }

    PotentialFieldLocalPlannerROS::~PotentialFieldLocalPlannerROS()
    {
		delete dsrv_;
    }

    void PotentialFieldLocalPlannerROS::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
    {
        ros::NodeHandle private_nh("~/" + name);

        global_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
		local_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
		next_heading_pub_ = private_nh.advertise<visualization_msgs::Marker>("maker", 10);

        tf_ = tf;
		costmap_ros_ = costmap_ros;

		linear_vel_.current_vel = 0;
		rotation_vel_.current_vel = 0;
		
		//create the actual planner that we'll use.. it'll configure itself from the parameter server
		dp_ = boost::shared_ptr<PotentialFieldLocalPlanner>(new PotentialFieldLocalPlanner(name, costmap_ros));

		dsrv_ = new dynamic_reconfigure::Server<PotentialFieldLocalPlannerConfig>(private_nh);
        dynamic_reconfigure::Server<PotentialFieldLocalPlannerConfig>::CallbackType cb = boost::bind(&PotentialFieldLocalPlannerROS::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);
    }

	void PotentialFieldLocalPlannerROS::reconfigureCB(PotentialFieldLocalPlannerConfig &config, uint32_t level)
    {
        ROS_INFO("PotentialFieldLocalPlanner reconfigureCB");

        map_frame_ = config.map_frame;
        linear_vel_.max_vel = config.max_linear_vel;
        linear_vel_.min_vel = config.min_linear_vel;
		linear_vel_.limit_acc = config.acc_linear_vel;
        rotation_vel_.max_vel = config.max_vel_theta;
        rotation_vel_.min_vel = config.min_vel_theta;
		rotation_vel_.limit_acc = config.acc_vel_theta;
        xy_tolerance_ = config.xy_goal_tolerance;
        yaw_tolerance_ = config.yaw_goal_tolerance;
		yaw_moving_tolerance_ = config.yaw_moving_tolerance;
        transform_timeout_ = config.timeout;

		dp_->reconfigureCB(config);
    }

    bool PotentialFieldLocalPlannerROS::isGoalReached()
    {
        return (state_ == Finished);
    }

	bool PotentialFieldLocalPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan)
	{
		last_time_ = ros::Time::now();
        global_plan_.clear();
		local_plan_.clear();

		// Make our copy of the global plan
		global_plan_ = global_plan;

        ROS_INFO("Got Plan.");

		curr_heading_index_ = 0;
		next_heading_index_ = 0;
		path_index_ = global_plan_.size() - 1;

        if(!costmap_ros_->getRobotPose(robot_pose_))
		{
			ROS_ERROR("path_executer: cannot get robot pose");
			return false;
		}

		// We need to compute the next heading point from the global plan.
		// Calculate potential field local planning.
		computeNextHeadingIndex(global_plan_);
		ROS_INFO("Next Index = %d", next_heading_index_);
		//local_plan_ = dp_->PotentialFieldLocal_Planner(robot_pose_, global_plan_[next_heading_index_]);

		// Calculate the rotation between the current odom and the vector created above
		double rotation = calDeltaAngle(robot_pose_, global_plan_[path_index_]);
		rotation = RestrictedForwardAngle(rotation);
		//ROS_INFO("delta_th = %f", rotation);

		if(linearDistance(robot_pose_.pose.position, global_plan_[path_index_].pose.position) <= xy_tolerance_)
		{
			state_ = RotatingToGoal;
		}
		else if(fabs(rotation) <= yaw_moving_tolerance_ &&
		        linearDistance(robot_pose_.pose.position, global_plan_[path_index_].pose.position) > xy_tolerance_)
		{
			state_ = Moving;
		}
		else if(fabs(rotation) > yaw_moving_tolerance_ &&
		        linearDistance(robot_pose_.pose.position, global_plan_[path_index_].pose.position) > xy_tolerance_)
		{
			// Set the state to RotatingToStart
			state_ = RotatingToStart;
		}

        nav_msgs::Path global_path_ = path_publisher("map", global_plan_);
		nav_msgs::Path local_path_ = path_publisher("map", local_plan_);
        global_plan_pub_.publish(global_path_);
		local_plan_pub_.publish(local_path_);
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

	bool PotentialFieldLocalPlannerROS::rotateToStart(geometry_msgs::Twist& cmd_vel)
	{
		geometry_msgs::PoseStamped rotate_goal;

		ros::Time now = ros::Time::now();
		global_plan_[next_heading_index_].header.stamp = now;

		try
		{
			geometry_msgs::TransformStamped trans = tf_->lookupTransform(robot_pose_.header.frame_id, global_plan_[next_heading_index_].header.frame_id, now, ros::Duration(transform_timeout_));
      		tf2::doTransform(global_plan_[next_heading_index_], rotate_goal, trans);
		}
		catch(tf2::LookupException& ex)
		{
			ROS_ERROR("Lookup Error: %s\n", ex.what());
			return false;
		}
		catch(tf2::ConnectivityException& ex)
		{
			ROS_ERROR("Connectivity Error: %s\n", ex.what());
			return false;
		}
		catch(tf2::ExtrapolationException& ex)
		{
			ROS_ERROR("Extrapolation Error: %s\n", ex.what());
			return false;
		}

	    double rotation = calDeltaAngle(robot_pose_, rotate_goal);
		rotation = RestrictedForwardAngle(rotation);
		//ROS_INFO("delta_th = %f", rotation);

		if(fabs(rotation) < yaw_moving_tolerance_)
		{
			state_ = Moving;
			return true;
		}

		cmd_vel.angular.z = calRotationVel(rotation);

		return true;
	}

	bool PotentialFieldLocalPlannerROS::move(geometry_msgs::Twist& cmd_vel)
	{
		publishNextHeading();

		geometry_msgs::PoseStamped move_goal;
		ros::Time now = ros::Time::now();
		global_plan_[next_heading_index_].header.stamp = now;

		try
		{
			geometry_msgs::TransformStamped trans = tf_->lookupTransform(robot_pose_.header.frame_id, global_plan_[next_heading_index_].header.frame_id, now, ros::Duration(transform_timeout_));
      		tf2::doTransform(global_plan_[next_heading_index_], move_goal, trans);
		}
		catch(tf2::LookupException& ex)
		{
			ROS_ERROR("Lookup Error: %s\n", ex.what());
			return false;
		}
		catch(tf2::ConnectivityException& ex)
		{
			ROS_ERROR("Connectivity Error: %s\n", ex.what());
			return false;
		}
		catch(tf2::ExtrapolationException& ex)
		{
			ROS_ERROR("Extrapolation Error: %s\n", ex.what());
			return false;
		}
	    
		double rotation = calDeltaAngle(robot_pose_, move_goal);
		rotation = RestrictedForwardAngle(rotation);
		//ROS_INFO("delta_th = %f", rotation);

		cmd_vel.angular.z = calRotationVel(rotation);

		if(fabs(rotation) <= yaw_moving_tolerance_)
		{
			// The robot has rotated to its next heading pose
			cmd_vel.angular.z = 0.0;
		}

		cmd_vel.linear.x = calLinearVel();
		double distance_to_next_heading = linearDistance(robot_pose_.pose.position, move_goal.pose.position);

		// We are approaching the goal position, slow down
		if(next_heading_index_ == (int)global_plan_.size() - 1)
		{
			// Reached the goal, now we can stop and rotate the robot to the goal position
			if(fabs(distance_to_next_heading) <= xy_tolerance_)
			{
				cmd_vel.linear.x = 0.0;
				cmd_vel.angular.z = 0.0;
				state_ = RotatingToGoal;
				return true;
			}
		}
		return true;
	}

	bool PotentialFieldLocalPlannerROS::rotateToGoal(geometry_msgs::Twist& cmd_vel)
	{
		geometry_msgs::PoseStamped rotate_goal;

		ros::Time now = ros::Time::now();
		global_plan_[next_heading_index_].header.stamp = now;

		try
		{
			geometry_msgs::TransformStamped trans = tf_->lookupTransform(robot_pose_.header.frame_id, global_plan_[next_heading_index_].header.frame_id, now, ros::Duration(transform_timeout_));
      		tf2::doTransform(global_plan_[next_heading_index_], rotate_goal, trans);
		}
		catch(tf2::LookupException& ex)
		{
			ROS_ERROR("Lookup Error: %s\n", ex.what());
			return false;
		}
		catch(tf2::ConnectivityException& ex)
		{
			ROS_ERROR("Connectivity Error: %s\n", ex.what());
			return false;
		}
		catch(tf2::ExtrapolationException& ex)
		{
			ROS_ERROR("Extrapolation Error: %s\n", ex.what());
			return false;
		}

		double rotation = rewrapAngleRestricted(tf2::getYaw(rotate_goal.pose.orientation) - tf2::getYaw(robot_pose_.pose.orientation));

		if(fabs(rotation) <= yaw_tolerance_)
		{
			state_ = Finished;
			ROS_INFO("Goal reached");
			return true;
		}

		cmd_vel.angular.z = calRotationVel(rotation);

		return true;
	}

	void PotentialFieldLocalPlannerROS::computeNextHeadingIndex(std::vector<geometry_msgs::PoseStamped> plan)
	{
		geometry_msgs::PoseStamped next_heading_pose;

		for(unsigned int i = curr_heading_index_; i < plan.size() - 1; ++i)
		{
			ros::Time now = ros::Time::now();
			plan[i].header.stamp = now;

			try
			{
				geometry_msgs::TransformStamped trans = tf_->lookupTransform(robot_pose_.header.frame_id, plan[i].header.frame_id, now, ros::Duration(transform_timeout_));
      			tf2::doTransform(plan[i], next_heading_pose, trans);
			}
			catch(tf2::LookupException& ex)
			{
				ROS_ERROR("Lookup Error: %s\n", ex.what());
				return;
			}
			catch(tf2::ConnectivityException& ex)
			{
				ROS_ERROR("Connectivity Error: %s\n", ex.what());
				return;
			}
			catch(tf2::ExtrapolationException& ex)
			{
				ROS_ERROR("Extrapolation Error: %s\n", ex.what());
				return;
			}

			next_heading_index_ = i;

			double dist = linearDistance(robot_pose_.pose.position, next_heading_pose.pose.position);
			double heading_lookahead_ = std::min(dp_->width_, dp_->height_) * dp_->resolution_ / 2;
			if(dist > heading_lookahead_)
			{
				break;
			}
		}
	}
	
	double PotentialFieldLocalPlannerROS::calLinearVel()
	{
		double vel = 0.0;

	    double straight_dist = linearDistance(robot_pose_.pose.position, global_plan_[next_heading_index_].pose.position);

		vel = use_BackForward == false ? 
		                         sqrt(2 * linear_vel_.limit_acc * straight_dist) :
								 -sqrt(2 * linear_vel_.limit_acc * straight_dist);

		double dt = (ros::Time::now() - last_time_).toSec();

		if(vel > linear_vel_.current_vel)
		{
			linear_vel_.current_vel += fmin(vel - linear_vel_.current_vel, linear_vel_.limit_acc * dt);
		}
		else
		{
			linear_vel_.current_vel += fmax(vel - linear_vel_.current_vel, -linear_vel_.limit_acc * dt);
		}

		if(linear_vel_.current_vel > linear_vel_.max_vel)
		{
			linear_vel_.current_vel = linear_vel_.max_vel;
		}
		else if (linear_vel_.current_vel < linear_vel_.min_vel)
		{
			linear_vel_.current_vel = linear_vel_.min_vel;
		}

		return linear_vel_.current_vel;
	}

	double PotentialFieldLocalPlannerROS::calRotationVel(double rotation)
	{
		double vel = 0.0;
		double dt = (ros::Time::now() - last_time_).toSec();
		vel = rotation >= 0 ? sqrt(2 * rotation_vel_.limit_acc * fabs(rotation)) : -sqrt(2 * rotation_vel_.limit_acc * fabs(rotation));

		if(vel > rotation_vel_.current_vel)
		{
			rotation_vel_.current_vel += fmin(vel - rotation_vel_.current_vel, rotation_vel_.limit_acc * dt);
		}
		else
		{
			rotation_vel_.current_vel += fmax(vel - rotation_vel_.current_vel, -rotation_vel_.limit_acc * dt);
		}

		if(rotation_vel_.current_vel > rotation_vel_.max_vel)
		{
			rotation_vel_.current_vel = rotation_vel_.max_vel;
		}
		else if(rotation_vel_.current_vel < rotation_vel_.min_vel)
		{
			rotation_vel_.current_vel = rotation_vel_.min_vel;
		}

		return rotation_vel_.current_vel;
	}

	double PotentialFieldLocalPlannerROS::linearDistance(geometry_msgs::Point p1, geometry_msgs::Point p2)
	{
		return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2));
	}

	double PotentialFieldLocalPlannerROS::calDeltaAngle(geometry_msgs::PoseStamped p1, geometry_msgs::PoseStamped p2)
	{
		double rotation_map;
		double delta_x = p2.pose.position.x - p1.pose.position.x;
		double delta_y = p2.pose.position.y - p1.pose.position.y;

		rotation_map = ::atan2(delta_y, delta_x) - tf2::getYaw(p1.pose.orientation);

		return rewrapAngleRestricted(rotation_map);
	}

	const inline double PotentialFieldLocalPlannerROS::rewrapAngleRestricted(const double angle)
	{
		double rewrap_angle;

		if(angle > PI)
		{
			rewrap_angle = angle - 2 * PI;
		}
		else if (angle < -PI)
		{
			rewrap_angle = angle + 2 * PI;
		}
		else
		{
			rewrap_angle = angle;
		}

		return rewrap_angle;
	}

	const inline double PotentialFieldLocalPlannerROS::RestrictedForwardAngle(const double angle)
	{
		if(angle > PI / 2)
		{
			use_BackForward = true;
			return angle - PI;
		}
		else if(angle < -PI / 2)
		{
			use_BackForward = true;
			return angle + PI;
		}
		else
		{
			use_BackForward = false;
			return angle;
		}
	}

	void PotentialFieldLocalPlannerROS::publishNextHeading(bool show)
	{
		const geometry_msgs::PoseStamped& next_pose = global_plan_[next_heading_index_];

		visualization_msgs::Marker marker;
		marker.id = 0;
		marker.header.stamp = ros::Time::now();
		marker.header.frame_id= next_pose.header.frame_id;
		marker.ns = "waypoints";
		marker.type = visualization_msgs::Marker::CYLINDER;

		if(show)
		{
			marker.action = visualization_msgs::Marker::MODIFY;
			marker.pose = next_pose.pose;
			marker.scale.x = 0.1;
			marker.scale.y = 0.1;
			marker.scale.z = 0.2;
			marker.color.a = 0.5;
			marker.color.r = 1.0;
			marker.color.g = 1.0;
			marker.color.b = 0.0;
		}
		else
		{
			marker.action = visualization_msgs::Marker::DELETE;
		}
		next_heading_pub_.publish(marker);
	}
}