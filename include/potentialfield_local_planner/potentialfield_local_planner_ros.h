#ifndef POTENTIALFIELD_LOCAL_PLANNER_ROS_H_
#define POTENTIALFIELD_LOCAL_PLANNER_ROS_H_

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

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
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>

#include <dynamic_reconfigure/server.h>
#include <potentialfield_local_planner/PotentialFieldLocalPlannerConfig.h>
#include <potentialfield_local_planner/potentialfield_local_planner.h>

#include <algorithm>

#define PI 3.1415926543

namespace potentialfield_local_planner
{
    class PotentialFieldLocalPlannerROS : public nav_core::BaseLocalPlanner
    {
        public:
           PotentialFieldLocalPlannerROS();
           ~PotentialFieldLocalPlannerROS();

           void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

           bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);
           bool setPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan);
           bool isGoalReached();

        private:
           void publishNextHeading(bool show = true);
           bool rotateToStart(geometry_msgs::Twist& cmd_vel);
           bool move(geometry_msgs::Twist& cmd_vel);
           bool rotateToGoal(geometry_msgs::Twist& cmd_vel);
           void computeNextHeadingIndex(std::vector<geometry_msgs::PoseStamped> plan);
           double calLinearVel();
           double calRotationVel(double rotation);
           double linearDistance(geometry_msgs::Point p1, geometry_msgs::Point p2);
           double calDeltaAngle(geometry_msgs::PoseStamped p1, geometry_msgs::PoseStamped p2);
           const inline double rewrapAngleRestricted(const double angle);
           const inline double RestrictedForwardAngle(const double angle);
           void reconfigureCB(PotentialFieldLocalPlannerConfig &config, uint32_t level);

           nav_msgs::Path path_publisher(std::string frame, std::vector<geometry_msgs::PoseStamped> plan);

           dynamic_reconfigure::Server<PotentialFieldLocalPlannerConfig> *dsrv_;

           std::vector<geometry_msgs::PoseStamped> global_plan_;
           std::vector<geometry_msgs::PoseStamped> local_plan_;

           ros::Publisher global_plan_pub_;
           ros::Publisher local_plan_pub_;
           ros::Publisher next_heading_pub_;

           int curr_heading_index_, next_heading_index_;
           int path_index_;

           typedef enum
           {
               RotatingToStart,
               Moving,
               RotatingToGoal,
               Finished
           } State;

           State state_;

           tf2_ros::Buffer* tf_;
           costmap_2d::Costmap2DROS* costmap_ros_;

           geometry_msgs::PoseStamped robot_pose_;

           // Parameters
           std::string map_frame_;

           struct constraint_vel
           {
               double max_vel;
               double min_vel;
               double limit_acc;
               double current_vel;
           };

           constraint_vel linear_vel_, rotation_vel_;
           
           double yaw_tolerance_, xy_tolerance_;
           double yaw_moving_tolerance_;
           double transform_timeout_;
           double heading_lookahead_;
           double headingSize_;

           bool use_BackForward;
           ros::Time last_time_;

           boost::shared_ptr<PotentialFieldLocalPlanner> dp_; ///< @brief The trajectory controller
    };
};

#endif