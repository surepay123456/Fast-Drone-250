#ifndef MPC_TRAJ_SERVER_H_
#define MPC_TRAJ_SERVER_H_
#include <math.h>
#include <quadrotor_common/trajectory.h>
#include <quadrotor_common/trajectory_point.h>
#include <quadrotor_msgs/Trajectory.h>
#include <ros/ros.h>

#include <cstdint>
#include <utility>
#include <vector>

#include "bspline_opt/uniform_bspline.h"
#include "nav_msgs/Odometry.h"
#include "quadrotor_msgs/PositionCommand.h"
#include "std_msgs/Empty.h"
#include "traj_utils/Bspline.h"
#include "visualization_msgs/Marker.h"
#include <quadrotor_msgs/Trajectory.h>
using ego_planner::UniformBspline;
class MpcTrajServer {
   public:
    ros::Subscriber bspine_sub;
    ros::Publisher pos_cmd_pub;
    ros::Publisher traj_cmd_pub;
    ros::Timer cmd_timer;

    quadrotor_msgs::PositionCommand cmd;
    double pos_gain[3] = {0, 0, 0};
    double vel_gain[3] = {0, 0, 0};
    bool receive_traj_ = false;
    vector<UniformBspline> traj_;
    double traj_duration_;
    ros::Time start_time_;
    int traj_id_;

    // yaw control
    double last_yaw_, last_yaw_dot_;
    double time_forward_;

    MpcTrajServer();
    ~MpcTrajServer();

    void cmdCallback(const ros::TimerEvent &e);
    void trajCmdCallback(const ros::TimerEvent& e);
    void bsplineCallback(traj_utils::BsplineConstPtr msg);

   private:
    std::pair<double, double> calculate_yaw(double t_cur, Eigen::Vector3d &pos, ros::Time &time_now,
                                            ros::Time &time_last);
    void convertBspline2Traj(vector<UniformBspline> &splines, const double &dt,
                             quadrotor_common::Trajectory &common_traj);
    void distancePerceptionYaw(double &yaw1, double &yaw2, double &d);
    void refinePerceptionTraj(quadrotor_common::Trajectory &, quadrotor_common::Trajectory &);
    Eigen::Quaterniond calculateQfromYawAcc(double &yaw, Eigen::Vector3d &acc);
    Eigen::Quaterniond calculateQfromYawAcc(Eigen::Vector2d& yaw_vec, Eigen::Vector3d &acc);
    double smooth_function(double d, double d0, double k) {
        return 1.0 / (1.0 + std::exp(-k * (d - d0)));}
    double getAnotherYaw(const quadrotor_common::TrajectoryPoint& p1, const quadrotor_common::TrajectoryPoint& p2);
};
#endif