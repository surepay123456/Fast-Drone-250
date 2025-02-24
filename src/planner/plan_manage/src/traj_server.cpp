#include "bspline_opt/uniform_bspline.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "ros/subscriber.h"
#include "ros/time.h"
#include "ros/timer.h"
#include "traj_utils/Bspline.h"
#include "quadrotor_msgs/PositionCommand.h"
#include "std_msgs/Empty.h"
#include "utils/types.h"
#include "visualization_msgs/Marker.h"
#include <Eigen/src/Core/Matrix.h>
#include <iostream>
#include <ros/ros.h>
#include <vector>
// #include "acados_simple_wrapper.hpp"
#include "perception_mpc_wrapper.hpp"
#include "dbg.h"
ros::Publisher pos_cmd_pub;
ros::Publisher optimal_list_pub;
quadrotor_msgs::PositionCommand cmd;
double pos_gain[3] = {0, 0, 0};
double vel_gain[3] = {0, 0, 0};

using ego_planner::UniformBspline;

bool receive_traj_ = false;
bool receive_odom_ = false;
bool receive_imu_ = false;
Eigen::VectorXd cmd_x_;
vector<UniformBspline> traj_;
double traj_duration_;
ros::Time start_time_;
int traj_id_;

// yaw control
double last_yaw_, last_yaw_dot_;
double time_forward_;

// odom current state x, y, vx, vy, ax, az, yaw
Eigen::VectorXd cur_state(7);
// AcadosSimpleWrapper acados_wrapper(NSTEPS);
AcadosPerceptionWrapper perception_wrapper(NSTEPS);
void displayMarkerList(ros::Publisher &pub, const vector<Eigen::Vector3d> &list, double scale,
                                            Eigen::Vector4d color, int id, bool show_sphere /* = true */ )
{
  visualization_msgs::Marker sphere, line_strip;
  sphere.header.frame_id = line_strip.header.frame_id = "world";
  sphere.header.stamp = line_strip.header.stamp = ros::Time::now();
  sphere.type = visualization_msgs::Marker::SPHERE_LIST;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  sphere.action = line_strip.action = visualization_msgs::Marker::ADD;
  sphere.id = id;
  line_strip.id = id + 1000;

  sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
  sphere.color.r = line_strip.color.r = color(0);
  sphere.color.g = line_strip.color.g = color(1);
  sphere.color.b = line_strip.color.b = color(2);
  sphere.color.a = line_strip.color.a = color(3) > 1e-5 ? color(3) : 1.0;
  sphere.scale.x = scale;
  sphere.scale.y = scale;
  sphere.scale.z = scale;
  line_strip.scale.x = scale / 2;
  geometry_msgs::Point pt;
  for (int i = 0; i < int(list.size()); i++)
  {
    pt.x = list[i](0);
    pt.y = list[i](1);
    pt.z = list[i](2);
    //if (show_sphere) sphere.points.push_back(pt);
    line_strip.points.push_back(pt);
  }
  //if (show_sphere) pub.publish(sphere);
  pub.publish(line_strip);
}
void displayOptimalList(Eigen::MatrixXd optimal_pts, int id)
{
  vector<Eigen::Vector3d> list;
  for (int i = 0; i < optimal_pts.cols(); i++)
  {
    // Eigen::Vector3d pt = optimal_pts.col(i).transpose();
    Eigen::Vector3d pt = optimal_pts.block(0, i, 3, 1);
    pt(2) = 1.5; 
    list.push_back(pt);
  }
  Eigen::Vector4d color(0, 0, 0, 1);
  displayMarkerList(optimal_list_pub, list, 0.15, color, id, true);
}


void bsplineCallback(traj_utils::BsplineConstPtr msg)
{
  // parse pos traj

  Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());

  Eigen::VectorXd knots(msg->knots.size());
  for (size_t i = 0; i < msg->knots.size(); ++i)
  {
    knots(i) = msg->knots[i];
  }

  for (size_t i = 0; i < msg->pos_pts.size(); ++i)
  {
    pos_pts(0, i) = msg->pos_pts[i].x;
    pos_pts(1, i) = msg->pos_pts[i].y;
    pos_pts(2, i) = msg->pos_pts[i].z;
  }

  UniformBspline pos_traj(pos_pts, msg->order, 0.1);
  pos_traj.setKnot(knots);

  start_time_ = msg->start_time;
  traj_id_ = msg->traj_id;

  traj_.clear();
  // p, v, a
  traj_.push_back(pos_traj);
  traj_.push_back(traj_[0].getDerivative());
  traj_.push_back(traj_[1].getDerivative());

  traj_duration_ = traj_[0].getTimeSum();

  receive_traj_ = true;
}

void odomCallback(const nav_msgs::OdometryConstPtr &msg)
{
  cur_state(0) = msg->pose.pose.position.x;
  cur_state(1) = msg->pose.pose.position.y;
  cur_state(2) = msg->twist.twist.linear.x;
  cur_state(3) = msg->twist.twist.linear.y;
  cur_state(4) = 0; // 加速度需要通过imu 获得
  cur_state(5) = 0; 
  Eigen::Quaterniond q;
  q.x() = msg->pose.pose.orientation.x;
  q.y() = msg->pose.pose.orientation.y;
  q.z() = msg->pose.pose.orientation.z;
  q.w() = msg->pose.pose.orientation.w;
  double yaw = std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()), 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
  cur_state(6) = yaw;
  if (receive_odom_ == false) {
      receive_odom_ = true;
  }
}

void imuCallback(const sensor_msgs::ImuConstPtr &msg)
{
  cur_state(4) = msg->linear_acceleration.x;
  cur_state(5) = msg->linear_acceleration.y;
  if (receive_imu_ == false) {
      receive_imu_ = true;
  }
}

std::pair<double, double> calculate_yaw(double t_cur, Eigen::Vector3d &pos, ros::Time &time_now, ros::Time &time_last)
{
  constexpr double PI = 3.1415926;
  constexpr double YAW_DOT_MAX_PER_SEC = PI / 2;
  // constexpr double YAW_DOT_DOT_MAX_PER_SEC = PI;
  std::pair<double, double> yaw_yawdot(0, 0);
  double yaw = 0;
  double yawdot = 0;

  Eigen::Vector3d dir = t_cur + time_forward_ <= traj_duration_ ? traj_[0].evaluateDeBoorT(t_cur + time_forward_) - pos : traj_[0].evaluateDeBoorT(traj_duration_) - pos;
  double yaw_temp = dir.norm() > 0.1 ? atan2(dir(1), dir(0)) : last_yaw_;
  double max_yaw_change = YAW_DOT_MAX_PER_SEC * (time_now - time_last).toSec();
  if (yaw_temp - last_yaw_ > PI)
  {
    if (yaw_temp - last_yaw_ - 2 * PI < -max_yaw_change)
    {
      yaw = last_yaw_ - max_yaw_change;
      if (yaw < -PI)
        yaw += 2 * PI;

      yawdot = -YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ > PI)
        yawdot = -YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
    }
  }
  else if (yaw_temp - last_yaw_ < -PI)
  {
    if (yaw_temp - last_yaw_ + 2 * PI > max_yaw_change)
    {
      yaw = last_yaw_ + max_yaw_change;
      if (yaw > PI)
        yaw -= 2 * PI;

      yawdot = YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ < -PI)
        yawdot = YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
    }
  }
  else
  {
    if (yaw_temp - last_yaw_ < -max_yaw_change)
    {
      yaw = last_yaw_ - max_yaw_change;
      if (yaw < -PI)
        yaw += 2 * PI;

      yawdot = -YAW_DOT_MAX_PER_SEC;
    }
    else if (yaw_temp - last_yaw_ > max_yaw_change)
    {
      yaw = last_yaw_ + max_yaw_change;
      if (yaw > PI)
        yaw -= 2 * PI;

      yawdot = YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ > PI)
        yawdot = -YAW_DOT_MAX_PER_SEC;
      else if (yaw - last_yaw_ < -PI)
        yawdot = YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
    }
  }

  if (fabs(yaw - last_yaw_) <= max_yaw_change)
    yaw = 0.5 * last_yaw_ + 0.5 * yaw; // nieve LPF
  yawdot = 0.5 * last_yaw_dot_ + 0.5 * yawdot;
  last_yaw_ = yaw;
  last_yaw_dot_ = yawdot;

  yaw_yawdot.first = yaw;
  yaw_yawdot.second = yawdot;

  return yaw_yawdot;
}

void tubeMpcCallback(const ros::TimerEvent &e)
{
  if (!receive_traj_ || !receive_odom_ || !receive_imu_)
      return;
  Eigen::Vector3d u0 = Eigen::Vector3d::Zero();
  // std::cout << "come in tubeMpcCallback" << std::endl;
  // acados_wrapper.set_initial_conditions(cur_state, u0);
  /********************************************* */
  Eigen::MatrixXd ref_traj(NX, NSTEPS);
  ros::Time time_now = ros::Time::now();
  double dt = 0.1;
  double t_cur = (time_now - start_time_).toSec();
  // if end of traj_, hover
  if (t_cur >= traj_duration_) {
      return;
  }
  for (int i = 0; i < NSTEPS; ++i) {
      double t = t_cur + i * dt;
      Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), acc(Eigen::Vector3d::Zero());
      double yaw = 0.0; 
      // 将计算结果存储到参考轨迹矩阵中
      if (t < traj_duration_ && t >= 0.0) {
          pos = traj_[0].evaluateDeBoorT(t);
          vel = traj_[1].evaluateDeBoorT(t);
          acc = traj_[2].evaluateDeBoorT(t);
          yaw = atan2(vel(1), vel(0));
      } else if (t >= traj_duration_) {
          pos = traj_[0].evaluateDeBoorT(traj_duration_);
          vel = traj_[1].evaluateDeBoorT(traj_duration_);
          acc = traj_[2].evaluateDeBoorT(traj_duration_);
          yaw= atan2(vel(1), vel(0));
      }
      else {
          std::cout << "[Traj server]: invalid time." << std::endl;
      }
      ref_traj(0, i) = pos(0);
      ref_traj(1, i) = pos(1);
      ref_traj(2, i) = vel(0);
      ref_traj(3, i) = vel(1);
      ref_traj(4, i) = acc(0);
      ref_traj(5, i) = acc(1);
      ref_traj(6, i) = yaw;
  }
  perception_wrapper.set_reference_trajectory(ref_traj);

  perception_wrapper.set_initial_conditions(cur_state, Eigen::VectorXd::Zero(3));

  // solve the optimal control problem
  int status = perception_wrapper.solve();

  Eigen::MatrixXd x(NX, NSTEPS + 1);
  Eigen::MatrixXd u(NU, NSTEPS);
  perception_wrapper.get_results(x, u);
  // dbg(u); 
  if (status != ACADOS_SUCCESS) {
    // dbg(ref_traj);
    // dbg(x);
  } 
  // cmd_x_ = x.col(0);
  displayOptimalList(x, 20);
}

void cmdCallback(const ros::TimerEvent &e)
{
  /* no publishing before receive traj_ */
  if (!receive_traj_)
    return;
  // if (cmd_x_.size() == 0) {
  //   return;
  // }
  ros::Time time_now = ros::Time::now();
  double t_cur = (time_now - start_time_).toSec();

  Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), acc(Eigen::Vector3d::Zero()), pos_f;
  std::pair<double, double> yaw_yawdot(0, 0);

  static ros::Time time_last = ros::Time::now();
  if (t_cur < traj_duration_ && t_cur >= 0.0)
  {
    pos = traj_[0].evaluateDeBoorT(t_cur);
    vel = traj_[1].evaluateDeBoorT(t_cur);
    acc = traj_[2].evaluateDeBoorT(t_cur);

    /*** calculate yaw ***/
    yaw_yawdot = calculate_yaw(t_cur, pos, time_now, time_last);
    /*** calculate yaw ***/

    double tf = min(traj_duration_, t_cur + 2.0);
    pos_f = traj_[0].evaluateDeBoorT(tf);
  }
  else if (t_cur >= traj_duration_)
  {
    /* hover when finish traj_ */
    pos = traj_[0].evaluateDeBoorT(traj_duration_);
    vel.setZero();
    acc.setZero();

    yaw_yawdot.first = last_yaw_;
    yaw_yawdot.second = 0;

    pos_f = pos;
    return;
  }
  else
  {
    cout << "[Traj server]: invalid time." << endl;
  }
  time_last = time_now;

  cmd.header.stamp = time_now;
  cmd.header.frame_id = "world";
  cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd.trajectory_id = traj_id_;

  cmd.position.x = pos(0);
  cmd.position.y = pos(1);
  cmd.position.z = pos(2);
  // cmd.position.x = cmd_x_(0);
  // cmd.position.y = cmd_x_(1);

  cmd.velocity.x = vel(0);
  cmd.velocity.y = vel(1);
  cmd.velocity.z = vel(2);
  // cmd.velocity.x = cmd_x_(2);
  // cmd.velocity.y = cmd_x_(3);

  cmd.acceleration.x = acc(0);
  cmd.acceleration.y = acc(1);
  cmd.acceleration.z = acc(2);
  // cmd.acceleration.x = cmd_x_(4);
  // cmd.acceleration.y = cmd_x_(5);

  cmd.yaw = yaw_yawdot.first;
  cmd.yaw_dot = yaw_yawdot.second;
  // cmd.yaw = cmd_x_(6);

  last_yaw_ = cmd.yaw;

  pos_cmd_pub.publish(cmd);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "traj_server");
  // ros::NodeHandle node;
  ros::NodeHandle nh("~");

  ros::Subscriber bspline_sub = nh.subscribe("planning/bspline", 10, bsplineCallback);
  ros::Subscriber odom_sub = nh.subscribe("odom", 10, odomCallback);
  ros::Subscriber imu_sub = nh.subscribe("imu", 10, imuCallback);

  pos_cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 50);
  optimal_list_pub = nh.advertise<visualization_msgs::Marker>("traj_server_tubempc", 2);

  ros::Timer cmd_timer = nh.createTimer(ros::Duration(0.01), cmdCallback);
  ros::Timer tube_mpc_timer = nh.createTimer(ros::Duration(0.1), tubeMpcCallback);

  /* control parameter */
  cmd.kx[0] = pos_gain[0];
  cmd.kx[1] = pos_gain[1];
  cmd.kx[2] = pos_gain[2];

  cmd.kv[0] = vel_gain[0];
  cmd.kv[1] = vel_gain[1];
  cmd.kv[2] = vel_gain[2];

  nh.param("traj_server/time_forward", time_forward_, -1.0);
  last_yaw_ = 0.0;
  last_yaw_dot_ = 0.0;

  ros::Duration(1.0).sleep();

  ROS_WARN("[Traj server]: ready.");

  double pi = M_PI;  // Use the constant for pi
  // set control constraints
  Eigen::VectorXd lbu(3);
  Eigen::VectorXd ubu(3);
  lbu << -50, -50, -pi / 2;
  ubu << 50, 50, pi / 2;
  perception_wrapper.set_control_bounds(lbu, ubu);

  //  state cost weights
  Eigen::VectorXd Q(NX);
  Q = 1e2 * Eigen::VectorXd::Ones(NX);
  Q << 1e3, 1e3, 1e1, 1e1, 1e0, 1e0, 1e0;
  Eigen::VectorXd R(NU);
  R =  1e-1 * Eigen::VectorXd::Ones(NU);
  R << 1e-1, 1e-1, 1e-1;
  perception_wrapper.set_cost_weights(Q, R);
  Q[0] = 1e3; 
  Q[1] = 1e3;
  perception_wrapper.set_cost_weights_end(Q);

  //  slack cost weights
  double zl = 1e2;
  double Zl = 1e4;
  double zu = 1e1;
  double Zu = 1e1;
  perception_wrapper.set_cost_slack_weights(zl, Zl, zu, Zu);
  double zl_0 = 500;
  double Zl_0 = 1e3;
  double zu_0 = 1e1;
  double Zu_0 = 1e1;
  perception_wrapper.set_cost_slack_begin_weights(zl_0, Zl_0, zu_0, Zu_0);

  //  constraint parameters set
  Eigen::VectorXd p(NP);
  p << pi / 6;  // half of Field of view
  perception_wrapper.set_params(p);
  ROS_WARN("[Tube Mpc]: ready.");

  ros::spin();

  return 0;
}