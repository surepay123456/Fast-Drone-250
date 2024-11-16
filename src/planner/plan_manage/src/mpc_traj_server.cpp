#include <plan_manage/mpc_traj_server.h>
#include <quadrotor_common/quaternion_functions.h>

MpcTrajServer::MpcTrajServer() {
    ros::NodeHandle nh("~");
    bspine_sub = nh.subscribe("planning/bspline", 10, &MpcTrajServer::bsplineCallback, this);
    traj_cmd_pub = nh.advertise<quadrotor_msgs::Trajectory>("/traj_cmd", 50);
    cmd_timer = nh.createTimer(ros::Duration(0.01), &MpcTrajServer::trajCmdCallback, this);
    ros::Duration(1.0).sleep();
    ROS_WARN("[Traj server]: ready.");
    ros::spin();
}

MpcTrajServer::~MpcTrajServer() {}

void MpcTrajServer::convertBspline2Traj(vector<UniformBspline> &splines, const double &dt,
                                        quadrotor_common::Trajectory &common_traj) {
    constexpr double small_dt_ = 0.1;
    ros::Time now_time = ros::Time::now();
    ros::Time start_time = start_time_;
    double t = (now_time - start_time).toSec();
    common_traj.timestamp = now_time;
    quadrotor_common::TrajectoryPoint point;
    // parse 20 knots
    for (int i = 0; i < 20; i++) {
        if (t >= splines[0].getTimeSum()) {
            t = splines[0].getTimeSum();
        }
        point.position = splines[0].evaluateDeBoorT(t);
        point.velocity = splines[1].evaluateDeBoorT(t);
        point.acceleration = splines[2].evaluateDeBoorT(t);
        point.time_from_start = ros::Duration(t);
        // caculate yaw use forword elur, and check the interval
        Eigen::Vector3d next_position_;
        double dx;
        double dy;
        if (t + small_dt_ >= splines[0].getTimeSum()) {
            next_position_ = splines[0].evaluateDeBoorT(t - small_dt_);
            dx = point.position[0] - next_position_[0];
            dy = point.position[1] - next_position_[1];
        } else {
            next_position_ = splines[0].evaluateDeBoorT(t + small_dt_);
            dx = next_position_[0] - point.position[0];
            dy = next_position_[1] - point.position[1];
        }
        // use yaw and acc to get q
        double yaw = atan2(dy, dx);
        double sin = std::sin(yaw);
        double cos = std::cos(yaw);
        double roll = (point.acceleration(0) * sin - point.acceleration(1) * cos) / 9.81;
        double pitch = (point.acceleration(0) * cos + point.acceleration(1) * sin) / 9.81;
        Eigen::Quaterniond q = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                               Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                               Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
        point.heading = yaw;
        point.orientation = q;
        common_traj.points.push_back(point);
        t += dt;
    }
}

void MpcTrajServer::trajCmdCallback(const ros::TimerEvent &e) {
    if (!receive_traj_) {
        return;
    }
    ros::Time time_now = ros::Time::now();
    ros::Duration dt{0.1};
    double t_cur = (time_now - start_time_).toSec();
    // get current 2s 0.1s traj and convert to quadrotor_common
    // create new common traj to pub
    quadrotor_common::Trajectory traj_cmd;
    quadrotor_msgs::Trajectory traj_msg;
    traj_cmd.timestamp = time_now;
    if (t_cur < traj_duration_ && t_cur >= 0.0) {
        // get current 2s 0.1s traj and convert to quadrotor_common
        for (int i = 0; i < 20; i++) {
            quadrotor_common::TrajectoryPoint p;
            p.position = traj_[0].evaluateDeBoorT(t_cur);
            p.velocity = traj_[1].evaluateDeBoorT(t_cur);
            p.acceleration = traj_[2].evaluateDeBoorT(t_cur);
            Eigen::Vector2d yaw_vec = traj_[3].evaluateDeBoorT(t_cur);
            p.orientation = calculateQfromYawAcc(yaw_vec, p.acceleration);
            p.time_from_start = dt * i;
        }
    } else if (t_cur >= traj_duration_) {
        // hover when finish traj_
        for (int i = 0; i < 20; i++) {
            quadrotor_common::TrajectoryPoint p;
            p.position = traj_[0].evaluateDeBoorT(traj_duration_);
            p.velocity = traj_[1].evaluateDeBoorT(traj_duration_);
            p.acceleration = traj_[2].evaluateDeBoorT(traj_duration_);
            Eigen::Vector2d yaw_vec = traj_[3].evaluateDeBoorT(traj_duration_);
            p.orientation = calculateQfromYawAcc(yaw_vec, p.acceleration);
            p.time_from_start = dt * i;
        }
    } else {
        // invalid time
        cout << "[Traj server]: invalid time." << endl;
    }
    traj_msg = traj_cmd.toRosMessage();
    traj_cmd_pub.publish(traj_msg);
}

void MpcTrajServer::cmdCallback(const ros::TimerEvent &e) {
    /* no publishing before receive traj_ */
    if (!receive_traj_) return;

    ros::Time time_now = ros::Time::now();
    double t_cur = (time_now - start_time_).toSec();

    Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()),
        acc(Eigen::Vector3d::Zero()), pos_f;
    std::pair<double, double> yaw_yawdot(0, 0);

    static ros::Time time_last = ros::Time::now();
    if (t_cur < traj_duration_ && t_cur >= 0.0) {
        pos = traj_[0].evaluateDeBoorT(t_cur);
        vel = traj_[1].evaluateDeBoorT(t_cur);
        acc = traj_[2].evaluateDeBoorT(t_cur);

        /*** calculate yaw ***/
        yaw_yawdot = calculate_yaw(t_cur, pos, time_now, time_last);
        /*** calculate yaw ***/

        double tf = min(traj_duration_, t_cur + 2.0);
        pos_f = traj_[0].evaluateDeBoorT(tf);
    } else if (t_cur >= traj_duration_) {
        /* hover when finish traj_ */
        pos = traj_[0].evaluateDeBoorT(traj_duration_);
        vel.setZero();
        acc.setZero();

        yaw_yawdot.first = last_yaw_;
        yaw_yawdot.second = 0;

        pos_f = pos;
        return;
    } else {
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

    cmd.velocity.x = vel(0);
    cmd.velocity.y = vel(1);
    cmd.velocity.z = vel(2);

    cmd.acceleration.x = acc(0);
    cmd.acceleration.y = acc(1);
    cmd.acceleration.z = acc(2);

    cmd.yaw = yaw_yawdot.first;
    cmd.yaw_dot = yaw_yawdot.second;

    last_yaw_ = cmd.yaw;

    pos_cmd_pub.publish(cmd);
}

void MpcTrajServer::bsplineCallback(traj_utils::BsplineConstPtr msg) {
    // parse pos traj
    Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());
    Eigen::MatrixXd yaw_pts(2, msg->yaw_traj.size());
    Eigen::VectorXd knots(msg->knots.size());
    for (size_t i = 0; i < msg->knots.size(); ++i) {
        knots(i) = msg->knots[i];
    }
    for (size_t i = 0; i < msg->pos_pts.size(); ++i) {
        pos_pts(0, i) = msg->pos_pts[i].x;
        pos_pts(1, i) = msg->pos_pts[i].y;
        pos_pts(2, i) = msg->pos_pts[i].z;
    }
    for (size_t i = 0; i < msg->yaw_traj.size(); ++i) {
        yaw_pts(0, i) = msg->yaw_traj[i].x;
        yaw_pts(1, i) = msg->yaw_traj[i].y;
    }
    UniformBspline pos_traj(pos_pts, msg->order, 0.1);
    UniformBspline yaws_t(yaw_pts, msg->order, 0.1);
    pos_traj.setKnot(knots);
    yaws_t.setKnot(knots);
    start_time_ = msg->start_time;
    traj_id_ = msg->traj_id;

    traj_.clear();
    traj_.push_back(pos_traj);
    traj_.push_back(traj_[0].getDerivative());
    traj_.push_back(traj_[1].getDerivative());
    traj_.push_back(yaws_t);
    traj_duration_ = traj_[0].getTimeSum();
    receive_traj_ = true;
}

void MpcTrajServer::distancePerceptionYaw(double &yaw1, double &yaw2, double &d) {
    double sin1 = std::sin(yaw1);
    double cos1 = std::cos(yaw1);

    double sin2 = std::sin(yaw2);
    double cos2 = std::cos(yaw2);

    double delta = smooth_function(d, 0.5, 2);
    double new_x = sin1 + delta * sin2;
    double new_y = cos1 + delta * cos2;
    yaw1 = atan2(new_y, new_x);
}

Eigen::Quaterniond MpcTrajServer::calculateQfromYawAcc(double &yaw, Eigen::Vector3d &acc) {
    double sin = std::sin(yaw);
    double cos = std::cos(yaw);
    double roll = (acc(0) * sin - acc(1) * cos) / 9.81;
    double pitch = (acc(0) * cos + acc(1) * sin) / 9.81;
    Eigen::Quaterniond q = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                           Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    return q;
}

Eigen::Quaterniond MpcTrajServer::calculateQfromYawAcc(Eigen::Vector2d &yaw_vec,
                                                       Eigen::Vector3d &acc) {
    double x = yaw_vec[0];
    double y = yaw_vec[1];
    double yaw = atan2(y, x);
    double sin_yaw = std::sin(yaw);
    double cos_yaw = std::cos(yaw);
    double roll = (acc(0) * sin_yaw - acc(1) * cos_yaw) / 9.81;
    double pitch = (acc(0) * cos_yaw + acc(1) * sin_yaw) / 9.81;
    Eigen::Quaterniond q = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                           Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    return q;
}
double MpcTrajServer::getAnotherYaw(const quadrotor_common::TrajectoryPoint &p1,
                                    const quadrotor_common::TrajectoryPoint &p2) {
    double yaw = atan2(p1.position(1) - p2.position(1), p1.position(0) - p2.position(0));
    return yaw;
}

void MpcTrajServer::refinePerceptionTraj(quadrotor_common::Trajectory &traj_1,
                                         quadrotor_common::Trajectory &traj_2) {
    // along the traj, change the yaw and q
    ros::Time now = ros::Time::now();
    ros::Duration dt{0.1};
    ros::Duration start_2 = now - traj_2.timestamp;
    int count = 0;
    for (auto it = traj_1.points.begin(); it != traj_1.points.end(); it++) {
        quadrotor_common::TrajectoryPoint p2 = traj_2.getStateAtTime(start_2 + dt * count);
        double yaw2 = getAnotherYaw(p2, *it);
        double d = sqrt(pow(p2.position[0] - it->position[0], 2) +
                        pow(p2.position[1] - it->position[1], 2));
        distancePerceptionYaw(it->heading, yaw2, d);
        it->orientation = calculateQfromYawAcc(it->heading, it->acceleration);
        count++;
        if (count == 20) {
            break;
        }
    }
}

// int main(int argc, char **argv)
// {
//   ros::init(argc, argv, "traj_server");
//   // ros::NodeHandle node;
//   ros::NodeHandle nh("~");

//   ros::Subscriber bspline_sub = nh.subscribe("planning/bspline", 10, bsplineCallback);

//   pos_cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 50);

//   ros::Timer cmd_timer = nh.createTimer(ros::Duration(0.01), cmdCallback);

//   /* control parameter */
//   cmd.kx[0] = pos_gain[0];
//   cmd.kx[1] = pos_gain[1];
//   cmd.kx[2] = pos_gain[2];

//   cmd.kv[0] = vel_gain[0];
//   cmd.kv[1] = vel_gain[1];
//   cmd.kv[2] = vel_gain[2];

//   nh.param("traj_server/time_forward", time_forward_, -1.0);
//   last_yaw_ = 0.0;
//   last_yaw_dot_ = 0.0;

//   ros::Duration(1.0).sleep();

//   ROS_WARN("[Traj server]: ready.");

//   ros::spin();

//   return 0;
// }

std::pair<double, double> MpcTrajServer::calculate_yaw(double t_cur, Eigen::Vector3d &pos,
                                                       ros::Time &time_now, ros::Time &time_last) {
    constexpr double PI = 3.1415926;
    constexpr double YAW_DOT_MAX_PER_SEC = PI;
    // constexpr double YAW_DOT_DOT_MAX_PER_SEC = PI;
    std::pair<double, double> yaw_yawdot(0, 0);
    double yaw = 0;
    double yawdot = 0;

    Eigen::Vector3d dir = t_cur + time_forward_ <= traj_duration_
                              ? traj_[0].evaluateDeBoorT(t_cur + time_forward_) - pos
                              : traj_[0].evaluateDeBoorT(traj_duration_) - pos;
    double yaw_temp = dir.norm() > 0.1 ? atan2(dir(1), dir(0)) : last_yaw_;
    double max_yaw_change = YAW_DOT_MAX_PER_SEC * (time_now - time_last).toSec();
    if (yaw_temp - last_yaw_ > PI) {
        if (yaw_temp - last_yaw_ - 2 * PI < -max_yaw_change) {
            yaw = last_yaw_ - max_yaw_change;
            if (yaw < -PI) yaw += 2 * PI;

            yawdot = -YAW_DOT_MAX_PER_SEC;
        } else {
            yaw = yaw_temp;
            if (yaw - last_yaw_ > PI)
                yawdot = -YAW_DOT_MAX_PER_SEC;
            else
                yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
        }
    } else if (yaw_temp - last_yaw_ < -PI) {
        if (yaw_temp - last_yaw_ + 2 * PI > max_yaw_change) {
            yaw = last_yaw_ + max_yaw_change;
            if (yaw > PI) yaw -= 2 * PI;

            yawdot = YAW_DOT_MAX_PER_SEC;
        } else {
            yaw = yaw_temp;
            if (yaw - last_yaw_ < -PI)
                yawdot = YAW_DOT_MAX_PER_SEC;
            else
                yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
        }
    } else {
        if (yaw_temp - last_yaw_ < -max_yaw_change) {
            yaw = last_yaw_ - max_yaw_change;
            if (yaw < -PI) yaw += 2 * PI;

            yawdot = -YAW_DOT_MAX_PER_SEC;
        } else if (yaw_temp - last_yaw_ > max_yaw_change) {
            yaw = last_yaw_ + max_yaw_change;
            if (yaw > PI) yaw -= 2 * PI;

            yawdot = YAW_DOT_MAX_PER_SEC;
        } else {
            yaw = yaw_temp;
            if (yaw - last_yaw_ > PI)
                yawdot = -YAW_DOT_MAX_PER_SEC;
            else if (yaw - last_yaw_ < -PI)
                yawdot = YAW_DOT_MAX_PER_SEC;
            else
                yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
        }
    }

    if (fabs(yaw - last_yaw_) <= max_yaw_change) yaw = 0.5 * last_yaw_ + 0.5 * yaw;  // nieve LPF
    yawdot = 0.5 * last_yaw_dot_ + 0.5 * yawdot;
    last_yaw_ = yaw;
    last_yaw_dot_ = yawdot;

    yaw_yawdot.first = yaw;
    yaw_yawdot.second = yawdot;

    return yaw_yawdot;
}