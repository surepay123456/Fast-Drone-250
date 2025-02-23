#include "acados_simple_wrapper.hpp"
#include <vector>
// 参考轨迹生成函数
Eigen::MatrixXd define_ref_traj(int N, double dt) {
    // 弧的半径
    double R = 10.0;
    // 角速度（rad/s），2秒内转动 pi/5 rad
    double omega = M_PI / 10.0;
    // 弧的中心位置
    double xc = 0.0, yc = 0.0;

    // 创建一个矩阵来存储参考轨迹 [px, py, vx, vy, ax, ay, yaw]
    // Eigen::MatrixXd ref_traj(N + 1, 7);
    Eigen::MatrixXd ref_traj(NX_CURRENT, NSTEPS);
    // std::cout << "ref_traj size " << ref_traj.rows() << " " << ref_traj.cols() << std::endl;
    for (int i = 0; i < N; ++i) {
        double t = (i+1) * dt;
        double theta = omega * t;  // 当前角度

        // 计算位置（px, py）
        double px = xc + R * cos(theta);
        double py = yc + R * sin(theta);

        // 计算速度（vx, vy）
        double vx = -R * omega * sin(theta);
        double vy = R * omega * cos(theta);

        // 计算加速度（ax, ay）
        double ax = -R * omega * omega * cos(theta);
        double ay = -R * omega * omega * sin(theta);

        // 偏航角（与轨迹切线方向一致，yaw 固定为 np.pi / 3）
        double yaw = M_PI / 3.0;

        // 将计算结果存储到参考轨迹矩阵中
        ref_traj(0, i) = px;
        ref_traj(1, i) = py;
        ref_traj(2, i) = vx;
        ref_traj(3, i) = vy;
        ref_traj(4, i) = ax;
        ref_traj(5, i) = ay;
        ref_traj(6, i) = yaw;
    }

    return ref_traj;
}

Eigen::MatrixXd define_ref_traj_2(int N, double dt) {
    // 提供的数据
    std::vector<double> px = {-8.21124, -8.0771, -7.9353, -7.78626, -7.63077, -7.46953, -7.30306, -7.13208, -6.95763, -6.78076, -6.60249, -6.42348, -6.24393, -6.06366, -5.88199, -5.69828, -5.51203, -5.32293, -5.1309, -4.93609};
    std::vector<double> py = {-0.567876, -0.642012, -0.727801, -0.827337, -0.940752, -1.06703, -1.20372, -1.34781, -1.49565, -1.64318, -1.78587, -1.91924, -2.03896, -2.1408, -2.22068, -2.27529, -2.30254, -2.30117, -2.27108, -2.21312};
    std::vector<double> vx = {1.30237, 1.38003, 1.45547, 1.524, 1.58453, 1.63941, 1.68893, 1.72892, 1.75827, 1.77736, 1.78688, 1.79301, 1.79806, 1.80852, 1.82604, 1.84898, 1.87659, 1.9056, 1.9348, 1.96073};
    std::vector<double> vy = {-0.694552, -0.793966, -0.925868, -1.06486, -1.20218, -1.31915, -1.40982, -1.46584, -1.48431, -1.45874, -1.3875, -1.2726, -1.11473, -0.915367, -0.676287, -0.412625, -0.129903, 0.157314, 0.443487, 0.712099};
    std::vector<double> ax = {0.785525, 0.767581, 0.726408, 0.644269, 0.573785, 0.523858, 0.45334, 0.346507, 0.241651, 0.140282, 0.0695936, 0.0530167, 0.0685198, 0.1406, 0.204565, 0.254215, 0.28768, 0.292601, 0.280449, 0.238189};
    std::vector<double> ay = {-0.820322, -1.16796, -1.38983, -1.38992, -1.2975, -1.04193, -0.745065, -0.375391, 0.0264336, 0.484959, 0.933528, 1.36454, 1.78822, 2.19898, 2.53479, 2.73845, 2.86999, 2.87435, 2.79694, 2.57531};
    std::vector<double> yaw = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    // 创建一个 NSTEPS x 7 的矩阵
    Eigen::MatrixXd ref_traj(NX_CURRENT, NSTEPS); // 每列是一个时间步的数据

    for (int i = 0; i < NSTEPS; i++) {
        ref_traj(0, i) = px[i];
        ref_traj(1, i) = py[i];
        ref_traj(2, i) = vx[i];
        ref_traj(3, i) = vy[i];
        ref_traj(4, i) = ax[i];
        ref_traj(5, i) = ay[i];
        ref_traj(6, i) = yaw[i];
    }

    // 调用 set_reference_trajectory 函数
    // set_reference_trajectory(ref_traj);
    return ref_traj;
}

int main(){

    AcadosSimpleWrapper acados_wrapper(NSTEPS);
    // set the cost  Q and R
    Eigen::VectorXd Q(NX_CURRENT);
    Q << 1e3, 1e3, 1e3, 1e3, 1e3, 1e3, 1e0;
    Eigen::VectorXd R(NU);
    R << 1e1, 1e1, 1e1;
    acados_wrapper.set_cost_weights(Q, R);
    // set the end cost Q
    Eigen::VectorXd Q_end(NX_CURRENT);
    Q_end << 1e5, 1e5, 1e3, 1e3, 1e3, 1e3, 1e0;
    acados_wrapper.set_cost_weights_end(Q_end);
    // set the control bounds
    Eigen::VectorXd lbu(NU);
    lbu << -5.0, -5.0, -5.0;
    Eigen::VectorXd ubu(NU);
    ubu << 5.0, 5.0, 5.0;
    acados_wrapper.set_control_bounds(lbu, ubu);

    // set initial conditions
    Eigen::VectorXd x_init(NX);
    x_init << -8.24216, -0.551492, 1.28353, -0.675927, 0.785525, -0.820322, -0.3, -8.24216, -0.551492, -0.3;
    Eigen::VectorXd u0(NU);
    u0 << 0.0, 0.0, 0.0;
    acados_wrapper.set_initial_conditions(x_init, u0);
    // set the reference trajectory
    Eigen::MatrixXd ref_traj = define_ref_traj_2(NSTEPS, 0.1);
    acados_wrapper.set_reference_trajectory(ref_traj);
    // set the parameters
    Eigen::VectorXd p(NP);
    // 视场角 视场距离 参考点x 参考点y tube半径平方
    p  << 0.3, 2, ref_traj(0, 0), ref_traj(1, 0), 0.25;
    acados_wrapper.set_params(p);
    // solve the optimal control problem
    acados_wrapper.solve();
    return 0; 
}