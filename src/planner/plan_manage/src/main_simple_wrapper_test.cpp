#include "acados_simple_wrapper.hpp"

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

int main(){

    AcadosSimpleWrapper acados_wrapper(NSTEPS);
    // set the cost  Q and R

    // set the constraints max and min

    // set initial conditions
    Eigen::VectorXd x_init(NX);
    x_init[0] = 10.1;
    x_init[1] = 0.0;
    x_init[2] = 0.0;
    x_init[3] = M_PI;
    x_init[4] = -M_PI * M_PI / 10.0; // 同Python中设置
    x_init[5] = 0.0;
    x_init[6] = M_PI / 2;
    x_init[7] = 10.1;
    x_init[8] = 0.0;
    x_init[9] = M_PI / 2;
    Eigen::VectorXd u0(NU);
    u0 << 0.0, 0.0, 0.0;
    acados_wrapper.set_initial_conditions(x_init, u0);
    // set the reference trajectory
    Eigen::MatrixXd ref_traj = define_ref_traj(NSTEPS, 0.1);
    acados_wrapper.set_reference_trajectory(ref_traj);
    // set the parameters
    Eigen::VectorXd p(NP);
    // 视场角 视场距离 参考点x 参考点y tube半径平方
    p << 0.2, 2, ref_traj(0, 0), ref_traj(1, 0), 9;
    acados_wrapper.set_params(p);
    // solve the optimal control problem
    acados_wrapper.solve();
    return 0; 
}