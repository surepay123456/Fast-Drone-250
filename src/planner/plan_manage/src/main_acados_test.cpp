#include "acados_mpc_wrapper.h"
// #define M_PI 3.14159265358979323846

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
    std::cout << "ref_traj size " << ref_traj.rows() << " " << ref_traj.cols() << std::endl;
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
    // Eigen::MatrixXd Q = 10 * Eigen::MatrixXd::Identity(7, 7);
    // Eigen::MatrixXd R = Eigen::MatrixXd::Identity(3, 3);
    // double max_jerk = 5.0;
    // double max_w = 5.0;
    // AcadosMpcWrapper acados_mpc_wrapper(Q, R, max_jerk, max_w, 0.1);

    // Eigen::VectorXd initial_state(NX);
    // initial_state(0)  = 10.1;
    // initial_state(1)  = 0.0;
    // initial_state(2)  = 0.0;
    // initial_state(3)  = M_PI;
    // initial_state(4)  = -M_PI * M_PI / 10.0;
    // initial_state(5)  = 0.0;
    // initial_state(6)  = M_PI / 2;
    // initial_state(7)  = 10.1;
    // initial_state(8)  = 0.0;
    // initial_state(9)  = M_PI / 2;
    // Eigen::MatrixXd initial_state_traj = initial_state.replicate(1, NSTEPS + 1);

    // acados_mpc_wrapper.SetIntialStateTraj(initial_state_traj);
    // std::cout << "initial state set" << std::endl;
    // Eigen::MatrixXd ref_traj = define_ref_traj(NSTEPS, 0.1);
    // std::cout << "ref traj defined" << std::endl;
    // // ref_traj = ref_traj.block(0, 1, NY, NSTEPS);
    // Eigen::MatrixXd extend_ref_traj = Eigen::MatrixXd::Zero(NY, NSTEPS);
    // extend_ref_traj.block(0, 0, NX_CURRENT, NSTEPS) = ref_traj;
    
    // acados_mpc_wrapper.SetRefTraj(extend_ref_traj);
    // std::cout << "ref traj set" << std::endl;
    
    // Eigen::VectorXd p(NP);
    // for (int i = 0; i < NSTEPS; i++) {
    //     p << 0.1, 2, ref_traj(0, i), ref_traj(1, i), 9; 
    //     acados_mpc_wrapper.update_params_at_t(p, i + 1);
    // }
    // double time = 0.0;
    // time = acados_mpc_wrapper.solve(initial_state); 
    // std::cout << " time consume " << time << std::endl;
    // acados_mpc_wrapper.get_traj_from_solver();
    
    return 0;
}