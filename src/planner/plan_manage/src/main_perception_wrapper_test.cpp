#include "perception_mpc_wrapper.hpp"
#include <vector>
#include <Eigen/Dense>
#include <cmath>

Eigen::MatrixXd define_ref_traj_2(int N) {
    // Data provided
    std::vector<double> px = {2.05024, 2.17461, 2.2933, 2.4069, 2.5161, 2.62196, 2.72569, 2.82848, 2.93112, 3.03425, 3.13848, 3.24428, 3.35205, 3.46214, 3.57469, 3.68971, 3.8072, 3.92686, 4.04826, 4.17095};
    std::vector<double> py = {-0.394741, -0.313512, -0.233161, -0.153921, -0.0760187, 0.000393903, 0.0751934, 0.148203, 0.218877, 0.286517, 0.350389, 0.409528, 0.46287, 0.509333, 0.547705, 0.576717, 0.595171, 0.602331, 0.597658, 0.580669};
    std::vector<double> vx = {1.27385, 1.21426, 1.16044, 1.11272, 1.07321, 1.04589, 1.03085, 1.02608, 1.02778, 1.03586, 1.04951, 1.06717, 1.0888, 1.11316, 1.13782, 1.16273, 1.18641, 1.20598, 1.22141, 1.23096};
    std::vector<double> vy = {0.81578, 0.808298, 0.798347, 0.786049, 0.771781, 0.756266, 0.739519, 0.719583, 0.692738, 0.658896, 0.616822, 0.564174, 0.500894, 0.426288, 0.339035, 0.239103, 0.128969, 0.013336, -0.107684, -0.232115};
    std::vector<double> ax = {-0.590954, -0.568757, -0.507686, -0.446614, -0.334648, -0.211833, -0.0890181, -0.0148636, 0.0489184, 0.1127, 0.156702, 0.196488, 0.236273, 0.245356, 0.247893, 0.250431, 0.216593, 0.175001, 0.133409, 0.0504558};
    std::vector<double> ay = {-0.0506895, -0.0877634, -0.111245, -0.134727, -0.149, -0.161311, -0.173621, -0.233461, -0.303433, -0.373405, -0.47333, -0.57964, -0.685951, -0.80914, -0.935928, -1.06272, -1.1294, -1.18326, -1.23713, -1.24308};
    // Calculate yaw using arctan2
    std::vector<double> yaw(N);
    for (int i = 0; i < N; ++i) {
        yaw[i] = std::atan2(vy[i], vx[i]);
    }
    // Create an 7 * N matrix for the reference trajectory
    Eigen::MatrixXd ref_traj(7, N);

    // Fill the matrix with the reference data
    for (int i = 0; i < N; i++) {
        ref_traj(0, i) = px[i];
        ref_traj(1, i) = py[i];
        ref_traj(2, i) = vx[i];
        ref_traj(3, i) = vy[i];
        ref_traj(4, i) = ax[i];
        ref_traj(5, i) = ay[i];
        ref_traj(6, i) = yaw[i];
    }
    // Return the reference trajectory matrix
    return ref_traj;
}

int main(){
    // Define parameters
    int N = 20;
    double pi = M_PI;  // Use the constant for pi
    Eigen::VectorXd initial_state(7);
    initial_state << 2.05024, -0.394741, 1.27385, 0.81578, -0.590954, -0.0506895, pi ; // yaw set to pi

    // Create an instance of the AcadosPerceptionWrapper class
    AcadosPerceptionWrapper perception_wrapper(N);

    // set init constraints
    perception_wrapper.set_initial_conditions(initial_state, Eigen::VectorXd::Zero(3));

    // set control constraints
    Eigen::VectorXd lbu(3);
    Eigen::VectorXd ubu(3);
    lbu << -50, -50, -pi;
    ubu << 50, 50, pi;
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
    p << pi / 6;  // Field of view
    perception_wrapper.set_params(p);

    // reference trajectory cost set
    Eigen::MatrixXd ref_traj = define_ref_traj_2(N);
    perception_wrapper.set_reference_trajectory(ref_traj);
    
    perception_wrapper.solve();
    perception_wrapper.print_results();
    return 0;

}
