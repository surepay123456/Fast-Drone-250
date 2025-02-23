// standard
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
// acados
#include "acados/utils/print.h"
#include "acados/utils/math.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"
#include "acados_solver_augmented_yaw_model.h"

// blasfeo
#include "blasfeo/include/blasfeo_d_aux_ext_dep.h"
#include "Eigen/Dense"

#define NX     AUGMENTED_YAW_MODEL_NX
#define NP     AUGMENTED_YAW_MODEL_NP
#define NU     AUGMENTED_YAW_MODEL_NU
#define NBX0   AUGMENTED_YAW_MODEL_NBX0
#define NP_GLOBAL   AUGMENTED_YAW_MODEL_NP_GLOBAL
#define M_PI 3.14159265358979323846


#define NX_CURRENT 7
#define NY  10
#define NSTEPS 20

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

int main()
{

    augmented_yaw_model_solver_capsule *acados_ocp_capsule = augmented_yaw_model_acados_create_capsule();
    // there is an opportunity to change the number of shooting intervals in C without new code generation
    int N = AUGMENTED_YAW_MODEL_N;
    // allocate the array and fill it accordingly
    double* new_time_steps = NULL;
    int status = augmented_yaw_model_acados_create_with_discretization(acados_ocp_capsule, N, new_time_steps);

    if (status)
    {
        printf("augmented_yaw_model_acados_create() returned status %d. Exiting.\n", status);
        exit(1);
    }

    ocp_nlp_config *nlp_config = augmented_yaw_model_acados_get_nlp_config(acados_ocp_capsule);
    ocp_nlp_dims *nlp_dims = augmented_yaw_model_acados_get_nlp_dims(acados_ocp_capsule);
    ocp_nlp_in *nlp_in = augmented_yaw_model_acados_get_nlp_in(acados_ocp_capsule);
    ocp_nlp_out *nlp_out = augmented_yaw_model_acados_get_nlp_out(acados_ocp_capsule);
    ocp_nlp_solver *nlp_solver = augmented_yaw_model_acados_get_nlp_solver(acados_ocp_capsule);
    void *nlp_opts = augmented_yaw_model_acados_get_nlp_opts(acados_ocp_capsule);
    /******************************************************************** */
    /* step 1 the init state set start! */
    // initial condition
    double lbx0[NBX0];
    double ubx0[NBX0];
    lbx0[0] = 10.1;
    ubx0[0] = 10.1;
    lbx0[1] = 0.0;
    ubx0[1] = 0.0;
    lbx0[2] = 0.0;
    ubx0[2] = 0.0;
    lbx0[3] = M_PI;
    ubx0[3] = M_PI;
    lbx0[4] = -M_PI * M_PI / 10.0;  
    ubx0[4] = -M_PI * M_PI / 10.0;
    lbx0[5] = 0.0;
    ubx0[5] = 0.0;
    lbx0[6] = M_PI / 2;
    ubx0[6] = M_PI / 2;
    lbx0[7] = 10.1;
    ubx0[7] = 10.1;
    lbx0[8] = 0.0;
    ubx0[8] = 0.0;
    lbx0[9] = M_PI / 2;
    ubx0[9] = M_PI / 2;

    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", lbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", ubx0);

    // initialization for state values
    double x_init[NX];

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

    // initial value for control input
    double u0[NU];
    u0[0] = 0.0;
    u0[1] = 0.0;
    u0[2] = 0.0;

    // 状态和控制输入设置
    for (int i = 0; i < N; i++) {
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "x", x_init);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "u", u0);
    }
    ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, N, "x", x_init);
    /***********************************************************/
    /* step 1 the init state set end! */

    /******************************************************** */
    /* step 2 define the ref traj and set ref start!*/
    Eigen::MatrixXd ref_traj_ = define_ref_traj(N, 0.1);
    for (int i = 1; i < N; i++) {
        Eigen::VectorXd yref(NY);
        yref.setZero();
        yref << ref_traj_.col(i - 1), 0 , 0, 0;
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "yref", yref.data());
    }
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, NSTEPS, "yref", ref_traj_.col(N - 1).data());
    /******************************************************* */
    /* step 2 define the ref traj and set ref end!*/

    /******************************************************* */
    /* step 3 set the param start!*/
    Eigen::VectorXd p(NP);  // 确保 NP 是定义的，并且 p 的大小正确
    p << 0.1, 2, ref_traj_(0, 0), ref_traj_(1, 0), 9;  // 假设 ref_traj 已经是一个 Eigen::MatrixXd 类型，确保其大小为 (2, NSTEPS)jkj
    // 假设 ref_traj 已经是一个 Eigen::MatrixXd 类型，确保其大小为 (2, NSTEPS)
    for (int i = 1; i <= NSTEPS; i++) {
        // 创建一个非 const 版
        Eigen::VectorXd p_non_const = p; 
        p_non_const[2] = ref_traj_(0, i - 1);  // 更新 p 的第 3 个元素
        p_non_const[3] = ref_traj_(1, i - 1);  // 更新 p 的第 4 个元素
        
        // 将更新后的 p_non_const 传递给 ocp_nlp_in_set
        ocp_nlp_in_set(nlp_config, nlp_dims, nlp_in, i, "parameter_values", p_non_const.data());
    }
    /******************************************************* */


    // prepare evaluation
    int NTIMINGS = 1;
    double min_time = 1e12;
    double kkt_norm_inf;
    double elapsed_time;
    int sqp_iter;

    double xtraj[NX * (N+1)];
    double utraj[NU * N];

    // solve ocp in loop
    for (int ii = 0; ii < NTIMINGS; ii++)
    {
        // // initialize solution
        // for (int i = 0; i < N; i++)
        // {
        //     ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "x", x_init);
        //     ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "u", u0);
        // }
        // ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, N, "x", x_init);
        status = augmented_yaw_model_acados_solve(acados_ocp_capsule);
        ocp_nlp_get(nlp_solver, "time_tot", &elapsed_time);
        min_time = MIN(elapsed_time, min_time);
    }

    /* print solution and statistics */
    for (int ii = 0; ii <= nlp_dims->N; ii++)
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, ii, "x", &xtraj[ii*NX]);
    for (int ii = 0; ii < nlp_dims->N; ii++)
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, ii, "u", &utraj[ii*NU]);

    printf("\n--- xtraj ---\n");
    d_print_exp_tran_mat( NX, N+1, xtraj, NX);
    printf("\n--- utraj ---\n");
    d_print_exp_tran_mat( NU, N, utraj, NU );
    // ocp_nlp_out_print(nlp_solver->dims, nlp_out);

    printf("\nsolved ocp %d times, solution printed above\n\n", NTIMINGS);

    if (status == ACADOS_SUCCESS)
    {
        printf("augmented_yaw_model_acados_solve(): SUCCESS!\n");
    }
    else
    {
        printf("augmented_yaw_model_acados_solve() failed with status %d.\n", status);
    }

    // get solution
    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "kkt_norm_inf", &kkt_norm_inf);
    ocp_nlp_get(nlp_solver, "sqp_iter", &sqp_iter);

    augmented_yaw_model_acados_print_stats(acados_ocp_capsule);

    printf("\nSolver info:\n");
    printf(" SQP iterations %2d\n minimum time for %d solve %f [ms]\n KKT %e\n",
           sqp_iter, NTIMINGS, min_time*1000, kkt_norm_inf);


    // free solver
    status = augmented_yaw_model_acados_free(acados_ocp_capsule);
    if (status) {
        printf("augmented_yaw_model_acados_free() returned status %d. \n", status);
    }
    // free solver capsule
    status = augmented_yaw_model_acados_free_capsule(acados_ocp_capsule);
    if (status) {
        printf("augmented_yaw_model_acados_free_capsule() returned status %d. \n", status);
    }

    return status;
}
