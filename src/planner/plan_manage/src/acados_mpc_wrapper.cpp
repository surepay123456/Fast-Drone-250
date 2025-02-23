#include "acados_mpc_wrapper.h"
#include <iostream>
// #include <acados_c/ocp_nlp_interface.h>
// #include <acados_solver_augmented_yaw_model.h>
// #include <iostream>
// #include <plan_env/obj_predictor.h>
AcadosMpcWrapper::AcadosMpcWrapper(const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R,
                                    double max_jerk, double max_w,  double new_time_step)
                                     : acados_ocp_capsule_(nullptr)
{
    acados_ocp_capsule_ = augmented_yaw_model_acados_create_capsule();
    int N = NSTEPS;
    // double *new_time_steps = nullptr;
    double* new_time_steps = (double*)malloc(N * sizeof(double));  // 为时间步长数组分配内存
    // 填充时间步长，假设我们想用均匀间隔的时间步长
    for (int i = 0; i < N; ++i) {
        new_time_steps[i] = new_time_step;  // 假设时间步长间隔为0.1
    }
    int status = augmented_yaw_model_acados_create_with_discretization(acados_ocp_capsule_, N, new_time_steps);
    if (status) {
        printf("quad_body_3d_acados_create() returned status %d. Exiting.\n", status);
    }
    else {
        printf("created and initialized acados_ocp_capsule \n");
    }
    acados_nlp_config = augmented_yaw_model_acados_get_nlp_config(acados_ocp_capsule_);
    acados_nlp_dims = augmented_yaw_model_acados_get_nlp_dims(acados_ocp_capsule_);
    acados_nlp_in = augmented_yaw_model_acados_get_nlp_in(acados_ocp_capsule_);
    acados_nlp_out = augmented_yaw_model_acados_get_nlp_out(acados_ocp_capsule_);
    acados_nlp_solver = augmented_yaw_model_acados_get_nlp_solver(acados_ocp_capsule_);
    acados_nlp_opts = augmented_yaw_model_acados_get_nlp_opts(acados_ocp_capsule_);
    
    Eigen::VectorXd u_bound(NU * 2);
    u_bound << -max_jerk, -max_jerk, -max_w, max_jerk, max_jerk, max_w;
    SetControlConstraints(u_bound);
    SetCostMatrix(Q, R);
}

AcadosMpcWrapper::~AcadosMpcWrapper()
{
    // free solver
    int status = augmented_yaw_model_acados_free(acados_ocp_capsule_);
    if (status) {
        printf("quad_body_3d_acados_free() returned status %d. \n", status);
    }
    status = augmented_yaw_model_acados_free_capsule(acados_ocp_capsule_);
}


/********************************/
/* getter */

// this function must be called after solve to get the result
void AcadosMpcWrapper::get_traj_from_solver(){
    // get the traj 
    for (int ii = 0; ii <= acados_nlp_dims->N; ii++)
        ocp_nlp_out_get(acados_nlp_config, acados_nlp_dims, acados_nlp_out, ii, "x", &Acados_xtraj_[ii*NX]);
    for (int ii = 0; ii < acados_nlp_dims->N; ii++)
        ocp_nlp_out_get(acados_nlp_config, acados_nlp_dims, acados_nlp_out, ii, "u", &Acados_utraj_[ii*NU]);  
    printf("\n--- xtraj ---\n");
    d_print_exp_tran_mat( NX, NSTEPS+1, Acados_xtraj_, NX);
    printf("\n--- utraj ---\n");
    d_print_exp_tran_mat( NU, NSTEPS, Acados_utraj_, NU );
}
void AcadosMpcWrapper::GetControls(Eigen::Ref<Eigen::MatrixXd> u_traj){
    u_traj = acados_utraj_;
}
void AcadosMpcWrapper::GetControl(Eigen::VectorXd& control){
    control =  acados_utraj_.col(0);
}
void AcadosMpcWrapper::GetStates(Eigen::Ref<Eigen::MatrixXd> x_traj){
    x_traj = acados_xtraj_;
}
/********************************/

/********************************/
/* setter */
void AcadosMpcWrapper::SetIntialState(const Eigen::VectorXd& state){
    acados_init_state_ = state;
    ocp_nlp_constraints_model_set(acados_nlp_config, acados_nlp_dims, acados_nlp_in, 0, "lbx", Acados_x_init_);
    ocp_nlp_constraints_model_set(acados_nlp_config, acados_nlp_dims, acados_nlp_in, 0, "ubx", Acados_x_init_);
}
void AcadosMpcWrapper::SetIntialStateTraj(Eigen::Ref<Eigen::MatrixXd> state_traj){
    SetIntialState(state_traj.col(0));
    acados_xtraj_ = state_traj;
}
void AcadosMpcWrapper::SetRefTraj(Eigen::Ref<Eigen::MatrixXd> traj)
{
    
    // std::cout << "acados_yref_ " << acados_yref_ << std::endl;
    acados_yref_ = traj; // NY * NSTEPS
    double y_ref_temp[NY];
    for (int i = 1; i < acados_nlp_dims->N; i++)
    {
        for (int j = 0; j < NY; j ++) {
            y_ref_temp[j] = Acados_y_ref_[(i -1) * NY + j];
        }
        ocp_nlp_cost_model_set(acados_nlp_config, acados_nlp_dims, acados_nlp_in, i, "yref", y_ref_temp);
    }
    double y_ref_temp_end[NX_CURRENT];
    for (int j = 0; j < NX_CURRENT; j ++) {
        y_ref_temp_end[j] = Acados_y_ref_[(NSTEPS - 1) * NY + j];
    }
    ocp_nlp_cost_model_set(acados_nlp_config, acados_nlp_dims, acados_nlp_in, NSTEPS, "yref", y_ref_temp_end);
}
// constraint for u and v
void AcadosMpcWrapper::SetControlConstraints(Eigen::VectorXd& constraint)
{
    int idxbu0[] = {0, 1, 2}; 
    for (int i = 0; i < NSTEPS; i++) {
        ocp_nlp_constraints_model_set(acados_nlp_config, acados_nlp_dims, acados_nlp_in, i, "idxbu", idxbu0);
    }
    ocp_nlp_constraints_model_set(acados_nlp_config, acados_nlp_dims, acados_nlp_in, 0, "idxbu", idxbu0);

    double lbu[NU]; 
    for (int i = 0; i < NU; i++) {
        lbu[i] = constraint[i];
    }
    double ubu[NU];
    for (int i = 0; i < NU; i++) {
        ubu[i] = constraint[i + NU];
    }

    for (int i = 0; i < NSTEPS; i++) {
        ocp_nlp_constraints_model_set(acados_nlp_config, acados_nlp_dims, acados_nlp_in, i, "lbu", lbu);
        ocp_nlp_constraints_model_set(acados_nlp_config, acados_nlp_dims, acados_nlp_in, i, "ubu", ubu);
    }

}

void AcadosMpcWrapper::SetCostMatrix(const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R)
{
    // Initialize Acados_cost_matrix_ using Eigen's block operations
    acados_cost_matrix_.block(0, 0, NX_CURRENT, NX_CURRENT) = Q;
    acados_cost_matrix_.block(NX_CURRENT, NX_CURRENT, NU, NU) = R;

    // Initialize Acados_cost_matrix_end using Eigen's block operations
    acados_cost_matrix_end.block(0, 0, NX_CURRENT, NX_CURRENT) = Q.block(0, 0, NX_CURRENT, NX_CURRENT);

    // Set the remaining elements of Acados_cost_matrix_ to 0
    acados_cost_matrix_.block(NX_CURRENT, 0, NU, NX_CURRENT).setZero();
    acados_cost_matrix_.block(0, NX_CURRENT, NX_CURRENT, NU).setZero();


    // Set the cost matrices for each stage
    for(int i = 1; i <= NSTEPS - 1; i++) {
        ocp_nlp_cost_model_set(acados_nlp_config, acados_nlp_dims, acados_nlp_in, i, "W", acados_cost_matrix_.data());
    }

    // Set the end cost matrix
    ocp_nlp_cost_model_set(acados_nlp_config, acados_nlp_dims, acados_nlp_in, NSTEPS, "W", acados_cost_matrix_end.data());
}

// int AcadosMpcWrapper::update_params(const Eigen::VectorXd& p){
//     acados_params_ = p;
//     for (int i = 1; i <= NSTEPS; i++) {
//         // ocp_nlp_constraints_model_set(acados_nlp_config, acados_nlp_dims, acados_nlp_in, i, "p", Acados_params_);
//     }
//     return 0;
// }

int AcadosMpcWrapper::update_params_at_t(const Eigen::VectorXd& p, int stage){
    for (int i = 0; i < NP; i++) {
        Acados_params_[(stage - 1) * NP + i] = p[i];
    }
    // std::cout << "Acados_params_ " << Acados_params_ << std::endl;
    // ocp_nlp_in_set_params_sparse(acados_nlp_config, acados_nlp_dims, acados_nlp_in, stage, "p", Acados_params_ + (stage - 1) * NP);
    ocp_nlp_in_set(acados_nlp_config, acados_nlp_dims, acados_nlp_in, stage, "parameter_values", p.data());
    std::cout << "update params at t " << stage << std::endl;
    return 0;
}
/******************************/

/******************************/
/* solver */
double AcadosMpcWrapper::solve(Eigen::VectorXd &state){
    SetIntialState(state);
    int status = augmented_yaw_model_acados_solve(acados_ocp_capsule_);
    if (status) {
        printf("acados returned status %d. \n", status);
    }
    ocp_nlp_get(acados_nlp_solver, "time_tot", &t_consume_);
    return t_consume_;
}
/******************************/



