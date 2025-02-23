#include "acados_simple_wrapper.hpp"

AcadosSimpleWrapper::AcadosSimpleWrapper(int N) : N_(N)
{
    // Create capsule and solver objects
    acados_ocp_capsule_ = augmented_yaw_model_acados_create_capsule();
    int status = augmented_yaw_model_acados_create_with_discretization(acados_ocp_capsule_, N_, NULL);
    if (status)
    {
        printf("augmented_yaw_model_acados_create() failed with status %d\n", status);
        exit(1);
    }

    // Get NLP solver components
    nlp_config_ = augmented_yaw_model_acados_get_nlp_config(acados_ocp_capsule_);
    nlp_dims_ = augmented_yaw_model_acados_get_nlp_dims(acados_ocp_capsule_);
    nlp_in_ = augmented_yaw_model_acados_get_nlp_in(acados_ocp_capsule_);
    nlp_out_ = augmented_yaw_model_acados_get_nlp_out(acados_ocp_capsule_);
    nlp_solver_ = augmented_yaw_model_acados_get_nlp_solver(acados_ocp_capsule_);
    nlp_opts_ = augmented_yaw_model_acados_get_nlp_opts(acados_ocp_capsule_);
}

AcadosSimpleWrapper::~AcadosSimpleWrapper()
{
    // Free solver
    int status = augmented_yaw_model_acados_free(acados_ocp_capsule_);
    if (status)
    {
        printf("augmented_yaw_model_acados_free() failed with status %d\n", status);
    }

    // Free solver capsule
    status = augmented_yaw_model_acados_free_capsule(acados_ocp_capsule_);
    if (status)
    {
        printf("augmented_yaw_model_acados_free_capsule() failed with status %d\n", status);
    }
}

void AcadosSimpleWrapper::set_initial_conditions(const Eigen::VectorXd &x_init, const Eigen::VectorXd &u0){
    if (x_init.size() != NX || u0.size() != NU)
    {
        printf("Initial conditions are not correct.\n");
        return;
    }
    // Copy the data from Eigen to the arrays
    std::memcpy(x_init_, x_init.data(), x_init.size() * sizeof(double));
    std::memcpy(u0_, u0.data(), u0.size() * sizeof(double));
    ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, 0, "lbx", x_init_);
    ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, 0, "ubx", x_init_);
    for (int i = 0; i < N_; i++) {
        ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, i, "x", x_init_);
        ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, i, "u", u0_);
    }
    ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, N_, "x", x_init_);
}


void AcadosSimpleWrapper::set_reference_trajectory(const Eigen::MatrixXd& ref_traj)
{
    //    Eigen::MatrixXd ref_traj(NX_CURRENT, NSTEPS);
    if (ref_traj.size() != NX_CURRENT * NSTEPS)  // Check if the size of the reference trajectory is correct
    {
        printf("Reference trajectory size is not correct.\n");
        return;
    }
    ref_traj_ = ref_traj;
    for (int i = 1; i < N_; i++) {
        Eigen::VectorXd yref(NY);
        yref.setZero();
        yref << ref_traj_.col(i - 1), 0, 0, 0; // add zeros for the last three elements control input
        ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, i, "yref", yref.data());
    }
    ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, N_, "yref", ref_traj_.col(N_ - 1).data());
}

// must be called after set_reference_trajectory
void AcadosSimpleWrapper::set_params(const Eigen::VectorXd& p)
{
    p_ = p;
    // Set parameters for the solver based on reference trajectory
    // p_ << 0.1, 2, ref_traj_(0, 0), ref_traj_(1, 0), 9;
    for (int i = 1; i <= N_; i++) {
        Eigen::VectorXd p_non_const = p;
        p_non_const[2] = ref_traj_(0, i - 1);
        p_non_const[3] = ref_traj_(1, i - 1);
        ocp_nlp_in_set(nlp_config_, nlp_dims_, nlp_in_, i, "parameter_values", p_non_const.data());
    }
}

void AcadosSimpleWrapper::set_control_bounds(const Eigen::VectorXd& lbu, const Eigen::VectorXd& ubu){
    if (lbu.size() != NU || ubu.size() != NU) {
        printf("Control bounds are not correct.\n");
        return;
    }
    for (int i = 0; i < NU; i++) {
        lbu_[i] = lbu(i);
        ubu_[i] = ubu(i);
    } 
    for (int i = 0; i < N_; i++) {
        ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, i, "lbu", lbu_);
        ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, i, "ubu", ubu_);
    }
}
void AcadosSimpleWrapper::set_cost_weights(const Eigen::VectorXd &Q,
                                           const Eigen::VectorXd &R) {
    if (Q.size() != NX_CURRENT || R.size() != NU) {
        printf("Cost weights are not correct.\n");
        return;
    }
    Q_ = Q.asDiagonal();
    R_ = R.asDiagonal();
    W_ = Eigen::MatrixXd::Zero(NY, NY);
    W_.block(0, 0, NX_CURRENT, NX_CURRENT) = Q_;
    W_.block(NX_CURRENT, NX_CURRENT, NU, NU) = R_;
    for (int i = 0; i < N_; i++) {
        ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, i, "W", W_.data());
    }
    ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, N_, "W", Q_.data());
}

void AcadosSimpleWrapper::set_cost_weights_end(const Eigen::VectorXd &Q) {
    if (Q.size() != NX_CURRENT) {
        printf("End cost weights are not correct.\n");
        return;
    }
    W_end_ = Q.asDiagonal();
    ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, N_, "W", W_end_.data());
}

int AcadosSimpleWrapper::solve()
{
    int status = augmented_yaw_model_acados_solve(acados_ocp_capsule_);
    if (status == ACADOS_SUCCESS)
    {
        // printf("augmented_yaw_model_acados_solve(): SUCCESS!\n");
    }
    else
    {
        printf("augmented_yaw_model_acados_solve() failed with status %d.\n", status);
        // augmented_yaw_model_acados_print_stats(acados_ocp_capsule_);
    }
        // double xtraj[NX * (N_ + 1)], utraj[NU * N_];
    for (int ii = 0; ii <= nlp_dims_->N; ii++) {
        ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, ii, "x", &xtraj_[ii * NX]);
    }
    for (int ii = 0; ii < nlp_dims_->N; ii++) {
        ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, ii, "u", &utraj_[ii * NU]);
    }
    // print_results();
    return status;
}

void AcadosSimpleWrapper::print_results()
{


    printf("\n--- xtraj ---\n");
    d_print_exp_tran_mat(NX, N_ + 1, xtraj_, NX);
    printf("\n--- utraj ---\n");
    d_print_exp_tran_mat(NU, N_, utraj_, NU);
    // prepare evaluation
    int NTIMINGS = 1;
    // double min_time = 1e12;
    double kkt_norm_inf;
    double elapsed_time;
    int sqp_iter;
    // get solution
    ocp_nlp_get(nlp_solver_, "time_tot", &elapsed_time);
    ocp_nlp_out_get(nlp_config_,nlp_dims_, nlp_out_, 0, "kkt_norm_inf", &kkt_norm_inf);
    ocp_nlp_get(nlp_solver_, "sqp_iter", &sqp_iter);

    augmented_yaw_model_acados_print_stats(acados_ocp_capsule_);

    printf("\nSolver info:\n");
    printf(" SQP iterations %2d\n minimum time for %d solve %f [ms]\n KKT %e\n",
           sqp_iter, NTIMINGS, elapsed_time*1000, kkt_norm_inf);
    printf("\nSolver info:\n");
    // printf(" SQP iterations %2d\n minimum time for solve %f [ms]\n", 0, elapsed_time);  // Adjust if needed

}

void AcadosSimpleWrapper::get_results(Eigen::MatrixXd &x, Eigen::MatrixXd &u){
    if (x.size() != NX * (N_ + 1) || u.size() != NU * N_) {
        printf("The size of the matrices is not correct.\n");
        return; 
    }
    // x.resize(NX, N_ + 1);
    // u.resize(NU, N_);
    for (int i = 0; i <= N_; i++) {
        for (int j = 0; j < NX; j++) {
            x(j, i) = xtraj_[i * NX + j];
        }
    }
    for (int i = 0; i < N_; i++) {
        for (int j = 0; j < NU; j++) {
            u(j, i) = utraj_[i * NU + j];
        }
    }
}