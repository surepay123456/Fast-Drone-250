#include "perception_mpc_wrapper.hpp"

AcadosPerceptionWrapper::AcadosPerceptionWrapper(int N) : N_(N)
{
    // Create capsule and solver objects
    acados_ocp_capsule_ = quad_point_2d_with_yaw_acados_create_capsule();
    int status = quad_point_2d_with_yaw_acados_create_with_discretization(acados_ocp_capsule_, N_, NULL);
    if (status)
    {
        printf("quad_point_2d_with_yaw_acados_create() failed with status %d\n", status);
        exit(1);
    }

    // Get NLP solver components
    nlp_config_ = quad_point_2d_with_yaw_acados_get_nlp_config(acados_ocp_capsule_);
    nlp_dims_ = quad_point_2d_with_yaw_acados_get_nlp_dims(acados_ocp_capsule_);
    nlp_in_ = quad_point_2d_with_yaw_acados_get_nlp_in(acados_ocp_capsule_);
    nlp_out_ = quad_point_2d_with_yaw_acados_get_nlp_out(acados_ocp_capsule_);
    nlp_solver_ = quad_point_2d_with_yaw_acados_get_nlp_solver(acados_ocp_capsule_);
    nlp_opts_ = quad_point_2d_with_yaw_acados_get_nlp_opts(acados_ocp_capsule_);
}

AcadosPerceptionWrapper::~AcadosPerceptionWrapper()
{
    // Free solver
    int status = quad_point_2d_with_yaw_acados_free(acados_ocp_capsule_);
    if (status)
    {
        printf("quad_point_2d_with_yaw_acados_free() failed with status %d\n", status);
    }

    // Free solver capsule
    status = quad_point_2d_with_yaw_acados_free_capsule(acados_ocp_capsule_);
    if (status)
    {
        printf("quad_point_2d_with_yaw_acados_free_capsule() failed with status %d\n", status);
    }
}

void AcadosPerceptionWrapper::set_initial_conditions(const Eigen::VectorXd &x_init, const Eigen::VectorXd &u0){
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

void AcadosPerceptionWrapper::set_reference_trajectory(const Eigen::MatrixXd& ref_traj)
{
    if (ref_traj.rows() != NX || ref_traj.cols() != NSTEPS)
    {
        printf("Reference trajectory is not correct.\n");
        return;
    }
    ref_traj_ = ref_traj;
    for (int i = 1; i < N_; i++) {
        Eigen::VectorXd yref(NY);
        yref.setZero();
        yref.head(NX) = ref_traj_.col(i - 1);
        ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, i, "yref", yref.data());
    }
    ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, N_, "yref", ref_traj_.col(N_ - 1).data());
}

void AcadosPerceptionWrapper::set_params(const Eigen::VectorXd &p)
{
    if (p.size() != NP)
    {
        printf("Parameters are not correct.\n");
        return;
    }
    for (int i = 0; i <= N_; i++) {
        Eigen::VectorXd p_non_const = p;

        ocp_nlp_in_set(nlp_config_, nlp_dims_, nlp_in_, i, "parameter_values", p_non_const.data());
    }
}

void AcadosPerceptionWrapper::set_control_bounds(const Eigen::VectorXd& lbu, const Eigen::VectorXd& ubu){
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

void AcadosPerceptionWrapper::set_cost_weights(const Eigen::VectorXd &Q, const Eigen::VectorXd &R)
{
    if (Q.size() != NX || R.size() != NU)
    {
        printf("Cost weights are not correct.\n");
        return;
    }
    Q_ = Q.asDiagonal();
    R_ = R.asDiagonal();
    W_ = Eigen::MatrixXd::Zero(NY, NY);
    W_.block(0, 0, NX, NX) = Q_;
    W_.block(NX, NX, NU, NU) = R_;
    for (int i = 0; i < N_; i++) {
        ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, i, "W", W_.data());
    }
    ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, N_, "W", Q_.data());
}

void AcadosPerceptionWrapper::set_cost_weights_end(const Eigen::VectorXd &Q)
{
    if (Q.size() != NX)
    {
        printf("End cost weights are not correct.\n");
        return;
    }
    W_end_ = Q.asDiagonal();
    ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, N_, "W", W_end_.data());
}

void AcadosPerceptionWrapper::set_cost_slack_weights(const double &zl, const double &Zl, const double &zu, const double &Zu){
    Eigen::MatrixXd zl_mat = zl * Eigen::MatrixXd::Ones(NH, 1);
    Eigen::MatrixXd Zl_mat = Zl * Eigen::MatrixXd::Ones(NH, 1);
    Eigen::MatrixXd zu_mat = zu * Eigen::MatrixXd::Ones(NH, 1);
    Eigen::MatrixXd Zu_mat = Zu * Eigen::MatrixXd::Ones(NH, 1);
    for (int i = 0 ; i < N_; i++) {
        ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, i, "zl", zl_mat.data());
        ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, i, "Zl", Zl_mat.data());
        ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, i, "zu", zu_mat.data());
        ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, i, "Zu", Zu_mat.data());
    }
    return;
}

void AcadosPerceptionWrapper::set_cost_slack_begin_weights(const double &zl_0, const double &Zl_0, const double &zu_0, const double &Zu_0){
    Eigen::MatrixXd zl_mat = zl_0 * Eigen::MatrixXd::Ones(NH, 1);
    Eigen::MatrixXd Zl_mat = Zl_0 * Eigen::MatrixXd::Ones(NH, 1);
    Eigen::MatrixXd zu_mat = zu_0 * Eigen::MatrixXd::Ones(NH, 1);
    Eigen::MatrixXd Zu_mat = Zu_0 * Eigen::MatrixXd::Ones(NH, 1);
    ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, 0, "zl", zl_mat.data());
    ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, 0, "Zl", Zl_mat.data());
    ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, 0, "zu", zu_mat.data());
    ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, 0, "Zu", Zu_mat.data());
    return;
}

void AcadosPerceptionWrapper::set_cost_slack_end_weights(const double &zl_N, const double &Zl_N, const double &zu_N, const double &Zu_N){
    Eigen::MatrixXd zl_mat = zl_N * Eigen::MatrixXd::Ones(NH, 1);
    Eigen::MatrixXd Zl_mat = Zl_N * Eigen::MatrixXd::Ones(NH, 1);
    Eigen::MatrixXd zu_mat = zu_N * Eigen::MatrixXd::Ones(NH, 1);
    Eigen::MatrixXd Zu_mat = Zu_N * Eigen::MatrixXd::Ones(NH, 1);
    ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, N_, "zl", zl_mat.data());
    ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, N_, "Zl", Zl_mat.data());
    ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, N_, "zu", zu_mat.data());
    ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, N_, "Zu", Zu_mat.data());
    return;
}

int AcadosPerceptionWrapper::solve()
{
    // Solve the optimization problem
    int status = quad_point_2d_with_yaw_acados_solve(acados_ocp_capsule_);
    if (status)
    {
        printf("quad_point_2d_with_yaw_acados_solve() failed with status %d\n", status);
    }

    // Get the solution
    for (int ii = 0; ii <= nlp_dims_->N; ii++)
    {
        ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, ii, "x", &xtraj_[ii * NX]);
    }
    for (int ii = 0; ii < nlp_dims_->N; ii++)
    {
        ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, ii, "u", &utraj_[ii * NU]);
    }

    return status;
}

void AcadosPerceptionWrapper::get_results(Eigen::MatrixXd &x, Eigen::MatrixXd &u)
{
    if (x.rows() != NX || x.cols() != NSTEPS + 1 || u.rows() != NU || u.cols() != NSTEPS)
    {
        printf("Results matrices are not correct.\n");
        return;
    }
    // Copy the solution to the Eigen matrices
    x = Eigen::Map<Eigen::MatrixXd>(xtraj_, NX, NSTEPS + 1);
    u = Eigen::Map<Eigen::MatrixXd>(utraj_, NU, NSTEPS);
}

void AcadosPerceptionWrapper::print_results()
{
    printf("\n--- xtraj ---\n");
    d_print_exp_tran_mat(NX, N_ + 1, xtraj_, NX);
    printf("\n--- utraj ---\n");
    d_print_exp_tran_mat(NU, N_, utraj_, NU);
    // ocp_nlp_out_print(nlp_solver->dims, nlp_out);

    printf("\nsolved ocp %d times, solution printed above\n\n", 1);
    int NTIMINGS = 1;
    double kkt_norm_inf;
    double elapsed_time;
    int sqp_iter;
    // get solution
    ocp_nlp_get(nlp_solver_, "time_tot", &elapsed_time);
    ocp_nlp_out_get(nlp_config_,nlp_dims_, nlp_out_, 0, "kkt_norm_inf", &kkt_norm_inf);
    ocp_nlp_get(nlp_solver_, "sqp_iter", &sqp_iter);
        printf("\nSolver info:\n");
    printf(" SQP iterations %2d\n minimum time for %d solve %f [ms]\n KKT %e\n",
           sqp_iter, NTIMINGS, elapsed_time*1000, kkt_norm_inf);
    printf("\nSolver info:\n");
    
}