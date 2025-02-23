#pragma once
// standard
#include <stdio.h>
#include <stdlib.h>
// acados
#include "acados/utils/print.h"
#include "acados/utils/math.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"
#include "./c_generated_code/acados_solver_quad_point_2d_with_yaw.h"

// blasfeo
#include "blasfeo/include/blasfeo_d_aux_ext_dep.h"
#include "Eigen/Dense"
#include "ocp_nlp/ocp_nlp_common.h"

#define NX     QUAD_POINT_2D_WITH_YAW_NX
#define NP     QUAD_POINT_2D_WITH_YAW_NP
#define NU     QUAD_POINT_2D_WITH_YAW_NU
#define NBX0   QUAD_POINT_2D_WITH_YAW_NBX0
#define NP_GLOBAL   QUAD_POINT_2D_WITH_YAW_NP_GLOBAL
#define NH     QUAD_POINT_2D_WITH_YAW_NH
#define NY     NX + NU
#define NSTEPS 20


class AcadosPerceptionWrapper{
    public:
        AcadosPerceptionWrapper(int N);
        ~AcadosPerceptionWrapper();
        int solve();  // Function to solve the optimal control problem
        void set_initial_conditions(const Eigen::VectorXd& x_init, const Eigen::VectorXd& u0);  // Function to set initial conditions
        void set_reference_trajectory(const Eigen::MatrixXd& ref_traj);  // Function to set the reference trajectory
        void set_params(const Eigen::VectorXd& p);  // Set parameters for the solver
        void set_control_bounds(const Eigen::VectorXd& lbu, const Eigen::VectorXd& ubu);  // Set control bounds
        void set_cost_weights(const Eigen::VectorXd& Q, const Eigen::VectorXd& R);  // Set cost weights
        void set_cost_weights_end(const Eigen::VectorXd& Q);  // Set cost weights for the final state
        void set_cost_slack_weights(const double& zl, const double& Zl, const double& zu, const double& Zu);  // Set cost weights for the slack variables
        void set_cost_slack_begin_weights(const double& zl_0, const double& Zl_0, const double& zu_0, const double& Zu_0);  // Set cost weights for the slack variables at the beginning
        void set_cost_slack_end_weights(const double& zl_N, const double& Zl_N, const double& zu_N, const double& Zu_N);  // Set cost weights for the slack variables at the beginning
        void get_results(Eigen::MatrixXd& x, Eigen::MatrixXd& u);  // Get the results of the optimization 
        void print_results();  // Print solution after optimization
    private:
        // acados capsule and solver objects
        quad_point_2d_with_yaw_solver_capsule *acados_ocp_capsule_;
        ocp_nlp_config  *nlp_config_;
        ocp_nlp_dims *nlp_dims_;
        ocp_nlp_in *nlp_in_;
        ocp_nlp_out *nlp_out_;
        ocp_nlp_solver *nlp_solver_;
        void *nlp_opts_;
        int N_;  // Number of shooting points

        // Initial state and control input values
        double x_init_[NX];
        double u0_[NU];
        Eigen::MatrixXd ref_traj_;  // Reference trajectory
        Eigen::VectorXd p_;  // Parameters for the solver p_[2] [3] are the reference point
        double xtraj_[NX * (NSTEPS + 1)];
        double utraj_[NU * NSTEPS];


        // constraints
        double lbu_[NU];
        double ubu_[NU];
        
        // cost matrix
        Eigen::Matrix<double, NX, NX> Q_;
        Eigen::Matrix<double, NU, NU> R_;
        Eigen::Matrix<double, NY, NY> W_;
        Eigen::Matrix<double, NX, NX> W_end_;

};

