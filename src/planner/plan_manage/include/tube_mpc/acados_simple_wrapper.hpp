// File: acados_simple_wrapper.hpp
#pragma once
// standard
// #include <cmath>
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
#include "ocp_nlp/ocp_nlp_common.h"

#define NX     AUGMENTED_YAW_MODEL_NX
#define NP     AUGMENTED_YAW_MODEL_NP
#define NU     AUGMENTED_YAW_MODEL_NU
#define NBX0   AUGMENTED_YAW_MODEL_NBX0
#define NP_GLOBAL   AUGMENTED_YAW_MODEL_NP_GLOBAL
#define M_PI 3.14159265358979323846


#define NX_CURRENT 7
#define NY  10
#define NSTEPS 20

class AcadosSimpleWrapper
{
    public:
        AcadosSimpleWrapper(int N);
        ~AcadosSimpleWrapper();
        int solve();  // Function to solve the optimal control problem
        void set_initial_conditions(const Eigen::VectorXd& x_init, const Eigen::VectorXd& u0);  // Function to set initial conditions
        void set_reference_trajectory(const Eigen::MatrixXd& ref_traj);  // Function to set the reference trajectory
        void set_params(const Eigen::VectorXd& p);  // Set parameters for the solver
        void get_results(Eigen::MatrixXd& x, Eigen::MatrixXd& u);  // Get the results of the optimization 
        void print_results();  // Print solution after optimization
    private:
        // acados capsule and solver objects
        augmented_yaw_model_solver_capsule *acados_ocp_capsule_;
        ocp_nlp_config  *nlp_config_;
        ocp_nlp_dims *nlp_dims_;
        ocp_nlp_in *nlp_in_;
        ocp_nlp_out *nlp_out_;
        ocp_nlp_solver *nlp_solver_;
        void *nlp_opts_;
        int N_;  // Number of shooting points

        // Initial state and control input values
        double* x_init_;
        double* u0_;
        Eigen::MatrixXd ref_traj_;  // Reference trajectory
        Eigen::VectorXd p_;  // Parameters for the solver
        double xtraj_[NX * (NSTEPS + 1)];
        double utraj_[NU * NSTEPS];
};



