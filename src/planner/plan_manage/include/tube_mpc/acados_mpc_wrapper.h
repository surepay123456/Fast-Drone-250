#ifndef ACADOS_MPC_WRAPPER_H
#define ACADOS_MPC_WRAPPER_H

// standard
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
// acados
// #include "Eigen/src/Core/Matrix.h"
// #include "Eigen/src/Core/Ref.h"
// #include "Eigen/src/Core/Matrix.h"
// #include "Eigen/src/Core/Ref.h"
// #include "Eigen/src/Core/Matrix.h"
// #include "Eigen/src/Core/Matrix.h"
// #include "Eigen/src/Core/Matrix.h"
#include "acados/utils/print.h"
#include "acados/utils/math.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"
#include "acados_solver_augmented_yaw_model.h"

// blasfeo
#include "blasfeo/include/blasfeo_d_aux_ext_dep.h"

#include "Eigen/Dense"
// #include "Eigen/"
// NP 就是参数列表 NP_GLOBAL就是全局的参数
#define NX     AUGMENTED_YAW_MODEL_NX // 10
#define NP     AUGMENTED_YAW_MODEL_NP // 5
#define NU     AUGMENTED_YAW_MODEL_NU // 3
#define NY     AUGMENTED_YAW_MODEL_NY // 10
#define NBX0   AUGMENTED_YAW_MODEL_NBX0 // 10
#define NP_GLOBAL   AUGMENTED_YAW_MODEL_NP_GLOBAL
#define NX_CURRENT 7

#define NSTEPS 20

class AcadosMpcWrapper
{
    public:
    //  Q : 7 * 7 , R : 3 * 3
        AcadosMpcWrapper(const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R,
                      double max_jerk, double max_w, double new_time_step);
        ~AcadosMpcWrapper();
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW


        //  not use RTI 
        double solve(Eigen::VectorXd& state);

        void get_traj_from_solver();
        void GetControls(Eigen::Ref<Eigen::MatrixXd> u_traj);
        void GetControl(Eigen::VectorXd& control);
        void GetStates(Eigen::Ref<Eigen::MatrixXd> x_traj);


        void SetIntialState(const Eigen::VectorXd& state);
        void SetIntialStateTraj(Eigen::Ref<Eigen::MatrixXd> state_traj);
        // reference setter
        // 前7个当前状态 + 后3个控制量
        void SetRefTraj(Eigen::Ref<Eigen::MatrixXd> traj);
        void SetRefPose(Eigen::VectorXd& pose){
            Eigen::MatrixXd traj = pose.replicate(1, NSTEPS);
            SetRefTraj(traj);
        }

        void SetControlConstraints(Eigen::VectorXd& constraints_u);
        void SetCostMatrix(const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R);
        int update_params(const Eigen::VectorXd& p);
        int update_params_at_t(const Eigen::VectorXd& p, int stage);

        // control setter
        // void SetIntialControl(const Eigen::VectorXd& control){
        //     acados_init_control_ = control;
        // }
        // void SetIntialControlTraj(Eigen::Ref<Eigen::MatrixXd> control_traj){
        //     acados_utraj_ = control_traj;
        // }



    private:
        // some classes to interact with acados
        augmented_yaw_model_solver_capsule *acados_ocp_capsule_;
        ocp_nlp_config *acados_nlp_config;
        ocp_nlp_dims *acados_nlp_dims;
        ocp_nlp_in *acados_nlp_in;
        ocp_nlp_out *acados_nlp_out;
        ocp_nlp_solver *acados_nlp_solver;
        void *acados_nlp_opts;
        double t_consume_;

        // data used to interact with ocp inside
        double Acados_x_init_[NX];
        double Acados_u_init_[NU];
        double Acados_y_ref_[NY * NSTEPS];
        double Acados_params_[NP * NSTEPS];

        double Acados_xtraj_[NX * (NSTEPS + 1)];
        double Acados_utraj_[NU * NSTEPS];

        double Acados_cost_matrix_[NY * NY]; // NY = NX_CURRENT + NU
        double Acados_cost_matrix_end_[NX_CURRENT * NX_CURRENT];

        // data used to interact with outside
        // before solve, data requried
        Eigen::Map<Eigen::Matrix<double, NX, 1>> acados_init_state_{Acados_x_init_};
        Eigen::Map<Eigen::Matrix<double, NU, 1>> acados_init_control_{Acados_u_init_};
        Eigen::Map<Eigen::Matrix<double, NP, NSTEPS>> acados_params_{Acados_params_};
        Eigen::Map<Eigen::Matrix<double, NY, NSTEPS>> acados_yref_{Acados_y_ref_};

        Eigen::Map<Eigen::Matrix<double, NY, NY>> acados_cost_matrix_{Acados_cost_matrix_};
        Eigen::Map<Eigen::Matrix<double, NX_CURRENT, NX_CURRENT>> acados_cost_matrix_end{Acados_cost_matrix_end_};

        // after solve, data get, also can be set
        Eigen::Map<Eigen::Matrix<double, NX, NSTEPS + 1>> acados_xtraj_{Acados_xtraj_};
        Eigen::Map<Eigen::Matrix<double, NU, NSTEPS>> acados_utraj_{Acados_utraj_};
};

#endif