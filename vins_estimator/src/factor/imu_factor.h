#pragma once
#include <ros/assert.h>
#include <iostream>
#include <eigen3/Eigen/Dense>

#include "../utility/utility.h"
#include "../parameters.h"
#include "integration_base.h"

#include <ceres/ceres.h>

class IMUFactor : public ceres::SizedCostFunction<15, 7, 9, 7, 9>    // imu预积分残差15维(PVQB -> 5x3=15)；7是位姿自由度，速度和零偏共9自由度(两帧)
{
  public:
    IMUFactor() = delete;   // ! 删除默认构造函数，说明有成员变量不可默认初始化
    IMUFactor(IntegrationBase* _pre_integration):pre_integration(_pre_integration)
    {
    }
    /**
     * @brief  // ! 使用ceres解析求导，必须重载这个函数，而真正的求jacobian的evaluate()函数在IntegrationBase中
     * 
     * @param[in] parameters 这是一个二维数组，每个参数块都是一个double数组，而一个观测会对多个参数块形成约束
     * @param[in] residuals 残差的计算结果，是一个一维数组，残差就是该观测量和约束的状态量通过某种关系形成残差
     * @param[in] jacobians 残差对参数块的雅克比矩阵，这也是一个二维数组，对任意一个参数块的雅克比矩阵都是一个一维数组
     * @return true 
     * @return false 
     */
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
    {
        // 便于后续计算，把参数块都转换成eigen
        // imu预积分的约束的参数是相邻两帧的位姿 速度和零偏 
        // ! PVQ、bias，这样看残差应该是16维，是因为此时的位姿用的是四元数，但是在预积分里用的是旋转向量，所以变成了15维
        Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);    // 位姿7自由度
        Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

        Eigen::Vector3d Vi(parameters[1][0], parameters[1][1], parameters[1][2]);  // 速度和零偏共9自由度
        Eigen::Vector3d Bai(parameters[1][3], parameters[1][4], parameters[1][5]);
        Eigen::Vector3d Bgi(parameters[1][6], parameters[1][7], parameters[1][8]);

        Eigen::Vector3d Pj(parameters[2][0], parameters[2][1], parameters[2][2]);
        Eigen::Quaterniond Qj(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);

        Eigen::Vector3d Vj(parameters[3][0], parameters[3][1], parameters[3][2]);
        Eigen::Vector3d Baj(parameters[3][3], parameters[3][4], parameters[3][5]);
        Eigen::Vector3d Bgj(parameters[3][6], parameters[3][7], parameters[3][8]);

//Eigen::Matrix<double, 15, 15> Fd;
//Eigen::Matrix<double, 15, 12> Gd;

//Eigen::Vector3d pPj = Pi + Vi * sum_t - 0.5 * g * sum_t * sum_t + corrected_delta_p;
//Eigen::Quaterniond pQj = Qi * delta_q;
//Eigen::Vector3d pVj = Vi - g * sum_t + corrected_delta_v;
//Eigen::Vector3d pBaj = Bai;
//Eigen::Vector3d pBgj = Bgi;

//Vi + Qi * delta_v - g * sum_dt = Vj;
//Qi * delta_q = Qj;

//delta_p = Qi.inverse() * (0.5 * g * sum_dt * sum_dt + Pj - Pi);
//delta_v = Qi.inverse() * (g * sum_dt + Vj - Vi);
//delta_q = Qi.inverse() * Qj;

#if 0
        if ((Bai - pre_integration->linearized_ba).norm() > 0.10 ||
            (Bgi - pre_integration->linearized_bg).norm() > 0.01)
        {
            pre_integration->repropagate(Bai, Bgi);
        }
#endif

        Eigen::Map<Eigen::Matrix<double, 15, 1>> residual(residuals);   // ! eigen映射相当于引用
        // 得到残差
        residual = pre_integration->evaluate(Pi, Qi, Vi, Bai, Bgi,
                                            Pj, Qj, Vj, Baj, Bgj);
        // ! 因为ceres没有g2o设置信息矩阵的接口，因此置信度直接乘在残差上，这里通过LLT分解，相当于将信息矩阵开根号
        Eigen::Matrix<double, 15, 15> sqrt_info = Eigen::LLT<Eigen::Matrix<double, 15, 15>>(pre_integration->covariance.inverse()).matrixL().transpose();
        //sqrt_info.setIdentity();
        // 这就是带有信息矩阵的残差
        residual = sqrt_info * residual;   // 应为eT * P * e，如果没有P则证明完全置信，先将P进行LLT分解得，eT * L * LT * e，则新的残差为LT * e
        // 关于雅克比的计算手动推导一下
        if (jacobians)
        {
            double sum_dt = pre_integration->sum_dt;
            Eigen::Matrix3d dp_dba = pre_integration->jacobian.template block<3, 3>(O_P, O_BA);
            Eigen::Matrix3d dp_dbg = pre_integration->jacobian.template block<3, 3>(O_P, O_BG);

            Eigen::Matrix3d dq_dbg = pre_integration->jacobian.template block<3, 3>(O_R, O_BG);

            Eigen::Matrix3d dv_dba = pre_integration->jacobian.template block<3, 3>(O_V, O_BA);
            Eigen::Matrix3d dv_dbg = pre_integration->jacobian.template block<3, 3>(O_V, O_BG);

            if (pre_integration->jacobian.maxCoeff() > 1e8 || pre_integration->jacobian.minCoeff() < -1e8)
            {
                ROS_WARN("numerical unstable in preintegration");
                //std::cout << pre_integration->jacobian << std::endl;
///                ROS_BREAK();
            }

            //!  jacobian:
            //  |                 15x7                                15x9                                      15x7                                             15x9                         |
            //  | |   偏e / 偏[Pk, Qk]   | |   偏e / 偏[Vk, biask]   | |    偏e / 偏[Pk+1, Qk+1]   | |   偏e / 偏[Vk+1, biask+1]   | |

            // ! 把雅克比分块
            if (jacobians[0])
            {
                Eigen::Map<Eigen::Matrix<double, 15, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);   // eigen map 映射，默认列优先，但是行优先更快
                jacobian_pose_i.setZero();

                // 这里存在Q取逆，是因为Q维护的是Imu相对于世界，而公式里对于P的误差里，前面乘了一个世界相对于imu的变换
                // block<3, 3>(O_P, O_P)  代表残差P对Pk求导，维数是3x3
                jacobian_pose_i.block<3, 3>(O_P, O_P) = -Qi.inverse().toRotationMatrix();    // ok
                jacobian_pose_i.block<3, 3>(O_P, O_R) = Utility::skewSymmetric(Qi.inverse() * (0.5 * G * sum_dt * sum_dt + Pj - Pi - Vi * sum_dt));  // ok

#if 0
            jacobian_pose_i.block<3, 3>(O_R, O_R) = -(Qj.inverse() * Qi).toRotationMatrix();
#else
                Eigen::Quaterniond corrected_delta_q = pre_integration->delta_q * Utility::deltaQ(dq_dbg * (Bgi - pre_integration->linearized_bg));
                jacobian_pose_i.block<3, 3>(O_R, O_R) = -(Utility::Qleft(Qj.inverse() * Qi) * Utility::Qright(corrected_delta_q)).bottomRightCorner<3, 3>();  // 按照论文公式 ok
#endif

                jacobian_pose_i.block<3, 3>(O_V, O_R) = Utility::skewSymmetric(Qi.inverse() * (G * sum_dt + Vj - Vi));  // ok

                jacobian_pose_i = sqrt_info * jacobian_pose_i;

                if (jacobian_pose_i.maxCoeff() > 1e8 || jacobian_pose_i.minCoeff() < -1e8)
                {
                    ROS_WARN("numerical unstable in preintegration");
                    //std::cout << sqrt_info << std::endl;
                    //ROS_BREAK();
                }
            }
            if (jacobians[1])
            {
                Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor>> jacobian_speedbias_i(jacobians[1]);
                jacobian_speedbias_i.setZero();
                jacobian_speedbias_i.block<3, 3>(O_P, O_V - O_V) = -Qi.inverse().toRotationMatrix() * sum_dt;  // ok
                jacobian_speedbias_i.block<3, 3>(O_P, O_BA - O_V) = -dp_dba;  // ok 
                jacobian_speedbias_i.block<3, 3>(O_P, O_BG - O_V) = -dp_dbg;  // ok

#if 0
            jacobian_speedbias_i.block<3, 3>(O_R, O_BG - O_V) = -dq_dbg;
#else
                //Eigen::Quaterniond corrected_delta_q = pre_integration->delta_q * Utility::deltaQ(dq_dbg * (Bgi - pre_integration->linearized_bg));
                //jacobian_speedbias_i.block<3, 3>(O_R, O_BG - O_V) = -Utility::Qleft(Qj.inverse() * Qi * corrected_delta_q).bottomRightCorner<3, 3>() * dq_dbg;
                jacobian_speedbias_i.block<3, 3>(O_R, O_BG - O_V) = -Utility::Qleft(Qj.inverse() * Qi * pre_integration->delta_q).bottomRightCorner<3, 3>() * dq_dbg;  // ok
#endif

                jacobian_speedbias_i.block<3, 3>(O_V, O_V - O_V) = -Qi.inverse().toRotationMatrix();  // ok
                jacobian_speedbias_i.block<3, 3>(O_V, O_BA - O_V) = -dv_dba;  // ok 
                jacobian_speedbias_i.block<3, 3>(O_V, O_BG - O_V) = -dv_dbg;  // ok

                jacobian_speedbias_i.block<3, 3>(O_BA, O_BA - O_V) = -Eigen::Matrix3d::Identity();  // ok

                jacobian_speedbias_i.block<3, 3>(O_BG, O_BG - O_V) = -Eigen::Matrix3d::Identity();  // ok

                jacobian_speedbias_i = sqrt_info * jacobian_speedbias_i;  // ok

                //ROS_ASSERT(fabs(jacobian_speedbias_i.maxCoeff()) < 1e8);
                //ROS_ASSERT(fabs(jacobian_speedbias_i.minCoeff()) < 1e8);
            }
            if (jacobians[2])
            {
                Eigen::Map<Eigen::Matrix<double, 15, 7, Eigen::RowMajor>> jacobian_pose_j(jacobians[2]);
                jacobian_pose_j.setZero();

                jacobian_pose_j.block<3, 3>(O_P, O_P) = Qi.inverse().toRotationMatrix();  // ok

#if 0
            jacobian_pose_j.block<3, 3>(O_R, O_R) = Eigen::Matrix3d::Identity();
#else
                Eigen::Quaterniond corrected_delta_q = pre_integration->delta_q * Utility::deltaQ(dq_dbg * (Bgi - pre_integration->linearized_bg));  // ok
                jacobian_pose_j.block<3, 3>(O_R, O_R) = Utility::Qleft(corrected_delta_q.inverse() * Qi.inverse() * Qj).bottomRightCorner<3, 3>();  // ok
#endif

                jacobian_pose_j = sqrt_info * jacobian_pose_j;

                //ROS_ASSERT(fabs(jacobian_pose_j.maxCoeff()) < 1e8);
                //ROS_ASSERT(fabs(jacobian_pose_j.minCoeff()) < 1e8);
            }
            if (jacobians[3])   // 零偏与k+1时刻无关
            {
                Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor>> jacobian_speedbias_j(jacobians[3]);
                jacobian_speedbias_j.setZero();

                jacobian_speedbias_j.block<3, 3>(O_V, O_V - O_V) = Qi.inverse().toRotationMatrix();  // ok

                jacobian_speedbias_j.block<3, 3>(O_BA, O_BA - O_V) = Eigen::Matrix3d::Identity();  // ok

                jacobian_speedbias_j.block<3, 3>(O_BG, O_BG - O_V) = Eigen::Matrix3d::Identity();  // ok

                jacobian_speedbias_j = sqrt_info * jacobian_speedbias_j;

                //ROS_ASSERT(fabs(jacobian_speedbias_j.maxCoeff()) < 1e8);
                //ROS_ASSERT(fabs(jacobian_speedbias_j.minCoeff()) < 1e8);
            }
        }

        return true;
    }

    //bool Evaluate_Direct(double const *const *parameters, Eigen::Matrix<double, 15, 1> &residuals, Eigen::Matrix<double, 15, 30> &jacobians);

    //void checkCorrection();
    //void checkTransition();
    //void checkJacobian(double **parameters);
    IntegrationBase* pre_integration;

};

