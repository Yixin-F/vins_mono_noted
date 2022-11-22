#pragma once

#include "../utility/utility.h"
#include "../parameters.h"

#include <ceres/ceres.h>
using namespace Eigen;

// ! 预积分的所有操作都在IntegrationBase里
// ! 前向积分-中值积分；协方差和雅克比更新
class IntegrationBase
{
  public:
    IntegrationBase() = delete;

    // ! 预积分构造函数 
    // 构造函数输入是初始时刻加速度、角速度、加速度零偏、角速度零偏
    // 雅克比赋值单位阵，15X15，其实只需要维护delta_p(alpha)(3)、delta_v(beta)(3)、delta_q(gamma)(3)(旋转向量形式)对两个零偏(6)的雅克比(导数)，但是这里直接维护15X15，一开始谁也与谁无关，所以初始化为单位阵
    // 协方差初始化为零矩阵，delta_p和delta_v初始化为0，delta_q初始化为单位四元数
    // 噪声noise 18X18，通过yaml来初始化，关于上一时刻加速度计(3)、陀螺仪噪声(3)，下一时刻加速度计(3)、陀螺仪噪声(3)，两个零偏噪声(6)(随机游走)
    IntegrationBase(const Eigen::Vector3d &_acc_0, const Eigen::Vector3d &_gyr_0,
                    const Eigen::Vector3d &_linearized_ba, const Eigen::Vector3d &_linearized_bg)
        : acc_0{_acc_0}, gyr_0{_gyr_0}, linearized_acc{_acc_0}, linearized_gyr{_gyr_0},
          linearized_ba{_linearized_ba}, linearized_bg{_linearized_bg},
            jacobian{Eigen::Matrix<double, 15, 15>::Identity()}, covariance{Eigen::Matrix<double, 15, 15>::Zero()},
          sum_dt{0.0}, delta_p{Eigen::Vector3d::Zero()}, delta_q{Eigen::Quaterniond::Identity()}, delta_v{Eigen::Vector3d::Zero()}

    {
        noise = Eigen::Matrix<double, 18, 18>::Zero();
        noise.block<3, 3>(0, 0) =  (ACC_N * ACC_N) * Eigen::Matrix3d::Identity();
        noise.block<3, 3>(3, 3) =  (GYR_N * GYR_N) * Eigen::Matrix3d::Identity();
        noise.block<3, 3>(6, 6) =  (ACC_N * ACC_N) * Eigen::Matrix3d::Identity();
        noise.block<3, 3>(9, 9) =  (GYR_N * GYR_N) * Eigen::Matrix3d::Identity();
        noise.block<3, 3>(12, 12) =  (ACC_W * ACC_W) * Eigen::Matrix3d::Identity();
        noise.block<3, 3>(15, 15) =  (GYR_W * GYR_W) * Eigen::Matrix3d::Identity();
    }

    //  ! 来一帧新的imu数据，存储时间、加速度计读数、陀螺仪读数，并且同时已经完成了Imu的预积分工作
    void push_back(double dt, const Eigen::Vector3d &acc, const Eigen::Vector3d &gyr)
    {
        // 相关时间差和传感器数据保留，方便后续repropagate，重新传播发生在vio初始化当中
        dt_buf.push_back(dt);
        acc_buf.push_back(acc);
        gyr_buf.push_back(gyr);
        propagate(dt, acc, gyr);   // ! 最基本的一次预积分
    }

    /**
     * @brief 根据新设置的imu零偏重新对该帧进行预积分
     * 
     * @param[in] _linearized_ba 
     * @param[in] _linearized_bg 
     */
    void repropagate(const Eigen::Vector3d &_linearized_ba, const Eigen::Vector3d &_linearized_bg)
    {
        // 状态量全部清零
        sum_dt = 0.0;
        acc_0 = linearized_acc;
        gyr_0 = linearized_gyr;
        delta_p.setZero();
        delta_q.setIdentity();
        delta_v.setZero();
        // 赋上设置的零偏值
        linearized_ba = _linearized_ba;
        linearized_bg = _linearized_bg;
        jacobian.setIdentity();
        covariance.setZero();
        // 用之前存下来的imu值重新预积分
        for (int i = 0; i < static_cast<int>(dt_buf.size()); i++)
            propagate(dt_buf[i], acc_buf[i], gyr_buf[i]);
    }

    // ! main
    // ! 中值积分用于离散化后相邻两帧imu之间的预积分，ESKF仅用于协方差和雅克比的更新
    // 时间，上一帧加速度计读数、陀螺仪读数，当前帧加速度计读数、陀螺仪读数，上一帧5个优化量，下一帧5个优化量，雅克比更新标志
    void midPointIntegration(double _dt, 
                            const Eigen::Vector3d &_acc_0, const Eigen::Vector3d &_gyr_0,
                            const Eigen::Vector3d &_acc_1, const Eigen::Vector3d &_gyr_1,
                            const Eigen::Vector3d &delta_p, const Eigen::Quaterniond &delta_q, const Eigen::Vector3d &delta_v,
                            const Eigen::Vector3d &linearized_ba, const Eigen::Vector3d &linearized_bg,
                            Eigen::Vector3d &result_delta_p, Eigen::Quaterniond &result_delta_q, Eigen::Vector3d &result_delta_v,
                            Eigen::Vector3d &result_linearized_ba, Eigen::Vector3d &result_linearized_bg, bool update_jacobian)
    {
        //ROS_INFO("midpoint integration");

        // Step 1 首先相邻两帧Imu之间中值积分更新状态量
        // |---------------|       “|”代表图像帧，“-”代表imu帧
        Vector3d un_acc_0 = delta_q * (_acc_0 - linearized_ba);  // ? 转到开始图像帧坐标系下，为什么看起来“乘以位姿应该是回到世界坐标系？”，实际上每两图像帧之间都各自的预积分类！
        Vector3d un_gyr = 0.5 * (_gyr_0 + _gyr_1) - linearized_bg;  // 陀螺仪中值
        result_delta_q = delta_q * Quaterniond(1, un_gyr(0) * _dt / 2, un_gyr(1) * _dt / 2, un_gyr(2) * _dt / 2);  // 后面一个四元数的构建，实部+3个虚部，里面有个delta_t里陀螺仪读数使得近似
        Vector3d un_acc_1 = result_delta_q * (_acc_1 - linearized_ba);
        Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);  // 加速度计中值
        result_delta_p = delta_p + delta_v * _dt + 0.5 * un_acc * _dt * _dt;
        result_delta_v = delta_v + un_acc * _dt;
        result_linearized_ba = linearized_ba;  // 零偏是假设不变的
        result_linearized_bg = linearized_bg;     

        // Step 2 随后更新方差矩阵及雅克比
        if(update_jacobian)
        {
            Vector3d w_x = 0.5 * (_gyr_0 + _gyr_1) - linearized_bg;
            Vector3d a_0_x = _acc_0 - linearized_ba;
            Vector3d a_1_x = _acc_1 - linearized_ba;
            Matrix3d R_w_x, R_a_0_x, R_a_1_x;

            // 事先计算了反对称矩阵
            R_w_x<<0, -w_x(2), w_x(1),
                w_x(2), 0, -w_x(0),
                -w_x(1), w_x(0), 0;
            R_a_0_x<<0, -a_0_x(2), a_0_x(1),
                a_0_x(2), 0, -a_0_x(0),
                -a_0_x(1), a_0_x(0), 0;
            R_a_1_x<<0, -a_1_x(2), a_1_x(1),
                a_1_x(2), 0, -a_1_x(0),
                -a_1_x(1), a_1_x(0), 0;

            // F矩阵的填充
            MatrixXd F = MatrixXd::Zero(15, 15);
            F.block<3, 3>(0, 0) = Matrix3d::Identity();
            F.block<3, 3>(0, 3) = -0.25 * delta_q.toRotationMatrix() * R_a_0_x * _dt * _dt + 
                                  -0.25 * result_delta_q.toRotationMatrix() * R_a_1_x * (Matrix3d::Identity() - R_w_x * _dt) * _dt * _dt;
            F.block<3, 3>(0, 6) = MatrixXd::Identity(3,3) * _dt;
            F.block<3, 3>(0, 9) = -0.25 * (delta_q.toRotationMatrix() + result_delta_q.toRotationMatrix()) * _dt * _dt;
            F.block<3, 3>(0, 12) = -0.25 * result_delta_q.toRotationMatrix() * R_a_1_x * _dt * _dt * -_dt;
            F.block<3, 3>(3, 3) = Matrix3d::Identity() - R_w_x * _dt;
            F.block<3, 3>(3, 12) = -1.0 * MatrixXd::Identity(3,3) * _dt;
            F.block<3, 3>(6, 3) = -0.5 * delta_q.toRotationMatrix() * R_a_0_x * _dt + 
                                  -0.5 * result_delta_q.toRotationMatrix() * R_a_1_x * (Matrix3d::Identity() - R_w_x * _dt) * _dt;
            F.block<3, 3>(6, 6) = Matrix3d::Identity();
            F.block<3, 3>(6, 9) = -0.5 * (delta_q.toRotationMatrix() + result_delta_q.toRotationMatrix()) * _dt;
            F.block<3, 3>(6, 12) = -0.5 * result_delta_q.toRotationMatrix() * R_a_1_x * _dt * -_dt;
            F.block<3, 3>(9, 9) = Matrix3d::Identity();
            F.block<3, 3>(12, 12) = Matrix3d::Identity();
            //cout<<"A"<<endl<<A<<endl;

            // V矩阵的填充，与之前的推导相差个负号，但是对白噪声来讲毫无影响
            MatrixXd V = MatrixXd::Zero(15,18);
            V.block<3, 3>(0, 0) =  0.25 * delta_q.toRotationMatrix() * _dt * _dt;
            V.block<3, 3>(0, 3) =  0.25 * -result_delta_q.toRotationMatrix() * R_a_1_x  * _dt * _dt * 0.5 * _dt;
            V.block<3, 3>(0, 6) =  0.25 * result_delta_q.toRotationMatrix() * _dt * _dt;
            V.block<3, 3>(0, 9) =  V.block<3, 3>(0, 3);
            V.block<3, 3>(3, 3) =  0.5 * MatrixXd::Identity(3,3) * _dt;
            V.block<3, 3>(3, 9) =  0.5 * MatrixXd::Identity(3,3) * _dt;
            V.block<3, 3>(6, 0) =  0.5 * delta_q.toRotationMatrix() * _dt;
            V.block<3, 3>(6, 3) =  0.5 * -result_delta_q.toRotationMatrix() * R_a_1_x  * _dt * 0.5 * _dt;
            V.block<3, 3>(6, 6) =  0.5 * result_delta_q.toRotationMatrix() * _dt;
            V.block<3, 3>(6, 9) =  V.block<3, 3>(6, 3);
            V.block<3, 3>(9, 12) = MatrixXd::Identity(3,3) * _dt;
            V.block<3, 3>(12, 15) = MatrixXd::Identity(3,3) * _dt;

            // Step 3 更新雅克比和协方差
            //step_jacobian = F;
            //step_V = V;
            jacobian = F * jacobian;  // 用于bias更新后的补偿
            covariance = F * covariance * F.transpose() + V * noise * V.transpose();   // 基于高斯分布的协方差更新
        }

    }

    void propagate(double _dt, const Eigen::Vector3d &_acc_1, const Eigen::Vector3d &_gyr_1)
    {
        dt = _dt;
        acc_1 = _acc_1;
        gyr_1 = _gyr_1;

        // ! 下面是优化的5个量
        Vector3d result_delta_p;
        Quaterniond result_delta_q;
        Vector3d result_delta_v;
        Vector3d result_linearized_ba;
        Vector3d result_linearized_bg;

        midPointIntegration(_dt, acc_0, gyr_0, _acc_1, _gyr_1, delta_p, delta_q, delta_v,
                            linearized_ba, linearized_bg,
                            result_delta_p, result_delta_q, result_delta_v,
                            result_linearized_ba, result_linearized_bg, 1);

        //checkJacobian(_dt, acc_0, gyr_0, acc_1, gyr_1, delta_p, delta_q, delta_v,
        //                    linearized_ba, linearized_bg);

        // 当前帧Imu变成上一帧
        delta_p = result_delta_p;
        delta_q = result_delta_q;
        delta_v = result_delta_v;
        linearized_ba = result_linearized_ba;
        linearized_bg = result_linearized_bg;
        delta_q.normalize();
        sum_dt += dt;
        acc_0 = acc_1;
        gyr_0 = gyr_1;  
     
    }

    // ! imu计算和给定相邻帧状态量的残差，作为帧间约束；这里将残差和ESKF联系起来了？
    // ! 这里所有的入参是imu和视觉重投影的整体优化值，如果系统最优应该是与单纯的imu预积分值接近，即前后两者残差为0
    Eigen::Matrix<double, 15, 1> evaluate(const Eigen::Vector3d &Pi, const Eigen::Quaterniond &Qi, const Eigen::Vector3d &Vi, const Eigen::Vector3d &Bai, const Eigen::Vector3d &Bgi,
                                          const Eigen::Vector3d &Pj, const Eigen::Quaterniond &Qj, const Eigen::Vector3d &Vj, const Eigen::Vector3d &Baj, const Eigen::Vector3d &Bgj)
    {
        Eigen::Matrix<double, 15, 1> residuals;

        Eigen::Matrix3d dp_dba = jacobian.block<3, 3>(O_P, O_BA);
        Eigen::Matrix3d dp_dbg = jacobian.block<3, 3>(O_P, O_BG);

        Eigen::Matrix3d dq_dbg = jacobian.block<3, 3>(O_R, O_BG);

        Eigen::Matrix3d dv_dba = jacobian.block<3, 3>(O_V, O_BA);
        Eigen::Matrix3d dv_dbg = jacobian.block<3, 3>(O_V, O_BG);

        Eigen::Vector3d dba = Bai - linearized_ba;
        Eigen::Vector3d dbg = Bgi - linearized_bg;

        Eigen::Quaterniond corrected_delta_q = delta_q * Utility::deltaQ(dq_dbg * dbg);    // 这边是 new = old + jaco * delta，仅考虑了bias对imu的影响
        Eigen::Vector3d corrected_delta_v = delta_v + dv_dba * dba + dv_dbg * dbg;
        Eigen::Vector3d corrected_delta_p = delta_p + dp_dba * dba + dp_dbg * dbg;

        residuals.block<3, 1>(O_P, 0) = Qi.inverse() * (0.5 * G * sum_dt * sum_dt + Pj - Pi - Vi * sum_dt) - corrected_delta_p;
        residuals.block<3, 1>(O_R, 0) = 2 * (corrected_delta_q.inverse() * (Qi.inverse() * Qj)).vec();
        residuals.block<3, 1>(O_V, 0) = Qi.inverse() * (G * sum_dt + Vj - Vi) - corrected_delta_v;
        residuals.block<3, 1>(O_BA, 0) = Baj - Bai;
        residuals.block<3, 1>(O_BG, 0) = Bgj - Bgi;
        return residuals;
    }

    double dt;
    Eigen::Vector3d acc_0, gyr_0;
    Eigen::Vector3d acc_1, gyr_1;

    const Eigen::Vector3d linearized_acc, linearized_gyr;
    Eigen::Vector3d linearized_ba, linearized_bg;

    Eigen::Matrix<double, 15, 15> jacobian, covariance;
    Eigen::Matrix<double, 15, 15> step_jacobian;
    Eigen::Matrix<double, 15, 18> step_V;
    Eigen::Matrix<double, 18, 18> noise;

    double sum_dt;
    Eigen::Vector3d delta_p;
    Eigen::Quaterniond delta_q;
    Eigen::Vector3d delta_v;

    std::vector<double> dt_buf;
    std::vector<Eigen::Vector3d> acc_buf;
    std::vector<Eigen::Vector3d> gyr_buf;

};

//     void eulerIntegration(double _dt, const Eigen::Vector3d &_acc_0, const Eigen::Vector3d &_gyr_0,
//                             const Eigen::Vector3d &_acc_1, const Eigen::Vector3d &_gyr_1,
//                             const Eigen::Vector3d &delta_p, const Eigen::Quaterniond &delta_q, const Eigen::Vector3d &delta_v,
//                             const Eigen::Vector3d &linearized_ba, const Eigen::Vector3d &linearized_bg,
//                             Eigen::Vector3d &result_delta_p, Eigen::Quaterniond &result_delta_q, Eigen::Vector3d &result_delta_v,
//                             Eigen::Vector3d &result_linearized_ba, Eigen::Vector3d &result_linearized_bg, bool update_jacobian)
//     {
//         result_delta_p = delta_p + delta_v * _dt + 0.5 * (delta_q * (_acc_1 - linearized_ba)) * _dt * _dt;
//         result_delta_v = delta_v + delta_q * (_acc_1 - linearized_ba) * _dt;
//         Vector3d omg = _gyr_1 - linearized_bg;
//         omg = omg * _dt / 2;
//         Quaterniond dR(1, omg(0), omg(1), omg(2));
//         result_delta_q = (delta_q * dR);   
//         result_linearized_ba = linearized_ba;
//         result_linearized_bg = linearized_bg;         

//         if(update_jacobian)
//         {
//             Vector3d w_x = _gyr_1 - linearized_bg;
//             Vector3d a_x = _acc_1 - linearized_ba;
//             Matrix3d R_w_x, R_a_x;

//             R_w_x<<0, -w_x(2), w_x(1),
//                 w_x(2), 0, -w_x(0),
//                 -w_x(1), w_x(0), 0;
//             R_a_x<<0, -a_x(2), a_x(1),
//                 a_x(2), 0, -a_x(0),
//                 -a_x(1), a_x(0), 0;

//             MatrixXd A = MatrixXd::Zero(15, 15);
//             // one step euler 0.5
//             A.block<3, 3>(0, 3) = 0.5 * (-1 * delta_q.toRotationMatrix()) * R_a_x * _dt;
//             A.block<3, 3>(0, 6) = MatrixXd::Identity(3,3);
//             A.block<3, 3>(0, 9) = 0.5 * (-1 * delta_q.toRotationMatrix()) * _dt;
//             A.block<3, 3>(3, 3) = -R_w_x;
//             A.block<3, 3>(3, 12) = -1 * MatrixXd::Identity(3,3);
//             A.block<3, 3>(6, 3) = (-1 * delta_q.toRotationMatrix()) * R_a_x;
//             A.block<3, 3>(6, 9) = (-1 * delta_q.toRotationMatrix());
//             //cout<<"A"<<endl<<A<<endl;

//             MatrixXd U = MatrixXd::Zero(15,12);
//             U.block<3, 3>(0, 0) =  0.5 * delta_q.toRotationMatrix() * _dt;
//             U.block<3, 3>(3, 3) =  MatrixXd::Identity(3,3);
//             U.block<3, 3>(6, 0) =  delta_q.toRotationMatrix();
//             U.block<3, 3>(9, 6) = MatrixXd::Identity(3,3);
//             U.block<3, 3>(12, 9) = MatrixXd::Identity(3,3);

//             // put outside
//             Eigen::Matrix<double, 12, 12> noise = Eigen::Matrix<double, 12, 12>::Zero();
//             noise.block<3, 3>(0, 0) =  (ACC_N * ACC_N) * Eigen::Matrix3d::Identity();
//             noise.block<3, 3>(3, 3) =  (GYR_N * GYR_N) * Eigen::Matrix3d::Identity();
//             noise.block<3, 3>(6, 6) =  (ACC_W * ACC_W) * Eigen::Matrix3d::Identity();
//             noise.block<3, 3>(9, 9) =  (GYR_W * GYR_W) * Eigen::Matrix3d::Identity();

//             //write F directly
//             MatrixXd F, V;
//             F = (MatrixXd::Identity(15,15) + _dt * A);
//             V = _dt * U;
//             step_jacobian = F;
//             step_V = V;
//             jacobian = F * jacobian;
//             covariance = F * covariance * F.transpose() + V * noise * V.transpose();
//         }

//     }     


//     void checkJacobian(double _dt, const Eigen::Vector3d &_acc_0, const Eigen::Vector3d &_gyr_0, 
//                                    const Eigen::Vector3d &_acc_1, const Eigen::Vector3d &_gyr_1,
//                             const Eigen::Vector3d &delta_p, const Eigen::Quaterniond &delta_q, const Eigen::Vector3d &delta_v,
//                             const Eigen::Vector3d &linearized_ba, const Eigen::Vector3d &linearized_bg)
//     {
//         Vector3d result_delta_p;
//         Quaterniond result_delta_q;
//         Vector3d result_delta_v;
//         Vector3d result_linearized_ba;
//         Vector3d result_linearized_bg;
//         midPointIntegration(_dt, _acc_0, _gyr_0, _acc_1, _gyr_1, delta_p, delta_q, delta_v,
//                             linearized_ba, linearized_bg,
//                             result_delta_p, result_delta_q, result_delta_v,
//                             result_linearized_ba, result_linearized_bg, 0);

//         Vector3d turb_delta_p;
//         Quaterniond turb_delta_q;
//         Vector3d turb_delta_v;
//         Vector3d turb_linearized_ba;
//         Vector3d turb_linearized_bg;

//         Vector3d turb(0.0001, -0.003, 0.003);

//         midPointIntegration(_dt, _acc_0, _gyr_0, _acc_1, _gyr_1, delta_p + turb, delta_q, delta_v,
//                             linearized_ba, linearized_bg,
//                             turb_delta_p, turb_delta_q, turb_delta_v,
//                             turb_linearized_ba, turb_linearized_bg, 0);
//         cout << "turb p       " << endl;
//         cout << "p diff       " << (turb_delta_p - result_delta_p).transpose() << endl;
//         cout << "p jacob diff " << (step_jacobian.block<3, 3>(0, 0) * turb).transpose() << endl;
//         cout << "q diff       " << ((result_delta_q.inverse() * turb_delta_q).vec() * 2).transpose() << endl;
//         cout << "q jacob diff " << (step_jacobian.block<3, 3>(3, 0) * turb).transpose() << endl;
//         cout << "v diff       " << (turb_delta_v - result_delta_v).transpose() << endl;
//         cout << "v jacob diff " << (step_jacobian.block<3, 3>(6, 0) * turb).transpose() << endl;
//         cout << "ba diff      " << (turb_linearized_ba - result_linearized_ba).transpose() << endl;
//         cout << "ba jacob diff" << (step_jacobian.block<3, 3>(9, 0) * turb).transpose() << endl;
//         cout << "bg diff " << (turb_linearized_bg - result_linearized_bg).transpose() << endl;
//         cout << "bg jacob diff " << (step_jacobian.block<3, 3>(12, 0) * turb).transpose() << endl;

//         midPointIntegration(_dt, _acc_0, _gyr_0, _acc_1, _gyr_1, delta_p, delta_q * Quaterniond(1, turb(0) / 2, turb(1) / 2, turb(2) / 2), delta_v,
//                             linearized_ba, linearized_bg,
//                             turb_delta_p, turb_delta_q, turb_delta_v,
//                             turb_linearized_ba, turb_linearized_bg, 0);
//         cout << "turb q       " << endl;
//         cout << "p diff       " << (turb_delta_p - result_delta_p).transpose() << endl;
//         cout << "p jacob diff " << (step_jacobian.block<3, 3>(0, 3) * turb).transpose() << endl;
//         cout << "q diff       " << ((result_delta_q.inverse() * turb_delta_q).vec() * 2).transpose() << endl;
//         cout << "q jacob diff " << (step_jacobian.block<3, 3>(3, 3) * turb).transpose() << endl;
//         cout << "v diff       " << (turb_delta_v - result_delta_v).transpose() << endl;
//         cout << "v jacob diff " << (step_jacobian.block<3, 3>(6, 3) * turb).transpose() << endl;
//         cout << "ba diff      " << (turb_linearized_ba - result_linearized_ba).transpose() << endl;
//         cout << "ba jacob diff" << (step_jacobian.block<3, 3>(9, 3) * turb).transpose() << endl;
//         cout << "bg diff      " << (turb_linearized_bg - result_linearized_bg).transpose() << endl;
//         cout << "bg jacob diff" << (step_jacobian.block<3, 3>(12, 3) * turb).transpose() << endl;

//         midPointIntegration(_dt, _acc_0, _gyr_0, _acc_1, _gyr_1, delta_p, delta_q, delta_v + turb,
//                             linearized_ba, linearized_bg,
//                             turb_delta_p, turb_delta_q, turb_delta_v,
//                             turb_linearized_ba, turb_linearized_bg, 0);
//         cout << "turb v       " << endl;
//         cout << "p diff       " << (turb_delta_p - result_delta_p).transpose() << endl;
//         cout << "p jacob diff " << (step_jacobian.block<3, 3>(0, 6) * turb).transpose() << endl;
//         cout << "q diff       " << ((result_delta_q.inverse() * turb_delta_q).vec() * 2).transpose() << endl;
//         cout << "q jacob diff " << (step_jacobian.block<3, 3>(3, 6) * turb).transpose() << endl;
//         cout << "v diff       " << (turb_delta_v - result_delta_v).transpose() << endl;
//         cout << "v jacob diff " << (step_jacobian.block<3, 3>(6, 6) * turb).transpose() << endl;
//         cout << "ba diff      " << (turb_linearized_ba - result_linearized_ba).transpose() << endl;
//         cout << "ba jacob diff" << (step_jacobian.block<3, 3>(9, 6) * turb).transpose() << endl;
//         cout << "bg diff      " << (turb_linearized_bg - result_linearized_bg).transpose() << endl;
//         cout << "bg jacob diff" << (step_jacobian.block<3, 3>(12, 6) * turb).transpose() << endl;

//         midPointIntegration(_dt, _acc_0, _gyr_0, _acc_1, _gyr_1, delta_p, delta_q, delta_v,
//                             linearized_ba + turb, linearized_bg,
//                             turb_delta_p, turb_delta_q, turb_delta_v,
//                             turb_linearized_ba, turb_linearized_bg, 0);
//         cout << "turb ba       " << endl;
//         cout << "p diff       " << (turb_delta_p - result_delta_p).transpose() << endl;
//         cout << "p jacob diff " << (step_jacobian.block<3, 3>(0, 9) * turb).transpose() << endl;
//         cout << "q diff       " << ((result_delta_q.inverse() * turb_delta_q).vec() * 2).transpose() << endl;
//         cout << "q jacob diff " << (step_jacobian.block<3, 3>(3, 9) * turb).transpose() << endl;
//         cout << "v diff       " << (turb_delta_v - result_delta_v).transpose() << endl;
//         cout << "v jacob diff " << (step_jacobian.block<3, 3>(6, 9) * turb).transpose() << endl;
//         cout << "ba diff      " << (turb_linearized_ba - result_linearized_ba).transpose() << endl;
//         cout << "ba jacob diff" << (step_jacobian.block<3, 3>(9, 9) * turb).transpose() << endl;
//         cout << "bg diff      " << (turb_linearized_bg - result_linearized_bg).transpose() << endl;
//         cout << "bg jacob diff" << (step_jacobian.block<3, 3>(12, 9) * turb).transpose() << endl;

//         midPointIntegration(_dt, _acc_0, _gyr_0, _acc_1, _gyr_1, delta_p, delta_q, delta_v,
//                             linearized_ba, linearized_bg + turb,
//                             turb_delta_p, turb_delta_q, turb_delta_v,
//                             turb_linearized_ba, turb_linearized_bg, 0);
//         cout << "turb bg       " << endl;
//         cout << "p diff       " << (turb_delta_p - result_delta_p).transpose() << endl;
//         cout << "p jacob diff " << (step_jacobian.block<3, 3>(0, 12) * turb).transpose() << endl;
//         cout << "q diff       " << ((result_delta_q.inverse() * turb_delta_q).vec() * 2).transpose() << endl;
//         cout << "q jacob diff " << (step_jacobian.block<3, 3>(3, 12) * turb).transpose() << endl;
//         cout << "v diff       " << (turb_delta_v - result_delta_v).transpose() << endl;
//         cout << "v jacob diff " << (step_jacobian.block<3, 3>(6, 12) * turb).transpose() << endl;
//         cout << "ba diff      " << (turb_linearized_ba - result_linearized_ba).transpose() << endl;
//         cout << "ba jacob diff" << (step_jacobian.block<3, 3>(9, 12) * turb).transpose() << endl;
//         cout << "bg diff      " << (turb_linearized_bg - result_linearized_bg).transpose() << endl;
//         cout << "bg jacob diff" << (step_jacobian.block<3, 3>(12, 12) * turb).transpose() << endl;

//         midPointIntegration(_dt, _acc_0 + turb, _gyr_0, _acc_1 , _gyr_1, delta_p, delta_q, delta_v,
//                             linearized_ba, linearized_bg,
//                             turb_delta_p, turb_delta_q, turb_delta_v,
//                             turb_linearized_ba, turb_linearized_bg, 0);
//         cout << "turb acc_0       " << endl;
//         cout << "p diff       " << (turb_delta_p - result_delta_p).transpose() << endl;
//         cout << "p jacob diff " << (step_V.block<3, 3>(0, 0) * turb).transpose() << endl;
//         cout << "q diff       " << ((result_delta_q.inverse() * turb_delta_q).vec() * 2).transpose() << endl;
//         cout << "q jacob diff " << (step_V.block<3, 3>(3, 0) * turb).transpose() << endl;
//         cout << "v diff       " << (turb_delta_v - result_delta_v).transpose() << endl;
//         cout << "v jacob diff " << (step_V.block<3, 3>(6, 0) * turb).transpose() << endl;
//         cout << "ba diff      " << (turb_linearized_ba - result_linearized_ba).transpose() << endl;
//         cout << "ba jacob diff" << (step_V.block<3, 3>(9, 0) * turb).transpose() << endl;
//         cout << "bg diff      " << (turb_linearized_bg - result_linearized_bg).transpose() << endl;
//         cout << "bg jacob diff" << (step_V.block<3, 3>(12, 0) * turb).transpose() << endl;

//         midPointIntegration(_dt, _acc_0, _gyr_0 + turb, _acc_1 , _gyr_1, delta_p, delta_q, delta_v,
//                             linearized_ba, linearized_bg,
//                             turb_delta_p, turb_delta_q, turb_delta_v,
//                             turb_linearized_ba, turb_linearized_bg, 0);
//         cout << "turb _gyr_0       " << endl;
//         cout << "p diff       " << (turb_delta_p - result_delta_p).transpose() << endl;
//         cout << "p jacob diff " << (step_V.block<3, 3>(0, 3) * turb).transpose() << endl;
//         cout << "q diff       " << ((result_delta_q.inverse() * turb_delta_q).vec() * 2).transpose() << endl;
//         cout << "q jacob diff " << (step_V.block<3, 3>(3, 3) * turb).transpose() << endl;
//         cout << "v diff       " << (turb_delta_v - result_delta_v).transpose() << endl;
//         cout << "v jacob diff " << (step_V.block<3, 3>(6, 3) * turb).transpose() << endl;
//         cout << "ba diff      " << (turb_linearized_ba - result_linearized_ba).transpose() << endl;
//         cout << "ba jacob diff" << (step_V.block<3, 3>(9, 3) * turb).transpose() << endl;
//         cout << "bg diff      " << (turb_linearized_bg - result_linearized_bg).transpose() << endl;
//         cout << "bg jacob diff" << (step_V.block<3, 3>(12, 3) * turb).transpose() << endl;

//         midPointIntegration(_dt, _acc_0, _gyr_0, _acc_1 + turb, _gyr_1, delta_p, delta_q, delta_v,
//                             linearized_ba, linearized_bg,
//                             turb_delta_p, turb_delta_q, turb_delta_v,
//                             turb_linearized_ba, turb_linearized_bg, 0);
//         cout << "turb acc_1       " << endl;
//         cout << "p diff       " << (turb_delta_p - result_delta_p).transpose() << endl;
//         cout << "p jacob diff " << (step_V.block<3, 3>(0, 6) * turb).transpose() << endl;
//         cout << "q diff       " << ((result_delta_q.inverse() * turb_delta_q).vec() * 2).transpose() << endl;
//         cout << "q jacob diff " << (step_V.block<3, 3>(3, 6) * turb).transpose() << endl;
//         cout << "v diff       " << (turb_delta_v - result_delta_v).transpose() << endl;
//         cout << "v jacob diff " << (step_V.block<3, 3>(6, 6) * turb).transpose() << endl;
//         cout << "ba diff      " << (turb_linearized_ba - result_linearized_ba).transpose() << endl;
//         cout << "ba jacob diff" << (step_V.block<3, 3>(9, 6) * turb).transpose() << endl;
//         cout << "bg diff      " << (turb_linearized_bg - result_linearized_bg).transpose() << endl;
//         cout << "bg jacob diff" << (step_V.block<3, 3>(12, 6) * turb).transpose() << endl;

//         midPointIntegration(_dt, _acc_0, _gyr_0, _acc_1 , _gyr_1 + turb, delta_p, delta_q, delta_v,
//                             linearized_ba, linearized_bg,
//                             turb_delta_p, turb_delta_q, turb_delta_v,
//                             turb_linearized_ba, turb_linearized_bg, 0);
//         cout << "turb _gyr_1       " << endl;
//         cout << "p diff       " << (turb_delta_p - result_delta_p).transpose() << endl;
//         cout << "p jacob diff " << (step_V.block<3, 3>(0, 9) * turb).transpose() << endl;
//         cout << "q diff       " << ((result_delta_q.inverse() * turb_delta_q).vec() * 2).transpose() << endl;
//         cout << "q jacob diff " << (step_V.block<3, 3>(3, 9) * turb).transpose() << endl;
//         cout << "v diff       " << (turb_delta_v - result_delta_v).transpose() << endl;
//         cout << "v jacob diff " << (step_V.block<3, 3>(6, 9) * turb).transpose() << endl;
//         cout << "ba diff      " << (turb_linearized_ba - result_linearized_ba).transpose() << endl;
//         cout << "ba jacob diff" << (step_V.block<3, 3>(9, 9) * turb).transpose() << endl;
//         cout << "bg diff      " << (turb_linearized_bg - result_linearized_bg).transpose() << endl;
//         cout << "bg jacob diff" << (step_V.block<3, 3>(12, 9) * turb).transpose() << endl;
//     }
//  