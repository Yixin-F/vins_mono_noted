#pragma once

#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>
#include "../utility/utility.h"

// 参数块，甚至可以理解为参数块计算过程的行为参数，比如确定优化量维度、雅克比计算等
class PoseLocalParameterization : public ceres::LocalParameterization
{
    // 虚函数和纯虚函数：https://zhuanlan.zhihu.com/p/147601339
    virtual bool Plus(const double *x, const double *delta, double *x_plus_delta) const;
    virtual bool ComputeJacobian(const double *x, double *jacobian) const;
    // ! GlobalSize()是ceres优化量维度，LocalSize() 是delta维度，可以看ceres::LocalParameterization基类
    virtual int GlobalSize() const { return 7; };  // 3平移 4四元数
    virtual int LocalSize() const { return 6; };  // 3平移 3旋转  共6个自由度   与GlobalSize()会相互转换
};
