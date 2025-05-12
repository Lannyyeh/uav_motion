#pragma once

#include <cmath>
#include <Eigen/Eigen>

Eigen::Vector3d sat(const Eigen::Vector3d &x);
Eigen::Vector3d sign(const Eigen::Vector3d &x);

Eigen::Vector3d sat(const Eigen::Vector3d &x)
{
    Eigen::Vector3d x_sat = x;
    for (uint8_t i = 0; i < 3; ++i)
    {
      if (x_sat(i) > 1)
        x_sat(i) = 1;
      else if (x_sat(i) < -1)
        x_sat(i) = -1;
    }
    return x_sat;
}

Eigen::Vector3d sign(const Eigen::Vector3d &x)
{
    Eigen::Vector3d x_sign = x;
    for (uint8_t i = 0; i < 3; ++i)
    {
      if (x_sign(i) > 0)
        x_sign(i) = 1;
      else if (x_sign(i) < 0)
        x_sign(i) = -1;
    }
    return x_sign;
}