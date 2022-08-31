/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once
#include <ros/assert.h>
#include <iostream>
#include <eigen3/Eigen/Dense>

#include "../utility/utility.h"
#include "../estimator/parameters.h"

#include <ceres/ceres.h>

class ResistFactor : public ceres::SizedCostFunction<1, 1>
{
  public:
    ResistFactor() = delete;
    ResistFactor(double _gamma):gamma(_gamma)
    {
    }
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
    {
        double sel = parameters[0][0];
        Eigen::Map<Eigen::Matrix<double, 1, 1>> residual(residuals);
        residual(0,0) = gamma * (1-sel);

        if (jacobians)
        {
            if (jacobians[0])
            {
              Eigen::Map<Eigen::Matrix<double, 1, 1>> jacobian_sel(jacobians[0]);
              jacobian_sel(0,0)=-gamma;
            }
        }

        return true;
    }

    double gamma;

};


class PriorFactor : public ceres::SizedCostFunction<1, 1>
{
  public:
    PriorFactor() = delete;
    PriorFactor(double _gamma,double _prior):gamma(_gamma),prior(_prior)
    {
    }
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
    {
        double sel = parameters[0][0];
        Eigen::Map<Eigen::Matrix<double, 1, 1>> residual(residuals);
        residual(0,0) = fabs(gamma * (prior-sel));

        if (jacobians)
        {
            if (jacobians[0])
            {
              Eigen::Map<Eigen::Matrix<double, 1, 1>> jacobian_sel(jacobians[0]);
              if(prior<sel) jacobian_sel(0,0) = gamma;
              else jacobian_sel(0,0) = -gamma;
            }
        }

        return true;
    }

    double gamma;
    double prior;

};

