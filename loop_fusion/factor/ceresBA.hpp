#include "ceres/ceres.h"
#include "ceres/rotation.h"




struct ReprojectionError {
  ReprojectionError(Eigen::Vector3d _pts3d,
                    Eigen::Vector2d _pts2d,
                    Eigen::Vector3d _trgP,Eigen::Quaterniond _trgQ,
                    Eigen::Vector3d _srcrelP,Eigen::Quaterniond _srcrelQ,
                    Eigen::Vector3d _i2cP,Eigen::Quaterniond _i2cQ)
    : pts3d(_pts3d), pts2d(_pts2d),
      trgP(_trgP),srcrelP(_srcrelP),i2cP(_i2cP),
      trgQ(_trgQ),srcrelQ(_srcrelQ),i2cQ(_i2cQ)
  {
    sqrt_info = 460 / 1.5 * Eigen::Matrix2d::Identity();
  }

  template <typename T>
  bool operator()(const T* const p_ts,
                  const T* const q_ts,
                  const T* const sel_ptr,
                  T* residuals) const {

    Eigen::Map<const Eigen::Matrix<T, 3, 1> > T_ts(p_ts);
    Eigen::Map<const Eigen::Quaternion<T> > Q_ts(q_ts);

    //pts3d = pts_camera_i
    Eigen::Vector3d pts_imu_trg = i2cQ * pts3d + i2cP;
    Eigen::Vector3d pts_imu_world = trgQ * pts_imu_trg + trgP;
    Eigen::Matrix<T, 3, 1>  pts_src_ref =
        Q_ts.template cast<T>().conjugate() * (pts_imu_world.template cast<T>()-T_ts);
    Eigen::Matrix<T, 3, 1> pts_imu_src =
        srcrelQ.template cast<T>().conjugate() * (pts_src_ref-srcrelP.template cast<T>());
    Eigen::Matrix<T, 3, 1> pts_cam_src =
        i2cQ.template cast<T>().conjugate() * (pts_imu_src-i2cP.template cast<T>());

    Eigen::Matrix<T, 2, 1> ptCamProj;
    ptCamProj<< pts_cam_src(0)  / pts_cam_src(2) - (T)pts2d(0),
                pts_cam_src(1)  / pts_cam_src(2) - (T)pts2d(1);


    Eigen::Map<Eigen::Matrix<T, 2, 1>> residual(residuals);
    residual =  sel_ptr[0] * ptCamProj;
    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(Eigen::Vector3d _pts3d,
                                     Eigen::Vector2d _pts2d,
                                     Eigen::Vector3d _trgP,Eigen::Quaterniond _trgQ,
                                     Eigen::Vector3d _srcrelP,Eigen::Quaterniond _srcrelQ,
                                     Eigen::Vector3d _i2cP,Eigen::Quaterniond _i2cQ) {
    return new ceres::AutoDiffCostFunction<ReprojectionError, 2, 3, 4, 1>(
              new ReprojectionError(_pts3d, _pts2d, _trgP,_trgQ, _srcrelP,
                                    _srcrelQ, _i2cP, _i2cQ));
  }

  Eigen::Vector3d pts3d;
  Eigen::Vector2d pts2d;
  Eigen::Vector3d trgP,srcrelP,i2cP;
  Eigen::Quaterniond trgQ, srcrelQ,i2cQ;
  Eigen::Matrix2d sqrt_info;
};

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
