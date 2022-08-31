/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#include "keyframe.h"

template <typename Derived>
static void reduceVector(vector<Derived> &v, vector<uchar> status)
{
  int j = 0;
  for (int i = 0; i < int(v.size()); i++)
    if (status[i])
      v[j++] = v[i];
  v.resize(j);
}

// create keyframe online
KeyFrame::KeyFrame(double _time_stamp, int _index, Vector3d &_vio_T_w_i, Matrix3d &_vio_R_w_i, cv::Mat &_image,
                           vector<cv::Point3f> &_point_3d,vector<cv::Point2f> &_point_2d_uv, vector<cv::Point2f> &_point_2d_normal,
                           vector<int> &_point_id,vector<double> &_point_weight, int _sequence)
{
  time_stamp = _time_stamp;
  index = _index;
  vio_T_w_i = _vio_T_w_i;
  vio_R_w_i = _vio_R_w_i;
  T_w_i = vio_T_w_i;
  R_w_i = vio_R_w_i;
  origin_vio_T = vio_T_w_i;
  origin_vio_R = vio_R_w_i;
  image = _image.clone();
  cv::resize(image, thumbnail, cv::Size(80, 60));
  point_3d = _point_3d;
  point_2d_uv = _point_2d_uv;
  point_2d_norm = _point_2d_normal;
  point_id = _point_id;
  point_weight = _point_weight;
  vector<uchar> status;
  for(int i=0;i<_point_weight.size();i++)
  {
    if(_point_weight[i]<MIN_SCORE)
    {
      status.push_back(0);
      dynamic_id.push_back(point_id[i]);
    }
    else status.push_back(1);
  }
  reduceVector(point_3d,status);
  reduceVector(point_2d_uv,status);
  reduceVector(point_2d_norm,status);
  reduceVector(point_id,status);
  reduceVector(point_weight,status);

  has_loop = false;
  loop_index = -1;
  has_fast_point = false;
  loop_info << 0, 0, 0, 0, 0, 0, 0, 0;
  sequence = _sequence;

  for(int i=0;i<point_3d.size();i++)
  {
    Vector3d w_pts_i;
    w_pts_i<<point_3d[i].x,point_3d[i].y,point_3d[i].z;
    Vector3d pts_i;
    pts_i = qic.transpose()*(origin_vio_R.transpose()*(w_pts_i - origin_vio_T)-tic);
    cv::Point3f pts_i_cv;
    pts_i_cv.x = pts_i.x();
    pts_i_cv.y = pts_i.y();
    pts_i_cv.z = pts_i.z();
    point_3d_ref.push_back(pts_i_cv);
  }
  computeCurrentBRIEFPoint();
  if(!DEBUG_IMAGE)
    image.release();
  else
    cv::cvtColor(image,image,cv::COLOR_GRAY2BGR);
}

void KeyFrame::getIndexById(const vector<int> absId,vector<int> & index)
{
  for(int i=0;i<absId.size();i++)
  {
    int idx = std::find(point_id.begin(), point_id.end(), absId[i]) - point_id.begin();
    if(idx==point_id.size()) index.push_back(-1);
    else index.push_back(idx);
  }
}

void KeyFrame::searchByBRIEFDes(std::vector<cv::Point2f> &matched_2d_old,
                                std::vector<cv::Point2f> &matched_2d_old_norm,
                                std::vector<int> &matched_old_id,
                                std::vector<uchar> &status,
                                const std::vector<BRIEF::bitset> &descriptors_old,
                                const std::vector<cv::KeyPoint> &keypoints_old,
                                const std::vector<cv::KeyPoint> &keypoints_old_norm)
{
    for(int i = 0; i < (int)brief_descriptors.size(); i++)
    {
        cv::Point2f pt(0.f, 0.f);
        cv::Point2f pt_norm(0.f, 0.f);
        int id = -1;
        if (searchInAera(brief_descriptors[i], descriptors_old, keypoints_old, keypoints_old_norm, id,pt, pt_norm))
          status.push_back(1);
        else
          status.push_back(0);
        matched_2d_old.push_back(pt);
        matched_2d_old_norm.push_back(pt_norm);
        matched_old_id.push_back(id);
    }

}

bool KeyFrame::searchInAera(const BRIEF::bitset window_descriptor,
                            const std::vector<BRIEF::bitset> &descriptors_old,
                            const std::vector<cv::KeyPoint> &keypoints_old,
                            const std::vector<cv::KeyPoint> &keypoints_old_norm,
                            int & best_id,
                            cv::Point2f &best_match,
                            cv::Point2f &best_match_norm)
{
    cv::Point2f best_pt;
    int bestDist = 128;
    int bestIndex = -1;
    for(int i = 0; i < (int)descriptors_old.size(); i++)
    {

        int dis = HammingDis(window_descriptor, descriptors_old[i]);
        if(dis < bestDist)
        {
            bestDist = dis;
            bestIndex = i;
        }
    }
    //printf("best dist %d", bestDist);
    if (bestIndex != -1 && bestDist < 80)
    {
      best_id = bestIndex;
      best_match = keypoints_old[bestIndex].pt;
      best_match_norm = keypoints_old_norm[bestIndex].pt;
      return true;
    }
    else
      return false;
}

int KeyFrame::HammingDis(const BRIEF::bitset &a, const BRIEF::bitset &b)
{
    BRIEF::bitset xor_of_bitset = a ^ b;
    int dis = xor_of_bitset.count();
    return dis;
}

bool KeyFrame::findConnection(KeyFrame* old_kf,Vector3d& rel_t,Quaterniond& rel_q,double& rel_yaw,
                              vector<int>& curId, vector<int>& tarId)
{
  std::vector<cv::Point2f> matched_2d_cur, matched_2d_old;
  std::vector<cv::Point2f> matched_2d_cur_norm, matched_2d_old_norm;
  std::vector<cv::Point3f> matched_3d;
  std::vector<uchar> status;

  matched_3d = point_3d;
  matched_2d_cur = point_2d_uv;
  matched_2d_cur_norm = point_2d_norm;
  curId = point_id;

  searchByBRIEFDes(matched_2d_old, matched_2d_old_norm,tarId, status, old_kf->brief_descriptors, old_kf->keypoints, old_kf->keypoints_norm);
  reduceVector(matched_2d_cur, status);
  reduceVector(matched_2d_old, status);
  reduceVector(matched_2d_cur_norm, status);
  reduceVector(matched_2d_old_norm, status);
  reduceVector(matched_3d, status);
  reduceVector(curId, status);
  reduceVector(tarId, status);

  status.clear();

  Eigen::Vector3d PnP_T_old;
  Eigen::Matrix3d PnP_R_old;
  Eigen::Vector3d relative_t;
  Quaterniond relative_q;
  double relative_yaw;
  if ((int)matched_2d_cur.size() > MIN_LOOP_NUM)
  {
      status.clear();
      PnPRANSAC(matched_2d_old_norm, matched_3d, status, PnP_T_old, PnP_R_old);
      reduceVector(matched_2d_cur, status);
      reduceVector(matched_2d_old, status);
      reduceVector(matched_2d_cur_norm, status);
      reduceVector(matched_2d_old_norm, status);
      reduceVector(matched_3d, status);
      reduceVector(curId, status);
      reduceVector(tarId, status);

  }
  if ((int)matched_2d_cur.size() > MIN_LOOP_NUM)
  {
      relative_t = PnP_R_old.transpose() * (origin_vio_T - PnP_T_old);
      relative_q = PnP_R_old.transpose() * origin_vio_R;
      relative_yaw = Utility::normalizeAngle(Utility::R2ypr(origin_vio_R).x() - Utility::R2ypr(PnP_R_old).x());


//      cout << "pnp relative_t " << relative_t.transpose() << endl;
//      cout << "pnp relative_yaw " << relative_yaw << endl;

      if (abs(relative_yaw) < 30.0 && relative_t.norm() < 20.0)
      {
        rel_t = relative_t;
        rel_q = relative_q;
        rel_yaw = relative_yaw;
        return true;
      }
  }
  return false;
}
void KeyFrame::PnPRANSAC(const vector<cv::Point2f> &matched_2d_old_norm,
                         const std::vector<cv::Point3f> &matched_3d,
                         std::vector<uchar> &status,
                         Eigen::Vector3d &PnP_T_old, Eigen::Matrix3d &PnP_R_old)
{
  //for (int i = 0; i < matched_3d.size(); i++)
  //	printf("3d x: %f, y: %f, z: %f\n",matched_3d[i].x, matched_3d[i].y, matched_3d[i].z );
  //printf("match size %d \n", matched_3d.size());
    cv::Mat r, rvec, t, D, tmp_r;
    cv::Mat K = (cv::Mat_<double>(3, 3) << 1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0);
    Matrix3d R_inital;
    Vector3d P_inital;
    Matrix3d R_w_c = origin_vio_R * qic;
    Vector3d T_w_c = origin_vio_T + origin_vio_R * tic;

    R_inital = R_w_c.inverse();
    P_inital = -(R_inital * T_w_c);

    cv::eigen2cv(R_inital, tmp_r);
    cv::Rodrigues(tmp_r, rvec);
    cv::eigen2cv(P_inital, t);

    cv::Mat inliers;
    TicToc t_pnp_ransac;

    if (CV_MAJOR_VERSION < 3)
        solvePnPRansac(matched_3d, matched_2d_old_norm, K, D, rvec, t, true, 100, 10.0 / 460.0, 100, inliers);
    else
    {
        if (CV_MINOR_VERSION < 2)
            solvePnPRansac(matched_3d, matched_2d_old_norm, K, D, rvec, t, true, 100, sqrt(10 / 460.0), 0.99, inliers);
        else
            solvePnPRansac(matched_3d, matched_2d_old_norm, K, D, rvec, t, true, 100, 10.0 / 460.0, 0.99, inliers);

    }

    for (int i = 0; i < (int)matched_2d_old_norm.size(); i++)
        status.push_back(0);

    for( int i = 0; i < inliers.rows; i++)
    {
        int n = inliers.at<int>(i);
        status[n] = 1;
    }

    cv::Rodrigues(rvec, r);
    Matrix3d R_pnp, R_w_c_old;
    cv::cv2eigen(r, R_pnp);
    R_w_c_old = R_pnp.transpose();
    Vector3d T_pnp, T_w_c_old;
    cv::cv2eigen(t, T_pnp);
    T_w_c_old = R_w_c_old * (-T_pnp);

    PnP_R_old = R_w_c_old * qic.transpose();
    PnP_T_old = T_w_c_old - PnP_R_old * tic;

}


void KeyFrame::removeFeature(const vector<int> featureId)
{
  std::vector<int> ptId(point_id);  //ASSUME SORTED.
  std::vector<int> rmId(featureId); //ASSUME SORTED.
  std::vector<uchar> status(ptId.size(),1);

  for (auto it1 = rmId.begin(), it2 = ptId.begin();
       it1 != rmId.end() && it2 != ptId.end();
       ++it2) {
      while (it1 != rmId.end() && *it1 < *it2) ++it1;
      if (it1 != rmId.end() && *it1 == *it2) {
          status[it2 - ptId.begin()] = 0;
      }
  }
  reduceVector(point_3d, status);
  reduceVector(point_3d_ref, status);
  reduceVector(point_2d_uv, status);
  reduceVector(point_2d_norm, status);
  reduceVector(point_id, status);
  reduceVector(point_weight, status);
  reduceVector(keypoints, status);
  reduceVector(brief_descriptors, status);
}

void KeyFrame::computeCurrentBRIEFPoint()
{
  BriefExtractor extractor(BRIEF_PATTERN_FILE.c_str());
  for(int i = 0; i < (int)point_2d_uv.size(); i++)
  {
    cv::KeyPoint key;
    key.pt = point_2d_uv[i];
    keypoints.push_back(key);
    Eigen::Vector3d tmp_p;
    m_camera->liftProjective(Eigen::Vector2d(keypoints[i].pt.x, keypoints[i].pt.y), tmp_p);
    cv::KeyPoint tmp_norm;
    tmp_norm.pt = cv::Point2f(tmp_p.x()/tmp_p.z(), tmp_p.y()/tmp_p.z());
    keypoints_norm.push_back(tmp_norm);
  }
  extractor(image, keypoints, brief_descriptors);
}

void BriefExtractor::operator() (const cv::Mat &im, vector<cv::KeyPoint> &keys, vector<BRIEF::bitset> &descriptors) const
{
  m_brief.compute(im, keys, descriptors);
}


void KeyFrame::getVioPose(Eigen::Vector3d &_T_w_i, Eigen::Matrix3d &_R_w_i)
{
  _T_w_i = vio_T_w_i;
  _R_w_i = vio_R_w_i;
}

void KeyFrame::getPose(Eigen::Vector3d &_T_w_i, Eigen::Matrix3d &_R_w_i)
{
  _T_w_i = T_w_i;
  _R_w_i = R_w_i;
}

void KeyFrame::updatePose(const Eigen::Vector3d &_T_w_i, const Eigen::Matrix3d &_R_w_i)
{
  T_w_i = _T_w_i;
  R_w_i = _R_w_i;
}

void KeyFrame::updateVioPose(const Eigen::Vector3d &_T_w_i, const Eigen::Matrix3d &_R_w_i)
{
  vio_T_w_i = _T_w_i;
  vio_R_w_i = _R_w_i;
  T_w_i = vio_T_w_i;
  R_w_i = vio_R_w_i;
}

Eigen::Vector3d KeyFrame::getLoopRelativeT()
{
  return Eigen::Vector3d(loop_info(0), loop_info(1), loop_info(2));
}

Eigen::Quaterniond KeyFrame::getLoopRelativeQ()
{
  return Eigen::Quaterniond(loop_info(3), loop_info(4), loop_info(5), loop_info(6));
}

double KeyFrame::getLoopRelativeYaw()
{
  return loop_info(7);
}

void KeyFrame::updateLoop(Eigen::Matrix<double, 8, 1 > &_loop_info)
{
  if (abs(_loop_info(7)) < 30.0 && Vector3d(_loop_info(0), _loop_info(1), _loop_info(2)).norm() < 20.0)
  {
    //printf("update loop info\n");
    loop_info = _loop_info;
  }
}

BriefExtractor::BriefExtractor(const std::string &pattern_file)
{
  // The DVision::BRIEF extractor computes a random pattern by default when
  // the object is created.
  // We load the pattern that we used to build the vocabulary, to make
  // the descriptors compatible with the predefined vocabulary

  // loads the pattern
  cv::FileStorage fs(pattern_file.c_str(), cv::FileStorage::READ);
  if(!fs.isOpened()) throw string("Could not open file ") + pattern_file;

  vector<int> x1, y1, x2, y2;
  fs["x1"] >> x1;
  fs["x2"] >> x2;
  fs["y1"] >> y1;
  fs["y2"] >> y2;

  m_brief.importPairs(x1, y1, x2, y2);
}


