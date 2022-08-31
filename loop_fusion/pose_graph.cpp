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

#include "pose_graph.h"

PoseGraph::PoseGraph()
{
  posegraph_visualization = new CameraPoseVisualization(1.0, 0.0, 1.0, 1.0);
  posegraph_visualization->setScale(0.1);
  posegraph_visualization->setLineWidth(0.01);
  earliest_loop_index = -1;
  t_drift = Eigen::Vector3d(0, 0, 0);
  yaw_drift = 0;
  r_drift = Eigen::Matrix3d::Identity();
  w_t_vio = Eigen::Vector3d(0, 0, 0);
  w_r_vio = Eigen::Matrix3d::Identity();
  global_index = 0;
  sequence_cnt = 0;
  sequence_loop.push_back(0);
  base_sequence = 1;
  use_imu = 0;
}

PoseGraph::~PoseGraph()
{
  t_optimization.detach();
}

void PoseGraph::registerPub(ros::NodeHandle &n)
{
  pub_pg_path = n.advertise<nav_msgs::Path>("pose_graph_path", 1000);
  pub_base_path = n.advertise<nav_msgs::Path>("base_path", 1000);
  pub_pose_graph = n.advertise<visualization_msgs::MarkerArray>("pose_graph", 1000);
  pub_pose_group = n.advertise<visualization_msgs::MarkerArray>("pose_group", 1000);
  for (int i = 1; i < 10; i++)
    pub_path[i] = n.advertise<nav_msgs::Path>("path_" + to_string(i), 1000);
}

void PoseGraph::setIMUFlag(bool _use_imu)
{
  use_imu = _use_imu;
  if(use_imu)
  {
    printf("VIO input, perfrom 4 DoF (x, y, z, yaw) pose graph optimization\n");
    t_optimization = std::thread(&PoseGraph::optimize4DoF, this);
  }
  else
  {
    printf("VO input, perfrom 6 DoF pose graph optimization\n");
    t_optimization = std::thread(&PoseGraph::optimize6DoF, this);
  }

}

void PoseGraph::loadVocabulary(std::string voc_path)
{
  voc = new BriefVocabulary(voc_path);
  db.setVocabulary(*voc, false, 0);
}

void PoseGraph::addKeyFrame(KeyFrame* cur_kf, bool flag_detect_loop)
{
  //shift to base frame
  Vector3d vio_P_cur;
  Matrix3d vio_R_cur;
  if (sequence_cnt != cur_kf->sequence)
  {
    sequence_cnt++;
    sequence_loop.push_back(0);
    w_t_vio = Eigen::Vector3d(0, 0, 0);
    w_r_vio = Eigen::Matrix3d::Identity();
    m_drift.lock();
    t_drift = Eigen::Vector3d(0, 0, 0);
    r_drift = Eigen::Matrix3d::Identity();
    m_drift.unlock();
  }

  cur_kf->getVioPose(vio_P_cur, vio_R_cur);
  vio_P_cur = w_r_vio * vio_P_cur + w_t_vio;
  vio_R_cur = w_r_vio *  vio_R_cur;
  cur_kf->updateVioPose(vio_P_cur, vio_R_cur);
  cur_kf->index = global_index;
  global_index++;


  //CHECK AND GENERATE NEW GROUP
  int overlapIdx,overlapSize;
//  TicToc t_groupchk;
  if(newGroupCheck(cur_kf,overlapSize,overlapIdx))
  {
//    printf("group check time: %f \n", t_groupchk.toc());
    if(!grouplist.empty())
    {
      removeDynamic(groupDynamic.back());

      Eigen::Vector3d P_s,P_e;
      Eigen::Matrix3d R_s,R_e;
      keyframelist[grouplist.back().front()]->getVioPose(P_s,R_s);
      keyframelist[grouplist.back().back()]->getVioPose(P_e,R_e);
      Vector3d relative_t = R_s.transpose() * (P_e - P_s);
      double relative_yaw = Utility::normalizeAngle(Utility::R2ypr(R_e).x() - Utility::R2ypr(R_s).x());
      if(grouplist.back().size()<MIN_KEYFRAME)// || abs(relative_yaw) > 20 )
      {
        grouplist.pop_back();
        groupFeatures.pop_back();
        groupFeaturesTrack.pop_back();
        groupDesc.pop_back();
//        std::cout<<"\033[1;31m REMOVE CURRENT GROUP \033[0m"<<std::endl;
      }
      else
      {
        loopInfo curLoops;
        m_optimize_buf.lock();
        groupLoop.push_back(curLoops);
        for(int i=0;i<grouplist.back().size();i++)
        {
          dbToIndex.insert(std::make_pair(db.size(),
                                          std::make_pair(grouplist.size()-1,
                                                         grouplist.back()[i])));
          indexToDB.insert(std::make_pair(grouplist.back()[i],db.size()));
          db.add(keyframelist[grouplist.back()[i]]->brief_descriptors);
        }
        TicToc t_group_match;
        groupMatch();
//        printf("matching time: %f \n", t_group_match.toc());

        lastindex_global = keyframelist[grouplist.back().back()]->index;
        m_optimize_buf.unlock();
      }
    }
//    std::cout<<"\033[1;31m NEW GROUP \033[0m"<<std::endl;
    std::vector<int> newGroupIdx = {cur_kf->index};
    grouplist.emplace_back(newGroupIdx);
    std::vector<std::vector<int>> newTrack;
    std::vector<int> initTrack = {0};
    newTrack.resize(cur_kf->point_id.size());
    std::fill (newTrack.begin(), newTrack.end(), initTrack);
    groupFeatures.emplace_back(cur_kf->point_id);
    groupFeaturesTrack.emplace_back(newTrack);
    groupDesc.emplace_back(cur_kf->brief_descriptors);
    groupDynamic.emplace_back(cur_kf->dynamic_id);
  }
  else
  {
    if(!(cur_kf->dynamic_id.empty()))
    {
      groupDynamic.back().insert(groupDynamic.back().end(),
                                 cur_kf->dynamic_id.begin(), cur_kf->dynamic_id.end()
                                 );
    }
    grouplist.back().emplace_back(cur_kf->index);
    for(int i=0;i<cur_kf->point_id.size();i++)
    {
      int idx = std::find(groupFeatures.back().begin(),groupFeatures.back().end(),
                          cur_kf->point_id[i]) - groupFeatures.back().begin();
      if(idx<groupFeatures.back().size())
      {
        groupFeaturesTrack.back()[idx].emplace_back(grouplist.back().size()-1);
      }
      else
      {
        groupFeatures.back().emplace_back(cur_kf->point_id[i]);
        groupDesc.back().emplace_back(cur_kf->brief_descriptors[i]);
        std::vector<int> newTrack = {grouplist.back().size()-1};
        groupFeaturesTrack.back().emplace_back(newTrack);
      }
    }
  }





  m_keyframelist.lock();
  Vector3d P;
  Matrix3d R;
  cur_kf->getVioPose(P, R);
  P = r_drift * P + t_drift;
  R = r_drift * R;
  cur_kf->updatePose(P, R);
  Quaterniond Q{R};
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.stamp = ros::Time(cur_kf->time_stamp);
  pose_stamped.header.frame_id = "world";
  pose_stamped.pose.position.x = P.x() + VISUALIZATION_SHIFT_X;
  pose_stamped.pose.position.y = P.y() + VISUALIZATION_SHIFT_Y;
  pose_stamped.pose.position.z = P.z();
  pose_stamped.pose.orientation.x = Q.x();
  pose_stamped.pose.orientation.y = Q.y();
  pose_stamped.pose.orientation.z = Q.z();
  pose_stamped.pose.orientation.w = Q.w();
  path[sequence_cnt].poses.push_back(pose_stamped);
  path[sequence_cnt].header = pose_stamped.header;
  pathGroup.push_back(cur_kf->index);

  //  if (SAVE_LOOP_PATH)
  //  {
  //    ofstream loop_path_file(VINS_RESULT_PATH+ "/vio_loop.csv", ios::app);
  //    loop_path_file.setf(ios::fixed, ios::floatfield);
  //    loop_path_file.precision(9);
  //    loop_path_file << cur_kf->time_stamp<< " ";
  //    loop_path_file.precision(6);
  //    loop_path_file << P.x() << " "
  //                   << P.y() << " "
  //                   << P.z() << " "
  //                   << Q.x() << " "
  //                   << Q.y() << " "
  //                   << Q.z() << " "
  //                   << Q.w() << endl;
  //    loop_path_file.close();
  //  }
  //draw local connection
  if (SHOW_S_EDGE)
  {
    vector<KeyFrame*>::reverse_iterator rit = keyframelist.rbegin();
    for (int i = 0; i < 4; i++)
    {
      if (rit == keyframelist.rend())
        break;
      Vector3d conncected_P;
      Matrix3d connected_R;
      if((*rit)->sequence == cur_kf->sequence)
      {
        (*rit)->getPose(conncected_P, connected_R);
        posegraph_visualization->add_edge(P, conncected_P);
      }
      rit++;
    }
  }
  //  if (SHOW_L_EDGE)
  //  {
  //    if (cur_kf->has_loop)
  //    {
  //      //printf("has loop \n");
  //      KeyFrame* connected_KF = getKeyFrame(cur_kf->loop_index);
  //      Vector3d connected_P,P0;
  //      Matrix3d connected_R,R0;
  //      connected_KF->getPose(connected_P, connected_R);
  //      //cur_kf->getVioPose(P0, R0);
  //      cur_kf->getPose(P0, R0);
  //      if(cur_kf->sequence > 0)
  //      {
  //        //printf("add loop into visual \n");
  //        posegraph_visualization->add_loopedge(P0, connected_P + Vector3d(VISUALIZATION_SHIFT_X, VISUALIZATION_SHIFT_Y, 0));
  //      }

  //    }
  //  }
  //posegraph_visualization->add_pose(P + Vector3d(VISUALIZATION_SHIFT_X, VISUALIZATION_SHIFT_Y, 0), Q);

  keyframelist.push_back(cur_kf);
  publish();
  m_keyframelist.unlock();
}

int PoseGraph::searchBRIEFs(const std::vector<BRIEF::bitset> &descripts,
                            const std::vector<BRIEF::bitset> &tar_descripts,
                            std::vector<int> & matched,
                            std::vector<int> & matchDist)
{
  int matchQuan = 0;
  for(int i=0;i<descripts.size();i++)
  {
    int distance;
    int matchpair = searchBRIEF(descripts[i],tar_descripts,distance);
    matched.emplace_back(matchpair);
    matchDist.emplace_back(distance);
    if(matchpair>-1)matchQuan++;
  }
  return matchQuan;
}

int PoseGraph::searchBRIEF(const BRIEF::bitset &descript,
                           const std::vector<BRIEF::bitset> &tar_descript,
                           int & distance)
{
  distance = 128;
  int bestIndex = -1;
  for(int i=0; i < tar_descript.size(); i++){
    int dis = HammingDis(descript, tar_descript[i]);
    if(dis < distance)
    {
      distance = dis;
      bestIndex = i;
    }
  }
  //printf("best dist %d", bestDist);
  if (bestIndex != -1 && distance < 80)
  {
    return bestIndex;
  }
  else
    return -1;
}


int PoseGraph::getGroupIdx(const int nodeIdx)
{
  for(int i=0;i<grouplist.size();i++)
  {
    if(grouplist[i].back() < nodeIdx) continue;
    if(grouplist[i].front() < nodeIdx) return -1;
    return i;
  }
}

void PoseGraph::groupMatch()
{
  if(grouplist.size()>2) //AT LEAST ONE MATCHABLE GROUP
  {
    int fidx = grouplist.size()-1;
    std::map<int,std::map<int,std::pair<int,int>>> matchedPairs; //GROUP - (CURID, (TARID,DISTANCE))
    for(int i=0;i<grouplist.back().size();i++)
    {
      QueryResults ret;
      db.queryL1NEW(keyframelist[grouplist.back()[i]]->brief_descriptors,ret,3,
          indexToDB[grouplist[fidx-1].front()],
          indexToDB[grouplist.back()[i]]);
      //std::cout<<"QUERY ID : "<<grouplist.back()[i]<<"("<<ret.size()<<")";
      if (ret.size() >= 1 &&ret[0].Score > 0.05)
      {
        for (int j = 1; j < ret.size(); j++)
        {
          if (ret[j].Score > 0.015)
          {
            int returnGrp = dbToIndex[ret[j].Id].first;
            if(returnGrp >= fidx-1) continue;
            int returnId = dbToIndex[ret[j].Id].second;
            Vector3d relPose;
            Quaterniond relQuat;
            double relYaw;
            vector<int> curId,tarId;
            if(keyframelist[grouplist.back()[i]]->findConnection(
                 keyframelist[returnId],relPose,relQuat,relYaw,curId,tarId))
            {
              groupLoop.back().inGroupId.push_back(i);
              groupLoop.back().sourceId.push_back(grouplist.back()[i]);
              groupLoop.back().targetId.push_back(returnId);
              groupLoop.back().targetGroup.push_back(returnGrp);
              groupLoop.back().sourceIdx.push_back(curId);
              groupLoop.back().targetIdx.push_back(tarId);
              groupLoop.back().hypoNum.push_back(-1);
              groupLoop.back().relPose.push_back(relPose);
              groupLoop.back().relRot.push_back(relQuat);
              groupLoop.back().relYaw.push_back(relYaw);

              //TEMPORARY INSERT. REFINED IN groupLoopRefine
              groupLoop.back().refPose.push_back(relPose);
              groupLoop.back().refRot.push_back(relQuat);
            }
          }
        }
      }
    }
    groupLoopRefine(groupLoop.size()-1);
  }
}

void PoseGraph::getLocalTQ(int refKeyframe,int curKeyframe,Vector3d& local_t,Quaterniond& local_q)
{
  Vector3d P_s0;
  Matrix3d R_s0;
  keyframelist[refKeyframe]->getVioPose(P_s0,R_s0); // BASE POSE OF CURRENT GROUP
  Eigen::Quaterniond Q_s(R_s0);
  Vector3d P_si;
  Matrix3d R_si;
  keyframelist[curKeyframe]->getVioPose(P_si,R_si);
  local_t   = R_s0.transpose() * (P_si-P_s0);
  local_q = R_s0.transpose() * R_si;
}

void PoseGraph::groupLoopRefine(int groupIdx)
{

  Vector3d P_s0;
  Matrix3d R_s0;
  keyframelist[grouplist[groupIdx][0]]->getVioPose(P_s0,R_s0); // BASE POSE OF CURRENT GROUP
  Eigen::Quaterniond Q_s(R_s0);


  std::vector<double> scorePerHypo_prev=groupLoop[groupIdx].scorePerHypo;
  std::vector<Vector3d>  refPPerHypo_prev=groupLoop[groupIdx].refPPerHypo;
  std::vector<double> refYawPerHypo_prev =groupLoop[groupIdx].refYawPerHypo;

  std::vector<Vector3d> hypoPose;
  std::vector<double> hypoYaw;
  std::vector<std::vector<double>> hypoYaws;
  std::vector<std::vector<int>> idxPerHypo; //LOCAL LOOP INDEX!

  for(int i=0;i<groupLoop[groupIdx].inGroupId.size();i++)
  {
    Vector3d P_tar,P_est;
    Matrix3d R_tar,R_est;
    keyframelist[groupLoop[groupIdx].targetId[i]]->getVioPose(P_tar,R_tar);
    P_est = P_tar + R_tar * groupLoop[groupIdx].relPose[i];
    R_est = R_tar * groupLoop[groupIdx].relRot[i];

    Vector3d P_si;
    Matrix3d R_si;
    keyframelist[groupLoop[groupIdx].sourceId[i]]->getVioPose(P_si,R_si);
    Eigen::Vector3d    local_t   = R_s0.transpose() * (P_si-P_s0);
    Eigen::Quaterniond local_q(R_s0.transpose() * R_si);

    Vector3d refPose;
    Quaterniond refQuat;
    refQuat = R_est * local_q.inverse();
    refPose = P_est - refQuat * local_t;

    groupLoop[groupIdx].refPose[i]=refPose;
    groupLoop[groupIdx].refRot[i]=refQuat;

    bool noHypo = true;
    Vector3d loopRefP = groupLoop[groupIdx].refPose[i];
    double loopRefYaw =  Utility::R2ypr(groupLoop[groupIdx].refRot[i].toRotationMatrix()).x();
    for(int j=0;j<hypoPose.size();j++)
    {
      double yawDiff = fabs(Utility::normalizeAngle(hypoYaw[j] - loopRefYaw));
      double distDiff = (hypoPose[j]-loopRefP).norm();

      if(yawDiff<MAX_HYPODIFF_YAW && distDiff <MAX_HYPODIFF_DIST)
      {
        Vector3d avgP = (idxPerHypo[j].size() * hypoPose[j] + loopRefP)/(idxPerHypo[j].size()+1);
        double x,y;
        x = y = 0;
        for(int k=0;k<hypoYaws[j].size();k++)
        {
          x += cos(hypoYaws[j][k]/180.0*M_PI);
          y += sin(hypoYaws[j][k]/180.0*M_PI);
        }
        double avgYaw = atan2(y,x);

        hypoPose[j] = avgP;
        hypoYaw[j]  = avgYaw / M_PI * 180.0;
        hypoYaws[j].push_back(loopRefYaw);
        idxPerHypo[j].push_back(i);
        noHypo=false;
        break;
      }
    }
    if(noHypo)
    {
      hypoPose.push_back(loopRefP);
      hypoYaw.push_back(loopRefYaw);
      std::vector<double> yaws;
      yaws.push_back(loopRefYaw);
      hypoYaws.push_back(yaws);
      std::vector<int> indexes;
      indexes.push_back(i);
      idxPerHypo.push_back(indexes);
    }
  }
  vector<int> hypos;
  hypos.push_back(0);
  hypos.push_back(1);
  if(idxPerHypo.size()==0) hypos.clear();
  else if(idxPerHypo.size()==1) hypos.pop_back();
  else if(idxPerHypo.size()>2)
  {
    int largestHypo      =   max_element(idxPerHypo.begin(), idxPerHypo.end(),
                                         [&](std::vector<int> &a, std::vector<int> &b) {
        return (a.size() < b.size());
  }) - idxPerHypo.begin();
    int secondLargestHypo =   max_element(idxPerHypo.begin(), idxPerHypo.end(),
                                          [&](std::vector<int> &a, std::vector<int> &b) {
        return (b.size()!=idxPerHypo[largestHypo].size())&&(a.size() < b.size());
  }) - idxPerHypo.begin();
    hypos[0] = largestHypo;
    hypos[1] = secondLargestHypo;
  }
  groupLoop[groupIdx].scorePerHypo.clear();
  groupLoop[groupIdx].refPPerHypo.clear();
  groupLoop[groupIdx].refPPerHypo.clear();
  std::fill(groupLoop[groupIdx].hypoNum.begin(),groupLoop[groupIdx].hypoNum.end(),-1);
  //  std::cout<<"\033[1;31m LOOPS OF GROUP "<<groupIdx<<" REFINED. TOTAL "<<hypos.size()<<" HYPOS, From "
  //            <<groupLoop[groupIdx].inGroupId.size()<<"Loops \033[0m"<<std::endl;
  for(int i=0;i<hypos.size();i++)
  {
    //    std::cout<<"HYPO "<<i <<"("<<idxPerHypo[hypos[i]].size() <<") : "<<hypoPose[hypos[i]].transpose()<<std::endl;
    groupLoop[groupIdx].scorePerHypo.push_back(1.0 / hypos.size());
    groupLoop[groupIdx].refPPerHypo.push_back(hypoPose[hypos[i]]);
    groupLoop[groupIdx].refYawPerHypo.push_back(hypoYaw[hypos[i]]);
    for(int j=0;j<idxPerHypo[hypos[i]].size();j++)
    {
      groupLoop[groupIdx].hypoNum[idxPerHypo[hypos[i]][j]] = i;
    }
  }

  if(!scorePerHypo_prev.empty())
  {
    for(int i=0;i<scorePerHypo_prev.size();i++)
    {
      for(int j=0;j<groupLoop[groupIdx].scorePerHypo.size();j++)
      {
        double yawDiff = fabs(Utility::normalizeAngle(
                                groupLoop[groupIdx].refYawPerHypo[j]-
                                refYawPerHypo_prev[i]));
        double distDiff = (groupLoop[groupIdx].refPPerHypo[j]-
                           refPPerHypo_prev[i]).norm();
        if(yawDiff<MAX_HYPODIFF_YAW && distDiff <MAX_HYPODIFF_DIST)
        {
          groupLoop[groupIdx].scorePerHypo[j] = scorePerHypo_prev[i];
        }

      }
    }

  }

  //  std::cout<<"\033[1;31m ------ \033[0m"<<std::endl;
}

void PoseGraph::optimizeBA(const std::vector<int> targetGroups,
                           const std::vector<int> sourceIndexes,
                           const std::vector<std::vector<int>> targetIndexes,
                           const std::vector<std::vector<int>> targetToSrcIndexes)
{
  bool if_debug = true;
  Eigen::Quaterniond Qic(qic);
  double featWeight[sourceIndexes.size()][1];

  Eigen::Vector3d P_s0;
  Eigen::Matrix3d R_s0;
  keyframelist[grouplist.back()[0]]->getVioPose(P_s0,R_s0); // BASE POSE OF QUERY GROUP


  Eigen::Vector3d Est_P_s = P_s0;
  Eigen::Quaterniond Est_Q_s(R_s0);

  std::vector<std::vector<int>> allSourceFeatureIdx;
  std::vector<std::pair<Eigen::Vector3d,Eigen::Quaterniond>> sourceRelPoses;
  for(int i=0;i<grouplist.back().size();i++)
  {
    Eigen::Vector3d P_s;
    Eigen::Matrix3d R_s;
    keyframelist[grouplist.back()[i]]->getVioPose(P_s,R_s);
    std::vector<int> sourceFeatureIdx;
    keyframelist[grouplist.back()[i]]->getIndexById(sourceIndexes,sourceFeatureIdx);
    allSourceFeatureIdx.emplace_back(sourceFeatureIdx);
    Eigen::Vector3d    relative_t   = R_s0.transpose() * (P_s-P_s0);
    Eigen::Quaterniond relative_q(R_s0.transpose() * R_s);
    sourceRelPoses.push_back(std::make_pair(relative_t,relative_q));
  }


  ceres::Problem problem;
  ceres::LossFunction *loss_function;
  loss_function = new ceres::HuberLoss(3.0);
  ceres::Solver::Options options;
  options.max_num_iterations = 1000;
  options.preconditioner_type = ceres::SCHUR_JACOBI;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  options.minimizer_progress_to_stdout = false;
  ceres::LocalParameterization* quaternion_local_parameterization =
      new ceres::EigenQuaternionParameterization;
  problem.AddParameterBlock(Est_P_s.data() , 3); //TARGET TO SOURCE
  problem.AddParameterBlock(Est_Q_s.coeffs().data(), 4);
  problem.SetParameterization(Est_Q_s.coeffs().data(),
                              quaternion_local_parameterization);


  for (int i=0;i<targetGroups.size();i++)
  {
//    std::cout<<"[BA]WITH GROUP "<<targetGroups[i]<<std::endl;
    for(int j=0;j<grouplist[targetGroups[i]].size();j++) //KEYFRAMES OF TARGET GROUP
    {
      Eigen::Vector3d P_t;
      Eigen::Matrix3d R_t;
      keyframelist[grouplist[targetGroups[i]][j]]->getVioPose(P_t,R_t);
      Eigen::Quaterniond Q_t(R_t);
      std::vector<int> targetFeatureIdx;
      keyframelist[grouplist[targetGroups[i]][j]]->getIndexById(targetIndexes[i],targetFeatureIdx);
      for(int m=0;m<allSourceFeatureIdx.size();m++)
      {
        for(int k=0;k<targetFeatureIdx.size();k++)
        {
          if(targetFeatureIdx[k]==-1) continue;
          int corrSrc = targetToSrcIndexes[i][k];
          if(allSourceFeatureIdx[m][corrSrc] ==-1) continue;

          cv::Point3f feat3DKT = keyframelist[grouplist[targetGroups[i]][j]]->point_3d[targetFeatureIdx[k]];
          cv::Point2f feat2DnormKS = keyframelist[grouplist.back()[m]]->point_2d_norm[allSourceFeatureIdx[m][corrSrc]];
          Eigen::Vector3d pts3d;
          pts3d<<feat3DKT.x,feat3DKT.y,feat3DKT.z;
          Eigen::Vector2d pts2d;
          pts2d<<feat2DnormKS.x,feat2DnormKS.y;
          ceres::CostFunction* cost_function =
              ReprojectionError::Create(pts3d,pts2d,
                                        P_t,Q_t,
                                        sourceRelPoses[m].first,
                                        sourceRelPoses[m].second,tic,Qic);

          problem.AddResidualBlock(cost_function, NULL,
                                   Est_P_s.data(),
                                   Est_Q_s.coeffs().data(),
                                   featWeight[k]
                                   );
        }
      }
    }
  }

  for(int i=0;i<sourceIndexes.size();i++)
  {
    featWeight[i][0] = 1.0;
    problem.AddParameterBlock(featWeight[i],1);
  }

  std::vector<int> unselectedWeight(sourceIndexes.size());
  std::fill(unselectedWeight.begin(),unselectedWeight.end(),1.0);
  int unselected = unselectedWeight.size();
  while(unselected>MIN_LOOP_NUM)
  {
    double costDiff = 0;
    int looptime = 0;
    bool optimizeWeight = true;
    int acceptFeat = 0;
    while(costDiff<0.99 && looptime<5)
    {

      if(optimizeWeight)
      {
        problem.SetParameterBlockConstant(Est_P_s.data());
        problem.SetParameterBlockConstant(Est_Q_s.coeffs().data());
      }
      else
      {
        problem.SetParameterBlockVariable(Est_P_s.data());
        problem.SetParameterBlockVariable(Est_Q_s.coeffs().data());
      }

      for(int i=0;i<sourceIndexes.size();i++) //For ABSOLUTE IDXES
      {
        ResistFactor *r = new ResistFactor(1.0);
        problem.AddResidualBlock(r,loss_function,featWeight[i]);

        if(optimizeWeight && (unselectedWeight[i] == 1))
        {
          problem.SetParameterBlockVariable(featWeight[i]);
          problem.SetParameterUpperBound(featWeight[i],0,1.0);
          problem.SetParameterLowerBound(featWeight[i],0,0.0);
        }
        else
        {
          problem.SetParameterBlockConstant(featWeight[i]);
        }


      }
      ceres::Solver::Summary summary;
      ceres::Solve(options, &problem, &summary);
      if(optimizeWeight)
      {
        optimizeWeight = false;
        acceptFeat = 0;
        for(int i=0;i<sourceIndexes.size();i++)
        {
          if(featWeight[i][0]>0.5) acceptFeat++;
        }
      }
      else
      {
        costDiff = summary.final_cost/summary.initial_cost;
//        std::cout<<"COSTDIFF :"<<costDiff<<std::endl;
        looptime++;
        optimizeWeight = true;
      }
    }
//    std::cout<<"FEATURE ACCEPTED : "<<acceptFeat<<"/"<<sourceIndexes.size()<<std::endl;
    if(acceptFeat<MIN_LOOP_NUM)
    {
//      std::cout<<"FAIL"<<std::endl;
      return;
    }
    for(int i=0;i<sourceIndexes.size();i++)
    {
      if(featWeight[i][0]>0.5)
      {
        unselectedWeight[i] = 0;
        featWeight[i][0] = 0;
      }
    }
//    std::cout<<"RESULT GROUP TF \n"<<Est_P_s.transpose()<<"\n - \n"<<Est_Q_s.toRotationMatrix()<<std::endl;
//    std::cout<<"PREV GROUP TF \n"<<P_s0.transpose()<<"\n - \n"<<R_s0<<std::endl;
    unselected = std::accumulate(unselectedWeight.begin(), unselectedWeight.end(), 0);
  }


}

int PoseGraph::findMatch(int targetGroup,int sourceGroup,
                         std::vector<int> &matchPointTarget,
                         std::vector<int> &matchPointSource)
{
  std::vector<int> distances;
  for(int i=0;i<groupDesc[sourceGroup].size();i++)
  {
    if(groupFeaturesTrack[sourceGroup][i].size()<3) continue;
    int targetSameIdx = std::find(groupFeatures[targetGroup].begin(),groupFeatures[targetGroup].end(),
                                  groupFeatures[sourceGroup][i]) - groupFeatures[targetGroup].begin();
    if(targetSameIdx<groupFeatures[targetGroup].size()) {
      if(groupFeaturesTrack[targetGroup][targetSameIdx].size()<(MIN_KEYFRAME/2)) continue;
      matchPointTarget.emplace_back(groupFeatures[sourceGroup][i]);
      matchPointSource.emplace_back(groupFeatures[sourceGroup][i]);
      distances.emplace_back(0);
      continue;
    }

    int bestDist = 128;
    int bestIndex = -1;
    for(int j=0;j<groupDesc[targetGroup].size();j++)
    {
      int dis = HammingDis(groupDesc[sourceGroup][i], groupDesc[targetGroup][j]);
      if(dis < bestDist)
      {
        bestDist = dis;
        bestIndex = j;
      }
    }
    if (bestIndex != -1 && bestDist < 80)
    {
      if(groupFeaturesTrack[targetGroup][bestIndex].size()<3) continue;
      int prevIdx = std::find(matchPointTarget.begin(),matchPointTarget.end(),
                              groupFeatures[targetGroup][bestIndex])
          - matchPointTarget.begin();
      if(prevIdx < matchPointTarget.size())
      {
        if(distances[prevIdx] > bestDist)
        {
          matchPointTarget.erase(matchPointTarget.begin()+prevIdx);
          matchPointSource.erase(matchPointSource.begin()+prevIdx);
          distances.erase(distances.begin()+prevIdx);
        }
        else continue;
      }
      matchPointTarget.emplace_back(groupFeatures[targetGroup][bestIndex]);
      matchPointSource.emplace_back(groupFeatures[sourceGroup][i]);
      distances.emplace_back(bestDist);
    }
  }
  return matchPointTarget.size();
}

void PoseGraph::removeDynamic(const vector<int> dynamicId)
{
  for(int i=0;i<groupFeatures.size();i++)
  {
    std::vector<int> ptId(groupFeatures[i]);  //ASSUME SORTED.
    std::vector<int> rmId(dynamicId); //ASSUME SORTED.
    std::vector<uchar> status(ptId.size(),1);

    for (auto it1 = rmId.begin(), it2 = ptId.begin();
         it1 != rmId.end() && it2 != ptId.end();
         ++it2) {
      while (it1 != rmId.end() && *it1 < *it2) ++it1;
      if (it1 != rmId.end() && *it1 == *it2) {
        status[it2 - ptId.begin()] = 0;
      }
    }
    reduceVector(groupFeaturesTrack[i], status);
    reduceVector(groupDesc[i], status);
    reduceVector(groupFeatures[i], status);
  }
}

bool PoseGraph::newGroupCheck(KeyFrame * keyframe,int& commonSize,int& overLapIdx)
{

  if(grouplist.empty() || grouplist.back().size()>MAX_KEYFRAME)
    return true;
  std::vector<int> common_data;
  KeyFrame * targetKeyf = getKeyFrame(grouplist.back().front());
  set_intersection(keyframe->point_id.begin(),keyframe->point_id.end(),
                   targetKeyf->point_id.begin(),targetKeyf->point_id.end(),
                   std::back_inserter(common_data));
  commonSize = common_data.size();
  if(commonSize<MIN_OVERLAP) return true;

  //FOR INSERT FEATURES INTO GROUPFEATURES
  auto it = std::upper_bound(keyframe->point_id.begin(),
                             keyframe->point_id.end(),
                             groupFeatures.back().back());
  overLapIdx = it - keyframe->point_id.begin();
  return false;
}

KeyFrame* PoseGraph::getKeyFrame(int index)
{
  //    unique_lock<mutex> lock(m_keyframelist);
  vector<KeyFrame*>::iterator it = keyframelist.begin();
  for (; it != keyframelist.end(); it++)
  {
    if((*it)->index == index)
      break;
  }
  if (it != keyframelist.end())
    return *it;
  else
    return NULL;
}

int PoseGraph::detectLoop(KeyFrame* keyframe, int frame_index)
{
  // put image into image_pool; for visualization
  cv::Mat compressed_image;
  if (DEBUG_IMAGE)
  {
    int feature_num = keyframe->keypoints.size();
    cv::resize(keyframe->image, compressed_image, cv::Size(376, 240));
    putText(compressed_image, "feature_num:" + to_string(feature_num), cv::Point2f(10, 10), CV_FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255));
    image_pool[frame_index] = compressed_image;
  }
  TicToc tmp_t;
  //first query; then add this frame into database!
  QueryResults ret;
  TicToc t_query;
  int minLoopDist = 20;
  db.query(keyframe->brief_descriptors, ret, 4, frame_index - minLoopDist);
  //printf("query time: %f", t_query.toc());
  //cout << "Searching for Image " << frame_index << ". " << ret << endl;

  TicToc t_add;
  db.add(keyframe->brief_descriptors);
  //printf("add feature time: %f", t_add.toc());
  // ret[0] is the nearest neighbour's score. threshold change with neighour score
  bool find_loop = false;
  cv::Mat loop_result;
  if (DEBUG_IMAGE)
  {
    loop_result = compressed_image.clone();
    if (ret.size() > 0)
      putText(loop_result, "neighbour score:" + to_string(ret[0].Score), cv::Point2f(10, 50), CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255));
  }
  // visual loop result
  if (DEBUG_IMAGE)
  {
    for (unsigned int i = 0; i < ret.size(); i++)
    {
      int tmp_index = ret[i].Id;
      auto it = image_pool.find(tmp_index);
      cv::Mat tmp_image = (it->second).clone();
      putText(tmp_image, "index:  " + to_string(tmp_index) + "loop score:" + to_string(ret[i].Score), cv::Point2f(10, 50), CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255));
      cv::hconcat(loop_result, tmp_image, loop_result);
    }
  }
  // a good match with its nerghbour
  if (ret.size() >= 1 &&ret[0].Score > 0.05)
    for (unsigned int i = 1; i < ret.size(); i++)
    {
      //if (ret[i].Score > ret[0].Score * 0.3)
      if (ret[i].Score > 0.015)
      {
        find_loop = true;
        int tmp_index = ret[i].Id;
        if (DEBUG_IMAGE && 0)
        {
          auto it = image_pool.find(tmp_index);
          cv::Mat tmp_image = (it->second).clone();
          putText(tmp_image, "loop score:" + to_string(ret[i].Score), cv::Point2f(10, 50), CV_FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255));
          cv::hconcat(loop_result, tmp_image, loop_result);
        }
      }

    }
  /*
    if (DEBUG_IMAGE)
    {
        cv::imshow("loop_result", loop_result);
        cv::waitKey(20);
    }
*/
  if (find_loop && frame_index > minLoopDist)
  {
    int min_index = -1;
    for (unsigned int i = 0; i < ret.size(); i++)
    {
      if (min_index == -1 || (ret[i].Id < min_index && ret[i].Score > 0.015))
        min_index = ret[i].Id;
    }
    return min_index;
  }
  else
    return -1;

}

void PoseGraph::addKeyFrameIntoVoc(KeyFrame* keyframe)
{
  // put image into image_pool; for visualization
  cv::Mat compressed_image;
  if (DEBUG_IMAGE)
  {
    int feature_num = keyframe->keypoints.size();
    cv::resize(keyframe->image, compressed_image, cv::Size(376, 240));
    putText(compressed_image, "feature_num:" + to_string(feature_num), cv::Point2f(10, 10), CV_FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255));
    image_pool[keyframe->index] = compressed_image;
  }

  db.add(keyframe->brief_descriptors);
}

void PoseGraph::optimize4DoF()
{
  while(true)
  {
    static int prevGroupLoopSize = 0;
    int cur_index = -1;
    std::vector<loopInfo> groupLoopNow;
    m_optimize_buf.lock();
    if(!groupLoop.empty())
    {
      groupLoopNow = groupLoop;
      cur_index = lastindex_global;
    }
    m_optimize_buf.unlock();

    if (!groupLoopNow.empty() && prevGroupLoopSize!=groupLoopNow.size())
    {
      TicToc t_optimize_time;

      prevGroupLoopSize = groupLoopNow.size();
//      std::cout<<"\033[1;32m OPTIMIZE POSE GRAPH WITH GROUPLOOP SIZE "<<prevGroupLoopSize<<" \033[0m"<<std::endl;
      TicToc tmp_t;
      m_keyframelist.lock();
      KeyFrame* cur_kf = getKeyFrame(cur_index);

      int max_length = cur_index + 1;

      // w^t_i   w^q_i
      double t_array[max_length][3];
      Quaterniond q_array[max_length];
      double euler_array[max_length][3];
      double sequence_array[max_length];

      ceres::Problem problem;
      ceres::Solver::Options options;
      options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
      //options.minimizer_progress_to_stdout = true;
      //options.max_solver_time_in_seconds = SOLVER_TIME * 3;
      options.max_num_iterations = 100;
      ceres::Solver::Summary summary;
      ceres::LossFunction *loss_function;
      loss_function = new ceres::HuberLoss(0.1);
      //loss_function = new ceres::CauchyLoss(1.0);
      ceres::LocalParameterization* angle_local_parameterization =
          AngleLocalParameterization::Create();

      vector<KeyFrame*>::iterator it;
//      std::cout<<"\033[1;32m ADD KEYFRAMES  \033[0m"<<std::endl;

      int idx = 0;
      for (it = keyframelist.begin(); it != keyframelist.end(); it++)
      {
        Quaterniond tmp_q;
        Matrix3d tmp_r;
        Vector3d tmp_t;
        (*it)->getVioPose(tmp_t, tmp_r);
        tmp_q = tmp_r;
        t_array[idx][0] = tmp_t(0);
        t_array[idx][1] = tmp_t(1);
        t_array[idx][2] = tmp_t(2);
        q_array[idx] = tmp_q;

        Vector3d euler_angle = Utility::R2ypr(tmp_q.toRotationMatrix());
        euler_array[idx][0] = euler_angle.x();
        euler_array[idx][1] = euler_angle.y();
        euler_array[idx][2] = euler_angle.z();

        sequence_array[idx] = (*it)->sequence;

        problem.AddParameterBlock(euler_array[idx], 1, angle_local_parameterization);
        problem.AddParameterBlock(t_array[idx], 3);

        if ((*it)->index == 0)
        {
          problem.SetParameterBlockConstant(euler_array[idx]);
          problem.SetParameterBlockConstant(t_array[idx]);
        }

        //add edge
        for (int j = 1; j < 5; j++)
        {
          if (idx - j >= 0)
          {
            Vector3d euler_conncected = Utility::R2ypr(q_array[idx-j].toRotationMatrix());
            Vector3d relative_t(t_array[idx][0] - t_array[idx-j][0], t_array[idx][1] - t_array[idx-j][1], t_array[idx][2] - t_array[idx-j][2]);
            relative_t = q_array[idx-j].inverse() * relative_t;
            double relative_yaw = euler_array[idx][0] - euler_array[idx-j][0];
            ceres::CostFunction* cost_function = FourDOFError::Create( relative_t.x(), relative_t.y(), relative_t.z(),
                                                                       relative_yaw, euler_conncected.y(), euler_conncected.z());
            problem.AddResidualBlock(cost_function, loss_function, euler_array[idx-j],
                t_array[idx-j],
                euler_array[idx],
                t_array[idx]);
          }
        }

        if ((*it)->index == cur_index)
          break;
        idx++;
      }

//      std::cout<<"\033[1;32m ADD LOOPCLOSURES  \033[0m"<<std::endl;
      double hypoWeights[groupLoopNow.size()][2];

      //add loop edge
      for(int j=0;j<groupLoopNow.size();j++)
      {
        problem.AddParameterBlock(hypoWeights[j],2);
        ceres::CostFunction* prior_cost = PriorFunctor::Create(HYPOPRIOR_GAMMA);
        problem.AddResidualBlock(prior_cost,NULL,hypoWeights[j]);


        for(int k=0;k<groupLoopNow[j].hypoNum.size();k++)
        {
          if(groupLoopNow[j].hypoNum[k]==-1) continue;
//          std::cout<<j<<","<<k<<","<<groupLoopNow[j].hypoNum[k]<<"/";
          int cur_index = groupLoopNow[j].sourceId[k];
          int connected_index = groupLoopNow[j].targetId[k];
          Vector3d euler_conncected = Utility::R2ypr(q_array[connected_index].toRotationMatrix());
          Vector3d relative_t = groupLoopNow[j].relPose[k];
          double relative_yaw = groupLoopNow[j].relYaw[k];
          int count = std::count(groupLoopNow[j].hypoNum.begin(), groupLoopNow[j].hypoNum.end(),
                                 groupLoopNow[j].hypoNum[k]);
          ceres::CostFunction* cost_function = FourDOFWeightError::Create( relative_t.x(), relative_t.y(), relative_t.z(),
                                                                           relative_yaw, euler_conncected.y(), euler_conncected.z(),
                                                                           groupLoopNow[j].hypoNum[k],
                                                                           1.0/(double)count);
          problem.AddResidualBlock(cost_function, NULL,
                                   euler_array[connected_index],
                                   t_array[connected_index],
                                   euler_array[cur_index],
                                   t_array[cur_index],
                                   hypoWeights[j]
                                   );

        }
      }
//      std::cout<<std::endl;

      m_keyframelist.unlock();

//      std::cout<<"\033[1;32m START TO SOLVE  \033[0m"<<std::endl;
      double costDiff = 0;
      bool optimizeWeight = true;
      for(int i=0;i<groupLoopNow.size();i++)
      {
//        std::cout<<"GLN "<<i<<":";
        for(int j=0;j<groupLoopNow[i].scorePerHypo.size();j++)
        {
          int count = std::count(groupLoopNow[i].hypoNum.begin(), groupLoopNow[i].hypoNum.end(), j);
          hypoWeights[i][j] = groupLoopNow[i].scorePerHypo[j];
//          std::cout<<j<<","<<hypoWeights[i][j]<<"("<<count<<");";
        }
        for(int j=groupLoopNow[i].scorePerHypo.size();j<2;j++)
        {
          int count = std::count(groupLoopNow[i].hypoNum.begin(), groupLoopNow[i].hypoNum.end(), j);
          hypoWeights[i][j] = 0.0;
//          std::cout<<j<<","<<hypoWeights[i][j]<<"("<<count<<");";
        }
//        std::cout<<std::endl;
      }
//      printf("optimize preprocess: %f \n", t_optimize_time.toc());

      TicToc t_optimize;
      while(costDiff<HYPOPRIOR_ALT_CONV || optimizeWeight)
      {
        if(optimizeWeight)
        {
          for(int i=0;i<idx;i++)
          {
            problem.SetParameterBlockConstant(euler_array[i]);
            problem.SetParameterBlockConstant(t_array[i]);
          }
          for(int i=0;i<groupLoopNow.size();i++)
          {
            for(int j=0;j<groupLoopNow[i].scorePerHypo.size();j++)
            {
              problem.SetParameterUpperBound(hypoWeights[i],j,1.0);
              problem.SetParameterLowerBound(hypoWeights[i],j,0.0);
            }
            for(int j=groupLoopNow[i].scorePerHypo.size();j<2;j++)
            {
              problem.SetParameterUpperBound(hypoWeights[i],j,0.01);
              problem.SetParameterLowerBound(hypoWeights[i],j,0.0);
            }
          }
          optimizeWeight = false;
//          std::cout<<"\033[1;32m SOLVE WEIGHT  \033[0m"<<std::endl;
        }
        else
        {
          for(int i=1;i<idx;i++)
          {
            problem.SetParameterBlockVariable(euler_array[i]);
            problem.SetParameterBlockVariable(t_array[i]);
          }
          for(int i=0;i<groupLoopNow.size();i++)
          {
            problem.SetParameterBlockConstant(hypoWeights[i]);
          }
          optimizeWeight = true;
//          std::cout<<"\033[1;32m SOLVE GRAPH  \033[0m"<<std::endl;
        }
        ceres::Solve(options, &problem, &summary);
        costDiff = summary.final_cost/summary.initial_cost;
//        std::cout<<"\033[1;32m SOLVED COST :"<<costDiff<<"  \033[0m"<<std::endl;
      }
//      printf("optimize cost: %f,%d \n", t_optimize.toc(),cur_index);

      for(int i=0;i<groupLoopNow.size();i++)
      {
//        std::cout<<"GROUP "<<i<<" RESULT ::";
        for(int j=0;j<groupLoopNow[i].scorePerHypo.size();j++)
        {
//          std::cout<<"HYPO "<<j<<" : "<<hypoWeights[i][j]<<" / ";
          groupLoopNow[i].scorePerHypo[j] = hypoWeights[i][j];
        }
//        std::cout<<std::endl;
      }
//      std::cout<<"\033[1;32m END SOLVE  \033[0m"<<std::endl;

      m_keyframelist.lock();
      idx = 0;
      for (it = keyframelist.begin(); it != keyframelist.end(); it++)
      {
        Quaterniond tmp_q;
        tmp_q = Utility::ypr2R(Vector3d(euler_array[idx][0], euler_array[idx][1], euler_array[idx][2]));
        Vector3d tmp_t = Vector3d(t_array[idx][0], t_array[idx][1], t_array[idx][2]);
        Matrix3d tmp_r = tmp_q.toRotationMatrix();
        (*it)-> updatePose(tmp_t, tmp_r);

        if ((*it)->index == cur_index)
          break;
        idx++;
      }

      Vector3d cur_t, vio_t;
      Matrix3d cur_r, vio_r;
      cur_kf->getPose(cur_t, cur_r);
      cur_kf->getVioPose(vio_t, vio_r);
      m_drift.lock();
      yaw_drift = Utility::R2ypr(cur_r).x() - Utility::R2ypr(vio_r).x();
      r_drift = Utility::ypr2R(Vector3d(yaw_drift, 0, 0));
      t_drift = cur_t - r_drift * vio_t;
      m_drift.unlock();


      it++;
      for (; it != keyframelist.end(); it++)
      {
        Vector3d P;
        Matrix3d R;
        (*it)->getVioPose(P, R);
        P = r_drift * P + t_drift;
        R = r_drift * R;
        (*it)->updatePose(P, R);
      }
      m_keyframelist.unlock();

      m_optimize_buf.lock();
      for(int i=0;i<groupLoopNow.size();i++)
      {
        groupLoop[i] = groupLoopNow[i];
      }
      m_optimize_buf.unlock();


      updatePath();

      TicToc t_loopfrefine;

      for(int i=0;i<prevGroupLoopSize;i++)
      {
        groupLoopRefine(i);
      }
//      printf("loop refine cost: %f \n", t_loopfrefine.toc());

    }

    std::chrono::milliseconds dura(2000);
    std::this_thread::sleep_for(dura);
  }
  return;
}


void PoseGraph::optimize6DoF()
{
  while(true)
  {
    int cur_index = -1;
    int first_looped_index = -1;
    m_optimize_buf.lock();
    while(!optimize_buf.empty())
    {
      cur_index = optimize_buf.front();
      first_looped_index = earliest_loop_index;
      optimize_buf.pop();
    }
    m_optimize_buf.unlock();
    if (cur_index != -1)
    {
//      printf("optimize pose graph \n");
      TicToc tmp_t;
      m_keyframelist.lock();
      KeyFrame* cur_kf = getKeyFrame(cur_index);

      int max_length = cur_index + 1;

      // w^t_i   w^q_i
      double t_array[max_length][3];
      double q_array[max_length][4];
      double sequence_array[max_length];

      ceres::Problem problem;
      ceres::Solver::Options options;
      options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
      //ptions.minimizer_progress_to_stdout = true;
      //options.max_solver_time_in_seconds = SOLVER_TIME * 3;
      options.max_num_iterations = 5;
      ceres::Solver::Summary summary;
      ceres::LossFunction *loss_function;
      loss_function = new ceres::HuberLoss(0.1);
      //loss_function = new ceres::CauchyLoss(1.0);
      ceres::LocalParameterization* local_parameterization = new ceres::QuaternionParameterization();

      vector<KeyFrame*>::iterator it;

      int i = 0;
      for (it = keyframelist.begin(); it != keyframelist.end(); it++)
      {
        if ((*it)->index < first_looped_index)
          continue;
        (*it)->local_index = i;
        Quaterniond tmp_q;
        Matrix3d tmp_r;
        Vector3d tmp_t;
        (*it)->getVioPose(tmp_t, tmp_r);
        tmp_q = tmp_r;
        t_array[i][0] = tmp_t(0);
        t_array[i][1] = tmp_t(1);
        t_array[i][2] = tmp_t(2);
        q_array[i][0] = tmp_q.w();
        q_array[i][1] = tmp_q.x();
        q_array[i][2] = tmp_q.y();
        q_array[i][3] = tmp_q.z();

        sequence_array[i] = (*it)->sequence;

        problem.AddParameterBlock(q_array[i], 4, local_parameterization);
        problem.AddParameterBlock(t_array[i], 3);

        if ((*it)->index == first_looped_index || (*it)->sequence == 0)
        {
          problem.SetParameterBlockConstant(q_array[i]);
          problem.SetParameterBlockConstant(t_array[i]);
        }

        //add edge
        for (int j = 1; j < 5; j++)
        {
          if (i - j >= 0 && sequence_array[i] == sequence_array[i-j])
          {
            Vector3d relative_t(t_array[i][0] - t_array[i-j][0], t_array[i][1] - t_array[i-j][1], t_array[i][2] - t_array[i-j][2]);
            Quaterniond q_i_j = Quaterniond(q_array[i-j][0], q_array[i-j][1], q_array[i-j][2], q_array[i-j][3]);
            Quaterniond q_i = Quaterniond(q_array[i][0], q_array[i][1], q_array[i][2], q_array[i][3]);
            relative_t = q_i_j.inverse() * relative_t;
            Quaterniond relative_q = q_i_j.inverse() * q_i;
            ceres::CostFunction* vo_function = RelativeRTError::Create(relative_t.x(), relative_t.y(), relative_t.z(),
                                                                       relative_q.w(), relative_q.x(), relative_q.y(), relative_q.z(),
                                                                       0.1, 0.01);
            problem.AddResidualBlock(vo_function, NULL, q_array[i-j], t_array[i-j], q_array[i], t_array[i]);
          }
        }

        //add loop edge

        if((*it)->has_loop)
        {
          assert((*it)->loop_index >= first_looped_index);
          int connected_index = getKeyFrame((*it)->loop_index)->local_index;
          Vector3d relative_t;
          relative_t = (*it)->getLoopRelativeT();
          Quaterniond relative_q;
          relative_q = (*it)->getLoopRelativeQ();
          ceres::CostFunction* loop_function = RelativeRTError::Create(relative_t.x(), relative_t.y(), relative_t.z(),
                                                                       relative_q.w(), relative_q.x(), relative_q.y(), relative_q.z(),
                                                                       0.1, 0.01);
          problem.AddResidualBlock(loop_function, loss_function, q_array[connected_index], t_array[connected_index], q_array[i], t_array[i]);
        }

        if ((*it)->index == cur_index)
          break;
        i++;
      }
      m_keyframelist.unlock();

      ceres::Solve(options, &problem, &summary);
      //std::cout << summary.BriefReport() << "\n";

      //printf("pose optimization time: %f \n", tmp_t.toc());
      /*
            for (int j = 0 ; j < i; j++)
            {
                printf("optimize i: %d p: %f, %f, %f\n", j, t_array[j][0], t_array[j][1], t_array[j][2] );
            }
            */
      m_keyframelist.lock();
      i = 0;
      for (it = keyframelist.begin(); it != keyframelist.end(); it++)
      {
        if ((*it)->index < first_looped_index)
          continue;
        Quaterniond tmp_q(q_array[i][0], q_array[i][1], q_array[i][2], q_array[i][3]);
        Vector3d tmp_t = Vector3d(t_array[i][0], t_array[i][1], t_array[i][2]);
        Matrix3d tmp_r = tmp_q.toRotationMatrix();
        (*it)-> updatePose(tmp_t, tmp_r);

        if ((*it)->index == cur_index)
          break;
        i++;
      }

      Vector3d cur_t, vio_t;
      Matrix3d cur_r, vio_r;
      cur_kf->getPose(cur_t, cur_r);
      cur_kf->getVioPose(vio_t, vio_r);
      m_drift.lock();
      r_drift = cur_r * vio_r.transpose();
      t_drift = cur_t - r_drift * vio_t;
      m_drift.unlock();
      //cout << "t_drift " << t_drift.transpose() << endl;
      //cout << "r_drift " << Utility::R2ypr(r_drift).transpose() << endl;

      it++;
      for (; it != keyframelist.end(); it++)
      {
        Vector3d P;
        Matrix3d R;
        (*it)->getVioPose(P, R);
        P = r_drift * P + t_drift;
        R = r_drift * R;
        (*it)->updatePose(P, R);
      }
      m_keyframelist.unlock();
      updatePath();
    }

    std::chrono::milliseconds dura(2000);
    std::this_thread::sleep_for(dura);
  }
  return;
}

void PoseGraph::updatePath()
{
  m_keyframelist.lock();
  vector<KeyFrame*>::iterator it;
  for (int i = 1; i <= sequence_cnt; i++)
  {
    path[i].poses.clear();
  }
  base_path.poses.clear();
  posegraph_visualization->reset();

  if (SAVE_LOOP_PATH)
  {
    ofstream loop_path_file_tmp(VINS_RESULT_PATH+ "/vio_loop.csv", ios::out);
    loop_path_file_tmp.close();
  }

  for (it = keyframelist.begin(); it != keyframelist.end(); it++)
  {
    Vector3d P;
    Matrix3d R;
    (*it)->getPose(P, R);
    Quaterniond Q;
    Q = R;

    pcl::PointCloud<pcl::PointXYZ> ptcl_ref;
    pcl::PointCloud<pcl::PointXYZ> ptcl_world;
    for(int j=0;j<(*it)->point_3d_ref.size();j++)
    {
      pcl::PointXYZ pt_ref;
      pt_ref.x = (*it)->point_3d_ref[j].x;
      pt_ref.y = (*it)->point_3d_ref[j].y;
      pt_ref.z = (*it)->point_3d_ref[j].z;
      ptcl_ref.push_back(pt_ref);
      pcl::PointXYZ pt_world;
      pt_world.x = (*it)->point_3d[j].x;
      pt_world.y = (*it)->point_3d[j].y;
      pt_world.z = (*it)->point_3d[j].z;
      ptcl_world.push_back(pt_world);
    }

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time((*it)->time_stamp);
    pose_stamped.header.frame_id = "world";
    pose_stamped.pose.position.x = P.x() + VISUALIZATION_SHIFT_X;
    pose_stamped.pose.position.y = P.y() + VISUALIZATION_SHIFT_Y;
    pose_stamped.pose.position.z = P.z();
    pose_stamped.pose.orientation.x = Q.x();
    pose_stamped.pose.orientation.y = Q.y();
    pose_stamped.pose.orientation.z = Q.z();
    pose_stamped.pose.orientation.w = Q.w();
    if((*it)->sequence == 0)
    {
      base_path.poses.push_back(pose_stamped);
      base_path.header = pose_stamped.header;
    }
    else
    {
      path[(*it)->sequence].poses.push_back(pose_stamped);
      path[(*it)->sequence].header = pose_stamped.header;
    }

    if (SAVE_LOOP_PATH)
    {
      ofstream loop_path_file(VINS_RESULT_PATH+ "/vio_loop.csv", ios::app);
      loop_path_file.setf(ios::fixed, ios::floatfield);
      loop_path_file.precision(9);
      loop_path_file << (*it)->time_stamp<< " ";
      loop_path_file.precision(6);
      loop_path_file << P.x() << " "
                     << P.y() << " "
                     << P.z() << " "
                     << Q.w() << " "
                     << Q.x() << " "
                     << Q.y() << " "
                     << Q.z() << endl;
      loop_path_file.close();
    }
    //draw local connection
  }
  publish();
  m_keyframelist.unlock();
}


void PoseGraph::publish()
{
  vector<KeyFrame*>::iterator it;
  for (it = keyframelist.begin(); it != keyframelist.end(); it++)
  {
    Vector3d P;
    Matrix3d R;
    (*it)->getPose(P, R);
    Quaterniond Q;
    Q = R;
  }
  for (int i = 1; i <= sequence_cnt; i++)
  {
    //if (sequence_loop[i] == true || i == base_sequence)
    if (1 || i == base_sequence)
    {
      pub_pg_path.publish(path[i]);
      if(i == base_sequence)
      {
        visualization_msgs::MarkerArray groupMarker;
        int prevgroup = -1;
        for(int j=0;j<path[i].poses.size();j++)
        {
          visualization_msgs::Marker marker;
          marker.header = path[i].header;
          marker.id = 10000+j;
          marker.type = visualization_msgs::Marker::SPHERE;
          marker.action = visualization_msgs::Marker::MODIFY;
          marker.pose = path[i].poses[j].pose;
          marker.scale.x = marker.scale.y = marker.scale.z = 0.05;
          int groupNum = -1;
          for(int k=0;k<grouplist.size();k++)
          {
            if(grouplist[k].back()< pathGroup[j]) continue;
            if(grouplist[k].front() > pathGroup[j]) break;
            groupNum = k;
          }
          cv::Vec3f color;
          color[0] =color[1]=color[2]= 1;
          if(groupNum>-1)
            color = heightcolor((double)groupNum/(double)(grouplist.size()-1));
          marker.color.r = color[0];
          marker.color.g = color[1];
          marker.color.b = color[2];
          marker.color.a = 1;
          groupMarker.markers.push_back(marker);
          if(prevgroup!=groupNum)
          {
            prevgroup = groupNum;
            if(groupNum == -1) continue;
            marker.id = 30000+groupNum;
            marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            marker.scale.z = 0.3;
            marker.pose.position.z = marker.pose.position.z+0.3;
            marker.text = std::to_string(groupNum);
            groupMarker.markers.push_back(marker);
          }
        }
        pub_pose_group.publish(groupMarker);
      }
      pub_path[i].publish(path[i]);
      posegraph_visualization->publish_by(pub_pose_graph, path[sequence_cnt].header);
    }
  }
  pub_base_path.publish(base_path);
  //posegraph_visualization->publish_by(pub_pose_graph, path[sequence_cnt].header);
}

void PoseGraph::searchByBRIEFDes(std::vector<int> &matched,
                                 const std::vector<BRIEF::bitset> &curgroup_descript,
                                 const std::map<int,BRIEF::bitset> &targroup_descript)
{
  for(int i = 0; i < (int)curgroup_descript.size(); i++)
  {
    matched.push_back(searchInAera(curgroup_descript[i],targroup_descript));
  }
}

int PoseGraph::searchInAera(const BRIEF::bitset descript,
                            const std::map<int,BRIEF::bitset> &targroup_descript)
{
  int bestDist = 128;
  int bestIndex = -1;
  for(auto it = targroup_descript.begin(); it != targroup_descript.end(); it++){
    int dis = HammingDis(descript, it->second);
    if(dis < bestDist)
    {
      bestDist = dis;
      bestIndex = it->first;
    }
  }
  //printf("best dist %d", bestDist);
  if (bestIndex != -1 && bestDist < 80)
  {
    return bestIndex;
  }
  else
    return -1;
}

int PoseGraph::HammingDis(const BRIEF::bitset &a, const BRIEF::bitset &b)
{
  BRIEF::bitset xor_of_bitset = a ^ b;
  int dis = xor_of_bitset.count();
  return dis;
}

cv::Vec3f PoseGraph::heightcolor(double h)
{
  if(h > 1) h = 1;
  if(h < 0) h = 0;

  h = h * 0.667;


  double color_R;
  double color_G;
  double color_B;
  // blend over HSV-values (more colors)

  double s = 1.0;
  double v = 1.0;

  h -= floor(h);
  h *= 6;
  int i;
  double m, n, f;

  i = floor(h);
  f = h - i;
  if (!(i & 1))
    f = 1 - f; // if i is even
  m = v * (1 - s);
  n = v * (1 - s * f);

  switch (i) {
  case 0:
    color_R = v; color_G = n; color_B = m;
    break;
  case 1:
    color_R = n; color_G = v; color_B = m;
    break;
  case 2:
    color_R = m; color_G = v; color_B = n;
    break;
  case 3:
    color_R = m; color_G = n; color_B = v;
    break;
  case 4:
    color_R = n; color_G = m; color_B = v;
    break;
  case 5:
    color_R = v; color_G = m; color_B = n;
    break;
  default:
    color_R = 1; color_G = 0.5; color_B = 0.5;
    break;
  }
  cv::Vec3f color;
  color[0] = color_R;
  color[1] = color_G;
  color[2] = color_B;
  return color;
}
