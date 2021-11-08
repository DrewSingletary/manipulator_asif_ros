#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/spatial/act-on-set.hpp"
#include "pinocchio/parsers/sample-models.hpp"
#include "pinocchio/utils/timer.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "common.h"

#include "pinocchio/parsers/sample-models.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/rnea-derivatives.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/aba-derivatives.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "math.h"
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Pose2D.h"
#include "std_msgs/Float32MultiArray.h"
#include "visualization_msgs/Marker.h"

#include <iostream>
#include <fstream>

#include <asif++.h>
#include "asif_timing.h"
#include <Eigen/QR> 

#include <chrono>
using namespace std::chrono;

using namespace Eigen;

ASIF::ASIF *asif;
double alpha = 1;
bool energy_based = false;
double alpha_e = 0;
double h_only = 0;
const int startJoint = 5;
const uint32_t DOF= 6;
const uint32_t nx = 6;
const uint32_t nu = 6;
const uint32_t npSS = DOF-startJoint;
const double lb[nu] = {-2,-2,-2,-2,-2,-2};
const double ub[nu] = {2,2,2,2,2,2};
ASIF_timing::ASIFtimer timer;

pinocchio::Model *model;
pinocchio::Data *data;

typedef VectorXd state_type;

state_type vdes = Eigen::VectorXd::Zero(DOF);
state_type rob_pos = Eigen::VectorXd::Zero(6);
state_type rob_vel = Eigen::VectorXd::Zero(6);
state_type human_pos = Eigen::VectorXd::Zero(3);

std::string sep = "\n----------------------------------------\n";
Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

ros::Publisher asif_pub;
ros::Publisher v_pub;
ros::Publisher h_pub;

double Kd_0 = 100;
double Kd_1 = 100;
double Kd_2 = 10;
double Kd_3 = 10;
double Kd_4 = 10;
double Kd_5 = 10;
double r_0 = 1.0;

// Updates robot joint positions
void rob_pos_cb(const std_msgs::Float32MultiArray::ConstPtr& msg) {
  for (int i = 0; i < DOF; ++i)
    rob_pos[i] = msg->data[i];
}

// Updates robot joint velocites
void rob_vel_cb(const std_msgs::Float32MultiArray::ConstPtr& msg) {
  for (int i = 0; i < DOF; ++i)
    rob_vel[i] = msg->data[i];
}

// Updates the human position to be used by CBF
void human_pos_cb(const geometry_msgs::Pose2D::ConstPtr& msg) {
    human_pos[0] = msg->x;
    human_pos[1] = msg->y;
}

//Safety set h(x), for both plain kinematic and energy-based barriers
void safetySet_(const double *x, double *h, double *Dh, 
                pinocchio::Model &model, pinocchio::Data &data)
{
  Map<Matrix<double,npSS,1>> hEigen(h);
  Map<Matrix<double,npSS,DOF>> DhEigen(Dh);
  state_type pos = Eigen::VectorXd::Zero(DOF);
  state_type q = Eigen::VectorXd::Zero(6);
  state_type vel = Eigen::VectorXd::Zero(DOF);
  double t = x[nx-1];

  hEigen.setZero();
  DhEigen.setZero();

  for (int i = 0;i<DOF;i++) {
    q[i] = x[i];
    pos[i] = x[i];
    vel[i] = x[i+DOF];
  }

  for (int i = 0;i<6-DOF;i++) {
    q[i+DOF] = 0;
  }


  pinocchio::Data::Matrix6x Jlocal(6, model.nv); 
  Jlocal.fill(0);

  pinocchio::forwardKinematics(model,data,q);
  pinocchio::computeJointJacobians(model, data, q);

  for (int i = startJoint;i<DOF;i++) {
    uint32_t jidx = i+1;
    Eigen::Vector3d Ec;
    Ec = data.oMi[jidx].translation();
    Jlocal.setZero();
    pinocchio::getJointJacobian(model, data, jidx, pinocchio::LOCAL_WORLD_ALIGNED, Jlocal);
    Eigen::Matrix<double,3,DOF> Jpos;
    Jpos = Jlocal.topRows(3);
    const double E[3] = {Ec[0],Ec[1],Ec[2]};

    Matrix<double, 3, nx> DKDx;
    DKDx.setZero();

    hEigen[i-startJoint] = (E[0]-human_pos[0])*(E[0]-human_pos[0])+(E[1]-human_pos[1])*(E[1]-human_pos[1])+
                (E[2]-r_0)*(E[2]-r_0)-(r_0*r_0+.25);
    // ROS_INFO("vals: %f, %f",hEigen[i-startJoint],h_tmp);

    Matrix<double,1,3> dhde;
    dhde[0] = 2*(E[0]-human_pos[0]);
    dhde[1] = 2*(E[1]-human_pos[1]);
    dhde[2] = 2*(E[2]-r_0);

    DKDx.block<3,DOF>(0,0) = Jpos;
    // DKDx.block<3,1>(0,nx-1) = Jpos*vel;

    DhEigen.block<1,nx>(i-startJoint,0) = dhde*DKDx;

    if (energy_based)
    {
      pinocchio::crba(model,data,rob_pos);
      data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
      Matrix<double,nx,nx> D = data.M;
      h_only = alpha_e * hEigen[i-startJoint];
      hEigen[i-startJoint] = hEigen[i-startJoint]*alpha_e - 0.5*rob_vel.transpose()*D*rob_vel;
      DhEigen.block<1,nx>(i-startJoint,0) = alpha_e*dhde*DKDx;
    }

 }
}

//The dynamics used here are simply the kinematics
void dynamics_(const double *x, double *f, double *g) {
  for (int i = 0; i < (int) DOF; i++) {
    for (int j = 0; j < (int) DOF; j++) {
      g[j] = 0;
    }
  }
  for (int i = 0; i < (int) DOF; i++) {
    f[i] = 0;
    g[i+DOF*i] = 1;
  }
}

//The desired velocity command from the controller node is filtered here.
void v_cb(const std_msgs::Float32MultiArray::ConstPtr& msg) {
  for (int i = 0; i < 6; ++i) {
    vdes[i] = msg->data[i];
  }
  std_msgs::Float32MultiArray u_msg;
  u_msg.data.clear();
  u_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
  u_msg.layout.dim[0].size = DOF;

  for (int i = 0; i < DOF; i++) {
    u_msg.data.push_back(0.0);
  }
  std_msgs::Float32MultiArray h_msg;
  h_msg.data.clear();
  h_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
  h_msg.layout.dim[0].size = DOF;
  for (int i = 0; i < npSS; i++) {
    if (energy_based)
    {
      h_msg.data.push_back(0.0);
      h_msg.data.push_back(0.0);
    }
    else
      h_msg.data.push_back(0.0);
  }

  std_msgs::Float32MultiArray v_msg;
  v_msg.data.clear();
  v_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
  v_msg.layout.dim[0].size = DOF;

  for (int i = 0; i < DOF; i++) {
    v_msg.data.push_back(0.0);
  }


  double xNow[DOF];
  std::copy(rob_pos.data(),rob_pos.data()+DOF,xNow);

  state_type vActNow = Eigen::VectorXd::Zero(DOF);

  ASIF_timing::tic(&timer);
  state_type h = Eigen::VectorXd::Zero(npSS);
  Matrix<double,npSS,DOF> dh; dh.setZero();
  safetySet_(rob_pos.data(),h.data(),dh.data(),*model,*data);
  Matrix<double,npSS,1> Lfh; Lfh.setZero();
  Matrix<double,npSS,nu> Lgh; Lgh.setZero();
  Matrix<double,DOF,1> f; f.setZero();
  Matrix<double,DOF,nu> g; g.setZero();
  dynamics_(rob_pos.data(),f.data(),g.data());
  Lfh = dh*f;
  Matrix<double,1,nu> BKqdot;
  BKqdot(0) = Kd_0*(rob_vel[0]-vdes(0));
  BKqdot(1) = Kd_1*(rob_vel[1]-vdes(1));
  BKqdot(2) = Kd_2*(rob_vel[2]-vdes(2));
  BKqdot(3) = Kd_3*(rob_vel[3]-vdes(3));
  BKqdot(4) = Kd_4*(rob_vel[4]-vdes(4));
  BKqdot(5) = Kd_5*(rob_vel[5]-vdes(5));
  Lgh = dh*g;
  if (energy_based)
    Lgh += BKqdot;
  Eigen::MatrixXd Dhinv =  dh.transpose()*(dh*dh.transpose()).inverse();
  Eigen::Matrix<double,DOF,1> vhard;
  vhard = vdes + -Dhinv*(dh*vdes+alpha*h);
  auto start = high_resolution_clock::now();
  int32_t rc = asif->filter(xNow,vdes.data(),vActNow.data(),Lfh.data(),Lgh.data());
  auto stop = high_resolution_clock::now();
  auto duration = duration_cast<microseconds>(stop - start);
  std::cout << "timing (us): " << duration.count() << std::endl;
  ASIF_timing::toc(&timer);

  if (rc != 1)
    std::cout << "QP failed!" << std::endl;

  for (int i = 0; i < nu; i++) {
    v_msg.data[i] = vActNow[i];
  }
  v_pub.publish(v_msg);

  //filtered velocity command converted to torques
  pinocchio::computeGeneralizedGravity(*model,*data,rob_pos);
  state_type tau = Eigen::VectorXd::Zero(DOF);
  tau = data->g;
  tau[0] += -Kd_0*(rob_vel[0]-vActNow[0]);
  tau[1] += -Kd_1*(rob_vel[1]-vActNow[1]);
  tau[2] += -Kd_2*(rob_vel[2]-vActNow[2]);
  tau[3] += -Kd_3*(rob_vel[3]-vActNow[3]);
  tau[4] += -Kd_4*(rob_vel[4]-vActNow[4]);
  tau[5] += -Kd_5*(rob_vel[5]-vActNow[5]);

  for (int i = 0;i<DOF;i++) {
    u_msg.data[i] = tau[i];
  }

  for (int i = 0;i<npSS;i++) 
  {
    if (energy_based)
    {
      //output both h and h_D if using energy-based method for logging
      h_msg.data[2*i] = h[i];
      h_msg.data[2*i+1] = h_only;
    }
    else
    {
      h_msg.data[i] = h[i];
    }
  }
  
  asif_pub.publish(u_msg);
  h_pub.publish(h_msg);
}

int main(int argc, char **argv) {

  model = new pinocchio::Model();
  pinocchio::urdf::buildModel(filename,*model);
  data = new pinocchio::Data(*model);

  //Set human position far away for initialization
  human_pos[0] = 10.0;
  human_pos[1] = 10.0;

  ros::init(argc, argv, "ASIF");

  ros::NodeHandle n;

  ros::Subscriber rob_pos_sub = n.subscribe<std_msgs::Float32MultiArray>("/irb6640/joint/position", 1, rob_pos_cb);
  ros::Subscriber rob_vel_sub = n.subscribe<std_msgs::Float32MultiArray>("/irb6640/joint/velocity", 1, rob_vel_cb);
  ros::Subscriber v_sub = n.subscribe<std_msgs::Float32MultiArray>("/irb6640/velocity_desired", 1, v_cb);

  ros::Subscriber human_pos_sub = n.subscribe<geometry_msgs::Pose2D>("/randy/position", 1, human_pos_cb);
  //ros::Subscriber rob_vel_sub = n.subscribe<std_msgs::Float32MultiArray>("/randy/velocity", 1, human_vel_cb);

  v_pub = n.advertise<std_msgs::Float32MultiArray>("/irb6640/velocity", 1);
  asif_pub = n.advertise<std_msgs::Float32MultiArray>("/irb6640/torques_actual", 1);
  h_pub = n.advertise<std_msgs::Float32MultiArray>("/h_val", 1);

  ros::param::get("~_Kd_0", Kd_0);
  ros::param::get("~_Kd_1", Kd_1);
  ros::param::get("~_Kd_2", Kd_2);
  ros::param::get("~_Kd_3", Kd_3);
  ros::param::get("~_Kd_4", Kd_4);
  ros::param::get("~_Kd_5", Kd_5);
  ros::param::get("~_energy_based", energy_based);
  ros::param::get("~_alpha", alpha);
  ros::param::get("~_alpha_e", alpha_e);
  ros::param::get("~_r_0", r_0);

  ASIF::ASIF::Options opts;
  opts.relaxLb = alpha;

  auto safetySet = [&]
  (const double *x, double *h, double *Dh)
  ->void
  {
    safetySet_(x, h, Dh, *model,*data);
  };

  auto dynamics = [&]
  (const double *x, double *f, double *g)
  ->void
  {
    dynamics_(x, f, g);
  };


  asif = new ASIF::ASIF(DOF,nu,npSS,safetySet,dynamics);

  ROS_INFO("INITIALIZING ASIF");
  asif->initialize(lb,ub,opts);
  ROS_INFO("ASIF INITIALIZED");


  while (ros::ok()) {
    ros::spinOnce();
  }
  return 0;
}