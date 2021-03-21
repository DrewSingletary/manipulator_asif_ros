#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/spatial/act-on-set.hpp"
#include "pinocchio/parsers/sample-models.hpp"
#include "pinocchio/utils/timer.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"

#include "pinocchio/parsers/sample-models.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/rnea-derivatives.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/aba-derivatives.hpp"

#include "common.h"

#include "math.h"
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Pose2D.h"
#include "std_msgs/Float32MultiArray.h"

#include <iostream>
#include <fstream>

#include <asif++.h>
#include "asif_timing.h"

using namespace Eigen;

ASIF::ASIFimplicit *asif;

bool logging = false;

const int startJoint = 2;
const uint32_t DOF= 6;
const uint32_t nx = 2*DOF+1;
const uint32_t nu = 6;
const uint32_t npSS = DOF-startJoint;
const uint32_t npBS = 2;
const uint32_t npBTSS = 6;

const double lb[nu] = {-10000,-15000,-3000,-100,-100,-100};
const double ub[nu] = {10000,15000,3000,100,100,100};

const double r_0 = .5;
const double v_max = 0;
const double H_h = 2;
const double t_max = 1;

const double kp_backup = 60;
const double kd_backup = 20; 

double Kp_1 = 100;
double Kp_2 = 100;
double Kd_0 = 100;
double Kd_1 = 100;
double Kd_2 = 10;
double Kd_3 = 10;
double Kd_4 = 10;
double Kd_5 = 10;

std::ofstream logfile;
std::ofstream logfile2;

ASIF_timing::ASIFtimer timer;

typedef VectorXd state_type;
typedef std::vector<double> state_t;

state_type udes = Eigen::VectorXd::Zero(DOF);

state_type rob_pos = Eigen::VectorXd::Zero(6);
state_type rob_vel = Eigen::VectorXd::Zero(6);
state_type human_pos = Eigen::VectorXd::Zero(3);

std::string sep = "\n----------------------------------------\n";
Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");


ros::Publisher asif_pub;



void rob_pos_cb(const std_msgs::Float32MultiArray::ConstPtr& msg) {
  for (int i = 0; i < DOF; ++i)
    rob_pos[i] = msg->data[i];
}

void rob_vel_cb(const std_msgs::Float32MultiArray::ConstPtr& msg) {
  for (int i = 0; i < DOF; ++i)
    rob_vel[i] = msg->data[i];
}

void human_pos_cb(const geometry_msgs::Pose2D::ConstPtr& msg) {
    human_pos[0] = msg->x;
    human_pos[1] = msg->y;
}

void safetySet_(const double *x, double *h, double *Dh, 
                pinocchio::Model &model, pinocchio::Data &data)
{
  Map<Matrix<double,npSS,1>> hEigen(h);
  Map<Matrix<double,npSS,nx>> DhEigen(Dh);
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
    pinocchio::getJointJacobian(model, data, jidx, pinocchio::LOCAL, Jlocal);
    Eigen::Matrix<double,3,DOF> Jpos;
    Jpos = data.oMi[jidx].rotation()*Jlocal.topRows(3);

    const double E[4] = {Ec[0],Ec[1],Ec[2],t};

    Matrix<double, 4, nx> DKDx;
    DKDx.setZero();

    hEigen[i-startJoint] = (E[0]-human_pos[0])*(E[0]-human_pos[0])+(E[1]-human_pos[1])*(E[1]-human_pos[1])+
                (E[2]-H_h/2)*(E[2]-H_h/2)/((H_h*H_h/4)/((r_0+v_max*E[3])*r_0+v_max*E[3]))-
                (r_0+v_max*E[3])*(r_0+v_max*E[3]);
    if (hEigen(i-startJoint)< 0) {
      ROS_INFO("h: %f End effector position of joint %i: (%f, %f, %f, %f)",hEigen(i-startJoint),i,E[0],E[1],E[2],E[3]);
    }
    Matrix<double,1,4> dhde;
    dhde[0] = 2*(E[0]-human_pos[0]);
    dhde[1] = 2*(E[1]-human_pos[1]);
    dhde[2] = 8*((r_0+E[3]*v_max)*(r_0+E[3]*v_max)*(E[2]-H_h/2))/(H_h*H_h);
    dhde[3] = -2*v_max*(r_0+E[3]*v_max)+(8*v_max*(r_0+E[3]*v_max)*(E[2]-H_h/2)*(E[2]-H_h/2))/(H_h*H_h);
    
    DKDx.block<3,DOF>(0,0) = Jpos;
    DKDx.block<3,1>(0,nx-1) = Jpos*vel;
    DKDx(3,nx-1) = 1;

    DhEigen.block<1,nx>(i-startJoint,0) = dhde*DKDx;

    // std::cout << "Human:" << std::endl;
    // std::cout << sep;
    // std::cout << human_pos.format(CleanFmt) << sep;
 }
}

void backupSet_(const double *x, double *h, double *Dh, 
                pinocchio::Model &model, pinocchio::Data &data)
{
  Map<Matrix<double,npBS,nx>> DhEigen(Dh);
  Map<Matrix<double,npBS,1>> hEigen(h);
  hEigen.setZero();
  h[0] = M_PI/6*M_PI/6-x[1]*x[1];
  h[1] = M_PI/6*M_PI/6-(x[2]+M_PI/2)*(x[2]+M_PI/2);
  DhEigen.setZero();
  DhEigen(0,1) = -2*x[1];
  DhEigen(1,2) = -2*x[2]-M_PI;
}

void backupController_(const double *x, double *u, double *Du,
                       pinocchio::Model &model, pinocchio::Data &data) {

  const double u0 = -914;
  const double u1 = -4000;

  Map<Matrix<double,nu,1>> uEigen(u);
  Map<Matrix<double,nu,nx>> DuEigen(Du); 
  uEigen.setZero();
  DuEigen.setZero(); 

  Eigen::Vector3d bck_pos_des;
  bck_pos_des(0) = 0;
  bck_pos_des(1) = -M_PI/2;

  uEigen(0) = -Kd_0*x[6];
  uEigen(2) = -Kp_2*((x[1]+x[2])-bck_pos_des(1))-Kd_2*x[8]+u0*cos(x[2]+x[1]);
  uEigen(1) = -Kp_1*(x[1]-bck_pos_des(0))-Kd_1*x[7]+uEigen(2)+u1*sin(x[1]);
  uEigen(3) = -Kd_3*x[9];
  uEigen(4) = -Kd_4*x[10];
  uEigen(5) = -Kd_5*x[11];

  DuEigen(0,6) = -Kd_0;

  DuEigen(1,1) = -Kp_1 -u0*sin(x[1]+x[2])-Kp_1+u1*cos(x[1]);
  DuEigen(1,2) = -Kp_1 -u0*sin(x[1]+x[2]);
  DuEigen(1,7) = -Kd_1;
  DuEigen(1,8) = -Kd_1;

  DuEigen(2,1) = -Kp_2 -u0*sin(x[1]+x[2]);
  DuEigen(2,2) = -Kp_2 -u0*sin(x[1]+x[2]);
  DuEigen(2,8) = -Kd_2;

  DuEigen(3,9) = -Kd_3;
  DuEigen(4,10) = -Kd_4;
  DuEigen(5,11) = -Kd_5;

  // std::cout << "Backup Controller" << std::endl;
  // std::cout << uEigen.format(CleanFmt) << sep;
  // std::cout << DuEigen.format(CleanFmt) << sep;

}

// void backupController_(const double *x, double *u, double *Du,
//                        pinocchio::Model &model, pinocchio::Data &data)
// {
//   Map<Matrix<double,nu,1>> uEigen(u);
//   Map<Matrix<double,nu,nx>> DuEigen(Du);
//   DuEigen.setZero();
//   uEigen.setZero();
//   state_type pos = Eigen::VectorXd::Zero(DOF);
//   state_type vel = Eigen::VectorXd::Zero(DOF);
//   double t = x[nx-1];

//   for (int i = 0;i<DOF;i++) {
//     pos[i] = x[i];
//     vel[i] = x[i+DOF];
//   }

//   // Eigen::Matrix<double, DOF, 1> pos;
//   // Eigen::Matrix<double, DOF, 1> vel;
//   Eigen::Matrix<double, DOF, 1> bck_ddq;
//   Eigen::Matrix<double, DOF, 1> bck_tau;

//   Eigen::Matrix<double, DOF, 1> pos_des;
//   Eigen::Matrix<double, DOF, 1> bck_diagonal;

//   Eigen::Matrix<double, DOF, DOF> rnea_partial_dq;
//   Eigen::Matrix<double, DOF, DOF> rnea_partial_dv;
//   Eigen::Matrix<double, DOF, DOF> rnea_partial_da;

//   Eigen::Matrix<double, DOF, 2*DOF> tau_partial_x;
//   bck_ddq.setZero();
//   bck_tau.setZero();
//   pos_des.setZero();
//   bck_diagonal.setZero();
//   rnea_partial_dq.setZero();
//   rnea_partial_dv.setZero();
//   rnea_partial_da.setZero();
//   tau_partial_x.setZero();

//   bck_diagonal << 0,1,1,0,0,0;

//   pos_des[0] = pos[0];
//   pos_des[1] = 0;
//   pos_des[2] = -M_PI/2;
//   pos_des[3] = pos[3];
//   pos_des[4] = pos[4];
//   pos_des[5] = pos[5];

//   bck_ddq = -kp_backup*(pos-pos_des) - kd_backup*vel;

//   bck_tau = pinocchio::rnea(model, data, pos, vel, bck_ddq);
//   pinocchio::computeRNEADerivatives(model, data, pos, vel, bck_ddq,
//     rnea_partial_dq, rnea_partial_dv, rnea_partial_da);

//   tau_partial_x.block<DOF, DOF>(0,0) = rnea_partial_dq+ rnea_partial_da * (-kp_backup) * (bck_diagonal.asDiagonal());
//   tau_partial_x.block<DOF, DOF>(0, DOF) = rnea_partial_dv + rnea_partial_da * (-kd_backup);

//   DuEigen.block<nu,nx-1>(0,0) = tau_partial_x;
//   uEigen = bck_tau;
// }

void dynamicsWithGradient_(const double *x, const double *u, double *f, 
                           double *g, double *d_fcl_dx,
                           pinocchio::Model &model, pinocchio::Data &data) 
{
  //   std::cout << "nx: " << nx << std::endl;
  // std::cout << "nu: " << nu << std::endl;
  // std::cout << "DOF: " << DOF << std::endl;

  Map<Matrix<double,nx,1>> fEigen(f);
  Map<Matrix<double,nx,nu>> gEigen(g);
  Map<Matrix<double,nx,nx>> d_fcl_dxEigen(d_fcl_dx);

  // std::cout << "Maped" << std::endl;
  fEigen.setZero();
  gEigen.setZero();
  d_fcl_dxEigen.setZero();

    // std::cout << "setZero" << std::endl;


  Eigen::Matrix<double, DOF, 1> cld_q;
  Eigen::Matrix<double, DOF, 1> cld_dq;
  Eigen::Matrix<double, DOF, 1> cld_ddq;
  cld_q.setZero();
  cld_dq.setZero();
  cld_ddq.setZero();

  Eigen::Matrix<double, DOF, 1> cld_tau;
  cld_tau.setZero();

  Eigen::Matrix<double, 2*DOF, 1> cld_f;
  Eigen::Matrix<double, 2*DOF, DOF> cld_g;
  Eigen::Matrix<double, 2*DOF, 2*DOF> cld_Dfcl_dx;
  cld_f.setZero();
  cld_g.setZero();
  cld_Dfcl_dx.setZero();

  Eigen::Matrix<double, DOF, DOF> aba_partial_dq;
  Eigen::Matrix<double, DOF, DOF> aba_partial_dv;
  Eigen::Matrix<double, DOF, DOF> aba_partial_dtau;
  aba_partial_dq.setZero();
  aba_partial_dv.setZero();
  aba_partial_dtau.setZero();

  // x = [q; dq]
  memcpy(cld_q.data(), x, sizeof(double) * DOF);
  memcpy(cld_dq.data(), x+DOF, sizeof(double) * DOF);
  memcpy(cld_tau.data(), u, sizeof(double) * nu);

  // std::cout << "q, dq, tau" << std::endl;
  // std::cout << cld_q.format(CleanFmt) << sep;
  // std::cout << cld_dq.format(CleanFmt) << sep;
  // std::cout << cld_tau.format(CleanFmt) << sep;

  // compute acceleration ddq
  cld_ddq = pinocchio::aba(model, data,
    cld_q,
    cld_dq,
    cld_tau);

  pinocchio::computeABADerivatives(model, data,
    cld_q, 
    cld_dq,
    cld_tau,
    aba_partial_dq,
    aba_partial_dv,
    aba_partial_dtau);

  // g = [0; ddq/dtau]
  cld_g.block<DOF, DOF>(0, 0) = Eigen::MatrixXd::Zero(DOF, DOF);
  cld_g.block<DOF, DOF>(DOF, 0) = aba_partial_dtau;
  cld_g.block<3,DOF>(DOF+3,0) = Eigen::MatrixXd::Zero(3, DOF);

  // f = [dq; ddq] - g * tau
  cld_f.head(DOF) = cld_dq;
  cld_f.tail(DOF) = cld_ddq;
  cld_f -= cld_g * cld_tau;
  cld_f(9) = 0;
  cld_f(10) = 0;
  cld_f(11) = 0;

  // Dfcl_dx
  cld_Dfcl_dx.block<DOF, DOF>(0,0) = Eigen::Matrix<double, DOF, DOF>::Zero();
  cld_Dfcl_dx.block<DOF, DOF>(0,DOF) = Eigen::Matrix<double, DOF, DOF>::Identity();
  cld_Dfcl_dx.block<DOF, DOF>(DOF,0) = aba_partial_dq;
  cld_Dfcl_dx.block<DOF, DOF>(DOF,DOF) = aba_partial_dv;
  cld_Dfcl_dx.block<3,DOF*2>(DOF+3,0) =Eigen::Matrix<double, 3, DOF*2>::Zero();

  fEigen.block<nx-1,1>(0,0) = cld_f;
  fEigen(nx-1,0) = 1.0;
  gEigen.block<nx-1,nu>(0,0) = cld_g;
  d_fcl_dxEigen.block<nx-1,nx-1>(0,0) = cld_Dfcl_dx;

  // std::cout << "f, g, Dxdot" << std::endl;
  // std::cout << fEigen.format(CleanFmt) << sep;
  // std::cout << gEigen.format(CleanFmt) << sep;
  // std::cout << d_fcl_dxEigen.format(CleanFmt) << sep;
}

void u_cb(const std_msgs::Float32MultiArray::ConstPtr& msg) {
  // ROS_INFO("HI");
  std_msgs::Float32MultiArray u_msg;
  u_msg.data.clear();
  u_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
  u_msg.layout.dim[0].size = DOF;

  for (int i = 0; i < DOF; i++) {
    u_msg.data.push_back(0.0);
  }
  for (int i = 0; i < 6; ++i) {
    udes[i] = msg->data[i];
  }

  double xNow[nx];
  std::copy(rob_pos.data(),rob_pos.data()+DOF,xNow);
  std::copy(rob_vel.data(),rob_vel.data()+DOF,xNow+DOF);
  xNow[nx-1] = 0;

  double uActNow[nu];

  ASIF_timing::tic(&timer);
  // std::cout << "starting asif filter" << std::endl;
  int32_t rc = asif->filter(xNow,udes.data(),uActNow);
  ASIF_timing::toc(&timer);
  // std::cout << timer.dt*1e6;

  // std::vector<state_t> &BT = asif->backTraj_;
  double h[npSS];
  double dh[npSS*nx];
  double x[nx];

  if (logging == true) {
  ASIF_timing::tic(&timer);

  // for (int i = 0; i < BT.size();i++) {
  //   for (int j = 0; j < nx;j++) {
  //     logfile << BT[i][j] << ",";
  //     x[j] = BT[i][j];
  //   }
  //   asif->safetySet_(x,h,dh);
  //   for (int j = 0; j < npSS;j++) {
  //     logfile << h[j] << ",";
  //   } 
  // }
  for (int i = 0; i < nx; i++) {
    logfile2 << xNow[i]<< ",";
  }
  for (int i = 0; i < nu; i++) {
    logfile2 << udes[i]<< ",";
  }
  for (int i = 0; i < nu; i++) {
    logfile2 << uActNow[i] << ",";
  }
  // asif->safetySet_(xNow,h,dh);
  // for (int j = 0; j < npSS;j++) {
  //      logfile2 << h[j] << ",";
  // } 
  // logfile << "\n";
  // logfile.flush();
  logfile2 << "\n";
  logfile2.flush();

  ASIF_timing::toc(&timer);
  // std::cout << timer.dt*1e6 << std::endl;
  }

  if (rc != 1)
    std::cout << "fail" << std::endl;
  // else
  //   std::cout << "succeed" << std::endl;

  for (int i = 0; i < nu; i++) {
    u_msg.data[i] = uActNow[i];
  }
  
  asif_pub.publish(u_msg);
}



int main(int argc, char **argv) {

  pinocchio::Model model;
  pinocchio::urdf::buildModel(filename,model);
  pinocchio::Data data = pinocchio::Data(model);

  human_pos[0] = 10.0;
  human_pos[1] = 10.0;

  {
    // ASIF::ASIFtimer timer;

    double h[npSS];
    double dh[npSS*nx];

    double x[nx] = {0.0};
    safetySet_(x,h,dh, model,data);
    Map<Matrix<double,npSS,1>> hEigen(h);
    Map<Matrix<double,npSS,nx>> DhEigen(dh);
    std::string sep = "\n----------------------------------------\n";
    Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
    std::cout << "Safety Set" << std::endl;
    std::cout << sep;
    std::cout << hEigen.format(CleanFmt) << sep;
    std::cout << DhEigen.format(CleanFmt) << sep;

    double bh[npBS];
    double bdh[npBS*nx];
    backupSet_(x,bh,bdh, model,data);
    Map<Matrix<double,npBS,1>> bhEigen(bh);
    Map<Matrix<double,npBS,nx>> bDhEigen(bdh);
    std::cout << "Backup Set" << std::endl;
    std::cout << bhEigen.format(CleanFmt) << sep;
    std::cout << bDhEigen.format(CleanFmt) << sep;

    double u[nu];
    double Du[nu*nx];
    backupController_(x, u, Du, model,data);
    Map<Matrix<double,nu,1>> uEigen(u);
    Map<Matrix<double,nu,nx>> DuEigen(Du);
    std::cout << "Backup Controller" << std::endl;
    std::cout << uEigen.format(CleanFmt) << sep;
    std::cout << DuEigen.format(CleanFmt) << sep;

    double f[nx];
    double g[nx*nu];
    double Df[nx*nx];
    dynamicsWithGradient_(x, u, f, g, Df, model,data);
    Map<Matrix<double,nx,1>> fEigen(f);
    Map<Matrix<double,nx,nu>> gEigen(g);
    Map<Matrix<double,nx,nx>> d_fcl_dxEigen(Df);
    std::cout << "Dynamics" << std::endl;
    std::cout << fEigen.format(CleanFmt) << sep;
    std::cout << gEigen.format(CleanFmt) << sep;
    std::cout << d_fcl_dxEigen.format(CleanFmt) << sep;
  }

  // // ros
  ros::init(argc, argv, "ASIF");

  ros::NodeHandle n;

  ros::param::get("~_Kp_1", Kp_1);
  ros::param::get("~_Kp_2", Kp_2);
  ros::param::get("~_Kd_0", Kd_0);
  ros::param::get("~_Kd_1", Kd_1);
  ros::param::get("~_Kd_2", Kd_2);
  ros::param::get("~_Kd_3", Kd_3);
  ros::param::get("~_Kd_4", Kd_4);
  ros::param::get("~_Kd_5", Kd_5);

  ros::Subscriber rob_pos_sub = n.subscribe<std_msgs::Float32MultiArray>("/irb6640/joint/position", 1, rob_pos_cb);
  ros::Subscriber rob_vel_sub = n.subscribe<std_msgs::Float32MultiArray>("/irb6640/joint/velocity", 1, rob_vel_cb);
  ros::Subscriber u_sub = n.subscribe<std_msgs::Float32MultiArray>("/irb6640/torques", 1, u_cb);

  ros::Subscriber human_pos_sub = n.subscribe<geometry_msgs::Pose2D>("/randy/position", 1, human_pos_cb);
  //ros::Subscriber rob_vel_sub = n.subscribe<std_msgs::Float32MultiArray>("/randy/velocity", 1, human_vel_cb);

  asif_pub = n.advertise<std_msgs::Float32MultiArray>("/irb6640/torques_actual", 1);

  ASIF::ASIFimplicit::Options opts;
  opts.backTrajHorizon = 1.5;
  opts.backTrajDt = 0.01;
  // opts.relaxReachLb = .0001;
  // opts.relaxSafeLb = .0001;

  auto safetySet = [&]
  (const double *x, double *h, double *Dh)
  ->void
  {
    safetySet_(x, h, Dh, model,data);
  };

  auto backupSet = [&]
  (const double *x, double *h, double *Dh)
  ->void
  {
    backupSet_(x, h, Dh, model,data);
  };

  auto dynamicsWithGradient = [&]
  (const double *x, const double *u, double *f, double *g, double *d_fcl_dx)
  ->void
  {
    dynamicsWithGradient_(x, u, f, g, d_fcl_dx, model,data);
  };

  auto backupController = [&]
  (const double *x, double *u, double *Du)
  ->void
  {
    backupController_(x, u, Du, model,data);
  };


  asif = new ASIF::ASIFimplicit(nx,nu,npSS,npBS,npBTSS,safetySet,
                                backupSet,dynamicsWithGradient,backupController);

  if (logging == true) {
    logfile.open ("/home/drew/Desktop/ASIF.csv");
    logfile2.open ("/home/drew/Desktop/ASIF_basic.csv");
  }
  ROS_INFO("INITIALIZING ASIF");
  asif->initialize(lb,ub,opts);
  ROS_INFO("ASIF INITIALIZED");


  while (ros::ok()) {
    ros::spinOnce();
  }
  return 0;
}