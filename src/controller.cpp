#include "pinocchio/parsers/sample-models.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/kinematics-derivatives.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "common.h"

#include "math.h"
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Float32MultiArray.h"
#include <boost/numeric/odeint.hpp>
#include <boost/numeric/odeint/external/eigen/eigen.hpp>

#include "asif_timing.h"

using namespace Eigen;

const int DOF=6;

typedef Eigen::VectorXd state_type;

bool backup = false;
Eigen::Vector3d xdes;  
state_type pos = Eigen::VectorXd::Zero(DOF);;
state_type vel = Eigen::VectorXd::Zero(DOF);;
state_type tau = Eigen::VectorXd::Zero(DOF);;

double BCK_X[2*DOF];
double BCK_U[DOF];
double BCK_DU[2*DOF*DOF];

double CLD_F[2*DOF];
double CLD_G[2*DOF*DOF];
double CLD_DFCK[4*DOF*DOF];

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

double timeScale = 1;

//Callback function for joystick node
void joy_cb(const sensor_msgs::Joy & msg) {
  if (msg.buttons[0] == 1){
    backup = false;
    ROS_INFO_THROTTLE(1,"CONTROLLER: TASK CONTROLLER ENGAGED");
  } else if (msg.buttons[1] == 1) {
    backup = true;
    ROS_INFO_THROTTLE(1,"CONTROLLER: BACKUP CONTROLLER ENGAGED");
  }
}

//Callback function for task_giver node
void end_effector_cb(const std_msgs::Float32MultiArray::ConstPtr& msg) {
  for (int i = 0; i < DOF/2; ++i)
    xdes[i] = msg->data[i];
}

//Callback function for to update position
void pos_cb(const std_msgs::Float32MultiArray::ConstPtr& msg) {
  for (int i = 0; i < DOF; ++i)
    pos[i] = msg->data[i];
}

//Callback function for to update velocity
void vel_cb(const std_msgs::Float32MultiArray::ConstPtr& msg) {
  for (int i = 0; i < DOF; ++i)
    vel[i] = msg->data[i];
}

void backup_controller(const double *x, double *u, double *Du) {

  const double u0 = -914;
  const double u1 = -4000;

  Map<Matrix<double,6,1>> uEigen(u);
  Map<Matrix<double,6,13>> DuEigen(Du); 
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
}

int main(int argc, char **argv) {
  pinocchio::Model model;
  pinocchio::urdf::buildModel(filename,model);
  pinocchio::Data data(model);
  pos = pinocchio::neutral(model);

  const int JOINT_ID = DOF; //ID of joint we are trying to track, DOF is the end effector tool position
  
  double kp = 2;       // proportional gain
  double kd = 1;       // derivative gain
  double max_acc = 1;  // max joint acceleration
  
  xdes << 1.912, 0, 2.055; //desired end effector position initialized here

  pinocchio::Data::Matrix6x J(6, model.nv); 
  J.setZero();
  unsigned int svdOptions = Eigen::ComputeThinU | Eigen::ComputeThinV;

  ros::init(argc, argv, "controller");
  ros::NodeHandle n;

  ros::param::get("~_kp", kp);
  ros::param::get("~_kd", kd);
  ros::param::get("~_Kp_1", Kp_1);
  ros::param::get("~_Kp_2", Kp_2);
  ros::param::get("~_Kd_0", Kd_0);
  ros::param::get("~_Kd_1", Kd_1);
  ros::param::get("~_Kd_2", Kd_2);
  ros::param::get("~_Kd_3", Kd_3);
  ros::param::get("~_Kd_4", Kd_4);
  ros::param::get("~_Kd_5", Kd_5);
  ros::param::get("~_max_acc", max_acc);
  ros::param::get("~_timeScale", timeScale); //timescale can used if computation is too slow

  ros::Subscriber end_effector_sub = n.subscribe<std_msgs::Float32MultiArray>("end_effector", 1, end_effector_cb);
  ros::Subscriber pos_sub = n.subscribe<std_msgs::Float32MultiArray>("joint/position", 1, pos_cb);
  ros::Subscriber vel_sub = n.subscribe<std_msgs::Float32MultiArray>("joint/velocity", 1, vel_cb);
  ros::Subscriber joy_sub = n.subscribe("/joy", 1, joy_cb);

  ros::Publisher controller_pub = n.advertise<std_msgs::Float32MultiArray>("torques", 1000);
  ros::Publisher v_controller_pub = n.advertise<std_msgs::Float32MultiArray>("velocity_desired", 1000);
  ros::Publisher a_controller_pub = n.advertise<std_msgs::Float32MultiArray>("acceleration_desired", 1000);

  // Initialize messages from controller
  std_msgs::Float32MultiArray msg;
  msg.data.clear();
  msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
  msg.layout.dim[0].size = DOF;
  for (int i = 0; i < DOF; i++) {
    msg.data.push_back(0.0);
  }

  std_msgs::Float32MultiArray vmsg;
  vmsg.data.clear();
  vmsg.layout.dim.push_back(std_msgs::MultiArrayDimension());
  vmsg.layout.dim[0].size = DOF;
  for (int i = 0; i < DOF; i++) {
    vmsg.data.push_back(0.0);
  }

  std_msgs::Float32MultiArray amsg;
  amsg.data.clear();
  amsg.layout.dim.push_back(std_msgs::MultiArrayDimension());
  amsg.layout.dim[0].size = DOF;
  for (int i = 0; i < DOF; i++) {
    amsg.data.push_back(0.0);
  }

  ros::Rate loop_rate(1000*timeScale);

  int count = 0;
  while (ros::ok()) {

    // Compute and send control action, desired velocity, and desired acceleration
    if (backup == false) {
      pinocchio::computeJointJacobians(model, data, pos);
      pinocchio::getJointJacobian(model, data, JOINT_ID, pinocchio::LOCAL, J);
      const Eigen::Vector3d & x   = data.oMi[JOINT_ID].translation();
      const Eigen::Matrix3d & R   = data.oMi[JOINT_ID].rotation();
      const Eigen::Vector3d & err = R.transpose()*(x-xdes);
      const Eigen::VectorXd vel_des = kp*-J.topRows<3>().bdcSvd(svdOptions).solve(err);

      for (int i = 0;i<DOF;i++) {
        vmsg.data[i] = vel_des[i];
      }
      v_controller_pub.publish(vmsg);

      // pd controller to get desired joint acceleration
      state_type acc = -kd * (vel - vel_des);
      acc = acc.cwiseMin(max_acc);
      acc = acc.cwiseMax(-max_acc);

      for (int i = 0;i<DOF;i++) {
        amsg.data[i] = acc[i];
      }
      a_controller_pub.publish(amsg);

      // calculate torques
      // tau = pinocchio::rnea(model, data, pos, vel, acc); //ALTERNATIVE controller uses RNEA to solve for torques

      pinocchio::computeGeneralizedGravity(model,data,pos);
      tau = data.g;
      tau[0] += -Kd_0*(vel[0]-vel_des[0]);
      tau[1] += -Kd_1*(vel[1]-vel_des[1]);
      tau[2] += -Kd_2*(vel[2]-vel_des[2]);
      tau[3] += -Kd_3*(vel[3]-vel_des[3]);
      tau[4] += -Kd_4*(vel[4]-vel_des[4]);
      tau[5] += -Kd_5*(vel[5]-vel_des[5]);
    }
    else {
      //to test backup controller (for implicit ASIF)

      memcpy(BCK_X, pos.data(), sizeof(double) * DOF);
      memcpy(BCK_X+DOF, vel.data(), sizeof(double) * DOF);

      backup_controller(BCK_X, BCK_U, BCK_DU);

      memcpy(tau.data(), BCK_U, sizeof(double) * DOF);
    }
    for (int i = 0;i<DOF;i++) {
      msg.data[i] = tau[i];
    }
    
    controller_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}