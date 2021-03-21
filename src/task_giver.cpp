#include "common.h"
#include "pinocchio/parsers/sample-models.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/rnea.hpp"

#include "math.h"
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include <boost/numeric/odeint.hpp>
#include <boost/numeric/odeint/external/eigen/eigen.hpp>

const int DOF = 6;

Eigen::VectorXd pos = Eigen::VectorXd::Zero(DOF);;

void pos_cb(const std_msgs::Float32MultiArray::ConstPtr& msg) {
  for (int i = 0; i < DOF; ++i)
    pos[i] = msg->data[i];
}

int main(int argc, char **argv) {

  pinocchio::Model model;
  pinocchio::urdf::buildModel(filename,model);
  pinocchio::Data data(model);


  int current_task = 0;
  const int total_tasks = 4;
  Eigen::Vector3d errVec;
  double err;
  Eigen::Matrix<double,DOF,total_tasks> qdes;
  Eigen::Matrix<double, 3, total_tasks> xdes;    
  for (int i = 0; i < total_tasks; i++) {
      qdes.col(i) = pinocchio::neutral(model);
  }  
  qdes(0,1) += M_PI/2;
  qdes(0,3) -= M_PI/2;
  qdes(1,1) += M_PI/8;
  qdes(1,3) += M_PI/8;
  qdes(1,0) += 3*M_PI/8;
  qdes(1,2) += 3*M_PI/8;
  for (int i = 0; i < total_tasks; i++) {
    Eigen::VectorXd qdesi = qdes.col(i);
    pinocchio::computeJointJacobians(model, data, qdesi);
    const Eigen::Vector3d & xdesi   = data.oMi[DOF].translation();        
    xdes.col(i) << xdesi;
  }
  // xdes.col(0) << 0, 0, 3.5;
  // xdes.col(1) << 0, 0, 3.5;
  // xdes.col(2) << 0, 0, 3.5;
  // xdes.col(3) << 0, 0, 3.5;
  // xdes.col(0) << 1.912, 0, 2.055;
  // xdes.col(1) << 1.912, 0, 2.055;
  // xdes.col(2) << 1.912, 0, 2.055;
  // xdes.col(3) << 1.912, 0, 2.055;
  
  pinocchio::Data::Matrix6x J(6, model.nv); 
  J.setZero();
  const int JOINT_ID = DOF;

  // ros
  ros::init(argc, argv, "task_giver");

  ros::NodeHandle n;

  ros::Subscriber pos_sub = n.subscribe<std_msgs::Float32MultiArray>("joint/position", 1, pos_cb);

  ros::Publisher end_pub = n.advertise<std_msgs::Float32MultiArray>("end_effector", 1000);

  std_msgs::Float32MultiArray msg;
  msg.data.clear();
  msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
  msg.layout.dim[0].size = 3;
  msg.data = {0,0,0};

  ros::Rate loop_rate(200);

  int count = 0;
  while (ros::ok()) {

    // solve for desired joint velocity
    pinocchio::computeJointJacobians(model, data, pos);
    pinocchio::getJointJacobian(model, data, JOINT_ID, pinocchio::LOCAL, J);
    const Eigen::Vector3d & x   = data.oMi[JOINT_ID].translation();
    ROS_INFO("%f %f %f",x[0],x[1],x[2]);
    ROS_INFO("%f %f %f",xdes(0,current_task),xdes(1,current_task),xdes(2,current_task));
    errVec(0) = x[0]-xdes(0,current_task);
    errVec(1) = x[1]-xdes(1,current_task);
    errVec(2) = x[2]-xdes(2,current_task);
    err = errVec.norm();

    ROS_INFO("error: %f",err);

    if (err < 0.1 && current_task<total_tasks-1) {
      current_task++;
      ROS_INFO("TASK: Task changed. Current task: %i", current_task);
    }
    else if (err < 0.1 && current_task==total_tasks-1) {
      current_task = 0;
      ROS_INFO("TASK: Task changed. Current task: %i", current_task);
    }

    msg.data[0] = xdes(0,current_task);
    msg.data[1] = xdes(1,current_task);
    msg.data[2] = xdes(2,current_task);
    
    end_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}