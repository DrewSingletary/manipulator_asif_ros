#include "pinocchio/multibody/model.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/aba-derivatives.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "common.h"
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include <boost/numeric/odeint.hpp>
#include <stdlib.h>
#include <sensor_msgs/JointState.h>

#include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

typedef Eigen::VectorXd state_type;

int DOF = 6;
uint64_t start, finish;
double t;
double timeScale = 1;

pinocchio::Model *model;
pinocchio::Data *data;

class Integrator {
public:
  Integrator() {
    q = pinocchio::neutral(*model);
    v = Eigen::VectorXd::Zero(model->nv);
    x.resize(2*model->nv,0);
    for (int i = 0; i < DOF; ++i)
    {
      x[i] = q[i];
      x[DOF+i] = v[i];
    }
    tau = pinocchio::rnea(*model, *data, q, v, Eigen::VectorXd::Zero(DOF));
  }

  void callback(const std_msgs::Float32MultiArray::ConstPtr& input) {
    for (int i = 0; i < DOF; i++) {
      tau[i] = input->data[i];
    } 
  }

  void rhs(const std::vector<double> &x, std::vector<double> &dxdt) {
    for (int i = 0; i < DOF; i++) {
      dxdt[i] = x[DOF+i];
    }
    acc = pinocchio::aba(*model, *data, q, v, tau);
    for (int i = 0; i < DOF; i++) {
      dxdt[DOF+i] = acc[i];
    }
    for (int i = 9; i < 12; i++) {
      dxdt[i] = 0;
    }
  }

  void integrate(double dt) {
    t = 0.0;
    boost::numeric::odeint::integrate(boost::bind(&Integrator::rhs, this, _1, _2), 
                                      x, t, t+dt, 0.001);
    t += dt;
    q = Eigen::Map<Eigen::VectorXd>(x.data(),DOF);
    v = Eigen::Map<Eigen::VectorXd>(x.data()+DOF,DOF);
  }

  void sendTransformCurrent(void)
  {
    static tf::TransformBroadcaster odom_broadcaster;
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = "world";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = 0.;
    odom_trans.transform.translation.y = 0.;
    odom_trans.transform.translation.z = 0.;

    tf2::Quaternion myQuaternion;
    myQuaternion.setRPY( 0, 0, 0 );
    geometry_msgs::Quaternion odom_quat;
    odom_quat.w = myQuaternion[0];
    odom_quat.x = myQuaternion[1];
    odom_quat.y = myQuaternion[2];
    odom_quat.z = myQuaternion[3];
    odom_trans.transform.rotation = odom_quat;

    odom_broadcaster.sendTransform(odom_trans);
  }

  std::vector<double> x;
  state_type tau;
  state_type acc;
  state_type q;
  state_type v;
};

int main(int argc, char **argv) {
  model = new pinocchio::Model();
  pinocchio::urdf::buildModel(filename,*model);
  data = new pinocchio::Data(*model);

  ros::init(argc, argv, "integrator");

  ros::NodeHandle n_;

  Integrator integrator;

  ros::param::get("~_timeScale", timeScale);
  bool ASIF_active;
  ros::param::get("~_ASIF_active", ASIF_active);

  //Ignore torques from ASIF if ASIF_active is not set
  ros::Subscriber sub_;
  if (ASIF_active)
  {
    sub_ = n_.subscribe("torques_actual", 1, &Integrator::callback, &integrator);
    ROS_INFO("ASIF ACTIVE");
  }
  else
  {
    sub_ = n_.subscribe("torques", 1, &Integrator::callback, &integrator);
    ROS_INFO("ASIF INACTIVE");
  }
  
  ros::Publisher pos_pub_ = n_.advertise<std_msgs::Float32MultiArray>("joint/position", 1);
  ros::Publisher vel_pub_ = n_.advertise<std_msgs::Float32MultiArray>("joint/velocity", 1);
  ros::Publisher joint_pub_ = n_.advertise<sensor_msgs::JointState>("jointstate", 1);

  std_msgs::Float32MultiArray pos_msg;
  std_msgs::Float32MultiArray vel_msg;

  pos_msg.data.clear();
  pos_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
  pos_msg.layout.dim[0].size = DOF;
  for (int i = 0; i < DOF; i++) {
    pos_msg.data.push_back(0.0);
  }

  vel_msg.data.clear();
  vel_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
  vel_msg.layout.dim[0].size = DOF;
  for (int i = 0; i < DOF; i++) {
    vel_msg.data.push_back(0.0);
  }

  int rate = 1000*timeScale;
  ros::Rate r(rate);

  ROS_INFO("integrator is ready");

  while (ros::ok()) {

    // integrate
    integrator.integrate(1./(rate/timeScale));

    // publish
    sensor_msgs::JointState joint_msg;
    for (int i = 0; i < DOF; ++i)
    {
      pos_msg.data[i] = integrator.x[i];
      vel_msg.data[i] = integrator.x[DOF+i];
      joint_msg.position.push_back(integrator.x[i]);
    }
    joint_msg.name.push_back("joint_1");
    joint_msg.name.push_back("joint_2");
    joint_msg.name.push_back("joint_3");
    joint_msg.name.push_back("joint_4");
    joint_msg.name.push_back("joint_5");
    joint_msg.name.push_back("joint_6");

    pos_pub_.publish(pos_msg);
    vel_pub_.publish(vel_msg);
    joint_pub_.publish(joint_msg);
    integrator.sendTransformCurrent();

    ros::spinOnce();
    r.sleep();
  }
  return 0;
}