#include <math.h>

#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "sensor_msgs/Joy.h"
#include <vector>
#include "visualization_msgs/Marker.h"
#include <tf2/LinearMath/Quaternion.h>

double r0;

class Human
{
public:

  Human() {
    pos = {0,0,0};
    v_lin = 0;
    v_ang = 0;
    maxvel_lin = 1;
    maxvel_ang = 1;
  }

  void joy_cb(const sensor_msgs::Joy & msg) {
    v_lin = maxvel_lin * msg.axes[1];
    v_ang = maxvel_ang * msg.axes[0];
  }

  void step(double dt) {
    pos[0] += dt * v_lin * cos(pos[2]);
    pos[1] += dt * v_lin * sin(pos[2]);
    pos[2] += dt * v_ang;    
  }

  std::vector<double> pos;
  double v_lin;
  double v_ang;

  double maxvel_lin;
  double maxvel_ang;
};


int main(int argc, char **argv) {

  ros::init(argc, argv, "human_control");

  Human human;

  ros::NodeHandle n;
 
  ros::param::get("~_x0", human.pos[0]);
  ros::param::get("~_y0", human.pos[1]);
  ros::param::get("~_r_0", r0);
  ros::param::get("~_theta0", human.pos[2]); 

  ros::param::get("~_maxvel_lin", human.maxvel_lin); 
  ros::param::get("~_maxvel_ang", human.maxvel_ang); 

  ros::Publisher pos_pub = n.advertise<geometry_msgs::Pose2D>("position", 1000);
  ros::Publisher vel_pub = n.advertise<geometry_msgs::Pose2D>("velocity", 1000);
  ros::Publisher vis_pub = n.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
  ros::Subscriber sub = n.subscribe("/joy", 1000, &Human::joy_cb, &human);

  int rate = 100;
  double dt = 1./rate;

  ros::Rate loop_rate(rate);
  geometry_msgs::Pose2D pos_msg;
  geometry_msgs::Pose2D vel_msg;

  visualization_msgs::Marker marker;
  marker.header.frame_id = "base_link";
  marker.header.stamp = ros::Time();
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 1;
  marker.pose.position.y = 1;
  marker.pose.position.z = r0/2;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = r0;
  marker.scale.y = r0;
  marker.scale.z = r0;
  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  ROS_INFO("human is ready");

  while (ros::ok())   {

    pos_msg.x = human.pos[0];
    pos_msg.y = human.pos[1];
    pos_msg.theta = human.pos[2];

    vel_msg.x = human.v_lin * cos(human.pos[2]);
    vel_msg.y = human.v_lin * sin(human.pos[2]);
    marker.pose.position.x = human.pos[0];
    marker.pose.position.y = human.pos[1];
    tf2::Quaternion myQuaternion;
    myQuaternion.setRPY( 0, 0, human.pos[2]);
    marker.pose.orientation.w = myQuaternion[3];
    marker.pose.orientation.x = myQuaternion[0];
    marker.pose.orientation.y = myQuaternion[1];
    marker.pose.orientation.z = myQuaternion[2];
    vel_msg.theta = human.v_ang;

    pos_pub.publish(pos_msg);
    vel_pub.publish(vel_msg);
    vis_pub.publish( marker );

    ros::spinOnce();

    human.step(dt);
    loop_rate.sleep();
  }

  return 0;
}