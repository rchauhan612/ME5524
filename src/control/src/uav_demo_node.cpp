#include <control/uav_demo.h>

#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "uav_demo");
  ros::NodeHandle rosNode;
  UavDemo::Boundary boundary;
  float frequency = 20.0;
  ros::Rate rosRate(frequency);

  boundary.xMin = -10.0;
  boundary.xMax = 10.0;
  boundary.yMin = -10.0;
  boundary.yMax = 10.0;
  boundary.zMin = 2.5;
  boundary.zMax = 10.0;

  rosNode.param(ros::this_node::getName() + "/x_min", boundary.xMin, boundary.xMin);
  rosNode.param(ros::this_node::getName() + "/x_max", boundary.xMax, boundary.xMax);
  rosNode.param(ros::this_node::getName() + "/y_min", boundary.yMin, boundary.yMin);
  rosNode.param(ros::this_node::getName() + "/y_max", boundary.yMax, boundary.yMax);
  rosNode.param(ros::this_node::getName() + "/z_min", boundary.zMin, boundary.zMin);
  rosNode.param(ros::this_node::getName() + "/z_max", boundary.zMax, boundary.zMax);

  UavDemo uavDemo(rosNode, boundary);
  sleep(5);
  uavDemo.takeoff();

  while (rosNode.ok()) {
    ros::spinOnce();
    rosRate.sleep();
  }
  return 0;
}
