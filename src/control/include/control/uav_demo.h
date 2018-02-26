#ifndef UAV_DEMO_H
#define UAV_DEMO_H

#include <ros/ros.h>
#include <std_msgs/UInt16.h>
#include <random>

class UavDemo
{
public:
  struct Boundary {
    float xMin;
    float xMax;
    float yMin;
    float yMax;
    float zMin;
    float zMax;
  };
  UavDemo(
    ros::NodeHandle &rosNode,
    UavDemo::Boundary boundary
  );
  void takeoff();
  void wander();

private:
  UavDemo();
  void controlEventCallback(const std_msgs::UInt16::ConstPtr& msg);
  float randomFloat(float min, float max);

  ros::NodeHandle* mRosNode;
  ros::ServiceClient mTakeoffClient;
  ros::ServiceClient mWaypointClient;
  ros::Subscriber mControlEventSubscriber;
  Boundary mBoundary;
  std::mt19937 mRandomEngine { std::random_device{}() };
};

#endif
