#include <control/uav_demo.h>
#include <control/RobotPose.h>

#include <std_srvs/Trigger.h>

static const unsigned int CONTROL_EVENT_TAKEOFF_COMPLETE = 0;
static const unsigned int CONTROL_EVENT_LAND_COMPLETE = 1;
static const unsigned int CONTROL_EVENT_WAYPOINT_COMPLETE = 2;

UavDemo::UavDemo(
  ros::NodeHandle &rosNode,
  UavDemo::Boundary boundary
) {
  this->mRosNode = &rosNode;
  mBoundary = boundary;
  this->mTakeoffClient = this->mRosNode->serviceClient<std_srvs::Trigger>("offboard_control/takeoff");
  this->mWaypointClient = this->mRosNode->serviceClient<control::RobotPose>("offboard_control/waypoint");
  this->mControlEventSubscriber = this->mRosNode->subscribe<std_msgs::UInt16>("offboard_control/event", 1, &UavDemo::controlEventCallback, this);
}

void UavDemo::takeoff() {
  std_srvs::Trigger srv;
  if (this->mTakeoffClient.call(srv)) {
    ROS_INFO("Takeoff command accepted.");
  }
  else {
    ROS_ERROR("Takeoff command rejected.");
    ROS_ERROR_STREAM(srv.response.message);
  }
}

void UavDemo::wander() {
  control::RobotPose poseSrv;
  poseSrv.request.position.x = this->randomFloat(this->mBoundary.xMin, this->mBoundary.xMax);
  poseSrv.request.position.y = this->randomFloat(this->mBoundary.yMin, this->mBoundary.yMax);
  poseSrv.request.position.z = this->randomFloat(this->mBoundary.zMin, this->mBoundary.zMax);
  if (this->mWaypointClient.call(poseSrv)) {
    ROS_INFO_STREAM(poseSrv.response.message);
  }
  else {
    ROS_ERROR("Waypoint command rejected.");
  }
}

void UavDemo::controlEventCallback(const std_msgs::UInt16::ConstPtr& msg) {
  switch (msg->data) {
  case CONTROL_EVENT_TAKEOFF_COMPLETE:
    // Start to wander after takeoff
    this->wander();
    break;
  case CONTROL_EVENT_WAYPOINT_COMPLETE:
    // Wander indefinitely
    this->wander();
    break;
  default:
    break;
  }
}

float UavDemo::randomFloat(float min, float max) {
  return std::uniform_real_distribution<float>{ min, max }(this->mRandomEngine);
}
