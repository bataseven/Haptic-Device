#ifndef ROSHandler_H
#define ROSHandler_H
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_srvs/SetBool.h>

#include "HapticDevice.h"
#include "ros_lib/wsg_50_common/Cmd.h"
#include "ros_lib/wsg_50_common/Status.h"

struct SpringObject {
  double k;
  double width;
};
struct SchunkGripper {
  String status;
  float width;
  float speed;
  float force;
  float lastCommandedPos;
  boolean isMoving;
  boolean isInContact;
  boolean prevIsInContact;
  SpringObject spring;
  wsg_50_common::Cmd cmd;
};

class ROSHandler {
 public:
  ros::NodeHandle nh;
  ROSHandler(HapticDevice*);
  void initialize();
  void update();

  void stopFingerMotorCallback(const std_srvs::SetBool::Request&, std_srvs::SetBool::Response&);
  void stopPalmMotorCallback(const std_srvs::SetBool::Request&, std_srvs::SetBool::Response&);
  void setKpCallback(const std_msgs::Float32MultiArray&);
  void setKiCallback(const std_msgs::Float32MultiArray&);
  void setKdCallback(const std_msgs::Float32MultiArray&);
  void setWaveformTypeCallback(const std_msgs::Float32MultiArray&);
  void setWaveformFreqCallback(const std_msgs::Float32MultiArray&);
  void setWaveformAmplitudeCallback(const std_msgs::Float32MultiArray&);
  void setWaveformOffsetCallback(const std_msgs::Float32MultiArray&);
  void commandedPositionCallback(const std_msgs::Float32&);
  void gripperStateCallback(const wsg_50_common::Status&);
  void gripperMovingCallback(const std_msgs::Bool&);

  // Service servers for stopping the motors
  ros::ServiceServer<std_srvs::SetBool::Request, std_srvs::SetBool::Response, ROSHandler> stopServer1;
  ros::ServiceServer<std_srvs::SetBool::Request, std_srvs::SetBool::Response, ROSHandler> stopServer2;

  // Subscribers for changing the PID position controller gains of the motors
  ros::Subscriber<std_msgs::Float32MultiArray, ROSHandler> setKpSubscriber; 
  ros::Subscriber<std_msgs::Float32MultiArray, ROSHandler> setKiSubscriber;
  ros::Subscriber<std_msgs::Float32MultiArray, ROSHandler> setKdSubscriber;

  // Subscribers for changing waveform parameters
  ros::Subscriber<std_msgs::Float32MultiArray, ROSHandler> setWaveformSubscriber;
  ros::Subscriber<std_msgs::Float32MultiArray, ROSHandler> setFreqSubscriber;
  ros::Subscriber<std_msgs::Float32MultiArray, ROSHandler> setAmplitudeSubscriber;
  ros::Subscriber<std_msgs::Float32MultiArray, ROSHandler> setOffsetSubscriber;

  // The use of this topic is flexible
  ros::Subscriber<std_msgs::Float32, ROSHandler> commandSubscriber;

  // Subscribers and publishers for the schunk gripper
  ros::Subscriber<wsg_50_common::Status, ROSHandler> wsg50StatusSubscriber;
  ros::Subscriber<std_msgs::Bool, ROSHandler> wsg50MovingSubscriber;
  ros::Publisher wsg50CommandPublisher;

  // Publishers for the generated waveforms and the encoder positions
  ros::Publisher positionPublisher;
  ros::Publisher velocityPublisher;
  ros::Publisher waveformPublisher;

  // Force sensor value publisher
  ros::Publisher forcePublisher;

  // Rosmsg to publish
  std_msgs::Int32MultiArray encoderPositions;
  std_msgs::Int32MultiArray encoderVelocities;
  std_msgs::Float32MultiArray generatedWaveform;
  std_msgs::Float32MultiArray forceValues;

  // Variables that determine ROS communication frequency
  long ROSPublisherTimer;
  long WSGPublisherTimer;

  HapticDevice* device;
  float commandedPos;
  const float ROS_PUBLISH_FREQUENCY = 100;
  const float WSG_PUBLISH_FREQUENCY = 75;
  SchunkGripper WSG50;
};
#endif