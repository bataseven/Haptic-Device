#include "ROSHandler.h"

ROSHandler::ROSHandler(HapticDevice *device) : stopServer1("stop_finger_motor", &ROSHandler::stopFingerMotorCallback, this),
                                               stopServer2("stop_palm_motor", &ROSHandler::stopPalmMotorCallback, this),
                                               setKpSubscriber("set_Kp", &ROSHandler::setKpCallback, this),
                                               setKiSubscriber("set_Ki", &ROSHandler::setKiCallback, this),
                                               setKdSubscriber("set_Kd", &ROSHandler::setKdCallback, this),
                                               setWaveformSubscriber("set_waveform_type", &ROSHandler::setWaveformTypeCallback, this),
                                               setFreqSubscriber("set_waveform_frequency", &ROSHandler::setWaveformFreqCallback, this),
                                               setAmplitudeSubscriber("set_waveform_amplitude", &ROSHandler::setWaveformAmplitudeCallback, this),
                                               setOffsetSubscriber("set_waveform_offset", &ROSHandler::setWaveformOffsetCallback, this),
                                               commandSubscriber("commanded_position", &ROSHandler::commandedPositionCallback, this),
                                               wsg50StatusSubscriber("wsg_50_driver/status", &ROSHandler::gripperStateCallback, this),
                                               wsg50MovingSubscriber("wsg_50_driver/moving", &ROSHandler::gripperMovingCallback, this),
                                               wsg50CommandPublisher("wsg_50_driver/goal_position", &WSG50.cmd),
                                               positionPublisher("encoder_position", &encoderPositions),
                                               velocityPublisher("encoder_velocity", &encoderVelocities),
                                               waveformPublisher("generated_waveform", &generatedWaveform),
                                               forcePublisher("force_sensor", &forceValues) {
  this->device = device;
  nh.initNode();
}

void ROSHandler::initialize() {
  // Advertise to and subscribe to topics
  nh.advertise(positionPublisher);
  nh.advertise(velocityPublisher);
  nh.advertise(waveformPublisher);

  nh.advertiseService(stopServer1);
  nh.advertiseService(stopServer2);
  nh.subscribe(commandSubscriber);

  nh.subscribe(setKpSubscriber);
  nh.subscribe(setKiSubscriber);
  nh.subscribe(setKdSubscriber);

  nh.subscribe(setWaveformSubscriber);
  nh.subscribe(setFreqSubscriber);
  nh.subscribe(setAmplitudeSubscriber);
  nh.subscribe(setOffsetSubscriber);

  nh.subscribe(wsg50StatusSubscriber);
  nh.subscribe(wsg50MovingSubscriber);
  nh.advertise(wsg50CommandPublisher);

  nh.advertise(forcePublisher);

    // Set the rosmsg size in the memory.
  encoderPositions.data = (int32_t *)malloc(sizeof(int32_t) * 2);
  encoderPositions.data_length = 2;
  encoderVelocities.data = (int32_t *)malloc(sizeof(int32_t) * 2);
  encoderVelocities.data_length = 2;
  generatedWaveform.data = (float *)malloc(sizeof(float) * 2);
  generatedWaveform.data_length = 2;
  forceValues.data = (float *)malloc(sizeof(float) * 4);
  forceValues.data_length = 4;
}

void ROSHandler::update() {
  if (millis() - ROSPublisherTimer > (1000.0 / ROS_PUBLISH_FREQUENCY)) {
    ROSPublisherTimer = millis();                         // Restart to timer to keep a constant publishing frequency
    encoderPositions.data[0] = device->finger->getPos();  // Fill in the variables for the rostopic
    encoderPositions.data[1] = device->palm->getPos();    // Fill in the variables for the rostopic
    positionPublisher.publish(&encoderPositions);         // Publish to that topic

    encoderVelocities.data[0] = device->finger->getVel();  // Fill in the variables for the rostopic
    encoderVelocities.data[1] = device->palm->getVel();    // Fill in the variables for the rostopic
    velocityPublisher.publish(&encoderVelocities);         // Publish to that topic

    generatedWaveform.data[0] = device->generatedFingerWave;  // Fill in the variables for the rostopic
    generatedWaveform.data[1] = device->generatedPalmWave;   // Fill in the variables for the rostopic

    waveformPublisher.publish(&generatedWaveform);  // Publish to that topic

    forceValues.data[0] = device->thumbOuterSensor->getForce();  // Fill in the variables for the rostopic
    forceValues.data[1] = device->thumbInnerSensor->getForce();  // Fill in the variables for the rostopic
    forceValues.data[2] = device->indexInnerSensor->getForce();  // Fill in the variables for the rostopic
    forceValues.data[3] = device->indexOuterSensor->getForce();  // Fill in the variables for the rostopic
    forcePublisher.publish(&forceValues);  // Publish to that topic
  }

  if (millis() - WSGPublisherTimer > (1000.0 / WSG_PUBLISH_FREQUENCY)) {
    WSGPublisherTimer = millis();                    // Restart to timer to keep a constant publishing frequency
    const float WSG_MOVEMENT_THRESHOLD_IN_MM = 0.1;  // Do not move the gripper until the new command is at least that much different from the current position.

    // Publish only if the gripper is not already moving  and threshold is passed
    if (abs(WSG50.cmd.pos - WSG50.lastCommandedPos) > WSG_MOVEMENT_THRESHOLD_IN_MM) {
      wsg50CommandPublisher.publish(&WSG50.cmd);  // Publish to that topic.
      WSG50.lastCommandedPos = WSG50.cmd.pos;     // Save the last commanded position
    }
  }

  nh.spinOnce();  // Call all the callbacks that are waiting to be called.
}

// Receives an array of length to 2 where the first element is the index of the
// motor(0 for the finger, 1 for the palm) and the second is the value.
void ROSHandler::setWaveformTypeCallback(const std_msgs::Float32MultiArray &indexAndValue) {
  int idx = indexAndValue.data[0];
  int value = indexAndValue.data[1];
  switch (idx) {
    case 0:
      device->waveformGeneratorFinger.setType(WaveformGenerator::WaveType(value));
      break;
    case 1:
      device->waveformGeneratorPalm.setType(WaveformGenerator::WaveType(value));
      break;
    default:
      break;
  }
}

// Receives an array of length to 2 where the first element is the index of the
// motor(0 for the finger, 1 for the palm) and the second is the value.
void ROSHandler::setWaveformFreqCallback(const std_msgs::Float32MultiArray &indexAndValue) {
  int idx = indexAndValue.data[0];
  float value = indexAndValue.data[1];
  switch (idx) {
    case 0:
      device->waveformGeneratorFinger.setFrequency(value);
      break;
    case 1:
      device->waveformGeneratorPalm.setFrequency(value);
      break;
    default:
      break;
  }
}

// Receives an array of length to 2 where the first element is the index of the
// motor(0 for the finger, 1 for the palm) and the second is the value.
void ROSHandler::setWaveformAmplitudeCallback(const std_msgs::Float32MultiArray &indexAndValue) {
  int idx = indexAndValue.data[0];
  float value = indexAndValue.data[1];
  switch (idx) {
    case 0:
      device->waveformGeneratorFinger.setAmplitude(value);
      break;
    case 1:
      device->waveformGeneratorPalm.setAmplitude(value);
      break;
    default:
      break;
  }
}

// Receives an array of length to 2 where the first element is the index of the
// motor(0 for the finger, 1 for the palm) and the second is the value.
void ROSHandler::setWaveformOffsetCallback(const std_msgs::Float32MultiArray &indexAndValue) {
  int idx = indexAndValue.data[0];
  float value = indexAndValue.data[1];
  switch (idx) {
    case 0:
      device->waveformGeneratorFinger.setOffset(value);
      break;
    case 1:
      device->waveformGeneratorPalm.setOffset(value);
      break;
    default:
      break;
  }
}

// Receives an array of length to 2 where the first element is the index of the
// motor(0 for the finger, 1 for the palm) and the second is the gain.
void ROSHandler::setKpCallback(const std_msgs::Float32MultiArray &indexAndGain) {
  int idx = indexAndGain.data[0];
  float gain = indexAndGain.data[1];
  switch (idx) {
    case 0:
      device->finger->setKp(gain);
      break;

    case 1:
      device->palm->setKp(gain);
      break;
    default:
      break;
  }
}

// Receives an array of length to 2 where the first element is the index of the
// motor(0 for the finger, 1 for the palm) and the second is the gain.
void ROSHandler::setKiCallback(const std_msgs::Float32MultiArray &indexAndGain) {
  int idx = indexAndGain.data[0];
  float gain = indexAndGain.data[1];
  switch (idx) {
    case 0:
      device->finger->setKi(gain);
      break;

    case 1:
      device->palm->setKi(gain);
      break;
    default:
      break;
  }
}

// Receives an array of length to 2 where the first element is the index of the
// motor(0 for the finger, 1 for the palm) and the second is the gain.
void ROSHandler::setKdCallback(const std_msgs::Float32MultiArray &indexAndGain) {
  int idx = indexAndGain.data[0];
  float gain = indexAndGain.data[1];
  switch (idx) {
    case 0:
      device->finger->setKd(gain);
      break;

    case 1:
      device->palm->setKd(gain);
      break;
    default:
      break;
  }
}

// Listen to this topic in case we need to change a number
void ROSHandler::commandedPositionCallback(const std_msgs::Float32 &commanded_position) {
  commandedPos = commanded_position.data;
}

// Service server callback for stopping the finger motor
void ROSHandler::stopFingerMotorCallback(const std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
  device->finger->hardStop = req.data;
  res.success = true;
  res.message = req.data ? "Finger motor stopped" : "Finger motor continued";
}

// Service server callback for stopping the palm motor
void ROSHandler::stopPalmMotorCallback(const std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
  device->palm->hardStop = req.data;
  res.success = true;
  res.message = req.data ? "Palm motor stopped" : "Palm motor continued";
}

// Listen to this topic for the gripper status
void ROSHandler::gripperStateCallback(const wsg_50_common::Status &status) {
  WSG50.status = status.status;
  WSG50.width = status.width;
  WSG50.speed = status.speed;
  WSG50.force = status.force;
  WSG50.prevIsInContact = WSG50.isInContact;
  WSG50.isInContact = status.force > 1.8; // 1.8N threshold for contact detection
  if (WSG50.isInContact && !WSG50.prevIsInContact) {
    WSG50.spring.width = status.width;
    // = device->finger->getPos();
  }
}

// Listen to this topic to see if the gripper is moving
void ROSHandler::gripperMovingCallback(const std_msgs::Bool &moving) {
  WSG50.isMoving = moving.data;
}
