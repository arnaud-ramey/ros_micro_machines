#ifndef WHEELRECEIVER_H
#define WHEELRECEIVER_H

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <ros_micro_machines/timer.h>

class WheelReceiver {
public:
  typedef int Speed;
  typedef int Angle;
  //! the error code when we could not read speed
  const static Speed ERROR_SPEED = - 1000;
  //! the error code when we could not read angle
  const static Angle ERROR_ANGLE = - 1000;
  const static Angle WHEEL_ANGLE_MAX = 180;
  const static Speed WHEEL_SPEED_MODULE_MIN = -30;
  const static Speed WHEEL_SPEED_MODULE_MAX = 300;

  /*! the time after which we consider the reception as bad
    and return 0 for speed and angle */
  const static vision_utils::Timer::Time TIMEOUT_MS = 1000;

  WheelReceiver()  {
    IMG_DIR = ros::package::getPath("ros_micro_machines") + "/data";
    speed_module = 0;
    wheel_angle = 0;
    // suscribe
    ros::NodeHandle nh_pubic;
    _cmd_vel_sub = nh_pubic.subscribe("cmd_vel", 1, &WheelReceiver::speed_cb, this);
  }

  ~WheelReceiver() {}

  /*! returns stg between WHEEL_SPEED_MODULE_MIN and WHEEL_SPEED_MODULE_MAX*/
  Speed get_speed_module() const {
    if (time_last_data.time() > TIMEOUT_MS)
      return 0;
    return speed_module;
  }

  /*! returns stg between WHEEL_ANGLE_MAX and WHEEL_ANGLE_MAX*/
  Angle get_wheel_angle() const {
    if (time_last_data.time() > TIMEOUT_MS)
      return 0;
    return wheel_angle;
  }

protected:
  void speed_cb(const geometry_msgs::TwistConstPtr & msg) {
    set_speed_module(msg->linear.x);
    set_wheel_angle(-msg->angular.z);
  }
  Speed set_speed_module(Speed d) {
    //maggieDebug2("set_speed_module(%i)", d);

    // emergency stop if needed
    if (d == ERROR_SPEED) {
      speed_module = 0;
      wheel_angle = 0;
      return 0;
    }

    time_last_data.reset();
    if (d < WHEEL_SPEED_MODULE_MIN)
      speed_module = WHEEL_SPEED_MODULE_MIN;
    else if (d > WHEEL_SPEED_MODULE_MAX)
      speed_module = WHEEL_SPEED_MODULE_MAX;
    else
      speed_module = d;
    return speed_module;
  }

  Angle set_wheel_angle(Angle angle) {
    //maggieDebug2("set_wheel_angle(%i)", angle);

    // emergency stop if needed
    if (angle == ERROR_ANGLE) {
      speed_module = 0;
      wheel_angle = 0;
      return 0;
    }

    time_last_data.reset();
    if (angle < -WHEEL_ANGLE_MAX)
      wheel_angle = -WHEEL_ANGLE_MAX;
    else if (angle > WHEEL_ANGLE_MAX)
      wheel_angle = WHEEL_ANGLE_MAX;
    else
      wheel_angle = angle;
    return wheel_angle;
  }

  ros::Subscriber _cmd_vel_sub;
  vision_utils::Timer time_last_data;
  int speed_module, wheel_angle;
  std::string IMG_DIR;
};

#endif // WHEELRECEIVER_H
