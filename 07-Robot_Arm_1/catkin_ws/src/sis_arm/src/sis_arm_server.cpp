#include <ros/ros.h>
#include <sis_arm/target_pose.h>
#include <std_msgs/Float64.h>
#include <cmath>
#include <iostream>

/* 
   Check if given value in range [lower, upper]
   Input:
     double val: value
     double upper: upper limit
     double lower: lower limit
   Output: 
     1: if in range
     0: if out of range
*/
bool in_range(double val, double upper, double lower)
{ 
  if(val <= upper and val >= lower) return true;
  else return false;
}

class SISArm{
  private:
    ros::NodeHandle nh_;
    ros::Publisher pub_pan_controller_, pub_tilt_controller_, pub_gripper_controller_;
    ros::ServiceServer goto_pose_srv_;
    // joint
    double q_[2];
    // sis_arm dh parameters, in meter
    double _d0 = 0.045, _d1 = 0.06, _a2 = 0.065, _d3 = 0.07;
    double _R2 = _a2 * _a2 + _d3 * _d3, _R = sqrt(_R2);
    // joint limits
    double _j1_upper = 2.616, _j1_lower = -2.616, _j2_upper = 0.8, _j2_lower = -2.0;
    // message string
    std::string message_;
    std_msgs::Float64 msg;
    /* 
       Check if given position in work space
       Input: 
         double x: x-axis distance, in meter
         double y: y-axis distance, in meter
         double z: z-axis distance, in meter
       Output:
         1: if in work space
         0: if out of work space
    */ 
    bool Cartesian_in_range(double x, double y, double z) {
      return (in_range(x, _R, -_R) and in_range(y, _d1 + _R, _d1 - _R) and 
              in_range(z, _d0 + _R, _d0 - _R));
    }
    /* 
       Check if given joint angle in range
       Input:
         double* joint: joint array
       Output:
         1: in joint range
         0: out of joint range
    */
    bool joint_in_range(double* joint) {
      return (in_range(joint[0], _j1_upper, _j1_lower) and 
              in_range(joint[1], _j2_upper, _j2_lower));
    }
    /*
       IK procedure
       Input:
         double x: TCP x-axis distance, in meter
         double y: TCP y-axis distance, in meter
         double z: TCP z-axis distance, in meter
       Output:
         1: if given position reachable
         0: if given position not reachable
    */
    bool IK(double x, double y, double z) {
      if(!Cartesian_in_range(x, y, z)) {
        message_ = "Given point out of range.";
        return false;
      }
      if(x == 0  && std::abs(z-_d0) == 0) {
        q_[0] = 0; // arbitrary value, assign 0
        q_[1] = atan2(-_d3, _a2);
      }
      else {
        double y_bar, z_bar, c1;
        q_[0] = (x == 0? 0 :atan2(-x, -z+_d0));
        c1 = cos(q_[0]);
        z_bar = _d0 - z;
        y_bar = y - _d1;
        q_[1] = atan2(_a2*z_bar-_d3*c1*y_bar, _a2*c1*y_bar+_d3*z_bar);
      }
      if(!joint_in_range(q_)) {
        printf("%f, %f", q_[0], q_[1]);
        message_ = "Joint out of range.";
        return false;
      }
      message_ = "Success.";
      return true;
    }
    /*
       Convert gripper finger percentage to rad
       Input:
         double percentage: finger close percentage
       Output:
         result: gripper motor radian
       Note: 
         percentage     rad
             0 %         0
            100 %      5pi/6
    */ 
    double to_gripper_rad(double percentage) {
      double result = 5*M_PI/6 * percentage / 100;
      return result;
    }
  public:
    SISArm() {
      pub_pan_controller_     = nh_.advertise<std_msgs::Float64>("/pan_controller/command", 1);
      pub_tilt_controller_    = nh_.advertise<std_msgs::Float64>("/tilt_controller/command", 1);
      pub_gripper_controller_ = nh_.advertise<std_msgs::Float64>("/gripper_controller/command", 1);
      goto_pose_srv_ = nh_.advertiseService("/sis_arm_control/goto_pose_srv", 
                                            &SISArm::GotoPoseService,this);
      ROS_INFO("Start pose control service");
    }
    bool GotoPoseService(sis_arm::target_pose::Request &req,
                         sis_arm::target_pose::Response &res) {
      ROS_INFO("Receive new goal");
      double x = static_cast<double> (req.x);
      double y = static_cast<double> (req.y);
      double z = static_cast<double> (req.z);

      if(!IK(x, y, z)) {
        res.plan_result = message_; 
        ROS_INFO_STREAM(message_);
        return true;
      }
      msg.data = q_[0]; pub_pan_controller_.publish(msg);  ros::Duration(0.5).sleep();
      msg.data = q_[1]; pub_tilt_controller_.publish(msg); ros::Duration(0.5).sleep();
      msg.data = to_gripper_rad(req.finger_percentage);
      pub_gripper_controller_.publish(msg); ros::Duration(0.5).sleep();
      res.plan_result = message_;
      ROS_INFO("joint: %f %f %f", q_[0], q_[1], msg.data);
      return true;
    }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sis_arm_control");
  SISArm sis_arm;
  ros::spin();
  return 0;
}
