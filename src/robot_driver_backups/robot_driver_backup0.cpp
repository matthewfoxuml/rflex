//ROS 2 COMMENTED OUT BELOW
//DONE

/*
#include <iostream>
#include <math.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include "std_msgs/Float64.h"

// Making this global is not required. There must be a safer way to do it.
// But for testing purposes I am initializing it here. To be removed later
double Distance = 0;
void xyDistanceCallBack(const std_msgs::Float64::ConstPtr& msg)
{
	Distance = (*msg).data;
}
double Heading = 0;
void xyHeadingCallBack(const std_msgs::Float64::ConstPtr& msg)
{
	Heading = (*msg).data;
}
class RobotDriver
{
private:
  //! The node handle we'll be using
  ros::NodeHandle nh_;
  //! We will be publishing to the "cmd_vel" topic to issue commands
  ros::Publisher cmd_vel_pub_;
  //! We will be listening to TF transforms as well
  tf::TransformListener listener_;
  //  We will be subscribing to distance to be covered that is calculated by python script
  ros::Subscriber subscriber_for_gpsdistance;
  ros::Subscriber subscriber_for_gpsheading;
public:
  //! ROS node initialization
  RobotDriver(ros::NodeHandle &nh)
  {
    nh_ = nh;
    //set up the publisher for the cmd_vel topic
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    // set up the subscriber for the gpstoxy topic
    subscriber_for_gpsdistance = nh_.subscribe("/gpsdistance",1, xyDistanceCallBack);
    subscriber_for_gpsheading = nh_.subscribe("/gpsheading",1, xyHeadingCallBack);
  }

  //! Drive forward a specified distance based on odometry information
  bool driveForwardOdom(double distance)
  {
    //wait for the listener to get the first message
    listener_.waitForTransform("base_link", "odom", 
                               ros::Time(0), ros::Duration(1.0));
    
    //we will record transforms here
    tf::StampedTransform start_transform;
    tf::StampedTransform current_transform;

    //record the starting transform from the odometry to the base frame
    listener_.lookupTransform("base_link", "odom", 
                              ros::Time(0), start_transform);
    
    //we will be sending commands of type "twist"
    geometry_msgs::Twist base_cmd;
    //the command will be to go forward at 0.25 m/s
    base_cmd.linear.y = base_cmd.angular.z = 0;
    base_cmd.linear.x = 0.3;
    
    ros::Rate rate(10.0);
    bool done = false;
    while (!done && nh_.ok())
    {
      //send the drive command
      cmd_vel_pub_.publish(base_cmd);
      rate.sleep();
      //get the current transform
      try
      {
        listener_.lookupTransform("base_link", "odom", 
                                  ros::Time(0), current_transform);
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s",ex.what());
        break;
      }
      //see how far we've traveled
      tf::Transform relative_transform = 
        start_transform.inverse() * current_transform;
      double dist_moved = relative_transform.getOrigin().length();

      if(dist_moved > distance) done = true;
    }
    if (done) 
    {
	base_cmd.linear.y = base_cmd.angular.z = 0;
    	base_cmd.linear.x = 0.0;
    	//send the drive command
    	cmd_vel_pub_.publish(base_cmd);
    	rate.sleep();
	return true;
    } 	
    return false;
  }


  bool turnOdom(bool clockwise, double radians)
  {
    while(radians < 0) radians += 2*M_PI;
    while(radians > 2*M_PI) radians -= 2*M_PI;

    //wait for the listener to get the first message
    listener_.waitForTransform("base_link", "odom", 
                               ros::Time(0), ros::Duration(1.0));
    
    //we will record transforms here
    tf::StampedTransform start_transform;
    tf::StampedTransform current_transform;

    //record the starting transform from the odometry to the base frame
    listener_.lookupTransform("base_link", "odom", 
                              ros::Time(0), start_transform);
    
    //we will be sending commands of type "twist"
    geometry_msgs::Twist base_cmd;
    //the command will be to turn at 0.75 rad/s
    base_cmd.linear.x = base_cmd.linear.y = 0.0;
    base_cmd.angular.z = 0.2;
    if (clockwise) base_cmd.angular.z = -base_cmd.angular.z;
    
    //the axis we want to be rotating by
    tf::Vector3 desired_turn_axis(0,0,1);
    if (!clockwise) desired_turn_axis = -desired_turn_axis;
    
    ros::Rate rate(10.0);
    bool done = false;
    while (!done && nh_.ok())
    {
      //send the drive command
      cmd_vel_pub_.publish(base_cmd);
      rate.sleep();
      //get the current transform
      try
      {
        listener_.lookupTransform("base_link", "odom", 
                                  ros::Time(0), current_transform);
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s",ex.what());
        break;
      }
      tf::Transform relative_transform = 
        start_transform.inverse() * current_transform;
      tf::Vector3 actual_turn_axis = 
        relative_transform.getRotation().getAxis();
      double angle_turned = relative_transform.getRotation().getAngle();
      if ( fabs(angle_turned) < 1.0e-2) continue;

      if ( actual_turn_axis.dot( desired_turn_axis ) < 0 ) 
        angle_turned = 2 * M_PI - angle_turned;

      if (angle_turned > radians) done = true;
    }
    if (done) 
    {
    	base_cmd.linear.y = base_cmd.angular.z = 0;
    	base_cmd.linear.x = 0.0;
    	//send the drive command
    	cmd_vel_pub_.publish(base_cmd);
    	rate.sleep();
	return true;
    }
    return false;
  }

};


int main(int argc, char** argv)
{
  //init the ROS node
  ros::init(argc, argv, "robot_driver");
  ros::NodeHandle nh;
  RobotDriver driver(nh);
//  double forwardDistance;
  float turningAngle = 0.0;

  ros::spinOnce();
  while (Distance == 0)
  {
  	ros::spinOnce();
  }

  do
  {
	sleep(2);
	ros::spinOnce();
	if ((Heading < -0.05) || (Heading > 0.05)) //if we are not within +- .05 radians (~3 degrees) then we need to turn
	{
	  if (Heading < 0) //if angle is negative, this is a counter-clockwise (left) turn.
	  {
	    turningAngle = -1 * Heading * (0.05); //make the angle positive and dial it down to 5% of the total angle
	    driver.turnOdom(false, turningAngle); //send the left turn command to the motors
	    std::cout << std::setprecision(12) << "L: " << turningAngle * 180 / M_PI << " : " << Heading * -1 * 180 / M_PI << std::endl;
	  }
	  else //this is a clockwise (right) turn
	  {
	    turningAngle = Heading * (0.05); //dial the angle down to 5%
	    driver.turnOdom(true, turningAngle);
	    std::cout << std::setprecision(12) << "R: " << turningAngle * 180 / M_PI << " : " << Heading * 180 / M_PI << std::endl;
	  }
	}
	ros::spinOnce();
	std::cout << std::setprecision(12) << "Dist: " << Distance << std::endl;  
	driver.driveForwardOdom(2);
  }while (Distance > 10);

std::cout << "Destination has been reached" << std::endl;

return 0;
//  std::cout << std::endl << "Enter the value to move forward" << std::endl;
//  std::cin >> forwardDistance;
// driver.driveForwardOdom(Distance);
//  std::cout << std::endl << "Enter the turning angle" << std::endl;
//  std::cin >> turningAngle;
//  driver.turnOdom(true,turningAngle);
}

*/
//ROS 2 STARTS BELOW


#include <iostream>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/Twist.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "std_msgs/msg/float64.hpp"

// Making these global is not required. There must be a safer way to do it.
// But for testing purposes, I am initializing them here. To be removed later.
double Distance = 0;
void xyDistanceCallBack(const std_msgs::msg::Float64::SharedPtr msg)
{
  Distance = msg->data;
}
double Heading = 0;
void xyHeadingCallBack(const std_msgs::msg::Float64::SharedPtr msg)
{
  Heading = msg->data;
}

class RobotDriver
{
private:
  //! The node handle we'll be using
  rclcpp::Node::SharedPtr nh_;
  //! We will be publishing to the "cmd_vel" topic to issue commands
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  //! We will be listening to TF transforms as well
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  // We will be subscribing to distance to be covered that is calculated by python script
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscriber_for_gpsdistance_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscriber_for_gpsheading_;

public:
  //! ROS node initialization
  RobotDriver(const rclcpp::NodeOptions &options)
    : nh_(rclcpp::Node::make_shared("robot_driver", options)),
      tf_buffer_(nh_->get_clock()),
      tf_listener_(tf_buffer_)
  {
    // set up the publisher for the cmd_vel topic
    cmd_vel_pub_ = nh_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
    // set up the subscriber for the gpstoxy topic
    subscriber_for_gpsdistance_ = nh_->create_subscription<std_msgs::msg::Float64>(
      "/gpsdistance", 1, xyDistanceCallBack);
    subscriber_for_gpsheading_ = nh_->create_subscription<std_msgs::msg::Float64>(
      "/gpsheading", 1, xyHeadingCallBack);
  }

  //! Drive forward a specified distance based on odometry information
  bool driveForwardOdom(double distance)
  {
    // wait for the listener to get the first message
    tf_buffer_.canTransform("base_link", "odom", tf2::TimePoint(), tf2::durationFromSec(1.0));

    // we will record transforms here
    geometry_msgs::msg::TransformStamped start_transform;
    geometry_msgs::msg::TransformStamped current_transform;

    // record the starting transform from the odometry to the base frame
    try
    {
      start_transform = tf_buffer_.lookupTransform("base_link", "odom", tf2::TimePoint());
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_ERROR(nh_->get_logger(), "%s", ex.what());
      return false;
    }

    // we will be sending commands of type "Twist"
    geometry_msgs::msg::Twist base_cmd;
    // the command will be to go forward at 0.3 m/s
    base_cmd.linear.y = base_cmd.angular.z = 0.0;
    base_cmd.linear.x = 0.3;

    rclcpp::Rate rate(10.0);
    bool done = false;
    while (!done && rclcpp::ok())
    {
      // send the drive command
      cmd_vel_pub_->publish(base_cmd);
      rate.sleep();
      // get the current transform
      try
      {
        current_transform = tf_buffer_.lookupTransform("base_link", "odom", tf2::TimePoint());
      }
      catch (tf2::TransformException &ex)
      {
        RCLCPP_ERROR(nh_->get_logger(), "%s", ex.what());
        break;
      }
      // see how far we've traveled
      tf2::Transform relative_transform;
      tf2::fromMsg(start_transform.transform, relative_transform);
      tf2::Transform current_transform_tf;
      tf2::fromMsg(current_transform.transform, current_transform_tf);
      relative_transform = relative_transform.inverse() * current_transform_tf;
      double dist_moved = relative_transform.getOrigin().length();

      if (dist_moved > distance)
      {
        done = true;
      }
    }

    if (done)
    {
      base_cmd.linear.y = base_cmd.angular.z = 0.0;
      base_cmd.linear.x = 0.0;
      // send the drive command
      cmd_vel_pub_->publish(base_cmd);
      rate.sleep();
      return true;
    }

    return false;
  }

  bool turnOdom(bool clockwise, double radians)
  {
    while (radians < 0)
      radians += 2 * M_PI;
    while (radians > 2 * M_PI)
      radians -= 2 * M_PI;

    // wait for the listener to get the first message
    tf_buffer_.canTransform("base_link", "odom", tf2::TimePoint(), tf2::durationFromSec(1.0));

    // we will record transforms here
    geometry_msgs::msg::TransformStamped start_transform;
    geometry_msgs::msg::TransformStamped current_transform;

    // record the starting transform from the odometry to the base frame
    try
    {
      start_transform = tf_buffer_.lookupTransform("base_link", "odom", tf2::TimePoint());
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_ERROR(nh_->get_logger(), "%s", ex.what());
      return false;
    }

    // we will be sending commands of type "Twist"
    geometry_msgs::msg::Twist base_cmd;
    // the command will be to turn at 0.2 rad/s
    base_cmd.linear.x = base_cmd.linear.y = 0.0;
    base_cmd.angular.z = 0.2;
    if (clockwise)
    {
      base_cmd.angular.z = -base_cmd.angular.z;
    }

    // the axis we want to be rotating by
    tf2::Vector3 desired_turn_axis(0, 0, 1);
    if (!clockwise)
    {
      desired_turn_axis = -desired_turn_axis;
    }

    rclcpp::Rate rate(10.0);
    bool done = false;
    while (!done && rclcpp::ok())
    {
      // send the drive command
      cmd_vel_pub_->publish(base_cmd);
      rate.sleep();
      // get the current transform
      try
      {
        current_transform = tf_buffer_.lookupTransform("base_link", "odom", tf2::TimePoint());
      }
      catch (tf2::TransformException &ex)
      {
        RCLCPP_ERROR(nh_->get_logger(), "%s", ex.what());
        break;
      }

      tf2::Transform relative_transform;
      tf2::fromMsg(start_transform.transform, relative_transform);
      tf2::Transform current_transform_tf;
      tf2::fromMsg(current_transform.transform, current_transform_tf);
      relative_transform = relative_transform.inverse() * current_transform_tf;
      tf2::Vector3 actual_turn_axis = relative_transform.getRotation().getAxis();
      double angle_turned = relative_transform.getRotation().getAngle();
      if (std::fabs(angle_turned) < 1.0e-2)
      {
        continue;
      }

      if (actual_turn_axis.dot(desired_turn_axis) < 0)
      {
        angle_turned = 2 * M_PI - angle_turned;
      }

      if (angle_turned > radians)
      {
        done = true;
      }
    }

    if (done)
    {
      base_cmd.linear.y = base_cmd.angular.z = 0.0;
      base_cmd.linear.x = 0.0;
      // send the drive command
      cmd_vel_pub_->publish(base_cmd);
      rate.sleep();
      return true;
    }

    return false;
  }
};

int main(int argc, char **argv)
{
  // Initialize the ROS node
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  RobotDriver driver(options);
  float turningAngle = 0.0;

  rclcpp::spin_some(driver);
  while (Distance == 0)
  {
    rclcpp::spin_some(driver);
  }

  do
  {
    rclcpp::sleep_for(std::chrono::seconds(2));
    rclcpp::spin_some(driver);
    if ((Heading < -0.05) || (Heading > 0.05)) // if we are not within +- .05 radians (~3 degrees), then we need to turn
    {
      if (Heading < 0) // if angle is negative, this is a counter-clockwise (left) turn
      {
        turningAngle = -1 * Heading * (0.05); // make the angle positive and dial it down to 5% of the total angle
        driver.turnOdom(false, turningAngle); // send the left turn command to the motors
        std::cout << std::setprecision(12) << "L: " << turningAngle * 180 / M_PI << " : " << Heading * -1 * 180 / M_PI << std::endl;
      }
      else // this is a clockwise (right) turn
      {
        turningAngle = Heading * (0.05); // dial the angle down to 5%
        driver.turnOdom(true, turningAngle);
        std::cout << std::setprecision(12) << "R: " << turningAngle * 180 / M_PI << " : " << Heading * 180 / M_PI << std::endl;
      }
    }
    rclcpp::spin_some(driver);
    std::cout << std::setprecision(12) << "Dist: " << Distance << std::endl;
    driver.driveForwardOdom(2);
  } while (Distance > 10);

  std::cout << "Destination has been reached" << std::endl;

  rclcpp::shutdown();
  return 0;
}
