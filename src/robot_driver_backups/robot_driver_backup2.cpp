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
  //  We will be subscribing to distance and heading to be covered that is calculated by python script
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
  bool driveForwardOdom(double distance, double speed)
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
    base_cmd.linear.x = speed;
    
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

  bool driveForward_safely(double distance)
  {
    if (distance > 2)
    {
      double d = distance - 2;
      driveForwardOdom(d,1);
      driveForwardOdom(0.5,0.8);
      driveForwardOdom(0.5,0.6);
      driveForwardOdom(0.5,0.4);
      driveForwardOdom(0.5,0.2);
    }
    else
    {
      driveForwardOdom(distance, 0.2); 
    }
  return true;
  }

  bool turn(double angle) //given an angle in radians, sends left/right turn message to motor function
  {
    double turningAngle = 0.0;

    if ((angle < -0.05) || (angle > 0.05)) //if we are not within +- .05 radians (~3 degrees) then we need to turn
    {
      if (Heading < 0) //if angle is negative, this is a counter-clockwise (left) turn.
      {
        turningAngle = -1 * angle; //make the angle positive
	turnOdom(false, turningAngle); //send the left turn command to the motors
	std::cout << std::setprecision(12) << "L: " << turningAngle * 180 / M_PI << std::endl;
      }
      else //this is a clockwise (right) turn
      {
	turningAngle = angle;
	turnOdom(true, turningAngle);
	std::cout << std::setprecision(12) << "R: " << turningAngle * 180 / M_PI << std::endl;
      }
    }
    else
    {
      std::cout << "No Turn" << std::endl;
    }

    return true;
  }

};

int main(int argc, char** argv)
{
  //init the ROS node
  ros::init(argc, argv, "robot_driver");
  ros::NodeHandle nh;
  RobotDriver driver(nh);

  ros::spinOnce();
  while (Distance == 0)
  {
  	ros::spinOnce();
  }

  do
  {
	sleep(5); //wait two seconds
	ros::spinOnce(); //get new data
	driver.turn(Heading); //turn based on new heading
	ros::spinOnce(); //get new data
	std::cout << std::setprecision(12) << "Dist: " << Distance << std::endl; //print distance left
	driver.driveForward_safely(4); //move forward 4 meters with a tapered speed at the end
	ros::spinOnce(); //get new data
  }while (Distance > 6); //check whether we are within 6m of our target

std::cout << "Destination has been reached" << std::endl;
std::cout << std::setprecision(12) << "Estimated Distance from Target: " << Distance << std::endl;
sleep(5);
ros::spinOnce();
driver.turn(Heading);

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
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_ros/transform_listener.h"
#include "std_msgs/msg/float64.hpp"

// Making this global is not required. There must be a safer way to do it.
// But for testing purposes I am initializing it here. To be removed later
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

class RobotDriver : public rclcpp::Node
{
public:
    RobotDriver()
        : Node("robot_driver")
    {
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
        subscriber_for_gpsdistance = this->create_subscription<std_msgs::msg::Float64>(
            "/gpsdistance", 1, std::bind(&RobotDriver::xyDistanceCallBack, this, std::placeholders::_1));
        subscriber_for_gpsheading = this->create_subscription<std_msgs::msg::Float64>(
            "/gpsheading", 1, std::bind(&RobotDriver::xyHeadingCallBack, this, std::placeholders::_1));

        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);
    }

    bool driveForwardOdom(double distance, double speed)
    {
        tf_buffer_.canTransform("base_link", "odom", tf2::TimePoint(), tf2::Duration(1.0));

        geometry_msgs::msg::TransformStamped start_transform;
        geometry_msgs::msg::TransformStamped current_transform;

        try
        {
            start_transform = tf_buffer_.lookupTransform("base_link", "odom", tf2::TimePoint());
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
            return false;
        }

        geometry_msgs::msg::Twist base_cmd;
        base_cmd.linear.y = base_cmd.angular.z = 0;
        base_cmd.linear.x = speed;

        rclcpp::Rate rate(10.0);
        bool done = false;
        while (!done && rclcpp::ok())
        {
            cmd_vel_pub_->publish(base_cmd);
            rate.sleep();

            try
            {
                current_transform = tf_buffer_.lookupTransform("base_link", "odom", tf2::TimePoint());
            }
            catch (tf2::TransformException &ex)
            {
                RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
                return false;
            }

            tf2::Transform relative_transform;
            tf2::fromMsg(start_transform.transform, relative_transform);
            tf2::Transform current_relative_transform;
            tf2::fromMsg(current_transform.transform, current_relative_transform);
            tf2::Transform dist_moved_transform = relative_transform.inverse() * current_relative_transform;
            double dist_moved = dist_moved_transform.getOrigin().length();

            if (dist_moved > distance)
            {
                done = true;
            }
        }

        if (done)
        {
            base_cmd.linear.y = base_cmd.angular.z = 0;
            base_cmd.linear.x = 0.0;
            cmd_vel_pub_->publish(base_cmd);
            rate.sleep();
        }

        return done;
    }

    bool turnOdom(bool clockwise, double radians)
    {
        while (radians < 0)
            radians += 2 * M_PI;
        while (radians > 2 * M_PI)
            radians -= 2 * M_PI;

        tf_buffer_.canTransform("base_link", "odom", tf2::TimePoint(), tf2::Duration(1.0));

        geometry_msgs::msg::TransformStamped start_transform;
        geometry_msgs::msg::TransformStamped current_transform;

        try
        {
            start_transform = tf_buffer_.lookupTransform("base_link", "odom", tf2::TimePoint());
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
            return false;
        }

        geometry_msgs::msg::Twist base_cmd;
        base_cmd.linear.x = base_cmd.linear.y = 0.0;
        base_cmd.angular.z = 0.2;
        if (clockwise)
        {
            base_cmd.angular.z = -base_cmd.angular.z;
        }

        tf2::Quaternion desired_turn_quaternion;
        desired_turn_quaternion.setRPY(0, 0, radians);
        if (!clockwise)
        {
            desired_turn_quaternion.setRPY(0, 0, -radians);
        }

        rclcpp::Rate rate(10.0);
        bool done = false;
        while (!done && rclcpp::ok())
        {
            cmd_vel_pub_->publish(base_cmd);
            rate.sleep();

            try
            {
                current_transform = tf_buffer_.lookupTransform("base_link", "odom", tf2::TimePoint());
            }
            catch (tf2::TransformException &ex)
            {
                RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
                return false;
            }

            tf2::Transform relative_transform;
            tf2::fromMsg(start_transform.transform, relative_transform);
            tf2::Transform current_relative_transform;
            tf2::fromMsg(current_transform.transform, current_relative_transform);
            tf2::Transform dist_moved_transform = relative_transform.inverse() * current_relative_transform;
            tf2::Quaternion current_turn_quaternion = dist_moved_transform.getRotation();

            tf2::Quaternion desired_turn_quaternion_inverse = desired_turn_quaternion.inverse();
            tf2::Quaternion error_quaternion = desired_turn_quaternion_inverse * current_turn_quaternion;
            double angle_turned = 2.0 * std::atan2(error_quaternion.getAxis().length(), error_quaternion.getW());

            if (angle_turned > radians)
            {
                done = true;
            }
        }

        if (done)
        {
            base_cmd.linear.y = base_cmd.angular.z = 0;
            base_cmd.linear.x = 0.0;
            cmd_vel_pub_->publish(base_cmd);
            rate.sleep();
        }

        return done;
    }

    bool driveForwardSafely(double distance)
    {
        if (distance > 2)
        {
            double d = distance - 2;
            driveForwardOdom(d, 1);
            driveForwardOdom(0.5, 0.8);
            driveForwardOdom(0.5, 0.6);
            driveForwardOdom(0.5, 0.4);
            driveForwardOdom(0.5, 0.2);
        }
        else
        {
            driveForwardOdom(distance, 0.2);
        }

        return true;
    }

    bool turn(double angle)
    {
        double turningAngle = 0.0;

        if ((angle < -0.05) || (angle > 0.05))
        {
            if (Heading < 0)
            {
                turningAngle = -1 * angle;
                turnOdom(false, turningAngle);
                RCLCPP_INFO(this->get_logger(), "L: %.2f : %.2f", turningAngle * 180 / M_PI, Heading * -1 * 180 / M_PI);
            }
            else
            {
                turningAngle = angle;
                turnOdom(true, turningAngle);
                RCLCPP_INFO(this->get_logger(), "R: %.2f : %.2f", turningAngle * 180 / M_PI, Heading * 180 / M_PI);
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "No Turn");
        }

        return true;
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscriber_for_gpsdistance;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscriber_for_gpsheading;

    tf2_ros::Buffer tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotDriver>());
    rclcpp::shutdown();

    return 0;
}
