#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <cmath> // สำหรับ cos() และ sin()

// =================================================================
// ส่วนที่ 1: Class Definition (การประกาศคลาส)
// =================================================================
class RobotBase 
{
public:
    // Constructor
    RobotBase();

    // Callback function
    void velocityCallback(const geometry_msgs::Twist& twist);

private:
    // ตัวแปรของ ROS
    ros::NodeHandle nh_;
    ros::Publisher odom_publisher_;
    ros::Subscriber velocity_subscriber_;
    tf::TransformBroadcaster odom_broadcaster_;
    ros::Time last_vel_time_;

    // ตัวแปรสำหรับคำนวณ Odometry
    double linear_scale_;
    double linear_velocity_x_;
    double linear_velocity_y_;
    double angular_velocity_z_;
    double vel_dt_;
    double x_pos_;
    double y_pos_;
    double heading_;
};

// =================================================================
// ส่วนที่ 2: Class Implementation (การเขียนการทำงานของฟังก์ชันในคลาส)
// =================================================================

// Constructor Implementation
RobotBase::RobotBase() :
        linear_velocity_x_(0.0),
        linear_velocity_y_(0.0),
        angular_velocity_z_(0.0),
        last_vel_time_(ros::Time::now()),
        vel_dt_(0.0),
        x_pos_(0.0),
        y_pos_(0.0),
        heading_(0.0) 
{
    ros::NodeHandle nh_private("~");

    // Advertise a topic for odometry
    odom_publisher_ = nh_.advertise<nav_msgs::Odometry>("odom", 50);

    // Subscribe to the velocity topic
    velocity_subscriber_ = nh_.subscribe("/xiaor/get_vel", 50, &RobotBase::velocityCallback, this);

    // Get parameters from the parameter server
    if (!nh_private.getParam("linear_scale", linear_scale_)) {
        linear_scale_ = 1.0; // Set a default value if not provided
        ROS_WARN("Linear scale not provided, using default value: 1.0");
    }
}

// Callback Function Implementation
void RobotBase::velocityCallback(const geometry_msgs::Twist& twist)
{
    ros::Time current_time = ros::Time::now();

    // Get velocities from the topic
    linear_velocity_x_ = twist.linear.x * linear_scale_;
    linear_velocity_y_ = twist.linear.y * linear_scale_;
    angular_velocity_z_ = twist.angular.z;

    // Calculate time delta
    vel_dt_ = (current_time - last_vel_time_).toSec();
    last_vel_time_ = current_time;

    // Calculate position delta
    double delta_heading = angular_velocity_z_ * vel_dt_;
    double delta_x = (linear_velocity_x_ * cos(heading_) - linear_velocity_y_ * sin(heading_)) * vel_dt_;
    double delta_y = (linear_velocity_x_ * sin(heading_) + linear_velocity_y_ * cos(heading_)) * vel_dt_;
    
    // Update position
    x_pos_ += delta_x;
    y_pos_ += delta_y;
    heading_ += delta_heading;

    // Create quaternion from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(heading_);

    // --- Broadcast the TF from odom to base_footprint ---
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";
    odom_trans.transform.translation.x = x_pos_;
    odom_trans.transform.translation.y = y_pos_;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
    odom_broadcaster_.sendTransform(odom_trans);
    // ----------------------------------------------------

    // --- Publish the Odometry message ---
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";
    
    // Set the position
    odom.pose.pose.position.x = x_pos_;
    odom.pose.pose.position.y = y_pos_;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
    
    // Set the velocity
    odom.twist.twist.linear.x = linear_velocity_x_;
    odom.twist.twist.linear.y = linear_velocity_y_;
    odom.twist.twist.angular.z = angular_velocity_z_;
    
    odom_publisher_.publish(odom);
    // ------------------------------------
}


// =================================================================
// ส่วนที่ 3: Main Function (จุดเริ่มต้นของโปรแกรม)
// =================================================================
int main(int argc, char** argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "odometry_publisher_node");

    // Create an object of the RobotBase class.
    // The constructor will be called, setting up publishers and subscribers.
    RobotBase my_robot_base;

    // Spin to keep the node alive and process callbacks
    ROS_INFO("Odometry Publisher Node is up and running.");
    ros::spin();

    return 0;
}