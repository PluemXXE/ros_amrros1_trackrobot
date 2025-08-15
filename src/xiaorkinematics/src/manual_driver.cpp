#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"
#include <cmath>

// ตัวแปรเก็บค่าคงที่ของหุ่นยนต์ (ควรจะดึงมาจาก Parameter Server ในอนาคต)
const double WHEEL_RADIUS = 0.038; // รัศมีล้อ (เมตร)
const double WHEEL_SEPARATION = 0.1563; // ระยะห่างระหว่างล้อ (เมตร)

// Global publishers
ros::Publisher left_wheel_pub;
ros::Publisher right_wheel_pub;

// Callback function ที่จะถูกเรียกทุกครั้งที่มีข้อความใหม่ใน /cmd_vel
void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    // ดึงค่าความเร็วเชิงเส้น (v) และความเร็วเชิงมุม (omega)
    double v = msg->linear.x;
    double omega = msg->angular.z;

    // --- Inverse Kinematics Calculation ---
    // คำนวณความเร็วเชิงมุมของล้อแต่ละข้าง (rad/s)
    // v_r = (2*v + omega*L) / (2*R)
    // v_l = (2*v - omega*L) / (2*R)
    double right_wheel_rad_s = (2.0 * v + omega * WHEEL_SEPARATION) / (2.0 * WHEEL_RADIUS);
    double left_wheel_rad_s  = (2.0 * v - omega * WHEEL_SEPARATION) / (2.0 * WHEEL_RADIUS);

    // สร้างข้อความที่จะ publish
    std_msgs::Float64 left_cmd;
    std_msgs::Float64 right_cmd;

    left_cmd.data = left_wheel_rad_s;
    right_cmd.data = right_wheel_rad_s;

    // Publish คำสั่งความเร็วไปยัง Joint Controllers ของ Gazebo
    left_wheel_pub.publish(left_cmd);
    right_wheel_pub.publish(right_cmd);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "manual_driver_node");
    ros::NodeHandle n;

    // สร้าง Subscriber เพื่อดักฟัง /cmd_vel
    ros::Subscriber cmd_vel_sub = n.subscribe("/cmd_vel", 10, cmdVelCallback);

    // สร้าง Publishers เพื่อส่งคำสั่งไปยัง Gazebo Joint Controllers
    // ชื่อ Topic ต้องตรงกับที่ Gazebo สร้างขึ้น: /<robot_name>/<joint_name>_velocity_controller/command
    // ในที่นี้สมมติว่า robot_name คือ transbot
    left_wheel_pub = n.advertise<std_msgs::Float64>("left_wheel/command", 10);
    right_wheel_pub = n.advertise<std_msgs::Float64>("right_wheel/command", 10);

    ROS_INFO("Manual Driver Node is running. Subscribing to /cmd_vel.");

    // ros::spin() จะทำให้ node ทำงานและรอ callback ไปเรื่อยๆ
    ros::spin();

    return 0;
}