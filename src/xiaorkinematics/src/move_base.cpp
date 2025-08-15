#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

// ฟังก์ชันสำหรับส่งคำสั่งหยุด
void stop_robot(ros::Publisher& pub) {
    geometry_msgs::Twist stop_msg;
    stop_msg.linear.x = 0.0;
    stop_msg.angular.z = 0.0;
    pub.publish(stop_msg);
}

int main(int argc, char **argv) {
    // Khởi tạo node ROS
    ros::init(argc, argv, "move_base_node");
    ros::NodeHandle n;

    // สร้าง Publisher เพื่อส่งข้อมูลไปยัง Topic /cmd_vel
    ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // รอสักครู่เพื่อให้ Publisher เชื่อมต่อกับ Subscriber ใน Gazebo
    ros::Duration(1.0).sleep();

    ROS_INFO("Starting to drive in a square...");

    // กำหนดค่าความเร็วและเวลา
    double linear_speed = 0.2;  // เมตร/วินาที
    double angular_speed = 0.5; // เรเดียน/วินาที
    double side_length = 1.0;   // ความยาวด้านของสี่เหลี่ยม (เมตร)
    double turn_angle = 1.57;   // มุม 90 องศา (PI/2)

    // คำนวณเวลาที่ต้องใช้
    double move_duration = side_length / linear_speed;
    double turn_duration = turn_angle / angular_speed;

    ros::Rate loop_rate(10); // ทำงานที่ 10 Hz

    // วนลูป 4 ครั้งเพื่อสร้างสี่เหลี่ยม
    for (int i = 0; i < 4; ++i) {
        ROS_INFO("Driving side %d/4...", i + 1);

        // 1. เคลื่อนที่ไปข้างหน้า
        geometry_msgs::Twist move_msg;
        move_msg.linear.x = linear_speed;
        ros::Time start_time = ros::Time::now();
        while (ros::Time::now() - start_time < ros::Duration(move_duration)) {
            cmd_vel_pub.publish(move_msg);
            ros::spinOnce();
            loop_rate.sleep();
        }

        // 2. หยุดหุ่นยนต์ก่อนเลี้ยว
        stop_robot(cmd_vel_pub);
        ros::Duration(0.5).sleep(); // หยุดนิ่งๆ แป๊บนึง

        ROS_INFO("Turning...");

        // 3. หมุน 90 องศา
        geometry_msgs::Twist turn_msg;
        turn_msg.angular.z = angular_speed;
        start_time = ros::Time::now();
        while (ros::Time::now() - start_time < ros::Duration(turn_duration)) {
            cmd_vel_pub.publish(turn_msg);
            ros::spinOnce();
            loop_rate.sleep();
        }

        // 4. หยุดหุ่นยนต์หลังเลี้ยว
        stop_robot(cmd_vel_pub);
        ros::Duration(0.5).sleep();
    }

    ROS_INFO("Finished driving in a square.");

    return 0;
}
