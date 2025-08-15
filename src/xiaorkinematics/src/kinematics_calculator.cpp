#include "ros/ros.h"
#include "tf2_ros/transform_listener.h" // หัวใจหลักในการดักฟัง TF
#include "geometry_msgs/TransformStamped.h"
#include <cmath> // สำหรับการคำนวณ sqrt และ pow

// ฟังก์ชันสำหรับคำนวณและพิมพ์ระยะทาง
void print_distance(const std::string& from_frame, const std::string& to_frame, const geometry_msgs::TransformStamped& transform) {
    double x = transform.transform.translation.x;
    double y = transform.transform.translation.y;
    double z = transform.transform.translation.z;

    // คำนวณระยะทางแบบยุคลิด (Euclidean distance)
    double distance = std::sqrt(std::pow(x, 2) + std::pow(y, 2) + std::pow(z, 2));

    ROS_INFO("Distance from '%s' to '%s': %.4f meters", from_frame.c_str(), to_frame.c_str(), distance);
    ROS_INFO("  -> (x: %.4f, y: %.4f, z: %.4f)", x, y, z);
}

int main(int argc, char **argv) {
    // Khởi tạo node ROS
    ros::init(argc, argv, "kinematics_calculator_node");
    ros::NodeHandle node;

    // สร้าง Buffer และ Listener สำหรับ TF
    // Listener จะเริ่มดักฟัง TF tree โดยอัตโนมัติ
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    // เราไม่จำเป็นต้องเช็คตลอดเวลา ตั้งค่าให้เช็คทุกๆ 1 วินาทีก็พอ
    ros::Rate rate(1.0);

    // รอสักครู่เพื่อให้ TF tree ถูก publish ขึ้นมาอย่างสมบูรณ์
    ROS_INFO("Waiting for TF tree to be published...");
    ros::Duration(2.0).sleep(); 
    ROS_INFO("Starting to look up transforms.");

    while (node.ok()) {
        geometry_msgs::TransformStamped base_to_left_wheel;
        geometry_msgs::TransformStamped base_to_right_wheel;
        geometry_msgs::TransformStamped base_to_camera;

        try {
            // ค้นหา Transform จาก base_link ไปยังล้อซ้าย
            base_to_left_wheel = tfBuffer.lookupTransform("base_link", "left_wheel_link", ros::Time(0));
            print_distance("base_link", "left_wheel_link", base_to_left_wheel);

            // ค้นหา Transform จาก base_link ไปยังล้อขวา
            base_to_right_wheel = tfBuffer.lookupTransform("base_link", "right_wheel_link", ros::Time(0));
            print_distance("base_link", "right_wheel_link", base_to_right_wheel);

            // ค้นหา Transform จาก base_link ไปยังกล้อง
            base_to_camera = tfBuffer.lookupTransform("base_link", "astra_link", ros::Time(0));
            print_distance("base_link", "astra_link", base_to_camera);

            // คำนวณ Wheel Separation (ระยะห่างระหว่างล้อ)
            double wheel_sep_x = base_to_left_wheel.transform.translation.x - base_to_right_wheel.transform.translation.x;
            double wheel_sep_y = base_to_left_wheel.transform.translation.y - base_to_right_wheel.transform.translation.y;
            double wheel_separation = std::sqrt(std::pow(wheel_sep_x, 2) + std::pow(wheel_sep_y, 2));
            ROS_INFO("----------------------------------------------------");
            ROS_INFO("Calculated Wheel Separation: %.4f meters", wheel_separation);
            ROS_INFO("----------------------------------------------------");


        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            ROS_WARN("Could not get transform. Is the simulation running and robot spawned?");
            ros::Duration(1.0).sleep();
            continue;
        }

        // หยุดการทำงานของ loop ชั่วคราว (ไม่จำเป็นต้องรันตลอด)
        // เราสามารถ break; เพื่อให้อ่านค่าครั้งเดียวแล้วจบโปรแกรมได้
        break; 
    }

    return 0;
}
