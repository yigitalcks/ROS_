#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

int main(int argc, char** argv) {

    ros::init(argc, argv, "broadcaster");

    ros::NodeHandle n;

    ros::Rate r(100);

    tf2_ros::TransformBroadcaster tb;
    geometry_msgs::TransformStamped transform1;
    geometry_msgs::TransformStamped transform2;

    tf2::Quaternion qt;
    qt.setRPY(0.0, 0.0, 0.0);

    transform1.header.frame_id = "base_link";
    transform1.child_frame_id = "base_laser";

    transform1.transform.translation.x = 0.1;
    transform1.transform.translation.y = 0.0;
    transform1.transform.translation.z = 0.2;

    transform1.transform.rotation.x = 0;
    transform1.transform.rotation.y = 0;
    transform1.transform.rotation.z = 0;
    transform1.transform.rotation.w = 1;

    transform2.header.frame_id = "world";
    transform2.child_frame_id = "base_link";

    transform2.transform.translation.x = 0.0;
    transform2.transform.translation.y = 0.0;
    transform2.transform.translation.z = 0.0;

    transform2.transform.rotation.x = qt.x();
    transform2.transform.rotation.y = qt.y();
    transform2.transform.rotation.z = qt.z();
    transform2.transform.rotation.w = qt.w();

    while(ros::ok()) {
        transform1.header.stamp = ros::Time::now();
        tb.sendTransform(transform1);
        transform2.header.stamp = ros::Time::now();
        tb.sendTransform(transform2);
        r.sleep();
    }


    return 0;
}
