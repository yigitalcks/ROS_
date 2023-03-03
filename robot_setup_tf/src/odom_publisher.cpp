#include "nav_msgs/Odometry.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "ros/ros.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "odom_publisher");

    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    tf2_ros::TransformBroadcaster br;

    double x    {0.0};
    double y    {0.0};
    double th   {0.0};

    double vx   {0.1};
    double vy   {-0.1};
    double vth  {0.1};

   ros::Time current_time, last_time;
    current_time = last_time = ros::Time::now();

    ros::Rate r(1.0);

    while(ros::ok()) {

        ros::spinOnce();               
        current_time = ros::Time::now();

        double dt = (current_time - last_time).toSec();
        double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
        double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
        double delta_th = vth * dt;

        x += delta_x;
        y += delta_y;
        th += delta_th;

        tf2::Quaternion qt;
        qt.setRPY(0, 0, th);

        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;

        odom_trans.transform.rotation.x = qt.x();
        odom_trans.transform.rotation.y = qt.y();
        odom_trans.transform.rotation.z = qt.z();
        odom_trans.transform.rotation.w = qt.w();

        br.sendTransform(odom_trans);

        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;

        odom.pose.pose.orientation.x = qt.x();
        odom.pose.pose.orientation.y = qt.y();
        odom.pose.pose.orientation.z = qt.z();
        odom.pose.pose.orientation.w = qt.w();

        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = vth;

        pub.publish(odom);


        last_time = current_time;
        r.sleep();

    }
}