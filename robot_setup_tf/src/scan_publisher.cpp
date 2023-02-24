#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

int main(int argc, char** argv) {

    ros::init(argc, argv, "scan_publisher");

    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<sensor_msgs::LaserScan>("scan", 50);

    unsigned int num_readings = 100;
    double laser_frequency = 40;
    double ranges[num_readings];
    double intensities[num_readings];

    int count = 0;
    ros::Rate r(1.0);

    while(ros::ok()) {
        for(int i = 0; i < num_readings; ++i){
            ranges[i] = count;
            intensities[i] = 100 + count;
        }

        sensor_msgs::LaserScan scan;

        scan.header.stamp = ros::Time::now();
        scan.header.frame_id = "base_laser";
        scan.angle_min = -1.57;
        scan.angle_max = 1.57;
        scan.angle_increment = 3.14 / num_readings;
        scan.time_increment = (1 / laser_frequency) / (num_readings);
        scan.range_min = 0.0;
        scan.range_max = 100.0;

        scan.ranges.resize(num_readings);
        scan.intensities.resize(num_readings);

        for(unsigned int i = 0; i < num_readings; ++i){
            scan.ranges[i] = ranges[i];
            scan.intensities[i] = intensities[i];
        }
        pub.publish(scan);
        ++count;
        r.sleep();
    }










    return 0;
}