#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>


const int RATE = 1;

/**
 * Purpose: Checks to see if range is infinity or NaN
 */
bool isValidRange(float range){
    return !(isinf(range) || isnan(range));
}


void laserScanReceived(const sensor_msgs::LaserScan &scanMessage){
    std::vector<float> ranges = scanMessage.ranges;

    for (int i = 0; i < int(ranges.size()); i++){
        float angle = scanMessage.angle_min + scanMessage.angle_increment * i;
        float range = ranges.at(i);

        if (isValidRange(range)){
            ROS_INFO_STREAM("ANGLE: " << angle << " Range: " << range);
            return;
        }
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "scan_reader");
	ros::NodeHandle nh;

//	ros::Subscriber laserSubcriber = nh.subscribe("/scan", 1000, &laserScanReceived);
    ros::Publisher twistPublisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop",1000);

    ros::Rate rate(RATE);

    while(ros::ok()){
        geometry_msgs::Twist twistObject;
        twistObject.linear.x = 1;
        twistPublisher.publish(twistObject);
        ros::spinOnce();
        rate.sleep();
    }
}
