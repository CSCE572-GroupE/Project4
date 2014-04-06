#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <algorithm>
#include <std_msgs/Int32.h>
#include <create_node/TurtlebotSensorState.h>

using namespace std;

const string IFNAME = "wlan0";
const int RATE = 100;
const int battery_wifi_publish_rate = 5 * RATE;
int cycle_count = 0;

int batteryPercentage;
int bumps_wheeldrops;


std_msgs::Int32 determineWifiStrength(){
    std_msgs::Int32 value;
    value.data = -1;

    ifstream input("/proc/net/wireless");
    if(!input)
    {
     	cout << "Couldn't open the file\n";
	return value;
    }

    string line;
    string word;
	
    while(getline(input, line)) {
        istringstream bp(line);	
        string fname;
        bp >> fname;
        if(fname == IFNAME + ':')
        {
            replace(line.begin(), line.end(), '.', ' ');
	    bp >> word; bp >> word;
            ROS_INFO_STREAM("Current WiFi Signal Strength: " << word << '\n');
            value.data = atoi(word.c_str());
 	    return value;    
 	}
    }

    return value;
}

void turtlebotSensorStateMessageReceived(create_node::TurtlebotSensorState sensorState){
	batteryPercentage = int(double(sensorState.charge)/sensorState.capacity * 100);	
	bumps_wheeldrops = sensorState.bumps_wheeldrops;
}

int main(int argc, char** argv)
{
	ros::init(argc,argv, "vitals_node");
	ros::NodeHandle nh;

	ros::Publisher wifiPublisher = nh.advertise <std_msgs::Int32>("/wifi_ss",1000);

	ros::Subscriber batterySubscriber = nh.subscribe("/turtlebot_node/sensor_state", 1000, &turtlebotSensorStateMessageReceived);
	ros::Publisher batteryPublisher = nh.advertise <std_msgs::Int32>("/battery_percentage", 1000);
	ros::Publisher wheeldropPublisher = nh.advertise <std_msgs::Int32>("/bump_wheeldrop", 1000);	

	ros::Rate rate(100);

	while(ros::ok()){
	    if(cycle_count % battery_wifi_publish_rate == 0){ 
	    	wifiPublisher.publish(determineWifiStrength());

	    	std_msgs::Int32 battery;
	   	 battery.data = batteryPercentage;
	    	batteryPublisher.publish(battery);
	    }	
  	    std_msgs::Int32 wheeldrop;
	    wheeldrop.data = bumps_wheeldrops;
	    wheeldropPublisher.publish(wheeldrop);	    
	    
	    cycle_count++;
	    rate.sleep();
	    ros::spinOnce();
	}
}


