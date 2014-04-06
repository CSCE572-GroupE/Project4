#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>

int last_bumpWheeldrop = 0;

void wifiMessageReceived(std_msgs::Int32 wifiStrengthObject){
	ROS_INFO_STREAM("Wifi Signal Strength: " << wifiStrengthObject.data << "%");
}

void batteryMessageReceived(std_msgs::Int32 batteryPercentageObject){
	ROS_INFO_STREAM("Create Battery Status: " << batteryPercentageObject.data << "%");
}

void diagnosticMessageRecieved(diagnostic_msgs::DiagnosticArray diagnosticArray){
	for (int i = 0; i < sizeof(diagnosticArray.status); i++){
 		diagnostic_msgs::DiagnosticStatus status = diagnosticArray.status[i];
		if (status.level == 1){
			ROS_INFO_STREAM("WARNING: " << status.name << " -- " << status.message);
		} else if (status.level == 2){
			ROS_INFO_STREAM("ERROR: " << status.name << " -- " << status.message);
		}
  	}
}

void contactBumperRight(){
	ROS_INFO_STREAM("Bumper contact right.");
}

void contactBumperLeft(){
	ROS_INFO_STREAM("Bumper contact left.");
}

void contactBumperCenter(){
	ROS_INFO_STREAM("Bumper contact center.");
}

void wheelDropRight(){
	ROS_INFO_STREAM("Wheel drop right.");
}

void wheelDropLeft(){
	ROS_INFO_STREAM("Wheel drop left.");
}

void wheelDropCaster(){
	ROS_INFO_STREAM("Wheel drop caster.");
}


void bumpWheeldropMessageReceived(std_msgs::Int32 bumpWheeldropObject){
	int bumpWheeldrop = bumpWheeldropObject.data;
	
	if (bumpWheeldrop == last_bumpWheeldrop){
		return;
	} else if (bumpWheeldrop > last_bumpWheeldrop){
		bumpWheeldrop -= last_bumpWheeldrop;
		last_bumpWheeldrop = bumpWheeldropObject.data;
	} else {
		last_bumpWheeldrop = bumpWheeldropObject.data;
		return;
	}
	
	int bump = bumpWheeldrop % 4;
	int wheeldrop = bumpWheeldrop - bump;
	
	switch(bump){
		case 0:
			//no bumps
		  	break;
		case 1:
    			//right bumper contact
    			contactBumperRight();
    			break;
    		case 2:
    			//left bumper contact
    			contactBumperLeft();
    			break;
   		case 3:
    			//right and left bumper contact
    			contactBumperCenter();
    			break;
		default:
			//default error action
		   	// Shouldn't be here
		break;
	}
	
	switch(wheeldrop) {
		case 0:
			//all wheels are on the ground and the front bumper is not colliding with any objects
			//send out no messages
		      	break;
	    	case 4:
	    		//right wheel dropping
	    		wheelDropRight();
	    		break;
	    	case 8:
	        	//left wheel dropping
	    		wheelDropLeft();
	    		break;
	    	case 12:
	    		//right and left wheels dropping
			wheelDropRight();
	    		wheelDropLeft();
	    		break;
	    	case 16:
	    		//caster wheel dropping
	    		wheelDropCaster();
	    		break;
	    	case 20:
	    		//right and caster wheel dropping
	    		wheelDropRight();
			wheelDropCaster();
	    		break;
	   	case 24:
	    		//left and caster wheel dropping
	    		wheelDropLeft();
	   		wheelDropCaster();
	    		break;
	    	case 28:
	    		//all three wheels dropped
	    		wheelDropRight();
	    		wheelDropLeft();
			wheelDropCaster();
	    		break;
	    	default:
	    		//default error action
	    		// Shouldn't be here
	    		break;
	 }	
	
}


int main(int argc, char **argv){
	ros::init(argc, argv, "monitor_node");
	ros::NodeHandle nh;
	
	ros::Subscriber wifiSubcriber = nh.subscribe("/wifi_ss", 1000, &wifiMessageReceived);
 	ros::Subscriber batterySubscriber = nh.subscribe("/battery_percentage", 1000, &batteryMessageReceived);
	ros::Subscriber diagnosticSubscriber = nh.subscribe("/diagnostics_agg", 1000, &diagnosticMessageRecieved);
	ros::Subscriber bumpWheeldropSubscriber = nh.subscribe("/bump_wheeldrop", 1000, &bumpWheeldropMessageReceived);
	ros::spin();
}
