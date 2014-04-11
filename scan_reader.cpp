#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/Int32.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>

using namespace std;

struct point {
    float range;
    float angle;

    point(){
        range = 0.0f;
        angle = 0.0f;
    }

    point(float range_val, float angle_val)
    {
        range = range_val;
        angle = angle_val;
    }


    /**
     * Purpose: Turns an angle and a range into coordinate (x,y) values
     */
    float getX(){
        return range * cos(angle);
    }

    float getY(){
        return range * sin(angle);
    }

};

struct pointObject {
    vector<point> points;

    pointObject(){
    }

    void addPoint(point p){
        points.push_back(p);
    }

    point lastPoint(){
        return points.back();
    }

    void clear(){
        points.clear();
    }

    int size(){
        return points.size();
    }
};


/***** Constants ******/
const float POINT_MAX_DISTANCE = .25f; //Defines max distance between one point in object and the adjacent point
const float POINT_RANGE_MAX = .5f; //Defines max difference in range that two points can have and still be considered for the same object

const int MIN_OBJECT_POINTS = 30; //Min number of points that must be connected in order to be considered an object
const int MIN_INF_OBJECT_POINTS = 50;
const float ANGLE_PADDING = M_PI/150.;
const int MIN_WIFI_STRENGTH_THRESHOLD = 20;

const int RATE = 30;

bool wifi_valid = true;
bool bump_sensor_valid = true;
bool diagnostics_valid = true;

std::vector < pointObject > objects;
std::vector < pointObject > inf_objects;

int last_motion = 0;
int multiplier = 1;
int loop_count = 0;
bool motion_left = false;
bool motion_right = false;
bool motion_straight = false;
float horizontal_movement = 0;
float angular_movmement = 0;


/**
 * Purpose: Checks to see if range is infinity or NaN
 */
bool isValidRange(float range){
    return !(isinf(range) || isnan(range));
}

/**
 * Purpose: Checks if two range values are close together
 */
bool similarRanges(float rangeA, float rangeB){
    if (!isValidRange(rangeA) || !isValidRange(rangeB)) return false;
    return (fabs(rangeA - rangeB) < POINT_RANGE_MAX);
}

/**
 * Purpose: Calulates distance between two points by using SAS Formula
 */
float distanceBetweenPoints(point pointA, point pointB){
    //Side-Angle-Side Formula: A^2 = B^2 + C^2 - 2*B*C*cos(angleA)
    return pow(pointA.range,2) + pow(pointB.range,2) - 2 * pointA.range * pointB.range * cos(fabs(pointA.angle - pointB.angle));
}

/**
 * Purpose: Determines if two points are close together
 */
bool similarPoints(point pointA, point pointB){
    return distanceBetweenPoints(pointA, pointB) < POINT_MAX_DISTANCE;
}

bool motionAllowed(){
    return wifi_valid && bump_sensor_valid && diagnostics_valid;
}

point calculateCenter(vector<point> object){
    float total_range = 0;
    float total_angle = 0;
    float object_size = object.size();
    for (int i = 0; i < int(object_size); i++){
        point p = object.at(i);
        total_range += p.range;
        total_angle += p.angle;
    }
    //ROS_INFO_STREAM("Calc Center: Range-" << total_range << " Angle-" << total_angle << " Size-" << object_size);
    return point(total_range/object_size, total_angle/object_size);
}

void laserScanReceived(const sensor_msgs::LaserScan &scanMessage){
    objects.clear();
    inf_objects.clear();
    float MAX_RANGE = scanMessage.range_max;
    std::vector<float> ranges = scanMessage.ranges;
    pointObject object;
    pointObject inf_object;
    point current_point;
    point last_point;
    for (int i = 0; i < int(ranges.size()); i++){
        float angle = scanMessage.angle_min + scanMessage.angle_increment * i;
        float range = ranges.at(i);
		current_point = point(range,angle);

        if (i == 0){
            last_point = current_point;
            continue;
        }
        if (isValidRange(range)){
            if (isValidRange(last_point.range) && similarPoints(last_point, current_point)){
                object.addPoint(current_point);
            } else {
                bool objectFound = false;
                for(int j = 0; j < int(objects.size()); j++){
                    pointObject obj = objects.at(j);
                    if(similarPoints(obj.lastPoint(), current_point)){
                        obj.addPoint(current_point);
                        objectFound = true;
                    }
                }
                if (!objectFound){
                    if (int(object.size()) > MIN_OBJECT_POINTS){
                        objects.push_back(object);
                    }
                    object.clear();
                    object.addPoint(current_point);
                }
            }
        } else { 
            point temp_point = point(MAX_RANGE, current_point.angle);
            if (!isValidRange(last_point.range)){
                inf_object.addPoint(temp_point);
            } else {
                if (int(inf_object.size()) > MIN_INF_OBJECT_POINTS){
                    inf_objects.push_back(inf_object);
                }
                inf_object.clear();
                inf_object.addPoint(temp_point);
            }
        }
        last_point = current_point;
    }
    objects.push_back(object);
}

void bumpWheeldropMessageReceived(std_msgs::Int32 bumpWheeldropObject){
    int bumpWheeldrop = bumpWheeldropObject.data;
    if (bumpWheeldrop != 0){
        bump_sensor_valid = false;
    } else {
        bump_sensor_valid = true;
    }
}

void wifiMessageReceived(std_msgs::Int32 wifiStrengthObject){
    ROS_INFO_STREAM("Wifi Signal Strength: " << wifiStrengthObject.data << "%");
    if (wifiStrengthObject.data <= MIN_WIFI_STRENGTH_THRESHOLD){
        wifi_valid = false;
    } else {
        wifi_valid = true;
    }
}


void diagnosticMessageRecieved(diagnostic_msgs::DiagnosticArray diagnosticArray){
    bool issue_found = false;
    for (int i = 0; i < sizeof(diagnosticArray.status); i++){
        diagnostic_msgs::DiagnosticStatus status = diagnosticArray.status[i];
        if (status.level == 1){
            ROS_INFO_STREAM("WARNING: " << status.name << " -- " << status.message);
            issue_found = true;
        } else if (status.level == 2){
            ROS_INFO_STREAM("ERROR: " << status.name << " -- " << status.message);
            issue_found = true;
        }
    }
    if (issue_found){
        diagnostics_valid = false;
    } else {
        diagnostics_valid = true;
    }
}


int main(int argc, char **argv){
    ros::init(argc, argv, "scan_reader");
	ros::NodeHandle nh;

    ros::Subscriber laserSubcriber = nh.subscribe("/scan", 1000, &laserScanReceived);

    ros::Subscriber wifiSubcriber = nh.subscribe("/wifi_ss", 1000, &wifiMessageReceived);
    ros::Subscriber diagnosticSubscriber = nh.subscribe("/diagnostics_agg", 1000, &diagnosticMessageRecieved);
    ros::Subscriber bumpWheeldropSubscriber = nh.subscribe("/bump_wheeldrop", 1000, &bumpWheeldropMessageReceived);

    ros::Publisher twistPublisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi",1000);

    ros::Publisher objectPublisher = nh.advertise<sensor_msgs::PointCloud>(
        "object_locations", 1000);

    ros::Rate rate(RATE);

    vector<point> centers;

    while(ros::ok()){

        centers.clear();
        // Create pointcloud to track people's movements
        sensor_msgs::PointCloud cloud;
        cloud.header.stamp = ros::Time::now();
        cloud.header.frame_id = "camera_depth_frame";

        int max_size = 0;
        pointObject max_inf_object;
        for (int i = 0; i < int(inf_objects.size()); i++){
            pointObject object = inf_objects.at(i);
            if (int(object.size()) > max_size){
                max_size = object.size();
                max_inf_object = object;
            }
        }

        point farthest_object_center;
        int object_max_distance = 0;
        for (int i = 0; i < int(objects.size()); i++){
            pointObject object = objects.at(i);
            point object_center = calculateCenter(object.points);
            if (distanceBetweenPoints(object_center, point()) > object_max_distance){
                object_max_distance = distanceBetweenPoints(object_center, point());
                farthest_object_center = object_center;
            }
            centers.push_back(object_center);
        }

        point target_point = calculateCenter(max_inf_object.points);

        cloud.points.resize(centers.size());

        // Map average points on PointCloud .
        for (int i = 0; i < int(centers.size()); i++){
            point p = centers.at(i);
            cloud.points[i].x = p.getX();
            cloud.points[i].y = p.getY();
            cloud.points[i].z = 0;
       }
        objectPublisher.publish(cloud);


        geometry_msgs::Twist twistObject;

        if (inf_objects.size() > 0){
            if(target_point.angle < -ANGLE_PADDING) {
                twistObject.linear.x = horizontal_movement < .05 ? 0 : horizontal_movement * .20f;
                twistObject.angular.z = -.3;//-target_point.angle > -.1 ? -.1 : target_point.angle;
//                 ROS_INFO_STREAM("RIGHT- x:" << twistObject.linear.x << " z:" << twistObject.angular.z);
                 motion_right = true;
            } else if (target_point.angle > ANGLE_PADDING){
                twistObject.linear.x = horizontal_movement < .05 ? 0 : horizontal_movement * .20f;
                twistObject.angular.z = .3; //target_point.angle < .1 ? .1 : target_point.angle;
//                ROS_INFO_STREAM("LEFT- x:" << twistObject.linear.x << " z:" << twistObject.angular.z);
                motion_left = true;
            } else {
                twistObject.linear.x = horizontal_movement == 0 ? .5f : horizontal_movement > 10 ? 10 : horizontal_movement * 1.1f;
                twistObject.angular.z = 0;
//              ROS_INFO_STREAM("STRAIGHT- x:" << twistObject.linear.x << " z:" << twistObject.angular.z);
                motion_straight = true;
            }


        } else {
            twistObject.linear.x = 0;
            twistObject.angular.z = -M_PI/4;
        }

        loop_count++;
        if (loop_count >= RATE){
            if (motion_right && motion_left && !motion_straight){
                twistObject.linear.x = -10.0f;
                twistObject.angular.z = -M_PI;
            }
            loop_count = 0;
            motion_right = false;
            motion_left = false;
            motion_straight = false;
        }
        if (!motionAllowed()){
            twistObject.linear.x = 0;
            twistObject.angular.z = 0;
        }

        horizontal_movement = twistObject.linear.x;
        angular_movmement = twistObject.angular.z;
        twistPublisher.publish(twistObject);

        ros::spinOnce();
        rate.sleep();
    }
}

