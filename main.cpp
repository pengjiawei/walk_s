#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_datatypes.h>
#include <vector>
#include <ostream>
#include <boost_161_pthread_condition_variable_fwd.h>
#include <boost/thread.hpp>
#include "tool.h"
#include <move_base_msgs/MoveBaseActionResult.h>

ros::Publisher goal_pub;
geometry_msgs::PoseStamped pose;
boost::condition_variable cv;
boost::mutex mutex;
bool ready = true;
double pose_x, pose_y,w;
double theta = M_PI_2;
double distance = 1.5;
const unsigned char SUCCEEDED = 3;

void turnAndGo(double theta, double distance);

int times = 1;

typedef enum {
    LEFT = 1,
    ONE_STEP,
    RIGHT,
    RUN
};
int state_array[8] = {LEFT, ONE_STEP, LEFT, RUN, RIGHT, ONE_STEP, RIGHT, RUN};
int current_state = 7;

double get_distance(double pose_x, double polygon_x) {
    return pose_x > polygon_x ? pose_x - polygon_x : polygon_x - pose_x;
}

//void footprint_callback(geometry_msgs::PolygonStamped poly) {
//    int size = poly.polygon.points.size();
//
//    pose.pose.position.x = (poly.polygon.points[0].x + poly.polygon.points[2].x) / 2;
//    pose.pose.position.y = (poly.polygon.points[0].y + poly.polygon.points[2].y) / 2;
////    ROS_INFO("CURRENT POSE = (%lf,%lf)", pose.pose.position.x, pose.pose.position.y);
//    pose_x = pose.pose.position.x;
//    pose_y = pose.pose.position.y;
//
//    distance = get_distance(pose_x, poly.polygon.points[0].x);
//
//    for (int i = 0; i < times; ++i) {
//        cv.notify_one();
//    }
//    times = 0;
//
//}
void amclpose_callback(geometry_msgs::PoseWithCovarianceStamped pose){
    pose_x = pose.pose.pose.position.x;
    pose_y = pose.pose.pose.position.y;
    w = pose.pose.pose.orientation.w;
    ROS_INFO("CURRENT POSE = (%lf,%lf),W = %lf", pose_x, pose_y,w);
    for (int i = 0; i < times; ++i) {
        cv.notify_one();
    }
    times = 0;
}

void turn(double theta) {

    geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(theta);
    //qian +x , zuo +y
    geometry_msgs::PoseStamped goal_pose;
    goal_pose.header.frame_id = "base_link";
    goal_pose.header.stamp = ros::Time::now();
    ROS_INFO("publish theta = %lf", theta);
    //turn left and go 0.5m
    goal_pose.pose.position.x = 0;
    goal_pose.pose.position.y = 0;
    goal_pose.pose.orientation = q;
    goal_pub.publish(goal_pose);
//        ready = false;

}

void turnleft() {
    ROS_INFO("turn left");
    turn(M_PI_2);

}

void turnright() {
    ROS_INFO("turn right");
    turn(-M_PI_2);
}

void GoAhead(double distance) {
    //qian +x , zuo +y
    geometry_msgs::PoseStamped goal_pose;
    goal_pose.header.frame_id = "base_link";
    goal_pose.header.stamp = ros::Time::now();
    ROS_INFO("publish distance = %lf", distance);
    //x + distance ,go ahead
    goal_pose.pose.position.x = distance;
    goal_pose.pose.position.y = 0;
    goal_pose.pose.orientation.w = 1;
    goal_pub.publish(goal_pose);
//        ready = false;
}

void oneStep() {
    ROS_INFO("one step");
    //it should be the radius of robot
    double step_size = 0.5;
    GoAhead(step_size);

}

void Run() {
    ROS_INFO("run");

    double run_distance = 5;
    GoAhead(run_distance);

}

void control() {
    while (ready) {
        boost::unique_lock<boost::mutex> lock(mutex);
        cv.wait(lock);
        switch (state_array[current_state]) {
            case LEFT:
                turnleft();
                break;
            case ONE_STEP:
                oneStep();
                break;
            case RIGHT:
                turnright();
                break;
            case RUN:
                Run();
                break;
            default:
                ROS_INFO("Switch control nothing");
                break;
        }
    }
}

void my_costmap_callback(nav_msgs::OccupancyGrid occ) {
    double origin_x = occ.info.origin.position.x;
    double origin_y = occ.info.origin.position.y;
    int width = occ.info.width;
    int height = occ.info.height;
    size_x = width;
    size_y = height;
    double resolution = occ.info.resolution;
    std::vector<int> frontiers;
    findFrontier(occ, frontiers);
    for (int i = 0; i < frontiers.size(); ++i) {
        double distance = DBL_MAX;
        double frontier_x = (frontiers[i] % width) * resolution + origin_x;
        double frontier_y = (frontiers[i] / width) * resolution + origin_y;
//        ROS_INFO("frontier_x = %lf,y = %lf",frontier_x,frontier_y);

    }
    ROS_INFO("origin (%lf,%lf),width = %d,height = %d,resolution = %lf,data size = %d", origin_x, origin_y, width,
             height, resolution, occ.data.size());
}

void result_callback(move_base_msgs::MoveBaseActionResult feedback) {
    ROS_INFO("feedback status = %d", feedback.status.status);
    if (feedback.status.status == SUCCEEDED) {
        ROS_INFO("status has been set SUCCEEDED");
        current_state = (++current_state) % 8;
        ready = true;
        cv.notify_one();
    }

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "lookup");
    ros::NodeHandle nh;
    goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1000);
//    ros::Subscriber footprint_sub = nh.subscribe("/move_base_node/global_costmap/footprint", 1000, footprint_callback);
    ros::Subscriber costmap_sub = nh.subscribe("/map", 1000, my_costmap_callback);
    ros::Subscriber feedback_sub = nh.subscribe("/move_base/result", 1000, result_callback);
    ros::Subscriber slam_pose_sub = nh.subscribe("/amcl_pose",1000,amclpose_callback);
    boost::thread thread(control);

    ros::spin();
    std::cout << "Hello, World!" << std::endl;
    return 0;
}