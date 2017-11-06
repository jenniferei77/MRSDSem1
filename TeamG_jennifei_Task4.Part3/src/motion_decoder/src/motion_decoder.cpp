#include <ros/ros.h>
#include <motion_decoder/image_converter.hpp>
#include <apriltags_ros/AprilTagDetectionArray.h>
#include <apriltags_ros/AprilTagDetection.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

using namespace std;

ros::Subscriber apDetect_sub;
ImageConverter* ic;

void apriltag_detection_callback(const apriltags_ros::AprilTagDetectionArray msg)
{
    for(int i=0; i<msg.detections.size(); i++) {
        //
        static tf::TransformBroadcaster broad;
        geometry_msgs::PoseStamped curr_pose = msg.detections[i].pose;
        double x_pose = curr_pose.pose.position.x;
        double y_pose = curr_pose.pose.position.y;
        double z_pose = curr_pose.pose.position.z;
        ROS_INFO("x: %f, y: %f, z: %f", x_pose, y_pose, z_pose);
        double q0 = curr_pose.pose.orientation.x;
        double q1 = curr_pose.pose.orientation.y;
        double q2 = curr_pose.pose.orientation.z;
        double q3 = curr_pose.pose.orientation.w;
        ROS_INFO("x: %f, y: %f, z: %f, w: %f", q0, q1, q2, q3);
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(x_pose, y_pose, z_pose));
        transform.setRotation(tf::Quaternion (q0, q1, q2, q3));
        broad.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera", "april_tf"));
        ic->setTagLocations(x_pose,y_pose,z_pose);

    }
    //TODO: Parse message and publish transforms as apriltag_tf and camera
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_converter");

    ros::NodeHandle n;
    ros::Subscriber apDetect_sub = n.subscribe("tag_detections", 1000, apriltag_detection_callback);
    //TODO: Add a subscriber to get the AprilTag detections The callback function skeleton is given.
    ImageConverter converter;
    ic = &converter;
    ros::Rate loop_rate(50);
    ROS_INFO("In main\n");
    while(ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}