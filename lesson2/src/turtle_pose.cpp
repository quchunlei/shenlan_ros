#include <ros/ros.h>
#include <turtlesim/Pose.h>

void poseCallback(const turtlesim::PoseConstPtr& msg)
{
  ROS_INFO("x: [%f]", msg->x);
  ROS_INFO("y: [%f]", msg->y);
  ROS_INFO("theta: [%f]", msg->theta);
}

int main(int argc, char** argv)
{
    // 初始化节点
    ros::init(argc, argv, "turtle2_pose");


    // 订阅乌龟的pose信息并打印
    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe("turtle2/pose", 10, &poseCallback);

    ros::spin();

    return 0;
};
