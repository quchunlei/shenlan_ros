#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>
#include <turtlesim/Kill.h>

int main(int argc, char** argv)
{
    // 初始化节点
    ros::init(argc, argv, "my_turtle");

    ros::NodeHandle node;
   
    // 通过服务调用，产生第二只乌龟turtle2
    ros::service::waitForService("spawn");
    ros::ServiceClient add_turtle = node.serviceClient<turtlesim::Spawn>("spawn");
    turtlesim::Spawn::Request req;
    turtlesim::Spawn::Response resp;
    req.x=5;
    req.y=5;
    req.theta=0;
    req.name="turtle2";
    bool success=add_turtle.call(req,resp);

    if(success)
    {
	ROS_INFO("Spawn a turtle nemed turtle2");
    }else{
        ROS_INFO("Spawn a turtle failed.");
     }


    ros::service::waitForService("kill");
    ros::ServiceClient kill_turtle = node.serviceClient<turtlesim::Kill>("kill");
    turtlesim::Kill turtle;
    turtle.request.name="turtle1";
    kill_turtle.call(turtle);
    // 定义turtle2的速度控制发布器
    ros::Publisher turtle_vel =
    node.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 10);

    ros::Rate rate(10.0);
    while (node.ok())
    {
        // 并发布速度控制指令，使turtle2做圆周运动
        geometry_msgs::Twist vel_msg;
        vel_msg.angular.z = 1;
        vel_msg.linear.x = 2;
        turtle_vel.publish(vel_msg);
        //发布turtle2的位置
        rate.sleep();
    }
    return 0;
};
