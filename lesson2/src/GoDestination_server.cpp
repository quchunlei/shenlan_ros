#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "lesson2/GoDestinationAction.h"
#include <geometry_msgs/Pose.h>

typedef actionlib::SimpleActionServer<lesson2::GoDestinationAction> Server;

// 收到action的goal后调用该回调函数
void execute(const lesson2::GoDestinationGoalConstPtr& goal, Server* as)
{
    ros::Rate r(1);
    lesson2::GoDestinationFeedback feedback;

    ROS_INFO("Destination [%d,%d] is going.", goal->x, goal->y);

    // 假设洗盘子的进度，并且按照1hz的频率发布进度feedback
    bool xdirection=true;
    int total=goal->x+goal->y;
    for(int i=0,j=0; i<goal->x||j<goal->y; )
    {
       if((j==goal->y|| xdirection) && i<goal->x )
       { 
		    i++;
			feedback.x=i;
		    xdirection=false;
		}
		else if(j<goal->y){
		    j++;
			feedback.y=j;
			xdirection=true;
		}

        feedback.percent_complete = ((i+j)*100.0/total);
        as->publishFeedback(feedback);
        r.sleep();
    }

    // 当action完成后，向客户端返回结果
    ROS_INFO("Destination [%d,%d] is arrived.", goal->x, goal->y);
    as->setSucceeded();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "go_destination_server");
    ros::NodeHandle n;

    // 定义一个服务器
    Server server(n, "go_destination", boost::bind(&execute, _1, &server), false);
    
    // 服务器开始运行
    server.start();

    ros::spin();

    return 0;
}
