#include <actionlib/client/simple_action_client.h>
#include "lesson2/GoDestinationAction.h"

typedef actionlib::SimpleActionClient<lesson2::GoDestinationAction> Client;

// 当action完成后会调用该回调函数一次
void doneCb(const actionlib::SimpleClientGoalState& state,
        const lesson2::GoDestinationResultConstPtr& result)
{
    ROS_INFO("Yay! The destination are now arrived");
    ros::shutdown();
}

// 当action激活后会调用该回调函数一次
void activeCb()
{
    ROS_INFO("Destination just went active");
}

// 收到feedback后调用该回调函数
void feedbackCb(const lesson2::GoDestinationFeedbackConstPtr& feedback)
{
ROS_INFO(" x : %d ", feedback->x);    
ROS_INFO(" y : %d ", feedback->y);
ROS_INFO(" percent_complete : %f ", feedback->percent_complete);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "go_destination_client");

    // 定义一个客户端
    Client client("go_destination", true);

    // 等待服务器端
    ROS_INFO("Waiting for action server to start.");
    client.waitForServer();
    ROS_INFO("Action server started, sending destination.");

    // 创建一个action的goal
    lesson2::GoDestinationGoal goal;
    goal.x = 4;
    goal.y = 0;

    // 发送action的goal给服务器端，并且设置回调函数
    client.sendGoal(goal,  &doneCb, &activeCb, &feedbackCb);

    ros::spin();

    return 0;
}
