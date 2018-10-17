#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Pose.h>

struct RobotGoal
{
   float x;
   float y;
   float yaw;
};

int main(int argc, char** argv)
{
    // 初始化节点
    ros::init(argc, argv, "rviz_pose");
    
    ros::NodeHandle node;
    ros::Rate rate(1);

    ros::Publisher pose_pub_ = node.advertise<geometry_msgs::PoseStamped>("pose", 10);
    struct RobotGoal goal={40, 50, 0.0};
    bool xdirection=true;

    while(node.ok())
	{
            geometry_msgs::PoseStamped pose;
	    pose.header.frame_id = "world";
            float startx=0,starty=0; 
	while(startx<goal.x || starty<goal.y )
	{
            pose.header.stamp = ros::Time::now();
	    if(( starty==goal.y|| xdirection) && startx<goal.x )
		{ 
			startx+=0.1;
			pose.pose.position.x =startx;
			xdirection=false;
		}
		else if( starty<goal.y){
			starty+=0.1;
			pose.pose.position.y = starty;
			xdirection=true;
		}

		pose.pose.orientation = tf::createQuaternionMsgFromYaw(goal.yaw); 
		pose_pub_.publish(pose);

		rate.sleep();
		ros::spinOnce();
	}
    }    


    return 0;
};
