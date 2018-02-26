#include "ros/ros.h"
#include "std_msgs/String.h"
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <iostream>

// Local headers
#include <leg_tracker/laser_processor.h>
#include <leg_tracker/cluster_features.h>

// Custom messages
#include <leg_tracker/Person.h>
#include <leg_tracker/PersonArray.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class RoutinePlanners
{
public:
    int people_id = -1;
    bool people_flag = true;
    move_base_msgs::MoveBaseGoal goal;
    //MoveBaseClient ac("move_base", true);
    //void chatterCallback(const leg_tracker::PersonArray::ConstPtr& msg);
    RoutinePlanners() {
	
	std::string listen_topic;
        nh.param("accelerater_param", accelerater_param, 2.0);
	nh.param("listen_topic", listen_topic, std::string("people_tracked"));

        sub = nh.subscribe(listen_topic, 1000, &RoutinePlanners::chatterCallback,this);
	//pub = nh.advertise<actionlib_msgs::GoalID>("/move_base/cancel",50);

	MoveBaseClient ac("move_base", true);
	while(!ac.waitForServer(ros::Duration(5.0))){
	    ROS_INFO("Waiting for the move_base action server to come up");
	}
	ros::Rate r(1); 
	std::cout << "Enter people ID: ";
	std::cin >> people_id;
	while (ros::ok())
	{
	    
	    if (people_flag)	ac.sendGoal(goal);
	    else {
		//pub.publish(" ");		
		ROS_INFO("cannot send goal!");
		ac.sendGoal({});
		std::cout << "Enter people ID again: ";
		std::cin >> people_id;
		people_flag = true;
		}
	    r.sleep();
	    try{
		ros::spinOnce();                   // Handle ROS events
		}
	    catch (int e)
	    {
	      ROS_ERROR_STREAM("caught exception: " << e);
	    }
	}	
    }
   
private:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    //ros::Publisher pub;
    //ros::Publisher pub2;

    double accelerater_param;
    

    void chatterCallback(const leg_tracker::PersonArray::ConstPtr& msg)
    {
      //ROS_INFO("%d",msg->header.seq);
	std_msgs::String ifo;
	if(!msg->people.empty() && people_id == msg->people[0].id){
	people_flag = true;
	goal.target_pose.header.frame_id = "base_link";
	goal.target_pose.header.stamp = ros::Time::now();

	goal.target_pose.pose.position.x = msg->people[0].pose.position.x * 0.85;
	goal.target_pose.pose.position.y = msg->people[0].pose.position.y * 0.85;
	goal.target_pose.pose.orientation.x = msg->people[0].pose.orientation.x;
	goal.target_pose.pose.orientation.y = msg->people[0].pose.orientation.y;
	goal.target_pose.pose.orientation.z = msg->people[0].pose.orientation.z;	
	goal.target_pose.pose.orientation.w = msg->people[0].pose.orientation.w;

        ROS_INFO("x is : %f, id is : %d",msg->people[0].pose.position.x,msg->people[0].id);
	}
	else people_flag = false;
    }

};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "cpp_listener");
  RoutinePlanners test;
  //ROS_INFO("Sending goal");
  return 0;
}
