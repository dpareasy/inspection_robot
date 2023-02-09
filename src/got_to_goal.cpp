/**
*\file ReachTarget.cpp
*\brief Autonomous navigation modality
*\author Parisi Davide Leo S4329668
*\version 1.0
*\date 08/04/2022
*\details
*
*Subscribes to: <BR>
* /move_base/feedback
*
* /move_base/goal
*
*Publishes to: <BR>
* /move_base/goal
*
* /move_base/cancel
*
*Description:
*
*This node simulate the autonomous nvigation of a robt within the environment. It asks the user to insert a goal point.
*Once inserted the user can decide to cancel the actual goal or to exit the modality. If a goal point is already defined
*the user can't choose another unless he cancel it.
**/

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "move_base_msgs/MoveBaseActionGoal.h"
#include "move_base_msgs/MoveBaseActionFeedback.h"
#include "geometry_msgs/PointStamped.h"
#include <iostream>
#include <string>
#include <chrono>


using namespace std;


//The publisher is defined as global variable because 
//I have to initialize it in the Main function but 
//use it also in the Reach_point function
//publish the target point
ros::Publisher pubGoal; ///< Publisher for publishing the goal point

ros::Publisher pubCancel; ///< Publisher to cancel the goal

move_base_msgs::MoveBaseActionGoal my_goal; ///< Variable for saving the chosen goal

actionlib_msgs::GoalID lastGoal; ///< Variable for the last goal to cancel 

std::string goalID; ///< Variable used to save the actual goal


double X, Y; ///< X and Y coordinates of the goal


float xG; ///< X variable to store the goal
float yG; ///< Y variable to store the goal

 
bool goalStatus = false; ///< Boolean to state if a goal is set


float error = 0.5; ///< Error for the position of the robot wrt the goal point

void SetGoal(){
    
	//setting x and y coordinates
	my_goal.goal.target_pose.pose.position.x = X;
	my_goal.goal.target_pose.pose.position.y = Y;
	// set the frame_id
	my_goal.goal.target_pose.header.frame_id = "map";
	//set the quaternion module equal to 1
	my_goal.goal.target_pose.pose.orientation.w = 1;
	
	//set the status equal to true
	goalStatus = true;
	
	pubGoal.publish(my_goal);
}


/**
*\brief Description of CancelGoal() function:
*
*In this function the current goal saved is canceled 
*and the variable that gives information about the 
*goal status is set to false, to state that no goal is set.
*If no goal is set, nothing would happen. 
**/
void CancelGoal(){

	if(goalStatus){
		
		system("clear");
		//set the goal to cancel equal to the current goal
		lastGoal.id = goalID;
		
		cout<<"Canceling goal"<<endl;
		
		pubCancel.publish(lastGoal);
		
		//set the status equal to false
		goalStatus = false;
	}
	else
		cout<<"No goal set"<<endl;
}




/**
*\brief Description of GoalStatus() function:
*
*\param msg it's a message published on /move_base/feedback topic.
*
*In this function the goal status is saved. When the robot reaches the target
*a message will notifies the user that the goal is reached, theen the robot is 
*ready for another target.
**/
void GoalStatus(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& msg){

	// set the goalID variable with the value of the actual goal id
	goalID = msg -> status.goal_id.id;
	
	// check the presence of a goal
	if(goalStatus){
	
		// check the distance of both coordinates to see if the robot is near the goal
		if(abs(msg -> feedback.base_position.pose.position.x - xG) <= error && abs(msg -> feedback.base_position.pose.position.y - yG) <= error)
		{
			system("clear");
			
			// print
			std::cout << "Goal reached successfully!\n";
			
			// cancel goal
			CancelGoal();
		}
		
	}
}


/**
*\brief Description of CurrentGoal() function:
*
*\param m it's a message published on /move_base/goal topic.
*
*In this function the goal position is saved to inform the user
*about the next target of the robot.
**/
void CurrentGoal(const move_base_msgs::MoveBaseActionGoal::ConstPtr& m)
{
	// get x coordinate of the current goal
	xG = m -> goal.target_pose.pose.position.x;
	// get y coordinate of the current goal
	yG = m -> goal.target_pose.pose.position.y;
	
	cout<<"Goal set, the robot is moving to ["<< xG <<", " << yG <<"]"<<endl;
}



int main(int argc, char **argv){
	
	//initialize the node, setup the NodeHandle for handling the communication with the ROS system
	ros::init(argc, argv, "reach_target_node");
	ros::NodeHandle nh;
	
	// publisher to send message for the goal of the robot
	pubGoal = nh.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 1);
	
	// publisher to send message for canceling the goal
	pubCancel = nh.advertise<actionlib_msgs::GoalID>("/move_base/cancel",1);
	
	// subscribe to the topic feedback to have the status always available and updated
	ros::Subscriber sub = nh.subscribe("/move_base/feedback", 1, GoalStatus);
	
	// subscribe to the topic goal to have the current status always available and updated
	ros::Subscriber subG = nh.subscribe("/move_base/goal", 1, CurrentGoal);
	
	//multi-threading
	ros::AsyncSpinner spinner(4);
	
	spinner.start();
	
		Decision();//enter in the menù	
	
	spinner.stop();
	
	
	return 0;
}


