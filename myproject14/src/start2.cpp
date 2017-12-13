
//#include "../../myproject13_no_moveit/include/visual_servoing.h"
#include <vs_msgs/start_vsAction.h>
#include <actionlib/client/simple_action_client.h>
//ROS Header
#include <ros/ros.h>
#include <ros/console.h>

using namespace std;

typedef actionlib::SimpleActionClient<vs_msgs::start_vsAction> Client;

int main(int argc, char **argv){

    ros::init(argc, argv, "visual_servoing2");

    Client client("vs_start_action", true);
    client.waitForServer();
    string eingabe;
    vs_msgs::start_vsGoal goal;
while(ros::ok()){
cout << "text eingeben: ";
cin >> eingabe;
goal.object_type = eingabe;


    client.sendGoal(goal);
    client.waitForResult(ros::Duration(1000.0));

    if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
           printf("Visual Servoing erfolgreich beendet.\n");

    if (client.getState() == actionlib::SimpleClientGoalState::ABORTED){
        printf("%s\n",client.getState().getText().c_str());
    }
}

    return 0;
}
