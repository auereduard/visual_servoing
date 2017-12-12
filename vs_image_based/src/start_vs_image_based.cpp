#include "vs_image_based.h"
#include <vs_msgs/start_vsAction.h>
#include <actionlib/server/simple_action_server.h>
//ROS Header
#include <ros/ros.h>
#include <ros/console.h>

typedef actionlib::SimpleActionServer<vs_msgs::start_vsAction> Server;

void execute(const vs_msgs::start_vsGoalConstPtr& goal, Server *as){

    VisualServoing vs;
    vs_msgs::start_vsResult a;

    a.r = vs.start_VS(goal->object_type);
    if(a.r == 1){
    	as->setSucceeded(a);
    }
    else {
        switch(a.r){
        	case 0: as->setAborted(a, "Fehler 0: AR-Tag-ID in object_parameter.yaml nicht gefunden"); break;
            case 2: as->setAborted(a, "Fehler 2"); break;
            case 3: as->setAborted(a, "Fehler 3"); break;
            case 4: as->setAborted(a, "Fehler 4"); break;
            case 5: as->setAborted(a, "Fehler 5"); break;
            default: as->setAborted(a, "unbekannter Fehler"); break;
        }
    }
}
void print_start();

int main(int argc, char** argv){

  ros::init(argc, argv, "vs_image_based");

  ros::NodeHandle n;

  Server server(n, "mein_test_action", boost::bind(&execute, _1, &server), false);

  ros::WallDuration(10.0).sleep();
  print_start();

  server.start();

  ros::spin();

  return 0;
}

void print_start(){

    for(int j = 0; j < 3; j++){
        for(int i = 0; i < 33; i++) {
            cout << "*";
        }
        cout << endl;
    }

    cout <<endl<< "     VISUAL SERVOING STARTED     "<<endl<<endl;

    for(int j = 0; j < 3; j++){
            for(int i = 0; i < 33; i++) {
                cout << "*";
            }
            cout << endl;
        }
}
