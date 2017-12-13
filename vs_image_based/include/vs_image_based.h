#ifndef VISUAL_SERVOING_H
#define VISUAL_SERVOING_H
/****************************************
 *     H E A D E R    F I L E S         *
 ****************************************/
#include <pid.h>
//ROS Header
#include <ros/ros.h>
#include <ros/console.h>

//tf Header
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>

// kinova_driver Header
#include "kinova_driver/kinova_arm.h" //ACHTUNG!! Originaldateien in kinova-ros wurden geändert!
#include "kinova_driver/kinova_ros_types.h"
#include "kinova/KinovaTypes.h"
#include "kinova_driver/kinova_comm.h" //ACHTUNG!! Originaldateien in kinova-ros wurden geändert!
#include "kinova_driver/kinova_api.h"

// msgs Header
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Pose.h>
#include <kinova_msgs/Stop.h>
#include <kinova_msgs/Start.h>
#include <kinova_msgs/HomeArm.h>
#include <kinova_msgs/JointVelocity.h>
#include <kinova_msgs/PoseVelocity.h>
#include <kinova_msgs/JointTorque.h>
#include <kinova_msgs/FingerPosition.h>
#include <kinova_msgs/JointAngles.h>
#include <kinova_msgs/KinovaPose.h>
#include <kinova_msgs/SetForceControlParams.h>
#include <kinova_msgs/SetEndEffectorOffset.h>
#include <kinova_msgs/SetNullSpaceModeState.h>
#include <kinova_msgs/SetTorqueControlMode.h>
#include <kinova_msgs/SetTorqueControlParameters.h>
#include <kinova_msgs/ClearTrajectories.h>
#include <kinova_msgs/AddPoseToCartesianTrajectory.h>
#include <kinova_msgs/ZeroTorques.h>
#include <kinova_msgs/RunCOMParametersEstimation.h>
#include <kinova_msgs/CartesianForce.h>
#include <kinova_msgs/SetFingersPositionAction.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include "kinova_msgs/Switch2CartControl.h"

//andere Header
#include <vector>
#include <cmath>
#include <fstream>
#include <sstream>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <kinova_msgs/SetFingersPositionAction.h>


using namespace std;

class VisualServoing{

private:
struct POSE{

	char name[30];
	double x,y,z,rot_x,rot_y,rot_z;

}robo_pose,object_pose,e,ausgang,reference_tracking,reference_grasp,u;

struct dh{

	double alpha,a,d,theta;

}i_1,i_2,i_3,i_4,i_5,i_6,i_7,i_8;

struct plane{
	double a,b,c,d;
	double x_min, x_max;
	double y_min, y_max;
	double z_min, z_max;
}planes[9];

tf::Vector3 VEKTOR_cam_data;
tf::Vector3 VEKTOR_erg;
tf::Vector3 robo_position_old;
tf::Vector3 robo_position_new;

tf::Matrix3x3 ROT_gripper2cam, ROT_root2gripper;
std::vector<double> reference_values_tracking, reference_values_tcp;

bool camera_data_received = false;
bool robot_data_received = false;
bool robot_angles_received = false;
bool object_visible = false;
bool object_moved = true;
bool robot_in_collision = false;

double frequency = 8;
double robot_basis_angle = 0; //35.8
double camera_angle = -30.0;
double control_precision = 0.003;
double control_angle_precision = 1.0;
double min_speed = 0.025;
double min_angle_speed = 0.2;
double object_grasp_distance = 0;

int joint1 = 0.0;
int joint2 = 0.0;
int joint3 = 0.0;
int joint4 = 0.0;
int joint5 = 0.0;
int joint6 = 0.0;
int connect_xyz = 1.5;
int counter1 = 0;
int counter2 = 0;

int timer1_value = 3000;
int timer2_value = 2000;

int error1_value = 20000;
int error2_value = 10000;
int error3_value = 10000;

int enable_check_collision = 0;

/**************************************************************************
 *                         O B J E K T E                                  *
 **************************************************************************/

std_msgs::String str_msg;
kinova_msgs::PoseVelocity pose_vel_msg;
kinova_msgs::Start start_srv;
kinova_msgs::SetFingersPositionGoal goal;
kinova_msgs::Switch2CartControl switch2cart;
PID pd_tracking_x, pd_tracking_y, pd_tracking_z, pd_approach_x, pd_approach_y, pd_approach_z;
PID pd_tracking_rot_x, pd_tracking_rot_y, pd_tracking_rot_z, pd_rot_z;

/**************************************************************************
 *                      F U N K T I O N E N                               *
 **************************************************************************/
public:
int start_VS(string type);
void init_VisualServoing(string type);
int check_collision();
void warte();
double b2w(double bogen);
double w2b(double winkel);
void ausgabe_pose(const POSE &p);
void robo_pose_data(const geometry_msgs::PoseStamped::ConstPtr &daten);
void robo_angles(const kinova_msgs::JointAngles::ConstPtr &daten);
void get_camera_data(const geometry_msgs::Pose::ConstPtr &daten);
double rnd(const double &wert, const int &stellen);
void reset_values();
double grasp_distance(const double &distance);
double calc_control_speed(double u, double e, double speed, double precision);
tf::Matrix3x3 calc_rotation(double x, double y, double z);
Eigen::Matrix4d create_4d_Matrix(double alpha, double a, double d, double theta);
void move_fingers(actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction> &finger_client, int distance);
void set_kollision(int value);
void set_objekt_quader(double width, double length, double height);
void set_objekt_cylinder(double radius, double height);
void set_tolerances(double linear, double rotation);
};
#endif
