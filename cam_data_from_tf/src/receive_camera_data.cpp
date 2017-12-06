/*  Dieses Programm bekommt als Parameter AR-Tag-ID und camera_frame_id.
 *  camera_frame_id wird in der Launch-Datei (visual_servoing/vs_image_based/launch/vs_image_based.launch) festgelegt.
 *  Damit werden die Posedaten aus tf geholt und in ein Topic weitergeleitet.
 *  In "vs_image_based.cpp" werden diese Daten weiterverarbeitet.*/

//tf Header
#include <tf/transform_listener.h>
#include <tf/tf.h>
//ROS Header
#include <ros/ros.h>
#include <ros/console.h>
//MSG Files
#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>

using namespace std;

string camera_frame_id, object_id, test;
std::vector<double> reference_values_tracking;
tf::Quaternion q;


struct POSE{

    char name[30];
    double x,y,z,rot_x,rot_y,rot_z;

}reference_tracking;

void id_callback(const std_msgs::String::ConstPtr &daten);


int main(int argc, char** argv){

  ros::init(argc, argv, "receive_camera_data");

  /*    Frequenz der Daten im Topic initialisieren.
   *    Dieser Wert wird mit dem Wert aus dem Parameterserver überschrieben.
   *    Wichtig! Hinweise in "visual_servoing/vs_image_based/launch/vs_image_based.launch" beachten!
   *    */
  double frequency = 8;

  ros::NodeHandle n;
  ros::NodeHandle pn("~");

  ros::Publisher pub_cam_data = n.advertise<geometry_msgs::Pose>("vs_cam_data_topic", 1);
  ros::Subscriber dsegs = n.subscribe("send_object_id", 10, id_callback);

  reference_values_tracking.resize(6);

  reference_tracking.x = 0;
  reference_tracking.y = 0;
  reference_tracking.z = 0;
  reference_tracking.rot_x = 0;
  reference_tracking.rot_y = 0;
  reference_tracking.rot_z = 0;

  q.setRPY(reference_tracking.rot_x, reference_tracking.rot_y, reference_tracking.rot_z);

  pn.getParam("camera_frame_id", camera_frame_id);
  pn.getParam("frequency", frequency);

  tf::TransformListener listener;
  geometry_msgs::Pose cd, cd_old; //camera_data, camera_data_old

  cd.position.x = reference_tracking.x;
  cd.position.y = reference_tracking.y;
  cd.position.z = reference_tracking.z;
  cd.orientation.w = q.getW();
  cd.orientation.x = q.getX();
  cd.orientation.y = q.getY();
  cd.orientation.z = q.getZ();

  ros::Rate rate(frequency);
  while(ros::ok()){

  if(listener.frameExists(object_id)){

      // tf Listener liefert die Transformation zwischen "camera_frame_id" und "object_id"
      tf::StampedTransform transform;
      listener.lookupTransform(camera_frame_id, object_id, ros::Time(0), transform);
      cd.position.x = transform.getOrigin().getX();
      cd.position.y = transform.getOrigin().getY();
      cd.position.z = transform.getOrigin().getZ();
      cd.orientation.w = transform.getRotation().getW();
      cd.orientation.x = transform.getRotation().getX();
      cd.orientation.y = transform.getRotation().getY();
      cd.orientation.z = transform.getRotation().getZ();

  }else{
      cd.position.x = reference_tracking.x;
      cd.position.y = reference_tracking.y;
      cd.position.z = reference_tracking.z;
      cd.orientation.w = q.getW();
      cd.orientation.x = q.getX();
      cd.orientation.y = q.getY();
      cd.orientation.z = q.getZ();
  }

  if(cd.orientation.x != cd_old.orientation.x && cd.orientation.y != cd_old.orientation.y && cd.orientation.z != cd_old.orientation.z && cd.orientation.w != cd_old.orientation.w &&
          cd.position.x != cd_old.position.x && cd.position.y != cd_old.position.y && cd.position.z != cd_old.position.z){

          pub_cam_data.publish(cd);
          cd_old = cd;
      }
  else{

      cd.position.x = reference_tracking.x;
      cd.position.y = reference_tracking.y;
      cd.position.z = reference_tracking.z;
      cd.orientation.w = q.getW();
      cd.orientation.x = q.getX();
      cd.orientation.y = q.getY();
      cd.orientation.z = q.getZ();

      pub_cam_data.publish(cd);
      }
  ros::spinOnce();
  rate.sleep();
  }
  return 0;
}

void id_callback(const std_msgs::String::ConstPtr &daten){

    ros::NodeHandle pn("~");
    string temp_str;

    /*  Erlauben das gleiche AR-Tag mit unterschiedlichen Parametern zu verwenden.
     *  Dazu wird in "object_parameter.yaml" das AR-Tag-ID
     *  mit einem oder mehreren belibigen Zeichen am Ende ergänzt.
     *  In der Parameter-Datei sollten keine IDs mit gleichen Namen sein.
     *  Die ersten 11 Zeichen stehen für ID.
     *  Beispiel:
     *  AR-Tag-ID: ar_marker_4          -> hat eigene Parameter
     *  AR-Tag-ID: ar_marker_4_1        -> hat eigene Parameter
     *  AR-Tag-ID: ar_marker_4hohoho    -> hat eigene Parameter
     *  Alle drei verwenden dieselben Pose-Daten von ar_marker_4.
     *  Erlaubte Zeichen sind:
     *  a-z, A-Z, 0-9, / und _
     * */
    temp_str = daten->data.c_str();
    object_id = temp_str.substr(0, 11);

    pn.getParam(object_id + "/reference_values", reference_values_tracking);
    reference_tracking.x = reference_values_tracking.data()[0];
    reference_tracking.y = reference_values_tracking.data()[1];
    reference_tracking.z = reference_values_tracking.data()[2];
    reference_tracking.rot_x = reference_values_tracking.data()[3];
    reference_tracking.rot_y = reference_values_tracking.data()[4];
    reference_tracking.rot_z = reference_values_tracking.data()[5];
    q.setRPY(reference_tracking.rot_x, reference_tracking.rot_y, reference_tracking.rot_z);
}
