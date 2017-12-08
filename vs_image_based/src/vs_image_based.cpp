/****************************************
 *     H E A D E R    F I L E S         *
 ****************************************/
#include <vs_image_based.h>
#include <pid.h>
//#include <actionlib/client/simple_action_client.h>
//#include <actionlib/client/terminal_state.h>
using namespace std;

/**************************************************************************
 *                      P R O G R A M M S T A R T                         *
 **************************************************************************/

int VisualServoing::start_VS(string type){

    ros::NodeHandle node;

    /*Initialisierung der Variablen und Parametern*/
    init_VisualServoing(type);

    double Kp_linear,Ki_linear,Kd_linear,Kp_rotation,Ki_rotation,Kd_rotation;
    double limit_linear, limit_rotation, Ta;

    /*  Reglerparameter festlegen */
    Kp_linear = 3.5;
    Ki_linear = 0.0;
    Kd_linear = 0.6;
    Kp_rotation = 0.03;
    Ki_rotation = 0.0;
    Kd_rotation = 0.007;
    limit_linear = 0.3;
    limit_rotation = 10.0;
    Ta = 1.0/frequency;

    //Erstellung der verschiedenen Regler
    PID pd_linear(Kp_linear, Ki_linear, Kd_linear, -limit_linear, limit_linear, Ta);
    PID pd_rotation(Kp_rotation, Ki_rotation, Kd_rotation, -limit_rotation, limit_rotation, Ta);

    pd_tracking_x = pd_tracking_y = pd_tracking_z = pd_approach_x = pd_approach_y = pd_approach_z = pd_linear;
    pd_tracking_rot_x = pd_tracking_rot_y = pd_tracking_rot_z = pd_rot_z = pd_rotation;

    /*Erstellung von Clients und abonnieren von Topics*/
    ros::ServiceClient start_client = node.serviceClient<kinova_msgs::Start>("/m1n6s200_driver/in/start");
    ros::ServiceClient stop_client = node.serviceClient<kinova_msgs::Stop>("/m1n6s200_driver/in/stop");
    ros::ServiceClient cart_client = node.serviceClient<kinova_msgs::Switch2CartControl>("/m1n6s200_driver/in/switch_to_cart_control");
    ros::ServiceClient move_home_client = node.serviceClient<kinova_msgs::HomeArm>("/m1n6s200_driver/in/home_arm");
    ros::Subscriber sub_robo_angles = node.subscribe("/m1n6s200_driver/out/joint_angles", 1, &VisualServoing::robo_angles, this);
    ros::Subscriber sub_robo_pose = node.subscribe("/m1n6s200_driver/out/tool_pose", 1, &VisualServoing::robo_pose_data, this);
    ros::Subscriber sub_object_pose = node.subscribe("vs_cam_data_topic", 1, &VisualServoing::get_camera_data, this);
    ros::Publisher pose_vel_pub = node.advertise<kinova_msgs::PoseVelocity>("/m1n6s200_driver/in/cartesian_velocity", 1);
    ros::Publisher id_pub = node.advertise<std_msgs::String>("send_object_id", 1);

    //Client für die Fingerbewegung
    actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction> finger_client("/m1n6s200_driver/fingers_action/finger_positions",true);
    finger_client.waitForServer();

    start_client.call(start_srv);
    ros::WallDuration(1.0).sleep();

    //Greifer vollständig öffnen
    move_fingers(finger_client, 0);

    //Objekt_ID zum Node "receive_camera_data" gesendet.
    str_msg.data = type;
    id_pub.publish(str_msg);

    cart_client.call(switch2cart);
    ros::WallDuration(0.5).sleep();

    cout << "Starte Regelung..."<< endl;

    /**************************************************************************
     *                      H A U P T R E G E L U N G                         *
     **************************************************************************/
    ros::Rate rate(100);
    while(ros::ok() && object_moved){
        counter2++;

        //Regelschleife nur dann durchlaufen, wenn die Daten angekommen sind
        if(camera_data_received){
    	    //Berechnung der Abweichung
            e.x = 0.0 - object_pose.x;
            e.y = 0.0 - object_pose.y;
            if(abs(e.x) < 0.04 && abs(e.y) < 0.04){
                if(object_pose.z <= 0.2)e.z = 0.0 - object_pose.z + connect_xyz*abs(e.x) + connect_xyz*abs(e.y);
                else e.z = 0.0 - object_pose.z;
            }else{e.z = 0.0;}

            e.rot_x = 0.0 - object_pose.rot_x;
            e.rot_y = 0.0 - object_pose.rot_y;
            e.rot_z = 0.0 - object_pose.rot_z;

            //Berechnung der Abweichung im Basis-KS
            VEKTOR_cam_data.setValue(e.x,e.y,e.z);

            ROT_gripper2cam = calc_rotation(camera_angle, 0, 180);
            ROT_root2gripper = calc_rotation(robo_pose.rot_x, robo_pose.rot_y, robo_pose.rot_z);

            VEKTOR_erg = ROT_root2gripper * ROT_gripper2cam * VEKTOR_cam_data;

            e.x = rnd(-VEKTOR_erg.getX(),4);
            e.y = rnd(-VEKTOR_erg.getY(),4);
            e.z = rnd(-VEKTOR_erg.getZ(),4);

            u.x = pd_tracking_x.calc(e.x);
            u.y = pd_tracking_y.calc(e.y);
            u.z = pd_tracking_z.calc(e.z);
            u.rot_x = pd_tracking_rot_x.calc(e.rot_x);
            u.rot_y = pd_tracking_rot_y.calc(e.rot_y);
            u.rot_z = pd_tracking_rot_z.calc(e.rot_z);
        }
        //Berechnung der Ansteuerung für PD-Regler in einem Toleranzbereich
    	if(Ki_linear == 0){
    	pose_vel_msg.twist_linear_x = u.x = calc_control_speed(u.x, e.x, min_speed,control_precision);
    	pose_vel_msg.twist_linear_y = u.y = calc_control_speed(u.y, e.y, min_speed,control_precision);
    	pose_vel_msg.twist_linear_z = u.z = calc_control_speed(u.z, e.z, min_speed,control_precision);
    	}else{
    	pose_vel_msg.twist_linear_x = u.x;
    	pose_vel_msg.twist_linear_y = u.y;
    	pose_vel_msg.twist_linear_z = u.z;
    	}

    	/*	Ansteuerung für die Orientierung: Reihehnfolge beachten
    	 	Orientierung wird nur dann geregelt, wenn die Abweichungen in x und y Richtung
    	 	kleiner als 0.04m sind.
    	 	Zuerst wird z-Orientierung geregelt. Wenn die Abweichung kleiner als 10° beträgt,
    	 	werden auch x- und y-Orientierungen geregelt.
    	 	  */
    	if(abs(e.x) < 0.04 && abs(e.y) < 0.04){

    	    pose_vel_msg.twist_angular_z = u.rot_z = calc_control_speed(u.rot_z, e.rot_z,min_angle_speed,control_angle_precision);

    	if(abs(e.rot_z)  < 10){

            pose_vel_msg.twist_angular_x = u.rot_x = calc_control_speed(u.rot_x, e.rot_x,min_angle_speed,control_angle_precision);
            pose_vel_msg.twist_angular_y = u.rot_y = calc_control_speed(u.rot_y, e.rot_y,min_angle_speed,control_angle_precision);

        }else{
            pose_vel_msg.twist_angular_x = u.rot_x = 0.0;
            pose_vel_msg.twist_angular_y = u.rot_y = 0.0;
        }

       	}else{
       		pose_vel_msg.twist_angular_x = u.rot_x = 0.0;
       		pose_vel_msg.twist_angular_y = u.rot_y = 0.0;
       		pose_vel_msg.twist_angular_z = u.rot_z = 0.0;
    	}
    	//Wenn objekt nicht sichbar, Bewegungen anhalten
    	if(!object_visible){
    	    pose_vel_msg.twist_linear_x = u.x = e.x = 0.0;
    	    pose_vel_msg.twist_linear_y = u.y = e.y = 0.0;
    	    pose_vel_msg.twist_linear_z = u.z = e.z = 0.0;
    	    pose_vel_msg.twist_angular_x = u.rot_x = e.rot_x = 0.0;
    	    pose_vel_msg.twist_angular_y = u.rot_y = e.rot_y = 0.0;
    	    pose_vel_msg.twist_angular_z = u.rot_z = e.rot_z = 0.0;
    	}

    	//Kolisionserkennung und -auswertung

    	int erg = check_collision();

    	switch(erg){

    	case 1: {

    				pose_vel_msg.twist_linear_x = 0.05;
    	    		pose_vel_msg.twist_linear_y = 0.0;
    	    		pose_vel_msg.twist_linear_z = 0.0;
    	    		pose_vel_msg.twist_angular_x = 0.0;
    	    		pose_vel_msg.twist_angular_y = 0.0;
    	    		pose_vel_msg.twist_angular_z = 0.0;
    	    		break;
    	}
    	case 2: {

    				pose_vel_msg.twist_linear_x = 0.0;
    				pose_vel_msg.twist_linear_y = -0.05;
    				pose_vel_msg.twist_linear_z = 0.0;
    				pose_vel_msg.twist_angular_x = 0.0;
    				pose_vel_msg.twist_angular_y = 0.0;
    				pose_vel_msg.twist_angular_z = 0.0;
    				break;
    	}

		case 3: {

					pose_vel_msg.twist_linear_x = 0.0;
					pose_vel_msg.twist_linear_y = 0.0;
					pose_vel_msg.twist_linear_z = 0.05;
					pose_vel_msg.twist_angular_x = 0.0;
					pose_vel_msg.twist_angular_y = 0.0;
					pose_vel_msg.twist_angular_z = 0.0;
					break;
		}
		case 4: {

					pose_vel_msg.twist_linear_x = 0.05;
					pose_vel_msg.twist_linear_y = 0.0;
					pose_vel_msg.twist_linear_z = 0.0;
					pose_vel_msg.twist_angular_x = 0.0;
					pose_vel_msg.twist_angular_y = 0.0;
					pose_vel_msg.twist_angular_z = 0.0;
					break;
		}

		case 5: {

					pose_vel_msg.twist_linear_x = 0.0;
					pose_vel_msg.twist_linear_y = -0.05;
					pose_vel_msg.twist_linear_z = 0.0;
					pose_vel_msg.twist_angular_x = 0.0;
					pose_vel_msg.twist_angular_y = 0.0;
					pose_vel_msg.twist_angular_z = 0.0;
					break;
		}
		case 6: {
					//cout << "6\n";
					pose_vel_msg.twist_linear_x = 0.0;
					pose_vel_msg.twist_linear_y = 0.0;
					pose_vel_msg.twist_linear_z = 0.05;
					pose_vel_msg.twist_angular_x = 0.0;
					pose_vel_msg.twist_angular_y = 0.0;
					pose_vel_msg.twist_angular_z = 0.0;
					break;
		}
		case 7: {

		            pose_vel_msg.twist_linear_x = 0.0;
		            pose_vel_msg.twist_linear_y = -0.05;
		            pose_vel_msg.twist_linear_z = 0.0;
		            pose_vel_msg.twist_angular_x = 0.0;
		            pose_vel_msg.twist_angular_y = 0.0;
		            pose_vel_msg.twist_angular_z = 0.0;
		            break;
		        }
		case 8: {

		            pose_vel_msg.twist_linear_x = 0.05;
		            pose_vel_msg.twist_linear_y = 0.0;
		            pose_vel_msg.twist_linear_z = 0.0;
		            pose_vel_msg.twist_angular_x = 0.0;
		            pose_vel_msg.twist_angular_y = 0.0;
		            pose_vel_msg.twist_angular_z = 0.0;
		            break;
		        }
		case 9: {

		            pose_vel_msg.twist_linear_x = 0.0;
		            pose_vel_msg.twist_linear_y = 0.0;
		            pose_vel_msg.twist_linear_z = 0.05;
		            pose_vel_msg.twist_angular_x = 0.0;
		            pose_vel_msg.twist_angular_y = 0.0;
		            pose_vel_msg.twist_angular_z = 0.0;
		            break;
		        }

		case 0: {
		            break;
		        }
    	}

    	//Ansteuerwerte an den Roboter senden
    	pose_vel_pub.publish(pose_vel_msg);

    	if(camera_data_received && erg == 0){

    		printf("e.x = %7.4f   e.y = %7.4f   e.z = %7.4f  ||  e.rox_x = %4.1f   e._rot_y = %4.1f   e.rot_z = %4.1f\n",e.x,e.y,e.z,e.rot_x,e.rot_y,e.rot_z);
    		printf("u.x = %7.4f   u.y = %7.4f   u.z = %7.4f  ||  u.rox_x = %4.1f   u._rot_y = %4.1f   u.rot_z = %4.1f\n\n",u.x,u.y,u.z,u.rot_x,u.rot_y,u.rot_z);

    	}

    	/*Timer 1 starten, wenn keine lineare Ansteuerung vorliegt.
    	 * Die Wartezeit kann in dem Launch-File geändert werden.
    	 * */
    	if(pose_vel_msg.twist_linear_x == 0.0 && pose_vel_msg.twist_linear_y == 0.0 && pose_vel_msg.twist_linear_z == 0.0 && object_visible &&
    	        pose_vel_msg.twist_angular_x == 0.0 && pose_vel_msg.twist_angular_y == 0.0 && pose_vel_msg.twist_angular_z == 0.0){
    	    counter1++;
    	}else{
    		counter1 = 0;
    	}

    	if(counter1 >= timer1_value){
    		object_moved = false;
    	}

    	//Wenn nach error1_value-Sekunden nicht ausgeregelt -> Verlassen mit Fehlercode 2
    	if(counter2 >= error1_value) return 2;

    	camera_data_received = false;
    	ros::spinOnce();
    	rate.sleep();
    }
    reset_values();
    cout << "Hauptregelung wurde verlassen..."<<endl;


    bool timer4 = true;

    double robo_position_x = 0;
    double robo_position_y = 0;
    double robo_position_z = 0;
    robo_position_old.setValue(robo_pose.x, robo_pose.y, robo_pose.z);

    cout << "Fahre zum Objekt..."<<endl;
    ros::Rate rate3(100);
    while(ros::ok() && timer4){
        counter2++;

        robo_position_new.setValue(robo_pose.x, robo_pose.y, robo_pose.z);
        ROT_root2gripper = calc_rotation(robo_pose.rot_x, robo_pose.rot_y, robo_pose.rot_z);

        robo_position_x = rnd((ROT_root2gripper.inverse()*(robo_position_new-robo_position_old)).getX(),4);
        robo_position_y = rnd((ROT_root2gripper.inverse()*(robo_position_new-robo_position_old)).getY(),4);
        robo_position_z = rnd((ROT_root2gripper.inverse()*(robo_position_new-robo_position_old)).getZ(),4);

        //Berechnung der Abweichung
        if(robot_data_received){
            e.x = reference_grasp.x - robo_position_x;
            e.y = reference_grasp.y - robo_position_y;
            e.z = reference_grasp.z - robo_position_z;

            VEKTOR_cam_data.setValue(e.x, e.y, e.z);


            VEKTOR_erg = ROT_root2gripper * VEKTOR_cam_data;
            e.x = rnd(VEKTOR_erg.getX(),4);
            e.y = rnd(VEKTOR_erg.getY(),4);
            e.z = rnd(VEKTOR_erg.getZ(),4);


            u.x = pd_approach_x.calc(e.x);
            u.y = pd_approach_y.calc(e.y);
            u.z = pd_approach_z.calc(e.z);

            robot_data_received = false;
        }
        pose_vel_msg.twist_linear_x = calc_control_speed(u.x, e.x,min_speed,control_precision);
        pose_vel_msg.twist_linear_y = calc_control_speed(u.y, e.y,min_speed,control_precision);
        pose_vel_msg.twist_linear_z = calc_control_speed(u.z, e.z,min_speed,control_precision);


        pose_vel_pub.publish(pose_vel_msg);

        if(pose_vel_msg.twist_linear_x == 0 && pose_vel_msg.twist_linear_y == 0 && pose_vel_msg.twist_linear_z == 0){

                counter1++;

        }else counter1 = 0;

        if(counter1 >= timer2_value) {
            timer4 = false;
        }

        //Wenn nach error2_value-Sekunden nicht ausgeregelt -> Verlassen mit Fehlercode 3
        if(counter2 >= error2_value) return 3;

        ros::spinOnce();
        rate3.sleep();

    }

    reset_values();
    cout << "Greife in 100ms..."<<endl;

    ros::WallDuration(0.1).sleep();

    //Action-Funktion zum schließen des Greifers aufrufen
    move_fingers(finger_client, grasp_distance(object_grasp_distance));

    ros::WallDuration(0.5).sleep();

    robo_position_z = robo_pose.z;
    reset_values();
    ros::Rate rate4(100);
    while(robo_pose.z <= robo_position_z + cos(w2b(35.8))*0.05){
        counter2++;
        pose_vel_msg.twist_linear_x = 0.0;
        pose_vel_msg.twist_linear_y = 0.2;
        pose_vel_msg.twist_linear_z = 0.2;

        pose_vel_pub.publish(pose_vel_msg);

        //Wenn nach error3_value-Sekunden nicht ausgeregelt -> Verlassen mit Fehlercode 4
        if(counter2 >= error3_value) return 4;
            rate4.sleep();
    }


    ros::WallDuration(3.0).sleep();

    move_fingers(finger_client, 0);
    cart_client.call(switch2cart);
    ros::WallDuration(0.5).sleep();
    ros::WallDuration(1.0).sleep();
return 1;
}

/*  Initialisierung von Variablen und Objekten.
 *  Werte aus dem Parameterserver laden.
 *  Kollisionsebenen initialisieren.
 * */
void VisualServoing::init_VisualServoing(string type){

	ros::NodeHandle pn("~");
	pn.getParam("frequency", frequency);
	pn.getParam("robot_basis_angle", robot_basis_angle);
	pn.getParam("camera_angle", camera_angle);
	pn.getParam("control_precision", control_precision);
	pn.getParam("control_angle_precision", control_angle_precision);
	pn.getParam("connect_xyz", connect_xyz);
	pn.getParam("object_length", object_length);
	pn.getParam("object_width", object_width);
	pn.getParam("object_height", object_height);

	pn.getParam("timer1_value", timer1_value);
	pn.getParam("timer2_value", timer2_value);

    pn.getParam("error1_value", error1_value);
    pn.getParam("error2_value", error2_value);
    pn.getParam("error3_value", error3_value);

	timer1_value /= 10;
	timer2_value /= 10;

	error1_value /= 10;
	error2_value /= 10;
	error3_value /= 10;

	pn.getParam("enable_check_collision", enable_check_collision);

	reference_values_tracking.resize(6);
	reference_values_tcp.resize(6);

	pn.getParam(type + "/grasp_distance", object_grasp_distance);
	pn.getParam(type + "/reference_values", reference_values_tracking);
	pn.getParam(type + "/move_tcp", reference_values_tcp);
	reference_tracking.x = reference_values_tracking.data()[0];
	reference_tracking.y = reference_values_tracking.data()[1];
	reference_tracking.z = abs(reference_values_tracking.data()[2]);
	reference_tracking.rot_x = -reference_values_tracking.data()[3];
	reference_tracking.rot_y = reference_values_tracking.data()[4];
	reference_tracking.rot_z = -reference_values_tracking.data()[5];

	reference_grasp.x = reference_values_tcp.data()[0];
	reference_grasp.y = reference_values_tcp.data()[1];
	reference_grasp.z = reference_values_tcp.data()[2];

	//Konstante DH Parameter
	i_1.alpha = 90;		i_1.a = 0;		i_1.d = 0.2755;
	i_2.alpha = 180;	i_2.a = 0.290;  i_2.d = 0.0;
	i_3.alpha = 90;		i_3.a = 0;		i_3.d = -0.007;
	i_4.alpha = 60;		i_4.a = 0;		i_4.d = -0.1660816549;
	i_5.alpha = 60;		i_5.a = 0;		i_5.d = -0.08556330989;
	i_6.alpha = 180;	i_6.a = 0;		i_6.d = -0.2027816549;
	i_7.alpha = 0;		i_7.a = 0.1;    i_7.d = 0.02;
	i_8.alpha = 180;	i_8.a = 0;		i_8.d = -0.2027816549;
//Ebene 1
	planes[0].a = 1;        planes[0].x_min = 0.11;                 planes[0].x_max = 0.11;
	planes[0].b = 0;        planes[0].y_min = -0.4;                 planes[0].y_max = 0.37;
	planes[0].c = 0;        planes[0].z_min = -0.12;                planes[0].z_max = 0.12;
	planes[0].d = 0.11;
//Ebene 2
	planes[1].a = 0;        planes[1].x_min = -0.54;                planes[1].x_max = 0.11;
	planes[1].b = -1;       planes[1].y_min = -0.4;                 planes[1].y_max = -0.4;
	planes[1].c = 0;        planes[1].z_min = -0.12;                planes[1].z_max = 0.12;
	planes[1].d = 0.4;
//Ebene 3
	planes[2].a = 0;        planes[2].x_min = -0.01;                planes[2].x_max = 0.11;
	planes[2].b = 0;        planes[2].y_min = -0.4;                 planes[2].y_max = 0.37;
	planes[2].c = 1;        planes[2].z_min = 0.12;                 planes[2].z_max = 0.12;
	planes[2].d = 0.12;
//Ebene 4
	planes[3].a = -1;       planes[3].x_min = -0.01;                planes[3].x_max = -0.01;
	planes[3].b = 0;        planes[3].y_min = -0.14;                planes[3].y_max = 0.01;
	planes[3].c = 0;        planes[3].z_min = 0.12;                 planes[3].z_max = 0.23;
	planes[3].d = 0.01;
//Ebene 5
	planes[4].a = 0;        planes[4].x_min = -0.54;                planes[4].x_max = -0.01;
	planes[4].b = -1;       planes[4].y_min = -0.14;                planes[4].y_max = -0.14;
	planes[4].c = 0;        planes[4].z_min = 0.12;                 planes[4].z_max = 0.23;
	planes[4].d = 0.14;
//Ebene 6
	planes[5].a = 0;        planes[5].x_min = -0.54;                planes[5].x_max = -0.01;
	planes[5].b = 0;        planes[5].y_min = -0.14;                planes[5].y_max = 0.01;
	planes[5].c = 1;        planes[5].z_min = 0.23;                 planes[5].z_max = 0.23;
	planes[5].d = 0.23;
//Ebene 7
	planes[6].a = 0;        planes[6].x_min = -0.54;                planes[6].x_max = -0.02;
	planes[6].b = 1;        planes[6].y_min = 0.01;                 planes[6].y_max = 0.01;
	planes[6].c = 0;        planes[6].z_min = 0.23;                 planes[6].z_max = 0.8;
	planes[6].d = 0.01;
//Ebene 8
	planes[7].a = -1;       planes[7].x_min = -0.01;                planes[7].x_max = -0.01;
	planes[7].b = 0;        planes[7].y_min = 0.01;                 planes[7].y_max = 0.37;
	planes[7].c = 0;        planes[7].z_min = 0.12;                 planes[7].z_max = 0.8;
	planes[7].d = 0.01;
//Ebene 9
	planes[8].a = 0;        planes[8].x_min = -0.8;                 planes[8].x_max = 0.8;
	planes[8].b = 0;        planes[8].y_min = -0.8;                 planes[8].y_max = 0.8;
	planes[8].c = -1;       planes[8].z_min = -0.12;                planes[8].z_max = -0.12;
	planes[8].d = 0.12;


	cout << "Fertig!!! Jetzt starte die Hauptfunktion():" << endl;
}

/*  Kollisionserkennung
 *  Überprüfung und Auswertung von Schnittpunkten der Ebenen und Geraden
 *  Die Ebenen sind im KS der mobilen Plattform vorgegeben und werden mit der
 *  Funktion "init_VisualServoing()" initialisiert.
 *
 *  Die Geraden werden zwischen den einzelnen benachbarten Gelenkpunkten des Armes gebildet.
 *
 *  Funktion liefert als Rückgabewert eine Integer-Zahl von 0 bis 9
 *  0 steht für keine Kollision
 *  1 bis 9 -> Kollision mit der jeweiligen Ebene
 *  */
int VisualServoing::check_collision(){

	if(!enable_check_collision) return 0;

	Eigen::Matrix4d m_erg,m_test;
	Eigen::Matrix4d matrices[8];
	Eigen::Matrix3d rotation_x;
	Eigen::Matrix4d f1,f2,tcp_backup;
	m_erg.setIdentity();
	Eigen::Vector3d point1;
	Eigen::Vector3d point2;
	Eigen::Vector3d intersection_point;
	Eigen::Vector3d position_vector;
	Eigen::Vector3d direction_vector;

	f1.setIdentity();
	f2.setIdentity();
	f1(0,3) = 0.085;
	f1(1,3) = 0.0;
	f1(2,3) = 0.02;

	f2(0,3) = -0.085;
	f2(1,3) = 0.0;
	f2(2,3) = 0.02;

	double t = 100;


	rotation_x(0,0) = 1; 	rotation_x(0,1) = 0;								rotation_x(0,2) = 0;
	rotation_x(1,0) = 0;	rotation_x(1,1) = cos(w2b(robot_basis_angle));		rotation_x(1,2) = -sin(w2b(robot_basis_angle));
	rotation_x(2,0) = 0;	rotation_x(2,1) = sin(w2b(robot_basis_angle));		rotation_x(2,2) = cos(w2b(robot_basis_angle));

	i_1.theta = -(joint1%360);
	i_2.theta = (joint2%360)-90;
	i_3.theta = (joint3%360)+90;
	i_4.theta = (joint4)%360;
	i_5.theta = (joint5%360)-180;
	i_6.theta = (joint6%360) + 90;


	matrices[0] = create_4d_Matrix(i_1.alpha, i_1.a, i_1.d, i_1.theta);
	matrices[1] = create_4d_Matrix(i_2.alpha, i_2.a, i_2.d, i_2.theta);
	matrices[2] = create_4d_Matrix(i_3.alpha, i_3.a, i_3.d, i_3.theta);
	matrices[3] = create_4d_Matrix(i_4.alpha, i_4.a, i_4.d, i_4.theta);
	matrices[4] = create_4d_Matrix(i_5.alpha, i_5.a, i_5.d, i_5.theta);
	matrices[5] = create_4d_Matrix(i_6.alpha, i_6.a, i_6.d, i_6.theta);
	matrices[6] = f1;
	matrices[7] = f2;


	m_erg *= matrices[0];

	point1(0) = m_erg(0,3);
	point1(1) = m_erg(1,3);
	point1(2) = m_erg(2,3);

	point1 = rotation_x * point1;

	for(int i = 1; i < 8; i++){

	    if(i == 7) m_erg = tcp_backup;
	    m_erg *= matrices[i];
		if(i == 5) tcp_backup = m_erg;


		point2(0) = m_erg(0,3);
		point2(1) = m_erg(1,3);
		point2(2) = m_erg(2,3);

		point2 = rotation_x * point2;

		direction_vector = point2 - point1;
		//cout << "-----------------------------------------------------------------------"<<endl;
		//printf("i:%d   P1: %f  %f  %f   P2: %f  %f  %f\n",i,point1(0),point1(1),point1(2),point2(0),point2(1),point2(2));
		//cout << "-----------------------------------------------------------------------"<<endl;

		for(int j = 0; j < 9; j++){

		    if((planes[j].a * direction_vector(0) + planes[j].b * direction_vector(1) + planes[j].c * direction_vector(2)) != 0){

			    t = (planes[j].d - planes[j].a * point1(0) - planes[j].b * point1(1) - planes[j].c * point1(2))/
				(planes[j].a * direction_vector(0) + planes[j].b * direction_vector(1) + planes[j].c * direction_vector(2));

			    intersection_point = point1 + t * direction_vector;
			    intersection_point(0) = rnd(intersection_point(0),4);
			    intersection_point(1) = rnd(intersection_point(1),4);
			    intersection_point(2) = rnd(intersection_point(2),4);


			    //printf("j:%d   t: %f   S: %f  %f  %f\n",j,t,intersection_point(0),intersection_point(1),intersection_point(2));

			    if(t >= 0.0 && t <= 1.0
			        && intersection_point(0) >= planes[j].x_min && intersection_point(0) <= planes[j].x_max
			        && intersection_point(1) >= planes[j].y_min && intersection_point(1) <= planes[j].y_max
			        && intersection_point(2) >= planes[j].z_min && intersection_point(2) <= planes[j].z_max){


				    if(i < 6) printf("Kollision: %d --- %d mit Ebene %d\n", i, i+1,j+1);
				    if(i == 6) printf("Kollision: 6 --- 7 mit Ebene %d\n",j+1);
				    if(i == 7) printf("Kollision: 6 --- 8 mit Ebene %d\n",j+1);
				    return j+1;
			    }
		    }
		}

		if(i != 6)point1 = point2;
	}
	return 0;
}


/*  Hilfsfunktion: unterbricht das laufende Programm
 * */
void VisualServoing::warte(){

	cout << endl << "...PAUSE..." << endl;

	while(1){;}

}


/*  Umrechnung vom Bogenmaß zu Winkel
 * */
double VisualServoing::b2w(double bogen){

	double winkel = (bogen*180)/M_PI;

	return winkel;

}

/*  Umrechnung vom Winkel zum Bogenmaß
 * */
double VisualServoing::w2b(double winkel){

	double bogen = (M_PI*winkel)/180;

	return bogen;

}

/*Hilfsfunktion: gibt die Daten von dem Objekt p aus*/
void VisualServoing::ausgabe_pose(const POSE &p){

	cout << p.name << endl << "x = " << p.x << endl;
	cout << "y = " << p.y << endl;
	cout << "z = " << p.z << endl;
	cout << "rot_x = " << p.rot_x << endl;
	cout << "rot_y = " << p.rot_y << endl;
	cout << "rot_z = " << p.rot_z << endl << endl;

}

/*  Callbackfunktion, die die aktuelle Pose des Roboters aus dem ROS-topic speichert.*/
void VisualServoing::robo_pose_data(const geometry_msgs::PoseStamped::ConstPtr &daten){

	robot_data_received = true;
	strcpy(robo_pose.name,"Roboter Position");

	robo_pose.x = rnd(daten->pose.position.x, 4);
	robo_pose.y = rnd(daten->pose.position.y, 4);
	robo_pose.z = rnd(daten->pose.position.z, 4);

	tf::Quaternion q(daten->pose.orientation.x, daten->pose.orientation.y, daten->pose.orientation.z, daten->pose.orientation.w);
	tf::Matrix3x3 m(q);
	m.getRPY(robo_pose.rot_x, robo_pose.rot_y, robo_pose.rot_z);

	robo_pose.rot_x = rnd(b2w(robo_pose.rot_x),1);
	robo_pose.rot_y = rnd(b2w(robo_pose.rot_y),1);
	robo_pose.rot_z = rnd(b2w(robo_pose.rot_z),1);

}

/*  Callbackfunktion, die die aktuellen Gelenkwinkel des Roboters aus dem ROS-topic speichert.
 *  Diese Daten werden für die Berechnung von variablen Winkeln theta (DH-Parameter) verwendet.
 * */
void VisualServoing::robo_angles(const kinova_msgs::JointAngles::ConstPtr &daten){

robot_angles_received = true;
joint1 = daten->joint1;
joint2 = daten->joint2;
joint3 = daten->joint3;
joint4 = daten->joint4;
joint5 = daten->joint5;
joint6 = daten->joint6;

}

/*  Callbackfunktion, die immer dann aufgerufen wird,
    wenn die Kameradaten im entsprechenden ROS-Topic ankommen.
    Diese Daten werden aus dem tf rausgenommen und in ein Topic weitergeleitet.
    Standardfrequenz mit AR-Tag: 8Hz
    Die Daten aus dem Topic werden global in dem Struct "object_pose" gespeichert.

*/
void VisualServoing::get_camera_data(const geometry_msgs::Pose::ConstPtr &daten){
    camera_data_received = true;
    strcpy(object_pose.name,"AR-Tag Daten");

    tf::Quaternion q(daten->orientation.x, daten->orientation.y, daten->orientation.z, daten->orientation.w);
    tf::Matrix3x3 c2o(q);
    tf::Matrix3x3 m_temp,g2c,rotate,erg;

    double temp_rot_x,temp_rot_y,temp_rot_z;
    c2o.getRPY(temp_rot_x, temp_rot_y, temp_rot_z);

    if(daten->position.x != reference_tracking.x && daten->position.y != reference_tracking.y && daten->position.z != reference_tracking.z &&
            temp_rot_x != reference_tracking.rot_x && temp_rot_y != reference_tracking.rot_y && temp_rot_z != reference_tracking.rot_z){

        object_visible = true;

        object_pose.x = rnd(daten->position.x,4)-reference_tracking.x;
        object_pose.y = rnd(daten->position.y,4)-reference_tracking.y;
        object_pose.z = rnd(daten->position.z,4)-reference_tracking.z;

        m_temp = calc_rotation(180, 0, 180);
        rotate = calc_rotation(reference_tracking.rot_x, reference_tracking.rot_y, reference_tracking.rot_z);
        g2c = calc_rotation(camera_angle, 0, 180);
        erg = m_temp * g2c * c2o * rotate;
        erg.getRPY(object_pose.rot_x, object_pose.rot_y, object_pose.rot_z);
        object_pose.rot_x = rnd(b2w(object_pose.rot_x),1);
        object_pose.rot_y = -rnd(b2w(object_pose.rot_y),1);
        object_pose.rot_z = rnd(b2w(object_pose.rot_z),1);

    }else{

        object_visible = false;
        object_pose.x = reference_tracking.x;
        object_pose.y = reference_tracking.y;
        object_pose.z = reference_tracking.z;
        object_pose.rot_x = reference_tracking.rot_x;
        object_pose.rot_y = reference_tracking.rot_y;
        object_pose.rot_z = reference_tracking.rot_z;

    }

/*

    ausgabe_pose(object_pose);
    /*cout << "robo_pose.x = " << robo_pose.x << endl;
    cout << "robo_pose.y = " << robo_pose.y << endl;
    cout << "robo_pose.z = " << robo_pose.z << endl;
    cout << "robo_pose.rot_x = " << robo_pose.rot_x << endl;
    cout << "robo_pose.rot_y = " << robo_pose.rot_y << endl;
    cout << "robo_pose.rot_z = " << robo_pose.rot_z << endl;


//*/

}

/*  Funktion rundet "mathematisch" den Eingangswert (wert)
 *  mit gegebenen Nachkommastellen (stellen) auf oder ab.
 *  Mathematisches runden: 0.5 als Grenze
 * */
double VisualServoing::rnd(const double &wert, const int &stellen){

	int basis = 1;

	for(int i = 0; i < stellen; i++) basis *= 10;


	if(wert > 0) return (double)((int)(wert * basis + 0.5))/basis;
	else return (double)((int)(wert * basis - 0.5))/basis;

}

/*  Resetet einige Parameter des Regelkreises
 * */
void VisualServoing::reset_values(){

    counter1 = 0;
    counter2 = 0;
	e.x = 0.0;
    e.y = 0.0;
    e.z = 0.0;
    e.rot_x = 0.0;
    e.rot_y = 0.0;
    e.rot_z = 0.0;

    u.x = 0.0;
    u.y = 0.0;
    u.z = 0.0;
    u.rot_x = 0.0;
    u.rot_y = 0.0;
    u.rot_z = 0.0;

    pose_vel_msg.twist_linear_x = 0.0;
    pose_vel_msg.twist_linear_y = 0.0;
    pose_vel_msg.twist_linear_z = 0.0;
    pose_vel_msg.twist_angular_x = 0.0;
    pose_vel_msg.twist_angular_y = 0.0;
    pose_vel_msg.twist_angular_z = 0.0;

}

/*  Berechnung die Greiferstellung für MoveIt!
    Ergebnis ist die Fingerstellung in Grad (0°...6000°)
    0° -> Greifer komplett geöffnet
    6000° -> Greifer komplett geschlossen
*/
double VisualServoing::grasp_distance(const double &distance){

	double result;

	result = (-36.3 * distance + 6750 + 1800);

	if(result > 6000) return 6000;
	else if(result < 0) return 0;

	return result;

}

/*  Berechnung der Ansteuerung für PD Regler,
    falls die Geschwindigkeit für die Bewegung nicht ausreichend ist
    und die Abweichung außerhalb des Toleranzbereiches liegt.
 *  Parameter:
 *  u: ist der Ausgang des PD-Reglers
    e: ist die aktuelle Abweichung
    speed: ist die notwendige Mindestgeschwindigkeit
    precision: ist die Genauigkeit (Toleranzbereich)
    */
double VisualServoing::calc_control_speed(double u, double e, double speed, double precision){

	if(abs(u) >= speed){
		return u;
	}else{
		if(u < speed && u > 0 && abs(e) > precision){
			return speed;
		}else{
			if(u > -speed && u < 0 && abs(e) > precision){
				return -speed;
			}else{
				return 0.0;
			}
		}
	}

}

/*  Berechnet die Rotationsmatrix mit RPY-Winkeln
 *  Parameter:
 *  x = Roll
 *  y = Pitch
 *  z = Yaw
 *  Winkel müssen in Grad angegeben werden.
 *  */
tf::Matrix3x3 VisualServoing::calc_rotation(double x, double y, double z){

	tf::Matrix3x3 ROT_X;
	tf::Matrix3x3 ROT_Y;
	tf::Matrix3x3 ROT_Z;

	ROT_X.setValue(1, 0, 0, 0, cos(w2b(x)), -sin(w2b(x)), 0, sin(w2b(x)), cos(w2b(x)));
	ROT_Y.setValue(cos(w2b(y)), 0, sin(w2b(y)), 0, 1, 0, -sin(w2b(y)), 0, cos(w2b(y)));
	ROT_Z.setValue(cos(w2b(z)), -sin(w2b(z)), 0, sin(w2b(z)), cos(w2b(z)), 0, 0, 0, 1);

	return ROT_Z * ROT_Y * ROT_X;

}

/*  Erstellt eine 4x4 Transformationsmatrix mit DH-Parametern
 *  Winkel alpha und theta spllten in Grad angegeben werden.
 *  Parameter a und d sind in [m]*/
Eigen::Matrix4d VisualServoing::create_4d_Matrix(double alpha, double a, double d, double theta){


	Eigen::Matrix4d m;
	m(0,0) = cos(w2b(theta));	m(0,1) = -sin(w2b(theta))*cos(w2b(alpha));	m(0,2) = sin(w2b(theta))*sin(w2b(alpha));	m(0,3) = a*cos(w2b(theta));
	m(1,0) = sin(w2b(theta));	m(1,1) = cos(w2b(theta))*cos(w2b(alpha));	m(1,2) = -cos(w2b(theta))*sin(w2b(alpha));	m(1,3) = a*sin(w2b(theta));
	m(2,0) = 0;					m(2,1) = sin(w2b(alpha));					m(2,2) = cos(w2b(alpha));					m(2,3) = d;
	m(3,0) = 0;					m(3,1) = 0;									m(3,2) = 0;									m(3,3) = 1;

	return m;
}

/*  Action-Finktion zum schließen des Greifers
 *  Erster Parameter ist der Client-Name (muss vorher definiert sein)
 *  Zweiter Parameter ist der Greifabstand. Dieser ist abhängig von der Breite des Objektes.
 *  Die Breite des Objektes ist so definiert, dass die x-Achse von dem AR-Tag-Bild
 *  entlang der Breite des Objektes verläuft.
 *  Die Breite kann in der Parameter-Datei (visual_servoing/vs_image_based/param/object_parameter.yaml)
 *  eingetragen und geändert werden.
 * */
void VisualServoing::move_fingers(actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction> &finger_client, int distance){

    goal.fingers.finger1 = goal.fingers.finger2 =  distance;
    finger_client.sendGoal(goal);
        bool finished_before_timeout = finger_client.waitForResult(ros::Duration(5.0));
        if(finished_before_timeout){
            actionlib::SimpleClientGoalState state = finger_client.getState();
            ROS_INFO("Action finished: %s",state.toString().c_str());
        }
        else ROS_INFO("Action did not finish before the time out.");

}
