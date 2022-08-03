#include "control.h"

std::shared_ptr<rclcpp::Node> node_publisher_drone[NDRONE],node_subscriber_drone[NDRONE],node_image[NDRONE],node_depth[NDRONE]; /*<Variables that will create all the publishers and subscriber for the drones.*/

rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_drone[NDRONE];
rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_drone[NDRONE];
rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_drone[NDRONE];
rclcpp::Subscription<VehicleOdometry>::SharedPtr odometry_subscriber_drone[NDRONE];
rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr test_image[NDRONE];
rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr test_depth[NDRONE];


std::shared_ptr<rclcpp::Node> node_publisher_rover[NROVER],node_subscriber_rover[NROVER];   /*<Variables that will create all the publishers and subscriber for the rovers.*/

rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_rover[NROVER];
rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_rover[NROVER];
rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_rover[NROVER];
rclcpp::Subscription<VehicleOdometry>::SharedPtr odometry_subscriber_rover[NROVER];

std::shared_ptr<rclcpp::Node> node_ui;  /*<Variable that will create the subscriver for the user_interface to receive all the actions to perform.*/
rclcpp::Subscription<interfaces::msg::Mymsg>::SharedPtr ui_subscriber;

std::shared_ptr<rclcpp::Node> node_return;  /*<Variable that will create the publisher for the user_interface to publish the coordinates to calculate the return.*/
rclcpp::Publisher<interfaces::msg::MsgRet>::SharedPtr client_return;

double x_drone[NDRONE],y_drone[NDRONE],z_drone[NDRONE];   /*<Variables to save the position of the drones.*/
double x_rover[NROVER],y_rover[NROVER],z_rover[NROVER];   /*<Variables to save the position of the rovers.*/

/**
 * A simple sleep function.
 * \param ms the number of milliseconds to sleep.
 **/
void delay(unsigned int ms) 
{
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}


int drone_requesting_position=-1;   /*!< variable to save the id of the drone publishing position information.  */
std::mutex mutex_position_drone;
/**
 * Function to trigger a drone on publishing position information.
 * \param[in] droneid id of the drone to be triggered.
 **/
void odom_callback_drone(int droneid){
    mutex_position_drone.lock();
    drone_requesting_position=droneid;  //cannot pass droneid to the callback function since it must accept only one parameter, the msg
    rclcpp::spin_some(node_subscriber_drone[droneid]);
}

/**
 * Callback function used when a drone publishes position information on topic /iris[drone_requesting_position]/fmu/vehicle_odometry/out
 * that save the x,y,z values on an array so that these values can be used by other functions.
 * \param msg[in] the message published by the drone on the said topic.
 **/
void save_drone_position(const VehicleOdometry::UniquePtr msg){
    if(drone_requesting_position!=-1){
        x_drone[drone_requesting_position]=msg->x;
	    y_drone[drone_requesting_position]=msg->y;
	    z_drone[drone_requesting_position]=msg->z; 
        drone_requesting_position=-1;
        mutex_position_drone.unlock();
    }else{
        std::cout<<"Error drone_requesting_position is -1!\n";
    }
}

int rover_requesting_position=-1;   /*!< variable to save the id of the rover publishing position information.  */
std::mutex mutex_position_rover;
/**
 * Function to trigger a rover on publishing position information.
 * \param[in] roverid id of the rover to be triggered.
 **/
void odom_callback_rover(int roverid){
    mutex_position_rover.lock();
    rover_requesting_position=roverid;  //cannot pass roverid to the callback function since it must accept only one parameter, the msg
    rclcpp::spin_some(node_subscriber_rover[roverid]);
    mutex_position_rover.unlock();
}

/**
 * Callback function used when a rover publishes position information on topic /rover[rover_requesting_position]/fmu/vehicle_odometry/out
 * that save the x,y,z values on an array so that these values can be used by other functions.
 * \param msg[in] the message published by the rover on the said topic.
 **/
void save_rover_position(const VehicleOdometry::UniquePtr msg){
    if(rover_requesting_position!=-1){
        x_rover[rover_requesting_position]=msg->x;
	    y_rover[rover_requesting_position]=msg->y;
	    z_rover[rover_requesting_position]=msg->z; 
        rover_requesting_position=-1;
        mutex_position_rover.unlock();
    }else{
        std::cout<<"Error rover_requesting_position is -1!\n";
    }
}

/**
 * Function that print on std_out the position of a drone, used for debug to verify that the vehicle is in the correct position
 * \param[in] droneid the id of the drone of which we want to verify the position
 **/
void print_pos_drone(int droneid){ 
    odom_callback_drone(droneid);
    std::cout<<"Drone "<<droneid<<": x: "<<x_drone[droneid]<<" y: "<<y_drone[droneid]<<" z: "<<-z_drone[droneid]<<"\n";
}

/**
 * Function that print on std_out the position of a rover, used for debug to verify that the vehicle is in the correct position
 * \param[in] roverid the id of the rover of which we want to verify the position
 **/
void print_pos_rover(int roverid){
    odom_callback_rover(roverid);
    std::cout<<"Rover "<<roverid<<": x: "<<x_rover[roverid]<<" y: "<<y_rover[roverid]<<" z: "<<-z_rover[roverid]<<"\n";
}

/**
 * Function that sends to a drone all the necessary command for a correct takeoff.
 * \param[in] droneid the id of the drone that we want to takeoff
 * \param[in] z the altitude value at which we consider the takeoff successfull
 **/
void takeoff_drone(int droneid,double z){
    VehicleCommand msg{};
    msg.param1 = 1; 
    msg.param2 = 6; 
    msg.command = VehicleCommand::VEHICLE_CMD_DO_SET_MODE;  //set offboard mode
    msg.target_system = NROVER+droneid+1;
    msg.target_component = 0;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;   

    VehicleCommand msg2{};
    
    msg2.param1 = 1.0;  //1 = arm, 0 = disarm
    //msg2.param2 = 0.0;
    msg2.command = VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;    //arm the drone
    msg2.target_system = NROVER+droneid+1;
    msg2.target_component = 0;
    msg2.source_system = 1;
    msg2.source_component = 1;
    msg2.from_external = true;

    OffboardControlMode msg3{};
	
	msg3.position = true;   //only control position
	msg3.velocity = false;
	msg3.acceleration = false;
	msg3.attitude = false;
	msg3.body_rate = false;

    odom_callback_drone(droneid);   
    TrajectorySetpoint msg4{};  //set destination coordinates
	msg4.x = x_drone[droneid];
	msg4.y = y_drone[droneid];
	msg4.z = -z;    //the frame of reference has the z axis pointed "downward"
	msg4.yaw = -PI;

	int offboard_setpoint_counter_ = 0;
	double dist_from_dest=DOUBLE_MAX;
	double x_dest=0.0,y_dest=0.0,z_dest=-z;
    int cont=0;
	do{
		odom_callback_drone(droneid);    //get current position
		dist_from_dest=sqrt(((x_drone[droneid]-x_dest)*(x_drone[droneid]-x_dest))+((y_drone[droneid]-y_dest)*(y_drone[droneid]-y_dest))+((z_drone[droneid]-z_dest)*(z_drone[droneid]-z_dest)));   //calculate distance from target
        #if DEBUG_MODE
            if(droneid==0){
		        std::cout<<"distance from target:"<<dist_from_dest<<std::endl;
                print_pos_drone(0);
            }
        #endif

		if (offboard_setpoint_counter_ == 11) {
			vehicle_command_publisher_drone[droneid]->publish(msg); //send offboard mode and arm commands
			vehicle_command_publisher_drone[droneid]->publish(msg2);
		}
		offboard_control_mode_publisher_drone[droneid]->publish(msg3);  
		trajectory_setpoint_publisher_drone[droneid]->publish(msg4);    //send some position command before the arm command to avoid arming and disarming without takeoff
		if (offboard_setpoint_counter_ < 11) {
			offboard_setpoint_counter_++;
		}
        rclcpp::spin_some(node_publisher_drone[droneid]);
		delay(100);
        cont++;
        if(cont==100){  //print position every ~10 seconds
            cont =0;
            print_pos_drone(droneid);
        }
	}while(dist_from_dest>TARGET_PRECISION);    //keep sen/ding command until it's close enough to destination
    std::cout<<"Drone "<<droneid<<": takeoff sent\n";
}

/**
 * Function to land a drone on the spot.
 * This function takes the position of the drone and use it to send a VehicleCommand::VEHICLE_CMD_NAV_LAND message.
 * \param[in] droneid id of the drone that we want to land.
 **/
void land_at_actual_location_drone(int droneid){ 
    VehicleCommand msg;
    int count=0;
    msg.command=VehicleCommand::VEHICLE_CMD_NAV_LAND;   //command to land
    odom_callback_drone(droneid);   //take position of the drone
    msg.param5=x_drone[droneid];
    msg.param6=y_drone[droneid];
    msg.param7=-z_drone[droneid];  
    msg.target_system = NROVER+droneid+1;
    msg.target_component=1;
    msg.source_system=1;
    msg.source_component=1;
    msg.from_external=true;
    while(rclcpp::ok() && count<10){
        vehicle_command_publisher_drone[droneid]->publish(msg); //send the command 10 times to be sure that the command is received
        count++;
        delay(100);
    }
    rclcpp::spin_some(node_publisher_drone[droneid]);
    std::cout<<"Drone "<<droneid<<": landing sent\n";
}

/**
 * Function to move a drone to specific coordinates relative to the spawning point.
 * \param[in] droneid id of the drone that we want to move.
 * \param[in] x_dest x coordiante of destination.
 * \param[in] y_dest y coordiante of destination.
 * \param[in] z_dest z coordiante of destination.
 * \param[in] yaw_dest desired yaw angle at destination.
 **/
void position_drone(int droneid,double x_dest,double y_dest,double z_dest,double yaw_dest){
    z_dest=-z_dest; //the reference system has the z axis facing downwards

    OffboardControlMode msg3{};
	msg3.position = true;
	msg3.velocity = false;
	msg3.acceleration = false;
	msg3.attitude = false;
	msg3.body_rate = false;

    TrajectorySetpoint msg4{};
	msg4.x = x_dest;
	msg4.y = y_dest;
	msg4.z = z_dest;    
	msg4.yaw = yaw_dest;
	
	double dist_from_dest=DOUBLE_MAX;
    int cont =0;
	do{
		odom_callback_drone(droneid);    //get current position
		dist_from_dest=sqrt(((x_drone[droneid]-x_dest)*(x_drone[droneid]-x_dest))+((y_drone[droneid]-y_dest)*(y_drone[droneid]-y_dest))+((z_drone[droneid]-z_dest)*(z_drone[droneid]-z_dest)));   //calculate distance from target
		#if DEBUG_MODE
		    std::cout<<"distance from target:"<<dist_from_dest<<"\n";
        #endif

		offboard_control_mode_publisher_drone[droneid]->publish(msg3);
		trajectory_setpoint_publisher_drone[droneid]->publish(msg4);

		delay(100);
        cont++;
        if(cont==100){
            cont=0;
            print_pos_drone(droneid);
        }
	}while(dist_from_dest>TARGET_PRECISION);
    std::cout<<"Drone "<<droneid<<": arrived at destination\n";
}

/**
 * Function to rotate a drone on the spot.
 * \param[in] droneid id of the drone that we want to rotate.
 * \param[in] new_yaw new yaw angle.
 **/
void yaw_rotation_drone(int droneid,double new_yaw){    
    odom_callback_drone(droneid);
    position_drone(droneid,x_drone[droneid],y_drone[droneid],-z_drone[droneid],new_yaw);
    delay(1000);
}

/**
 * Function to disarm a drone.
 * \param[in] droneid id of the drone we want to disarm.
 **/
void disarm_drone(int droneid){
    int count = 0;
    VehicleCommand msg2{};
    msg2.param1 = 0.0;
    msg2.command = VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
    msg2.target_system = 1+droneid;
    msg2.target_component = 1;
    msg2.source_system = 1;
    msg2.source_component = 1;
    msg2.from_external = true;

    while(rclcpp::ok() && count<10){
        vehicle_command_publisher_drone[droneid]->publish(msg2);
        count++;
        delay(100);
    }
    rclcpp::spin_some(node_publisher_drone[droneid]);
}

std::mutex m; 
int drone_shooting=-1;  /*!< variable to save the id of the drone publishing an image.  */
int serial[NDRONE];

/**
 * Callback function used when a drone publishes an image.
 * The function takes the received encoded image, decode and convert it on a opencv image format so that the function cv::imwrite can be used to save the image.
 * \param[in] msg message published by the camera installed on the drone.
 **/
void image_callback(const sensor_msgs::msg::Image::SharedPtr msg){
    if(drone_shooting!=-1){ 
        //data size is step*height
        std::cout<<"Image recieved\n";
        cv_bridge::CvImagePtr cv_image;
        
        cv_image = cv_bridge::toCvCopy(msg, msg->encoding);    //convert image from uint8 array to opencv image
        cv::Mat frame= cv_image->image; //convert image to matrix
        try{
            cv::imwrite("/home/drone/Planning-drones-ROS2/robots_control/images/drone_"+std::to_string(drone_shooting)+"_n_"+std::to_string(serial[drone_shooting])+".jpg",frame);   //save image
            serial[drone_shooting]++;   //update serial for future images
        }catch(...){
            std::cout<<"Error!";
        }
        drone_shooting=-1;
    }
    m.unlock();  
}
std::mutex m_depth;
int drone_depth=-1;
double z_sensor[NDRONE];
/**
 * Callback function used when the range sensor publish information about the height of a drone (used in the distributed strategy).
 * \param[in] msg message published by the range sensor (infrared) mounted on the drone.
 **/
void depth_callback(const sensor_msgs::msg::Range::SharedPtr msg){
    if(drone_depth!=-1){
        z_sensor[drone_depth]=msg->range;
        drone_depth=-1;
    }
    m_depth.unlock();
        
    //std::cout<<"Distance: "<<msg->range<<"\n";
}


/**
 * Function used to trigger a drone on publishing an image on the topic.
 * \param[in] droneid id of the drone by which we want to obtain an image.
 **/
void get_image(int droneid){
    std::cout<<"Drone "<<droneid<<": getting image\n";
    m.lock();   //since that i use a single callback for both the cameras, i want to avoid conflicts of any type
    drone_shooting=droneid; //save the droneid that is taking the photo
    rclcpp::spin_some(node_image[droneid]);    //get the photo (trigger image_callback)
    
}

/**
 * Function to trigger a drone on publishing altitude information through range sensor.
 * \param[in] droneid id of the drone tby which we want to obtain the altitude.
 **/
void get_depth(int droneid){
    m_depth.lock();
    drone_depth=droneid;
    rclcpp::spin_some(node_depth[droneid]);
}

std::list<int> drone_action[NDRONE], rover_action[NROVER];
std::list<float> drone_x[NDRONE],drone_y[NDRONE],drone_z[NDRONE];
std::list<float> rover_x[NDRONE],rover_y[NDRONE],rover_z[NDRONE];

//std::list<int> drone_0_action, drone_1_action, rover_action;    /*!<Lists used to store all the action that a vehicle will perform (takeoff, land, move and take picture). */
//std::list<float> drone_0_x,drone_1_x,rover_x, drone_0_y,drone_1_y,rover_y,drone_0_z,drone_1_z,rover_z;  /*<Lists used to store the positions at which the action will be performed */

/**
 * Function that send all the necessary commands to a rover so that it can perform its mission.
 * \param[in] roverid id of the rover that will perform the mission.
 **/
void perform_mission_rover(int roverid){
    VehicleCommand msg{};
    msg.param1 = 1; 
    msg.param2 = 6; 
    msg.command = VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
    msg.target_system = 1+roverid;
    msg.target_component = 0;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;   

    VehicleCommand msg2{};  //since that, unlike drones, the rover doesn't have a takeoff procedure, the rover is armed here
    
    msg2.param1 = 1.0;
    msg2.command = VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
    msg2.target_system = 1+roverid;
    msg2.target_component = 0;
    msg2.source_system = 1;
    msg2.source_component = 1;
    msg2.from_external = true;

    OffboardControlMode msg3{};
	
	msg3.position = true;
	msg3.velocity = false;
	msg3.acceleration = false;
	msg3.attitude = false;
	msg3.body_rate = false;

    bool firsttime = true;
    while(rover_action[roverid].size()>0){
        int action_dest = rover_action[roverid].front();
        rover_action[roverid].pop_front();
        TrajectorySetpoint msg4{};
        
        double x_dest = rover_x[roverid].front();
        double y_dest = rover_y[roverid].front();
        double z_dest = rover_z[roverid].front();
        msg4.x = x_dest;
        msg4.y = y_dest;
        msg4.z = z_dest;
        rover_x[roverid].pop_front();
        rover_y[roverid].pop_front();
        rover_z[roverid].pop_front();
        #if TEST_MODE
            msg4.z = 0.0;
        #endif
        msg4.yaw = 0.0;
        std::cout<<"Rover "<<roverid<<": action: "<<action_dest<<" x: "<<x_dest<<" y: "<<y_dest<<"\n";

        int offboard_setpoint_counter_ = 0;
        double dist_from_dest=DOUBLE_MAX;
        int cont =0;

        if(action_dest==2){ //actio==2 means move rover at coodrinates
            do{ 
                odom_callback_rover(roverid);
                dist_from_dest=sqrt(((x_rover[roverid]-x_dest)*(x_rover[roverid]-x_dest))+((y_rover[roverid]-y_dest)*(y_rover[roverid]-y_dest))/*+((z_rover[roverid]-z_dest)*(z_rover[roverid]-z_dest))*/);   //calculate distance from target
                #if DEBUG_MODE
                    std::cout<<"distance from target:"<<dist_from_dest<<std::endl;
                    print_pos_rover();
                #endif
                if (offboard_setpoint_counter_ == 11 && firsttime) {
                    vehicle_command_publisher_rover[roverid]->publish(msg);
                    vehicle_command_publisher_rover[roverid]->publish(msg2);
                }
                offboard_control_mode_publisher_rover[roverid]->publish(msg3);
                trajectory_setpoint_publisher_rover[roverid]->publish(msg4);
                if (offboard_setpoint_counter_ < 11) {
                    offboard_setpoint_counter_++;
                }
                rclcpp::spin_some(node_publisher_rover[roverid]);
                delay(100);
                cont++;
                if(cont==100){
                    cont=0;
                    print_pos_rover(0);
                }
            }while(dist_from_dest>1.5);
            firsttime = false;
        }else{  //the planner service returned a wrong id
            std::cout<<"Errore action != 2\n";
        }
        
    }
    std::cout<<"Rover "<<roverid<<" arrived at destination\n";
}

int cont_callback = 0;

/**
 * function called when the user interface publishes actions on the "/ui" topic.
 * The first time the user interface publish something (cont_callback==1) the function store all the recieved action on the appropriate lists, then, for each vehicle, it takes the last element of the list 
 * (the last position to visit) and send this information back to the user interface that calculate the return path (with another service) and publish again on the "/ui" topic this information.
 * At this point the function (cont_callback == 2) store the new positions on the same lists and starts all the missions.
 * It is not possible to calculate the return path while the missions are running because it take some time to do all the calculations and, if a drone finishes its mission before receving the return path, 
 * it simply disable the offboard mode and it lands on the spot (it may be dangerous to land wherever the drone is).
 * \param[in] msg message published by the user_interface 
 **/
void ui_callback(const interfaces::msg::Mymsg::SharedPtr msg){ 
    cont_callback++;

    int length = msg->length;

    for(int i=0;i<length;i++){
        int action = msg->action[i];
        double x = msg->x[i];
        double y = msg->y[i];
        double z = msg->z[i];

        if(msg->vehicle[i]=="d1"){  //each action is stored in the list of the vehicle that will perform that action (done in both first and second call)
            drone_action[0].push_back(action);
            drone_x[0].push_back(x);
            drone_y[0].push_back(y);
            drone_z[0].push_back(z);
        }else if(msg->vehicle[i]=="d2"){
            drone_action[1].push_back(action);
            drone_x[1].push_back(x);
            drone_y[1].push_back(y);
            drone_z[1].push_back(z);
        }else{
            rover_action[0].push_back(action);
            rover_x[0].push_back(x);
            rover_y[0].push_back(y);
            rover_z[0].push_back(z);
        }
    
    }

    if(cont_callback==1){   //first time the callback is called
        auto request = interfaces::msg::MsgRet();

        //get the last position of each vehicle (origin if the vehicle didn't move) and save it to calculate the return path
        if(rover_x[0].size()!=0){
            request.x_rover = rover_x[0].back();
            request.y_rover = rover_y[0].back();
        }else{
            request.x_rover = 0.0;
            request.y_rover = 0.0;
        }

        if(drone_x[0].size()!=0){
            request.x_drone_0 = drone_x[0].back();
            request.y_drone_0 = drone_y[0].back();
            request.z_drone_0 = drone_z[0].back();
        }else{
            request.x_drone_0 = 0.0;
            request.y_drone_0 = 0.0;
            request.z_drone_0 = 0.0;
        }

        if(drone_x[1].size()!=0){
            request.x_drone_1 = drone_x[1].back();
            request.y_drone_1 = drone_y[1].back();
            request.z_drone_1 = drone_z[1].back();
        }else{
            request.x_drone_1 = 0.0;
            request.y_drone_1 = 0.0;
            request.z_drone_1 = 0.0;
        }

        delay(5000);
        client_return->publish(request);    //publish on the "/ret" topic the previous positions to calculate the return path
    }
    
    if(cont_callback==2){   //second time the callback is called (return path)
        std::cout<<"Drone 0: # commands: "<<drone_action[0].size()<<"\nDrone 1: # commands: "<<drone_action[1].size()<<"\nRover: # commands: "<<rover_action[0].size()<<"\n";
        //start all the missions
        std::thread mission0(perform_mission_drone,0);  
        std::thread mission1(perform_mission_drone,1);
        std::thread mission2(perform_mission_rover,0);

        mission0.join();
        mission1.join();
        mission2.join();
    }
    
   
}

/**
 * Function to initialize all the necessary variables when the program starts, in particluar it inizialize all the publishers and subscriber that will be used during the program.
 **/
void init_all(int argc,char* argv[]){   
    rclcpp::init(argc, argv);
    node_ui =rclcpp::Node::make_shared("ui_subscriber");
    node_return = rclcpp::Node::make_shared("return_client");

    ui_subscriber = node_ui->create_subscription<interfaces::msg::Mymsg>("/ui",10, ui_callback);   //subscribe to "/ui" to receive actions for the vehicle to perform
    client_return = node_return->create_publisher<interfaces::msg::MsgRet>("/ret",10); //client to publish on "/ret" to send to the ui the necessary informations to calculate the return path
    
    for(int i=0;i<NDRONE;i++){
        node_publisher_drone[i] = rclcpp::Node::make_shared("minimal_publisher_drone"+std::to_string(i));
        node_subscriber_drone[i] = rclcpp::Node::make_shared("minimal_subscriber_drone"+std::to_string(i));
        node_image[i] = rclcpp::Node::make_shared("image_subscriber_drone"+std::to_string(i));
        node_depth[i] = rclcpp::Node::make_shared("range_subscriber_drone"+std::to_string(i));

        serial[i]=0;    //serial number of the photos taken by every drone (initialized to 0)

        offboard_control_mode_publisher_drone[i] = node_publisher_drone[i]->create_publisher<OffboardControlMode>("iris"+std::to_string(i)+"/fmu/offboard_control_mode/in",10);
        trajectory_setpoint_publisher_drone[i] = node_publisher_drone[i]->create_publisher<TrajectorySetpoint>("iris"+std::to_string(i)+"/fmu/trajectory_setpoint/in",10);
        vehicle_command_publisher_drone[i] = node_publisher_drone[i]->create_publisher<VehicleCommand>("iris"+std::to_string(i)+"/fmu/vehicle_command/in",10);

        odometry_subscriber_drone[i]=node_subscriber_drone[i]->create_subscription<VehicleOdometry>("iris"+std::to_string(i)+"/fmu/vehicle_odometry/out",10,save_drone_position);
        
        test_image[i] = node_image[i]->create_subscription<sensor_msgs::msg::Image>("/custom_ns"+std::to_string(i+1)+"/camera"+std::to_string(i+1)+"/image_raw",10,image_callback);
        test_depth[i] = node_depth[i]->create_subscription<sensor_msgs::msg::Range>("/ultrasonic_sensor_"+std::to_string(i+1),1,depth_callback);
    }

    for(int i=0;i<NROVER;i++){      
        node_publisher_rover[i] = rclcpp::Node::make_shared("minimal_publisher_rover"+std::to_string(i));
        node_subscriber_rover[i] = rclcpp::Node::make_shared("minimal_subscriber_rover"+std::to_string(i));

        offboard_control_mode_publisher_rover[i] = node_publisher_rover[i]->create_publisher<OffboardControlMode>("rover"+std::to_string(i)+"/fmu/offboard_control_mode/in",10);
        trajectory_setpoint_publisher_rover[i] = node_publisher_rover[i]->create_publisher<TrajectorySetpoint>("rover"+std::to_string(i)+"/fmu/trajectory_setpoint/in",10);
        vehicle_command_publisher_rover[i] = node_publisher_rover[i]->create_publisher<VehicleCommand>("rover"+std::to_string(i)+"/fmu/vehicle_command/in",10);

        odometry_subscriber_rover[i]=node_subscriber_rover[i]->create_subscription<VehicleOdometry>("rover"+std::to_string(i)+"/fmu/vehicle_odometry/out",10,save_rover_position);
    }

}

/**
 * Function that send all the necessary commands to a drone so that it can perform its mission.
 * This function uses the drone_x_action and drone_x_[x,y,z] lists to get all the actions to perform once at time and, based on their value, call the correct function.
 * \param[in] droneid the id of the drone that will perform the mission.
 **/
void perform_mission_drone(int droneid){
    double x,y,z;
    int action;
    while(drone_action[droneid].size()>0){ //iterate on all actions
        //perform 1 action at time

        //get the first element of the lists (drone_?_action contains the action to perform while drone_?_x, drone_?_y, drone_?_z contains the coordinates)
        action = drone_action[droneid].front();
        drone_action[droneid].pop_front();

        x = drone_x[droneid].front();
        drone_x[droneid].pop_front();

        y = drone_y[droneid].front();
        drone_y[droneid].pop_front();

        if(action!=5)
            z = drone_z[droneid].front();
        drone_z[droneid].pop_front();
        #if TEST_MODE
            z =200.0;
        #endif

        std::cout<<"Drone: "<<droneid<<" action: "<<action<<" x: "<<x<<" y: "<<y<<" z: "<<z<<"\n";

        switch (action){
            case 1:{    //move the drone to destination
                try{
                    position_drone(droneid,x,y,z,PI); 
                }catch(...){
                    std::cout<<droneid<<" "<<action<<"\n"; std::perror("Error");
                }
                break;
            }
            case 2:{    //action = 2 only for rovers
                std::cout<<"Error! Drone "<<droneid<<" cannot perform action with value 2!\n";
                break;
            }
            case 3:{    //takeoff
                try{
                    takeoff_drone(droneid,z);                    
                }catch(...){
                    std::cout<<droneid<<" "<<action<<"\n"; std::perror("Error");
                }   
                break;        
            }
            case 4:{    //land
                try{
                    land_at_actual_location_drone(droneid);
                }catch(...){
                    std::cout<<droneid<<" "<<action<<"\n"; std::perror("Error");
                }  
                break;
            }
            case 5:{    //move to destination, take photo, turn 180 degrees, take photo
                try{
                    odom_callback_drone(droneid);   //get position wrt the start location
                    
                    //position_drone(droneid,x,y,z,PI);
                    /*delay(4000);
                    get_depth(droneid); //get distance from terrain
                    double new_z = -z_drone[droneid] - z_sensor[droneid] + 5.0;  //calculate the height wrt the starting location that is 5 meters above the ground in the point that the drone is.
                    std::cout<<"Distance from terrain: "<<z_sensor[droneid]<<" height from spawn: "<<-z_drone[droneid]<<" target height: "<<new_z<<"\n";*/
                    position_drone(droneid,x,y,z,PI);
                    delay(2000);
                    get_image(droneid);
                    position_drone(droneid,x,y,z,0.0);
                    delay(2000);
                    get_image(droneid);
                }catch(...){
                    std::cout<<droneid<<" "<<action<<"\n"; std::perror("Error");
                }  
                break;
            }
            default:{
                std::cout<<"Error! Wrong action value!\n";
                std::cout<<"Message: action:"<<action<<" drone: "<<droneid<<" x: "<<x<<" y: "<<y<<" z: "<<z<<"\n";
                break;
            }
        }
        delay(1000);
    }
    std::cout<<"Drone "<<droneid<<" finished\n";
}   

/**
 * Function to perform a demo of all the actions that the drones and the rover can perform.
 **/
void perform_demo(){
    /*drone_action[0].push_back(3);    //takeoff drone 0
    drone_x[0].push_back(0.0);
    drone_y[0].push_back(0.0);
    drone_z[0].push_back(221.0);

    
    drone_action[0].push_back(1);
    drone_x[0].push_back(-350.0);
    drone_y[0].push_back(-350.0);
    drone_z[0].push_back(152.0);

    drone_action[0].push_back(1);
    drone_x[0].push_back(-700.0);
    drone_y[0].push_back(0.0);
    drone_z[0].push_back(228.0);

    drone_action[0].push_back(1);
    drone_x[0].push_back(-1050.0);
    drone_y[0].push_back(350.0);
    drone_z[0].push_back(314.0);
    
    drone_action[0].push_back(5);
    drone_x[0].push_back(-1345.0);
    drone_y[0].push_back(631.0);
    drone_z[0].push_back(467.0);*/

    /*drone_action[1].push_back(3);    //takeoff drone 1
    drone_x[1].push_back(0.0);
    drone_y[1].push_back(0.0);
    drone_z[1].push_back(40.0);*/


   /* drone_action[0].push_back(1);    //move drone 0
    drone_x[0].push_back(0.0);
    drone_y[0].push_back(40.0);
    drone_z[0].push_back(40.0);*/
    
    /*drone_action[1].push_back(1);    //move drone 1
    drone_x[1].push_back(0.0);
    drone_y[1].push_back(40.0);
    drone_z[1].push_back(40.0);*/
    

   /* drone_action[0].push_back(1);    //move drone 0
    drone_x[0].push_back(100.0);
    drone_y[0].push_back(70.0);
    drone_z[0].push_back(20.0);
    
    drone_action[1].push_back(1);    //move drone 1
    drone_x[1].push_back(100.0);
    drone_y[1].push_back(70.0);
    drone_z[1].push_back(20.0);*/

    
    /*drone_action[0].push_back(5);    //take photo drone 0
    drone_x[0].push_back(110.0);
    drone_y[0].push_back(45.0);
    drone_z[0].push_back(40.0);*/
    
    /*drone_action[1].push_back(5);    //take photo drone 1
    drone_x[1].push_back(110.0);
    drone_y[1].push_back(45.0);
    drone_z[1].push_back(40.0);*/

   /* drone_action[0].push_back(4);    //land drone 0
    drone_x[0].push_back(110.0);
    drone_y[0].push_back(45.0);
    drone_z[0].push_back(0.0);*/
    
   /* drone_action[1].push_back(4);    //land drone 1
    drone_x[1].push_back(110.0);
    drone_y[1].push_back(45.0);
    drone_z[1].push_back(0.0);*/

    rover_action[0].push_back(2);  //move rover
    rover_x[0].push_back(-350.0);
    rover_y[0].push_back(-350.0);
    rover_z[0].push_back(0.0);

    rover_action[0].push_back(2);  //move rover
    rover_x[0].push_back(-700.0);
    rover_y[0].push_back(-700.0);
    rover_z[0].push_back(0.0);

    delay(5000);


    std::cout<<"Drone 0: # commands: "<<drone_action[0].size()<<"\nDrone 1: # commands: "<<drone_action[1].size()<<"\nRover: # commands: "<<rover_action[0].size()<<"\n";
    //start all the missions
    //std::thread mission0(perform_mission_drone,0);  
    //std::thread mission1(perform_mission_drone,1);
    std::thread mission2(perform_mission_rover,0);

    //mission0.join();
    //mission1.join();
    mission2.join();
}

int main(int argc, char* argv[]){
    init_all(argc,argv);
    //std::cout<<"Demo mode is: "<<(DEMO_MODE == true? "True":"False")<<"\nTest mode is: "<<(TEST_MODE== true? "True":"False")<<"\nDebug mode is: "<<(DEBUG_MODE== true? "True":"False")<<"\n";

    #if DEMO_MODE
        perform_demo();
    #else
        rclcpp::spin(node_ui);
    #endif
    
    rclcpp::shutdown();
}