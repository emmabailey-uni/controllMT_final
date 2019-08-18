#include "Header.h"

ros::NodeHandle nh;

//Publisher message definition
geometry_msgs::Pose2D pose_msg;
std_msgs::float32 sensor_msg;

//Publisher Topics
ros::Publisher pose("pose", &pose_msg);
ros::Publisher front_dis("front_dis", &sensor_msg); 
ros::Publisher left_dis("left_dis", &sensor_msg); 
ros::Publisher right_dis("right_dis", &sensor_msg); 

//Subscriber Topics)
ros::Subscriber<std_msgs::UInt8MultiArray> sub_leds("rgb_leds", setLED);
ros::Subscriber<geometry_msgs::Twist> sub_cmd_vel("cmd_vel", readVelocity);
ros::Subscriber<geometry_msgs::Pose2D> sub_set_pose("set_pose", setPos);

// Set desired robo velocities
//float Vd = 0.07; // in range [-0.08, 0.08] [m/s]
//float Wd= 0; // in range [-1.7, 1.7] [rad/s]

void setup() {
  //Initialise ROS serial node
  nh.initNode();

  //Advertising Publisher Topics (Initilisation)
  nh.advertise(pose);
  nh.advertise(front_dis);
  nh.advertise(left_dis);
  nh.advertise(right_dis);

  //Subscriber Initilisation
  nh.subscribe(sub_leds);
  nh.subscribe(sub_cmd_vel);
  nh.subscribe(sub_set_pose);
  
  nh.getHardware()->setBaud(115200);

  //Defining type of LED, fills array with LED objects
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);

  //Controls brightness of LEDs
  FastLED.setBrightness(255);
  
  pinMode(PWMR,OUTPUT);
  pinMode(PWML,OUTPUT);
  pinMode(DIRR,OUTPUT);
  pinMode(DIRL,OUTPUT);

  Timer3.initialize(10000); // 10000 microseconds = 0.01 s
  Timer3.attachInterrupt(MotorSpeedControl); // Update speed when timer overlows (100Hz)  
}

void MotorSpeedControl(void)
{   
    encUpdate();
    poseUpdate();
    cmd_vel2wheel(Vd,Wd,&WLd, &WRd);
    cmd_vel2wheel(Vr,Wr,&WLr, &WRr);
    ModelController(Vd, Wd, WLr, WRr);
}

void loop() {
  current_time=micros();

  // Start 10Hz loop
  if (current_time-previous_time>= sampling_time){
    previous_time=current_time;

    pose_msg pose_MSG;
    pose_MSG.x = robot_pos.x;
    pose_MSG.y = robot_pos.y;
    pose_MSG.theta = robot_pos.theta;
    

    //Publish Topics
    //DONE WRONG MUST CHANGE
    pose.publish(pose_MSG);
    front_dis.publish(front_distance);
    left_dis.publish(left_distance);
    right_dis.publish(right_distance);

    //Spin node to process callbacks
    nh.spinOnce();

  }
}



  /////////////////////////////////////////////////////////////////////
  //Serial prints
  /*Serial.print("X = ");
  Serial.print(current_pos.x);
  
  Serial.print("Y = ");
  Serial.println(current_pos.y);
  
  Serial.print("Theta = ");
  Serial.println(current_pos.theta);
  
  Serial.print("NR = ");
  Serial.print(NR);
  
  Serial.print("NL = ");
  Serial.println(NL);   
  
  
  //Serial.print("real WL = ");
  
  
  //Serial.print("real WR = ");
  //Serial.println(WRr);
  
  //print all the values of the IR sensors
  readSensors(&left_distance, &middle_distance, &right_distance); // distance from each sensor [m]
  Serial.println("Left: "); 
  Serial.println(left_distance);
  Serial.println("Middle: ");  
  Serial.println(middle_distance);
  Serial.println("Right: "); 
  Serial.println(right_distance);
  */
    
