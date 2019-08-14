void setLED(const std_msgs::UInt8MultiArray& LED_msg ){

  //Turn both LED's on - same colour
  leds[0] = CRGB(255,20,147);
  FastLED.show(); 
  leds[1] = CRGB(255,20,147);
  FastLED.show();
  
}

void readVelocity(const geometry_msgs::Twist vel_msg){

  Vd = vel_msg.linear.x;
  Wd = vel_msg.angular.z;
  
}

void setPos(const geometry_msgs::Pose2D pos_set_msg){

  robot_pos.x = pose_set_msg.x;
  robot_pos.y = pose_set_msg.y;
  robot_pos.theta = pose_set_msg.theta;
  
}
