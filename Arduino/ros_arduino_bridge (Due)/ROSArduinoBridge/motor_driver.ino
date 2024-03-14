/***************************************************************
   Motor driver definitions
   
   Add a "#elif defined" block to this file to include support
   for a particular motor driver.  Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.
   
   *************************************************************/

#ifdef USE_BASE

#ifdef L298_MOTOR_DRIVER
  void initMotorController() {
    // digitalWrite(RIGHT_MOTOR_ENABLE, HIGH);
    // digitalWrite(LEFT_MOTOR_ENABLE, HIGH);
  }
  
  void setMotorSpeed(int i, int spd) {
    unsigned char reverse = 0;
  
    if (spd < 0)
    {
      spd = -spd;
      reverse = 1;
    }
    if (spd > 255)
      spd = 255;
    
    if (i == FRONT_LEFT) { 
      if      (reverse == 0) { analogWrite(FRONT_LEFT_MOTOR_FORWARD, spd); analogWrite(FRONT_LEFT_MOTOR_BACKWARD, 0); }
      else if (reverse == 1) { analogWrite(FRONT_LEFT_MOTOR_BACKWARD, spd); analogWrite(FRONT_LEFT_MOTOR_FORWARD, 0); }
    }
    else if (i == FRONT_RIGHT){
      if      (reverse == 0) { analogWrite(FRONT_RIGHT_MOTOR_FORWARD, spd); analogWrite(FRONT_RIGHT_MOTOR_BACKWARD, 0); }
      else if (reverse == 1) { analogWrite(FRONT_RIGHT_MOTOR_BACKWARD, spd); analogWrite(FRONT_RIGHT_MOTOR_FORWARD, 0); }
    }
    else if (i == BACK_LEFT){
      if      (reverse == 0) { analogWrite(BACK_LEFT_MOTOR_FORWARD, spd); analogWrite(BACK_LEFT_MOTOR_BACKWARD, 0); }
      else if (reverse == 1) { analogWrite(BACK_LEFT_MOTOR_BACKWARD, spd); analogWrite(BACK_LEFT_MOTOR_FORWARD, 0); }
    }
    else{
      if      (reverse == 0) { analogWrite(BACK_RIGHT_MOTOR_FORWARD, spd); analogWrite(BACK_RIGHT_MOTOR_BACKWARD, 0); }
      else if (reverse == 1) { analogWrite(BACK_RIGHT_MOTOR_BACKWARD, spd); analogWrite(BACK_RIGHT_MOTOR_FORWARD, 0); }
    }
  }
  
  void setMotorSpeeds(int frontleftSpeed, int frontrightSpeed, int backleftSpeed, int backrightSpeed) {
    setMotorSpeed(FRONT_LEFT, frontleftSpeed);
    setMotorSpeed(FRONT_RIGHT, frontrightSpeed);
    setMotorSpeed(BACK_LEFT, backleftSpeed);
    setMotorSpeed(BACK_RIGHT, backrightSpeed);
  }
#else
  #error A motor driver must be selected!
#endif

#endif
