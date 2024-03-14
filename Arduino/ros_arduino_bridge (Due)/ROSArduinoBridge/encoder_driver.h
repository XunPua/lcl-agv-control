/* *************************************************************
   Encoder driver function definitions - by James Nugen
   ************************************************************ */
   
   
#ifdef ARDUINO_ENC_COUNTER

  #define FRONT_LEFT_ENC_PIN_A 23  //23
  #define FRONT_LEFT_ENC_PIN_B 25  //25
  
  #define FRONT_RIGHT_ENC_PIN_A 37  //29
  #define FRONT_RIGHT_ENC_PIN_B 39   //31

  #define BACK_LEFT_ENC_PIN_A 27  //35
  #define BACK_LEFT_ENC_PIN_B 29   //37

  #define BACK_RIGHT_ENC_PIN_A 41 //41
  #define BACK_RIGHT_ENC_PIN_B 43   //43
#endif
   
long readEncoder(int i);
void resetEncoder(int i);
void resetEncoders();
void isr_front_left_enc();
void isr_front_right_enc();
void isr_back_left_enc();
void isr_back_right_enc();

