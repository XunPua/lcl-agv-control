/* *************************************************************
   Encoder definitions
   
   Add an "#ifdef" block to this file to include support for
   a particular encoder board or library. Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.
   
   ************************************************************ */
   
#ifdef USE_BASE

#ifdef ARDUINO_ENC_COUNTER
  volatile long front_left_enc_pos = 0L;
  volatile long front_right_enc_pos = 0L;
  volatile long back_left_enc_pos = 0L;
  volatile long back_right_enc_pos = 0L;
  static const int8_t ENC_STATES [] = {0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0};  //encoder lookup 
  
  void isr_front_left_enc(){
    static uint8_t enc_last=0;
    enc_last <<=2; //shift previous state two places
    enc_last |= ((digitalRead(FRONT_LEFT_ENC_PIN_B) << 1) | (digitalRead(FRONT_LEFT_ENC_PIN_A)));
  
  	front_left_enc_pos += ENC_STATES[(enc_last & 0x0f)];
  }

  void isr_front_right_enc(){
    static uint8_t enc_last=0;
          	
    enc_last <<=2; //shift previous state two places
    enc_last |= ((digitalRead(FRONT_RIGHT_ENC_PIN_B) << 1) | (digitalRead(FRONT_RIGHT_ENC_PIN_A))); //read the current state into lowest 2 bits (reads PC4 and 5)
    
  	front_right_enc_pos += ENC_STATES[(enc_last & 0x0f)];
  }

  void isr_back_left_enc(){
    static uint8_t enc_last=0;
    enc_last <<=2; //shift previous state two places
    enc_last |= ((digitalRead(BACK_LEFT_ENC_PIN_B) << 1) | (digitalRead(BACK_LEFT_ENC_PIN_A)));
  
  	back_left_enc_pos += ENC_STATES[(enc_last & 0x0f)];
  }

  void isr_back_right_enc(){
    static uint8_t enc_last=0;
          	
    enc_last <<=2; //shift previous state two places
    enc_last |= ((digitalRead(BACK_RIGHT_ENC_PIN_B) << 1) | (digitalRead(BACK_RIGHT_ENC_PIN_A))); //read the current state into lowest 2 bits (reads PC4 and 5)
    
  	back_right_enc_pos += ENC_STATES[(enc_last & 0x0f)];
  }
  
  /* Wrap the encoder reading function */
  long readEncoder(int i) {
    if (i == FRONT_LEFT) return front_left_enc_pos;
    else if (i == FRONT_RIGHT) return front_right_enc_pos;
    else if (i == BACK_LEFT) return back_left_enc_pos;
    else return back_right_enc_pos;
  }

  /* Wrap the encoder reset function */
  void resetEncoder(int i) {
    if (i == FRONT_LEFT){
      front_left_enc_pos=0L;
      return;
    }
    else if (i == FRONT_RIGHT){
      front_right_enc_pos=0L;
      return;
    } 
    else if (i == BACK_LEFT){
      back_left_enc_pos=0L;
      return;
    } 
    else { 
      back_right_enc_pos=0L;
      return;
    }
  }
#else
  #error A encoder driver must be selected!
#endif

/* Wrap the encoder reset function */
void resetEncoders() {
  resetEncoder(FRONT_LEFT);
  resetEncoder(FRONT_RIGHT);
  resetEncoder(BACK_LEFT);
  resetEncoder(BACK_RIGHT);
}

#endif

