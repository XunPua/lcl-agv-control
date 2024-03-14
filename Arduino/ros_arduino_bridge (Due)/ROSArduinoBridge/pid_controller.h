/* Functions and type-defs for PID control.

   Taken mostly from Mike Ferguson's ArbotiX code which lives at:
   
   http://vanadium-ros-pkg.googlecode.com/svn/trunk/arbotix/
*/

/* PID setpoint info For a Motor */
typedef struct {
  double TargetTicksPerFrame;    // target speed in ticks per frame
  long Encoder;                  // encoder count
  long PrevEnc;                  // last encoder count

  /*
  * Using previous input (PrevInput) instead of PrevError to avoid derivative kick,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
  */
  int PrevInput;                // last input
  //int PrevErr;                   // last error

  /*
  * Using integrated term (ITerm) instead of integrated error (Ierror),
  * to allow tuning changes,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
  //int Ierror;
  int ITerm;                    //integrated term

  long output;                    // last motor setting
}
SetPointInfo;

SetPointInfo frontleftPID, frontrightPID, backleftPID, backrightPID;

/* PID Parameters */
int Kp = 20;
int Kd = 12;
int Ki = 0;
int Ko = 50;

unsigned char moving = 0; // is the base in motion?

/*
* Initialize PID variables to zero to prevent startup spikes
* when turning PID on to start moving
* In particular, assign both Encoder and PrevEnc the current encoder value
* See http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
* Note that the assumption here is that PID is only turned on
* when going from stop to moving, that's why we can init everything on zero.
*/
void resetPID(){
   frontleftPID.TargetTicksPerFrame = 0.0;
   frontleftPID.Encoder = readEncoder(FRONT_LEFT);
   frontleftPID.PrevEnc = frontleftPID.Encoder;
   frontleftPID.output = 0;
   frontleftPID.PrevInput = 0;
   frontleftPID.ITerm = 0;

   frontrightPID.TargetTicksPerFrame = 0.0;
   frontrightPID.Encoder = readEncoder(FRONT_RIGHT);
   frontrightPID.PrevEnc = frontrightPID.Encoder;
   frontrightPID.output = 0;
   frontrightPID.PrevInput = 0;
   frontrightPID.ITerm = 0;

   backleftPID.TargetTicksPerFrame = 0.0;
   backleftPID.Encoder = readEncoder(BACK_LEFT);
   backleftPID.PrevEnc = backleftPID.Encoder;
   backleftPID.output = 0;
   backleftPID.PrevInput = 0;
   backleftPID.ITerm = 0;

   backrightPID.TargetTicksPerFrame = 0.0;
   backrightPID.Encoder = readEncoder(BACK_RIGHT);
   backrightPID.PrevEnc = backrightPID.Encoder;
   backrightPID.output = 0;
   backrightPID.PrevInput = 0;
   backrightPID.ITerm = 0;
}

/* PID routine to compute the next motor commands */
void doPID(SetPointInfo * p) {
  long Perror;
  long output;
  int input;

  //Perror = p->TargetTicksPerFrame - (p->Encoder - p->PrevEnc);
  input = p->Encoder - p->PrevEnc;
  Perror = p->TargetTicksPerFrame - input;


  /*
  * Avoid derivative kick and allow tuning changes,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
  //output = (Kp * Perror + Kd * (Perror - p->PrevErr) + Ki * p->Ierror) / Ko;
  // p->PrevErr = Perror;
  output = (Kp * Perror - Kd * (input - p->PrevInput) + p->ITerm) / Ko;
  p->PrevEnc = p->Encoder;

  output += p->output;
  // Accumulate Integral error *or* Limit output.
  // Stop accumulating when output saturates
  if (output >= MAX_PWM)
    output = MAX_PWM;
  else if (output <= -MAX_PWM)
    output = -MAX_PWM;
  else
  /*
  * allow turning changes, see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
    p->ITerm += Ki * Perror;

  p->output = output;
  p->PrevInput = input;
}

/* Read the encoder values and call the PID routine */
void updatePID() {
  /* Read the encoders */
  frontleftPID.Encoder = readEncoder(FRONT_LEFT);
  frontrightPID.Encoder = readEncoder(FRONT_RIGHT);
  backleftPID.Encoder = readEncoder(BACK_LEFT);
  backrightPID.Encoder = readEncoder(BACK_RIGHT);
  
  /* If we're not moving there is nothing more to do */
  if (!moving){
    /*
    * Reset PIDs once, to prevent startup spikes,
    * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
    * PrevInput is considered a good proxy to detect
    * whether reset has already happened
    */
    if (frontleftPID.PrevInput != 0 || frontrightPID.PrevInput != 0 || backleftPID.PrevInput != 0 || backrightPID.PrevInput != 0) resetPID();
    return;
  }

  /* Compute PID update for each motor */
  doPID(&frontleftPID);
  doPID(&frontrightPID);
  doPID(&backleftPID);
  doPID(&backrightPID);

  /* Set the motor speeds accordingly */
  setMotorSpeeds(frontleftPID.output, frontrightPID.output, backleftPID.output, backrightPID.output);
}

