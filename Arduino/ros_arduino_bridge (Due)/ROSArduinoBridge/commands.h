/* Define single-letter commands that will be sent by the PC over the
   serial link.
*/

#ifndef COMMANDS_H
#define COMMANDS_H

#define ROT_VECT        'v'
#define GYRO            'g'
#define LIN_ACCEL       'a'

#define READ_ENCODERS   'e'
#define MOTOR_SPEEDS    'm'
#define MOTOR_RAW_PWM   'o'
#define RESET_ENCODERS  'r'
#define UPDATE_PID      'u'

#define FRONT_LEFT      0
#define FRONT_RIGHT     1
#define BACK_LEFT       2
#define BACK_RIGHT      3

#endif


