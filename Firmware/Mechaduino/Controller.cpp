//Contains TC5 Controller definition
//The main control loop is executed by the TC5 timer interrupt:

#include "State.h"
#include "Utils.h"
#include "A4954.h"
#include "board.h"
#include "Configuration.h"
#include "Configurationals.h"
#include "Encoder.h"
#include "Serial.h"



const int16_t u_LPF = 1000;
const int16_t u_LPFa = ((2048.0 * exp(u_LPF * -2.0 * 3.14159283 / FPID)) + 0.5); // z = e^st pole mapping
const int16_t u_LPFb = 2048 - u_LPFa;


// ----- gets called with FPID -----
// ----- calculates the target velocity and PID settings -----
void TC5_Handler() {

 // static int32_t t_1 = micros();
 // int32_t t;
 // int32_t dt;

  int32_t domega_target;

  //----- Variables -----
  static int32_t ITerm;
  static int32_t DTerm;

  static int32_t e_1;        // last error term
  static int32_t omega_target_1;

  int16_t phase_advanced;

  int32_t u_pid;
  int32_t u_ff;
  int32_t u_acc;

  static uint16_t serial_loop_counter;


  //----- Calculations -----

  if (TC5->COUNT16.INTFLAG.bit.OVF == 1  || frequency_test == true) {  // A overflow caused the interrupt
    serial_loop_counter++;

    domega_target = omega_target - omega_target_1;

    // start the calculations only if the motor is enabled
    if (enabled) {

      // ----------- PID loop -----------------------------------------
      // calculate the I-Term
      if (abs(u) < uMAX) {
        ITerm = ITerm + (int_Ki * error);
      }

      // constrain the ITerm
      if (ITerm > ITerm_max) {
        ITerm = ITerm_max;
      }
      else if (ITerm < -ITerm_max) {
        ITerm = -ITerm_max;
      }

      //calculate the D-Term
      DTerm = int_Kd * (error - e_1);

      // calculate the whole PID effort
      u_pid = ((int_Kp * error) + ITerm + DTerm) / 1024;


      // calculate the feedforward effort
      u_ff = (int_Kff * omega_target) / (10000 * 1024);


      // calculate the acceleration effort
      u_acc = (int_Kacc * domega_target) / (10000 * 1024);



      // ------ Add up the Effort -------------------------------------
      //u = u_pid + u_ff + u_acc;
      // lowpass filter with 1000 Hz cutoff

      u = ((u_LPFa * u) + (u_LPFb * (u_pid + u_ff + u_acc))) / 2048;


    }
    else {
      omega_target = 0;
      step_target = y / stepangle;
      u = 0;
      ITerm = 0;
    }


    // constrain the effort to the user spedified maximum
    if (u > uMAX) {
      u = uMAX;
      error_register = error_register | 0B0000000000000100;    // log error in register
    }
    else if (u < -uMAX) {
      u = -uMAX;
      error_register = error_register | 0B0000000000000100;    // log error in register
    }


    // calculate the phase advance term
    phase_advanced = (sign(u) * PA) + (omega / FPID);

    if (phase_advanced >= PA) {
      phase_advanced = PA;
    }
    else if (phase_advanced <= -PA) {
      phase_advanced = -PA;
    }


    // calculate the electrical angle for the motor coils
    electric_angle = y + phase_advanced;


    // write the output
    output(electric_angle, u);


    // buffer the variables for the next loop
    e_1 = error;
    omega_target_1 = omega_target;


    if (abs(error) > max_e) {
      // set error register if the error was to high at some point
      error_register = error_register | 0B0000000000000010;
    }


    if (streaming) {

      if (serial_loop_counter >= max_serial_counter) {

        fifo_counter++;

        byte fifo_position = mod(fifo_counter, 49);

        fifo[0][fifo_position] = (1000000 * max_serial_counter) / FPID;
        fifo[1][fifo_position] = r;
        fifo[2][fifo_position] = y;
        fifo[3][fifo_position] = error;
        fifo[4][fifo_position] = omega_target;
        fifo[5][fifo_position] = omega;
        fifo[6][fifo_position] = u;
        fifo[7][fifo_position] = electric_angle;

        serial_loop_counter = 0;
      }
    }

    TC5->COUNT16.INTFLAG.bit.OVF = 1;    // writing a one clears the flag ovf flag
  }
}








// ----- reads the current shaft angle with 2*FPID -----
// ----- Oversamples the shaft angle to reduce noise -----
void TC4_Handler() {

  static int32_t y_temp;

  static int32_t r_1;
  static int32_t y_1;

  static uint16_t o_target_counter;
  static uint16_t y_counter;

  if (TC4->COUNT16.INTFLAG.bit.OVF == 1) {

    o_target_counter++;
    y_counter ++;

    // read the current angle
    y_temp = y_temp + readAngle();


    if (y_counter >= 4) {

      // calculate the current angle
      y = y_temp / y_counter;


      // calculate the current velocity
      omega = (omega + ((y - y_1) * (Fs / y_counter)) ) / 2;


      // buffer the variables for the next loop
      y_temp = 0;
      y_counter = 0;
      y_1 = y;
    }


    // calculate the current target from the received steps and the angle per step
    r = step_target * stepangle;


    //calculate the error
    error = ((error_LPFa * error) + (error_LPFb * (r - y))) / 2048;


    //calculate the target velocity
    if (r != r_1) {
      omega_target = (omega_target + ((r - r_1) * (Fs / o_target_counter) )) / 2;

      o_target_counter = 0;

      // buffer the variables for the next loop
      r_1 = r;
    }



    TC4->COUNT16.INTFLAG.bit.OVF = 1;    // writing a one clears the flag ovf flag
  }
}
