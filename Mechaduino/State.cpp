//Contains the declaration of the state variables for the control loop
#include "State.h"
#include "Configuration.h"
#include "Configurationals.h"
#include "math.h"


//---- interrupt vars ----
volatile int r = 0;                 //target angle
volatile int y = 0;                //current angle

volatile int u = 0;                 // control effort

volatile bool dir = true;           // flag for dir setting
volatile bool enabled = true;       // flag for enabled setting
volatile bool response = false;   // flag for step response

volatile bool frequency_test = false;        // flag for frequency test

const int pLPFa = (1000 * exp(pLPF * -2 * 3.14159283 / FPID)); // z = e^st pole mapping
const int pLPFb = (1000 - pLPFa);

const int positionLPFa = (1000 * exp(positionLPF * -2 * 3.14159283 / FPID)); // z = e^st pole mapping
const int positionLPFb = (1000 - positionLPFa);


const int uLPFa = (1000 * exp(uLPF * -2 * 3.14159283 / FPID)); // z = e^st pole mapping
const int uLPFb = (1000 - uLPFa);

const int RASa = (1000 * exp((1000 / RAS) * -2 * 3.14159283 / FPID)); // z = e^st pole mapping
const int RASb = (1000 - RASa);


volatile int int_Kp = Kp * 1000;
volatile int int_Ki = Ki * 1000;
volatile int int_Kd = Kd * 1000;

volatile int int_Kvff = Kvff * 1000;

volatile int int_Kfr = Kfr * 1000;


// calculated from F = m*a
// calculating the moment from the Force with M = F * r 
// r = mm_rev / (2*Pi)
// calculating omega_dot from a 
// a = omega_dot * mm/rad  --> mm/rad = mm_rev /2

int J_load = (mass_load * mm_rev ^ 2) / (10 * 10 * 4 * 3.14159283);

// calculating the necessary current 
// u = M / (Nm/mA) 
// u = M * mA_M_max / M_max
// 10000000 to fit the units
// 1000 to get a int from the float
volatile int int_J = 1000 * (((J + J_load) * A_max) / (10000000 * M_max));

// variable for the target in steps
volatile int step_target = 0;      // target as step gets incremented if an step is received
