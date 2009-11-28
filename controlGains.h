// control gains.
// YAWKP is the proportional feedback gain for the rudder
// YAWKD is the yaw gyro feedback gain for the rudder
// YAWBOOST is the additional gain multiplier for the manually commanded rudder deflection
// PITCHGAIN is the pitch proportional feedback gain for the elevator
// RUDDERELEVMIX is the degree of elevator adjustment for rudder and banking

// All gains should be positive real numbers.
// Typical values for the red board are:
//#define YAWKP 0.0625
//#define YAWKD 0.75
//#define YAWBOOST 1.0
//#define PITCHGAIN 0.25
//#define RUDDERELEVMIX 1.0

// maximum allowable values for the gains are 2.0

// experiment with these values to fine tune the performance of the controls in your plane

//#define YAWKP 0.0625
#define YAWKP 0.100
#define YAWKD 0.75
#define YAWBOOST 1.0
#define PITCHGAIN 0.250
#define RUDDERELEVMIX 1.0

// return to launch pitch down in degrees, a real number.
// this is the real angle in degrees that the nose of the plane will pitch downward during a return to launch.
// it is used to increase speed (and wind penetration) during a return to launch.
// set it to zero if you do not want to use this feature.

//#define RTLPITCHDOWN 2.0

#define RTLPITCHDOWN 0.0


// the real number SERVOSAT limits servo throw by controlling pulse width saturation.
// set it to 1.0 if you want full servo throw, otherwise set it to the portion that you want

#define SERVOSAT 1.0

// the following define is used to test the above gains and parameters.
// if you define TestGains, there functions will be enabled, even without GPS or Tx turned on.

//#define TestGains		// uncomment this line if you want to test your gains without using GPS



