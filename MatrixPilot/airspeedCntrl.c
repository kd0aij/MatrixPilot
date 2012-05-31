// This file is part of MatrixPilot.
//
//    http://code.google.com/p/gentlenav/
//
// Copyright 2009-2011 MatrixPilot Team
// See the AUTHORS.TXT file for a list of authors of MatrixPilot.
//
// MatrixPilot is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// MatrixPilot is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with MatrixPilot.  If not, see <http://www.gnu.org/licenses/>.


#include "defines.h"

#if(ALTITUDE_GAINS_VARIABLE != 1)

// If mavlink is being used but the gains are not variable
// implement the malink parameter variables for airspeed here
#if(SERIAL_OUTPUT_FORMAT == SERIAL_MAVLINK)
	#include "airspeedCntrl.h"

	int minimum_groundspeed		= MINIMUM_GROUNDSPEED * 100;
	int minimum_airspeed		= MINIMUM_AIRSPEED * 100;
	int maximum_airspeed		= MAXIMUM_AIRSPEED * 100;
	int cruise_airspeed			= CRUISE_AIRSPEED * 100;
	
	int airspeed_pitch_adjust_rate	= (AIRSPEED_PITCH_ADJ_RATE*(RMAX/(57.3 * 40.0)));
	
	int airspeed_pitch_ki_limit	= (AIRSPEED_PITCH_KI_MAX*(RMAX/57.3));
	fractional airspeed_pitch_ki = (AIRSPEED_PITCH_KI * RMAX);
	
	int airspeed_pitch_min_aspd = (AIRSPEED_PITCH_MIN_ASPD*(RMAX/57.3));
	int airspeed_pitch_max_aspd = (AIRSPEED_PITCH_MAX_ASPD*(RMAX/57.3));
#endif	//SERIAL_MAVLINK


#else	//ALTITUDE_GAINS_VARIABLE == 1


#include "airspeedCntrl.h"

extern int desiredSpeed;

int 	airspeed		= 0;
int 	groundspeed		= 0;
int 	airspeedError	= 0;

int minimum_groundspeed		= MINIMUM_GROUNDSPEED * 100;
int minimum_airspeed		= MINIMUM_AIRSPEED * 100;
int maximum_airspeed		= MAXIMUM_AIRSPEED * 100;
int cruise_airspeed			= CRUISE_AIRSPEED * 100;

int airspeed_pitch_adjust_rate	= (AIRSPEED_PITCH_ADJ_RATE*(RMAX/(57.3 * 40.0)));

// Remember last adjustment to limit rate of adjustment.
fractional last_aspd_pitch_adj	= 0;

// Integral of airspeed error
// lower word is underflow.  Upper word is output in degrees.
union longww airspeed_integration = {0};
int airspeed_pitch_ki_limit	= (AIRSPEED_PITCH_KI_MAX*(RMAX/57.3));
fractional airspeed_pitch_ki = (AIRSPEED_PITCH_KI * RMAX);

int airspeed_pitch_min_aspd = (AIRSPEED_PITCH_MIN_ASPD*(RMAX/57.3));
int airspeed_pitch_max_aspd = (AIRSPEED_PITCH_MAX_ASPD*(RMAX/57.3));

// Calculate the airspeed.
// Note that this airspeed is a magnitude regardless of direction.
// It is not a calculation of forward airspeed.
void calc_airspeed(void)
{
	int speed_component ;
	long fwdaspd2;

	speed_component = IMUvelocityx._.W1 - estimatedWind[0] ;
	fwdaspd2 = __builtin_mulss ( speed_component , speed_component ) ;

	speed_component = IMUvelocityy._.W1 - estimatedWind[1] ;
	fwdaspd2 += __builtin_mulss ( speed_component , speed_component ) ;

	speed_component = IMUvelocityz._.W1 - estimatedWind[2] ;
	fwdaspd2 += __builtin_mulss ( speed_component , speed_component ) ;

	airspeed  = sqrt_long(fwdaspd2);
}

// Calculate the groundspeed
void calc_groundspeed(void) // computes (1/2gravity)*( actual_speed^2 - desired_speed^2 )
{
	long gndspd2;

	gndspd2 = __builtin_mulss ( IMUvelocityx._.W1 , IMUvelocityx._.W1 ) ;
	gndspd2 += __builtin_mulss ( IMUvelocityy._.W1 , IMUvelocityy._.W1 ) ;
	gndspd2 += __builtin_mulss ( IMUvelocityz._.W1 , IMUvelocityz._.W1 ) ;

	groundspeed 	= sqrt_long(gndspd2);
}

// Calculate the required airspeed in cm/s.  desiredSpeed is in dm/s
void calc_target_airspeed(void)
{
	union longww accum ;
	accum.WW = __builtin_mulsu ( desiredSpeed , 10 ) ;
	target_airspeed = accum._.W0 ;

	if(groundspeed < minimum_groundspeed)
		target_airspeed += (minimum_groundspeed - groundspeed);

	if(target_airspeed > maximum_airspeed)
		target_airspeed = maximum_airspeed;

	if(target_airspeed < minimum_airspeed)
		target_airspeed = minimum_airspeed;

	//Some airspeed error filtering
	airspeedError = airspeedError >> 1;
	airspeedError += ( (target_airspeed - airspeed) >> 1);

	airspeed_integration.WW += __builtin_mulss( airspeed_pitch_ki, airspeedError ) << 2;

	if(airspeed_integration._.W1 > airspeed_pitch_ki_limit)
		airspeed_integration._.W1 = airspeed_pitch_ki_limit;
	else if(airspeed_integration._.W1 < -airspeed_pitch_ki_limit)
		airspeed_integration._.W1 = -airspeed_pitch_ki_limit;
}

//Calculate and return pitch target adjustment for target airspeed
fractional airspeed_pitch_adjust(void)
{
	union longww accum ;

	// linear interpolation between target airspeed and cruise airspeed.
	// calculating demand airspeed to pitch feedforward
	int aspd_tc_delta = target_airspeed - cruise_airspeed;
	int aspd_tc_range;
	int pitch_range = 0;
	fractional aspd_pitch_adj = 0;
	
	if(aspd_tc_delta > 0)
	{
		aspd_tc_range = maximum_airspeed - cruise_airspeed;
		pitch_range = airspeed_pitch_max_aspd;
	}
	else if(aspd_tc_delta < 0)
	{
		aspd_tc_range = cruise_airspeed - minimum_airspeed;
		pitch_range = airspeed_pitch_min_aspd;
		aspd_tc_delta = -aspd_tc_delta;
	}
	else
	{
		aspd_tc_range = 1;
	}

	accum.WW = 0;
	accum._.W1 = aspd_tc_delta;
	accum._.W1 = __builtin_divsd( accum.WW >> 2,  aspd_tc_range );
	accum.WW = __builtin_mulss( accum._.W1, pitch_range ) << 2;
	aspd_pitch_adj = accum._.W1;

	aspd_pitch_adj -= airspeed_integration._.W1;

	// Pitch adjust for airspeed on glide only.
	if(throttle_control >= 100)
	{
		aspd_pitch_adj = 0;
		airspeed_integration.WW = 0;
	}

	// limit the rate of the airspeed pitch adjustment
	if(aspd_pitch_adj > last_aspd_pitch_adj)
	{
		if( (last_aspd_pitch_adj + airspeed_pitch_adjust_rate) < aspd_pitch_adj)
			aspd_pitch_adj = (last_aspd_pitch_adj + airspeed_pitch_adjust_rate);
	}
	else
	{
		if( (last_aspd_pitch_adj - airspeed_pitch_adjust_rate) > aspd_pitch_adj)
			aspd_pitch_adj = (last_aspd_pitch_adj - airspeed_pitch_adjust_rate);
	}

	last_aspd_pitch_adj = aspd_pitch_adj;

	return aspd_pitch_adj;
}


#endif		//(ALTITUDE_GAINS_VARIABLE == 1)
