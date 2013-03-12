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


#include "../MatrixPilot/defines.h"
#include "fbw_options.h"
#include "airframe.h"
//#include "airspeedCntrlFBW.h"
#include "../libDCM/libDCM.h"
#include "minifloat.h"
#include "inputCntrl.h"		// For limitRMAX()

#define AFRM_MULT1		2.0
#define AFRM_MULT2		128.0

#define INVERSE_GLIDE_RATIO (RMAX / CRUISE_GLIDE_RATIO)

#define AFRM_EFFECTIVE_AREA (AFRM_WING_AREA * AFRM_EFFECTIVE_AREA_RATIO)



// This constant is directly related to wing loading so has limited dynamic range
#define AFRM_CL_CALC_CONST	(2.0 * AFRM_AIRCRAFT_MASS / (AFRM_AIR_DENSITY * AFRM_EFFECTIVE_AREA))

// Adjust the calcualtion constant from g to m/s**2 and to RMAX with MULT1 scaling
//#define AFRM_CL_CALC_CONST_SCALED (RMAX * (AFRM_MULT1 * AFRM_CL_CALC_CONST * AFRM_GRAVITY / GRAVITY))

//#define AFRM_ACCN_CALC_CONST_SCALED (RMAX / ( AFRM_CL_CALC_CONST * AFRM_GRAVITY ))

// CL calculation constant for minifloat based calculation only
#define AFRM_CL_CALC_CONST_G (AFRM_CL_CALC_CONST * AFRM_GRAVITY)

//  Effective tail volume for calculating elevator Cl
#define AFRM_TAIL_VOLUME	((AFRM_TAIL_AREA/AFRM_WING_AREA) * (AFRM_TAIL_MOMENT / AFRM_MAC))

//  Inverse tail volume for easier calculation
#define AFRM_INV_TAIL_VOLUME	(1.0 / AFRM_TAIL_VOLUME)

// Constant used to calculate requried rudder Cl
// This is false since it uses aircraft mass as a scaling.
// It should use rotational moments
#define AFRM_RUDD_CL_CALC_CONST (2.0 * AFRM_AIRCRAFT_MASS / (AFRM_AIR_DENSITY * AFRM_FIN_AREA * AFRM_FIN_MOMENT))

int successive_interpolation(int X, int X1, int X2, int Y1, int Y2);
_Q16 successive_interpolation_Q16(_Q16 X, _Q16 X1, _Q16 X2, _Q16 Y1, _Q16 Y2);


minifloat afrm_aspdcm_to_m(int airspeedCm)
{
	minifloat aspdmf = ltomf(airspeedCm);
	minifloat tempmf = {164, -6};		// 0.01
	aspdmf = mf_mult(aspdmf, tempmf);
	return aspdmf;
}


// Get the required lift coefficient for the airspeed and load
// airspeed in cm/s
// load in GRAVITY scale
// Calculates Cl=2*m*g*load / (p * A * V^2)
// p=air density
// A = wing area
// m = mass, g = gravity constant
//  
minifloat afrm_get_required_Cl_mf(int airspeed, minifloat load)
{
	minifloat Clmf = {0,0};
	union longww temp;

	// If airspeed is lower than 1m/s then don't try this calculation
	if(airspeed < 100) return Clmf;

	minifloat aspdmf = afrm_aspdcm_to_m(airspeed);
	minifloat aspd2mf = mf_mult(aspdmf, aspdmf);		// Airspeed^2

	temp.WW = (long) (AFRM_CL_CALC_CONST_G * AFRM_Q16_SCALE);
	minifloat constmf = Q16tomf(temp.WW);

	Clmf = mf_mult(load, constmf);		// load * Cl calc constant
		
	Clmf = mf_div(Clmf, aspd2mf);			// Cl = load * Cl calc constant / aispeed^2

	return Clmf;
}


//  NOT TESTED
minifloat afrm_get_required_alpha_mf(int airspeed, minifloat Clmf)
{
	_Q16 temp;
	minifloat mf;

	temp = mftoQ16(Clmf);

	temp =  
		successive_interpolation_Q16(temp, 
			normal_polars[0].points[0].Cl, 
			normal_polars[0].points[1].Cl, 
			normal_polars[0].points[0].alpha, 
			normal_polars[0].points[1].alpha);

	mf = Q16tomf(temp);

	return mf;
}


minifloat afrm_get_required_alpha(int airspeed, minifloat Cl)
{
	union longww temp;
	temp.WW = mftoQ16(Cl);

	temp.WW = successive_interpolation_Q16(temp.WW, 
			normal_polars[0].points[0].Cl, 
			normal_polars[0].points[1].Cl, 
			normal_polars[0].points[0].alpha, 
			normal_polars[0].points[1].alpha);
	
	return Q16tomf(temp.WW);
}

// Tail coefficient of lift = Wing Cm / effective tail volume
// wing_aoa in degrees
minifloat afrm_get_tail_required_Cl_mf(minifloat wing_aoa)
{
	_Q16 tempQ16;
	minifloat Cm_mf;
	minifloat mf_temp;

	tempQ16 = mftoQ16(wing_aoa);

	// Find wing Cm
	tempQ16 = successive_interpolation_Q16(tempQ16, 
			normal_polars[0].points[0].alpha, 
			normal_polars[0].points[1].alpha, 
			normal_polars[0].points[0].Cm, 
			normal_polars[0].points[1].Cm);

	Cm_mf = Q16tomf(tempQ16);
	
	mf_temp = Q16tomf(AFRM_Q16_SCALE * AFRM_INV_TAIL_VOLUME);

	// Multiply wing Cm by 1 / tail volume
	mf_temp = mf_mult(mf_temp, Cm_mf);

	return mf_temp;
}

minifloat afrm_get_tail_required_alpha(minifloat Clmf_tail)
{
	union longww temp;

	// Scale Cl to alpha
	minifloat mf = {160, 4}; // 10.0 degrees at Cl = 1;
	mf = mf_mult(mf, Clmf_tail);

	return mf;
}


// return the RMAX scale control requried for an required elevator pitch
fractional lookup_elevator_control( minifloat pitch )
{
	_Q16 elev_pitch = mftoQ16(pitch);

	int index;
	// Make sure that if the angle is out of bounds then the limits are returned
	if(elev_pitch < elevator_angles[0].surface_deflection)
		return elevator_angles[0].ap_control;

	if(elev_pitch > elevator_angles[elevator_angle_points - 1].surface_deflection)
		return elevator_angles[elevator_angle_points - 1].ap_control;

	index = elevator_angle_points - 1;
	while(elev_pitch < elevator_angles[index - 1].surface_deflection)
	{
		index--;
	}

	return successive_interpolation_Q16(elev_pitch, 
			elevator_angles[index-1].surface_deflection,
			elevator_angles[index].surface_deflection, 
			elevator_angles[index-1].ap_control,
			elevator_angles[index].ap_control);
}


// Find the Cl required from the rudder for the yaw moment
// Yaw moment in Nm
// airspeed in cm/s
// Moment = force * length of moment arm
// Force = Cl * p * A / 
minifloat afrm_get_rudd_required_Cl(int airspeed, minifloat yaw_moment)
{
	minifloat Clmf = {0,0};
	union longww temp;

	// If airspeed is lower than 1m/s then don't try this calculation
	if(airspeed < 100) return Clmf;

	minifloat aspdmf = afrm_aspdcm_to_m(airspeed);
	minifloat aspd2mf = mf_mult(aspdmf, aspdmf);		// Airspeed^2

	temp.WW = (long) (AFRM_RUDD_CL_CALC_CONST * AFRM_Q16_SCALE);
	minifloat constmf = Q16tomf(temp.WW);

	Clmf = mf_mult(yaw_moment, constmf);		// load * Cl calc constant
		
	Clmf = mf_div(Clmf, aspd2mf);			// Cl = load * Cl calc constant / aispeed^2

	return Clmf;
}

// Convert rudder aoa into rudder command
fractional lookup_rudder_control( minifloat aoa )
{
	_Q16 rudd_angle = mftoQ16(aoa);

	int index;
	// Make sure that if the angle is out of bounds then the limits are returned
	if(rudd_angle < rudder_angles[0].surface_deflection)
		return rudder_angles[0].ap_control;

	if(rudd_angle > rudder_angles[rudder_angle_points - 1].surface_deflection)
		return rudder_angles[elevator_angle_points - 1].ap_control;

	index = rudder_angle_points - 1;
	while(rudd_angle < rudder_angles[index - 1].surface_deflection)
	{
		index--;
	}

	return successive_interpolation_Q16(rudd_angle, 
			rudder_angles[index-1].surface_deflection,
			rudder_angles[index].surface_deflection, 
			rudder_angles[index-1].ap_control,
			rudder_angles[index].ap_control);
}


// Calculate the expected descent rate in cm/s at the given airspeed in cm/s
// if the aircraft were gliding.  This is a measure of expected energy loss.
// It is also known as Cl/Cd or the Lift/Drag coefficent.
// Output is positive for descending.
int expected_glide_descent_rate(int airspeed)
{
	// Get the expected descent rate
	union longww temp;
	temp.WW = __builtin_muluu( air_speed_3DIMU, INVERSE_GLIDE_RATIO);
	temp.WW <<= 2;
	return temp._.W1;
}

// Calculate the expected climb rate depending on a throttle setting and airspeed
// glide_ratio.  rate and airspeed in cm/s.  Descent rate is positive falling.
// Returned value is positive rising in cm/s
int feedforward_climb_rate(fractional throttle, int glide_descent_rate, int airspeed)
{
	// TODO - Correct the assumption that scale changes with descent rate.
	int rateScale = (MAX_THROTTLE_CLIMB_RATE * 100) + glide_descent_rate;

	union longww temp;
	temp.WW = __builtin_mulss( throttle, rateScale);
	temp.WW <<= 2;
	temp._.W1 -= glide_descent_rate;
	return temp._.W1;
}



// Airframe data interpolation funcitons

int successive_interpolation(int X, int X1, int X2, int Y1, int Y2)
{
	int X1temp = X1;
	int X2temp = X2;
	int Y1temp = Y1;
	int Y2temp = Y2;
	int Xtemp;

	// Test for out of limit.  Return limit if so.
	if( (X2-X1) > 0)
	{
		if(X > X2) return Y2;
		if(X < X1) return Y1;
	}
	else
	{
		if(X < X2) return Y2;
		if(X > X1) return Y1;
	}

	// Repeat approximation until magnitude difference between X estiamtes is <= 1
	while( ((X2temp - X1temp) >> 1) != 0)
	{ 
		int deltaX = (X2temp - X1temp) >> 1;
		int deltaY = (Y2temp - Y1temp) >> 1;
		Xtemp  = X - X1temp;

		if(deltaX > 0)
		{
			if(Xtemp > deltaX)
			{
				X1temp += deltaX;
				Y1temp += deltaY;
			}
			else
			{
				X2temp -= deltaX;
				Y2temp -= deltaY;
			}
		}
		else
		{
			if(Xtemp < deltaX)
			{
				X1temp -= deltaX;
				Y1temp -= deltaY;
			}
			else
			{
				X2temp += deltaX;
				Y2temp += deltaY;
			}
		}

	}

	// Last selection is when |X1-X2| <= 1
	if(X == X1temp)
		return Y1temp;
	else
		return Y2temp;
}


// Succesive interpolation between X inputs of Y outputs
_Q16 successive_interpolation_Q16(_Q16 X, _Q16 X1, _Q16 X2, _Q16 Y1, _Q16 Y2)
{
	_Q16 X1temp = X1;
	_Q16 X2temp = X2;
	_Q16 Y1temp = Y1;
	_Q16 Y2temp = Y2;
	_Q16 Xtemp;

	// Test for out of limit.  Return limit if so.
	if( (X2-X1) > 0)
	{
		if(X > X2) return Y2;
		if(X < X1) return Y1;
	}
	else
	{
		if(X < X2) return Y2;
		if(X > X1) return Y1;
	}

	// Repeat approximation until magnitude difference between X estiamtes is <= 1
	while( ((X2temp - X1temp) >> 1) != 0)
	{ 
		_Q16 deltaX = (X2temp - X1temp) >> 1;
		_Q16 deltaY = (Y2temp - Y1temp) >> 1;
		Xtemp  = X - X1temp;

		if(deltaX > 0)
		{
			if(Xtemp > deltaX)
			{
				X1temp += deltaX;
				Y1temp += deltaY;
			}
			else
			{
				X2temp -= deltaX;
				Y2temp -= deltaY;
			}
		}
		else
		{
			if(Xtemp < deltaX)
			{
				X1temp -= deltaX;
				Y1temp -= deltaY;
			}
			else
			{
				X2temp += deltaX;
				Y2temp += deltaY;
			}
		}

	}

	// Last selection is when |X1-X2| <= 1
	if(X == X1temp)
		return Y1temp;
	else
		return Y2temp;
}
