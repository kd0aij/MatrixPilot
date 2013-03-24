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
#include "airframe_options.h"
#include "airframe.h"
#include "inputCntrl.h"

aero_condition_point camber_aero_data[] = 
	{
	{-RMAX,	{1000, 2000, 1200, 0}},
	{0,		{1200, 3000, 1500, 0}},
	{RMAX,	{1500, 5000, 2500, 0}},
	};

int camber_aero_datapoints = (sizeof(camber_aero_data) / sizeof(aero_condition_point));


polar normal_polars[AFRM_OPT_POLARS_PER_CONDITION] =
	{
		{ 1300,	{	{ -2 * AFRM_Q16_SCALE, AFRM_Q16_SCALE * 0,		AFRM_Q16_SCALE * 0.008,	AFRM_Q16_SCALE * -0.06},
					{ 10 * AFRM_Q16_SCALE, AFRM_Q16_SCALE * 1.15, 	AFRM_Q16_SCALE * 0.05, 	AFRM_Q16_SCALE * -0.02}
				},			
		},
	};


// Table of elevator deflection vs RMAX scaled control input
// Positive angle is elevator deflection up, not aircraft pitch up
// Should always be negative to positive for both control and angle
control_surface_angle elevator_angles[] = 
	{
		{-RMAX	, AFRM_Q16_SCALE * -11.5 },
		{0 		, AFRM_Q16_SCALE * -1.5 },
		{RMAX	, AFRM_Q16_SCALE * 8.5 }
	};

int elevator_angle_points = (sizeof(elevator_angles) / sizeof(control_surface_angle));


control_surface_angle rudder_angles[] = 
	{
		{-RMAX	, AFRM_Q16_SCALE * -10.0 },
		{0 		, AFRM_Q16_SCALE * 0 },
		{RMAX	, AFRM_Q16_SCALE * 10.0 }
	};

int rudder_angle_points = (sizeof(rudder_angles) / sizeof(control_surface_angle));

// Decribes delta aileron angles.  Will be replaced by polars
extern control_surface_angle aileron_angles[];
extern int ;

control_surface_angle aileron_angles[] = 
	{
		{-RMAX	, AFRM_Q16_SCALE * -15.0 },
		{0 		, AFRM_Q16_SCALE * 0 },
		{RMAX	, AFRM_Q16_SCALE * 15.0 }
	};

int aileron_angle_points = (sizeof(aileron_angles) / sizeof(control_surface_angle));


// Decribes flap angles
control_surface_angle flap_angles[] =
	{
		{-RMAX	, AFRM_Q16_SCALE * 5.0 },
		{0 	, AFRM_Q16_SCALE * 0 },
		{RMAX/2	, AFRM_Q16_SCALE * 5.0 },
		{RMAX	, AFRM_Q16_SCALE * 15.0 }
	};


int flap_angle_points = (sizeof(flap_angles) / sizeof(control_surface_angle));
