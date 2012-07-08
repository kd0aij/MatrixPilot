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

// FLY BY WIRE module
// Responsible for turning user control demands into actions
// roll control -> roll angle demand + turn rate demand
// pitch control + camber control -> airspeed demand


#include "defines.h"
#include "inputCntrl.h"
#include "fbw_options.h"
#include "airframe.h"

#if(USE_FBW == 1)

// Interpolate between two input points X1,Y1 and X2,Y2 where the input value is
// between X1 and X2.
// NOTE: X2 MUST BE LARGER THAN X1.
fractional interpolate(fractional input, fractional X1, fractional Y1, fractional X2, fractional Y2)
{
	fractional 		output;
	fractional 		delta;
	union longww 	ltemp;

	int 			gain = 0;

	input -= X1;
	if(X2 <= X1) return Y1;
	if(Y1 == Y2) return Y1;

	delta = X2 - X1;

	// Find the gain required to increase delta to be in range RMAX to RMAX/2
	while(delta < RMAX/2)
	{
		gain++;
		delta <<= 1;
	}

	if(delta > RMAX)
	{
		gain--;
		delta >>= 1;
	}

	ltemp.WW = 0;
	ltemp._.W1 = (Y2 - Y1);		//  does this need to be inverted???


	// Limit numerator to +-RMAX/4 and adjust gain
	if(ltemp.WW > 0)
	{
		while(ltemp._.W1 > RMAX/4)
		{
			gain++;
			ltemp.WW >>= 1;
		}
	}
	else
	{
		while(ltemp._.W1 < -RMAX/4)
		{
			gain++;
			ltemp.WW >>= 1;
		}
	}

	output = __builtin_divsd( ltemp.WW,  delta ); //(int) (fractional)

	
	ltemp.WW = __builtin_mulss(output, input);
	ltemp.WW <<= gain;
	output = (fractional) (ltemp._.W1 + Y1);

	return output;
}

int find_aero_data_index_for_ref_input(aero_condition_point* pCondList, int maxConds, fractional input)
{
	if(input < pCondList[0].condition_point) return -1;
	int index;
	int condLo, condHi;

	for(index=0; index < (maxConds-1); index++)
	{
		condLo = pCondList[index].condition_point;
		condHi = pCondList[index+1].condition_point;
		if( (input >= condLo) && 
			(input <  condHi) ) 
				return index;
	}
	return maxConds;
}


// Fly by wire demand control.  Turns user input into demand.
void fbwDemandCntrl( void )
{
	int index = find_aero_data_index_for_ref_input(camber_aero_data, camber_aero_datapoints, in_cntrls[IN_CNTRL_CAMBER]);
	int aspd = 0;

	if(index == -1)
	{
		aspd = camber_aero_data[0].data_point.airspeed_cruise;		
	}
	else if(index == camber_aero_datapoints)
	{
		aspd = camber_aero_data[camber_aero_datapoints].data_point.airspeed_cruise;
	}
	else
	{
		aspd = interpolate(in_cntrls[IN_CNTRL_CAMBER],
							camber_aero_data[index].condition_point,
							camber_aero_data[index].data_point.airspeed_cruise,
							camber_aero_data[index+1].condition_point,
							camber_aero_data[index+1].data_point.airspeed_cruise);
	}

	union longww temp ;
	temp.WW = __builtin_mulss(aspd , (RMAX * 0.1) );
	temp.WW <<= 2;
	desiredSpeed = temp._.W1;
}

#endif	//#if(USE_FBW == 1)
