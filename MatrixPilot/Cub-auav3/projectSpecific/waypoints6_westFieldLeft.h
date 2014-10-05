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

////////////////////////////////////////////////////////////////////////////////
// Waypoint handling

// Move on to the next waypoint when getting within this distance of the current goal (in meters)
#define WAYPOINT_RADIUS 		10

// AAM West Field runway center  39°50'31.97"N  105°13'10.17"W (105.2194917, 39.842213889)
#define USE_FIXED_ORIGIN		1
#define FIXED_ORIGIN_LOCATION	{ -1052194917, 398422138, 1817.0 }

////////////////////////////////////////////////////////////////////////////////
// This is a lefthand pattern for takeoff to the east

#define USE_FIXED_ORIGIN		1
const struct waypointDef waypoints[] = {
	{ { 84, 3, 20 } , F_NORMAL + F_TAKEOFF, CAM_VIEW_LAUNCH } , //Waypoint 1
	{ { 83, 49, 30 } , F_NORMAL , CAM_VIEW_LAUNCH } , //Waypoint 2
	{ { -103, 53, 30 } , F_NORMAL , CAM_VIEW_LAUNCH } , //Waypoint 3
	{ { -103, 7, 25 } , F_NORMAL , CAM_VIEW_LAUNCH } , //Waypoint 4
	{ { -14, 4, 20 } , F_NORMAL , CAM_VIEW_LAUNCH } , //Waypoint 5
	{ { 15, 3, 20 } , F_NORMAL + F_TRIGGER , CAM_VIEW_LAUNCH } , //Waypoint 6
};


const struct waypointDef rtlWaypoints[] = {
		{ { 0, 30,  30 } , F_LAND, CAM_VIEW_LAUNCH }
} ;
