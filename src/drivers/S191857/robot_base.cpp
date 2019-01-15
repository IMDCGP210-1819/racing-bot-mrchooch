/***************************************************************************

    file                 : robot_base.cpp
    created              : Mon 13 Feb 11:40:23 GMT 2017
    copyright            : (C) 2002 Author

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#ifdef _WIN32
#include <windows.h>
#endif

#include <stdio.h>
#include <stdlib.h> 
#include <string.h> 
#include <math.h>

#include <tgf.h> 
#include <track.h> 
#include <car.h> 
#include <raceman.h> 
#include <robottools.h>
#include <robot.h>

static tTrack	*curTrack;

static void initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s); 
static void newrace(int index, tCarElt* car, tSituation *s); 
static void drive(int index, tCarElt* car, tSituation *s); 
static void endrace(int index, tCarElt *car, tSituation *s);
static void shutdown(int index);
static int  InitFuncPt(int index, void *pt); 
static int getGear(tCarElt *car);
static float getAllowedSpeed(tTrackSeg *segment);
static float getAccel(tCarElt* car);
static float getDistToSegEnd(tCarElt* car);
static float getBrake(tCarElt* car);

const float gravity = 9.81;

/* 
 * Module entry point  
 */ 
extern "C" int 
S191857(tModInfo *modInfo)
{
    memset(modInfo, 0, 10*sizeof(tModInfo));

    modInfo->name    = strdup("S191857");		/* name of the module (short) */
    modInfo->desc    = strdup("");	/* description of the module (can be long) */
    modInfo->fctInit = InitFuncPt;		/* init function */
    modInfo->gfId    = ROB_IDENT;		/* supported framework version */
    modInfo->index   = 1;

    return 0; 
} 

/* Module interface initialization. */
static int 
InitFuncPt(int index, void *pt) 
{ 
    tRobotItf *itf  = (tRobotItf *)pt; 

    itf->rbNewTrack = initTrack; /* Give the robot the track view called */ 
				 /* for every track change or new race */ 
    itf->rbNewRace  = newrace; 	 /* Start a new race */
    itf->rbDrive    = drive;	 /* Drive during race */
    itf->rbPitCmd   = NULL;
    itf->rbEndRace  = endrace;	 /* End of the current race */
    itf->rbShutdown = shutdown;	 /* Called before the module is unloaded */
    itf->index      = index; 	 /* Index used if multiple interfaces */
    return 0; 
} 

/* Called for every track change or new race. */
static void  
initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s) 
{ 
    curTrack = track;
    *carParmHandle = NULL; 
} 

/* Start a new race. */
static void  
newrace(int index, tCarElt* car, tSituation *s) 
{ 
} 

/* Drive during race. */
static void  
drive(int index, tCarElt* car, tSituation *s) 
{ 
    memset((void *)&car->ctrl, 0, sizeof(tCarCtrl)); 

	float angle = RtTrackSideTgAngleL(&(car->_trkPos)) - car->_yaw; //Subtract road angle from steering angle
	NORM_PI_PI(angle); // Normalise it
	angle -= 1.0 * car->_trkPos.toMiddle / car->_trkPos.seg->width; //Steer car towards middle if not near middle

	// Set car variables
	car->ctrl.steer = angle / car->_steerLock;
	car->ctrl.gear = getGear(car);
	car->ctrl.brakeCmd = getBrake(car);
	if (car->ctrl.brakeCmd == 0.0) {
		car->ctrl.accelCmd = getAccel(car);
	}
	else {
		car->ctrl.accelCmd = 0.0;
	}
}

/* Get the correct gear */
static int getGear(tCarElt *car)
{
	
	if (car->_gear <= 0) return 1; //If in neutral or reverse, got to gear 1
	
	float omega = car->_enginerpmRedLine / (car->_gearRatio[car->_gear + car->_gearOffset]); //Used to calculate if gear needs to go up

	//If faster than the maximum speed for the current gear, go up a gear
	if (car->_speed_x > (omega * (car->_wheelRadius(2)) *  0.9)) {
		return car->_gear + 1;
	}
	else {
		omega = car->_enginerpmRedLine / (car->_gearRatio[car->_gear + car->_gearOffset - 1]); //Used to calculate if gear needs to go down

		//If above gear 1 and speed is significantly lower than the maximum speed for the current gear, go down a gear
		if (car->_gear > 1 && car->_speed_x + 4 < (omega * (car->_wheelRadius(2)) *  0.9)) {
			return car->_gear - 1;
		}
	}
	//If speed is fine, stay in current gear
	return car->_gear;
}

/* Get the allowed speed on a segment */
static float getAllowedSpeed(tTrackSeg *segment)
{
	//If the track is straight, go full speed
	if (segment->type == TR_STR) {
		return FLT_MAX;
	}
	else {
		//If not, calculate a safe speed to go
		float mu = segment->surface->kFriction;
		return sqrt(mu*gravity*segment->radius);
	}
}

/* Get the distance to the end of the segment */
static float getDistToSegEnd(tCarElt* car)
{
	//If the track is straight, the distance is length of the track minus how far into it the car has gone
	if (car->_trkPos.seg->type == TR_STR) {
		return car->_trkPos.seg->length - car->_trkPos.toStart;
	}
	else {
		//If the track is not straight, do maths to calculate the distance left to go
		return (car->_trkPos.seg->arc - car->_trkPos.toStart)*car->_trkPos.seg->radius;
	}
}

/* Get appropriate acceleration */
static float getAccel(tCarElt* car)
{
	float allowedspeed = getAllowedSpeed(car->_trkPos.seg);

	//Accelerate if car is below max safe speed
	if (car->_speed_x + 1.0 < allowedspeed) {
		return 1.0;
	}
	else {
		return allowedspeed / car->_wheelRadius(REAR_RGT)* (car->_gearRatio[car->_gear + car->_gearOffset]) / (car->_enginerpmRedLine);
	}
}

/* Get if car should brake */
static float getBrake(tCarElt* car)
{
	tTrackSeg *segptr = car->_trkPos.seg;
	float currentspeedsqr = car->_speed_x*car->_speed_x;
	float mu = segptr->surface->kFriction;
	float maxlookaheaddist = currentspeedsqr / (2.0*mu*gravity);
	float lookaheaddist = getDistToSegEnd(car);
	float allowedspeed = getAllowedSpeed(segptr);

	//If car is going too fast
	if (car->_speed_x > allowedspeed) return 1.0;
	segptr = segptr->next;
	while (lookaheaddist < maxlookaheaddist) {
		allowedspeed = getAllowedSpeed(segptr);
		//If car is still going too fast
		if (car->_speed_x > allowedspeed) {
			float brakedist = (currentspeedsqr - (allowedspeed * allowedspeed)) / (2.0*mu*gravity);
			if (brakedist > lookaheaddist) {
				return 1.0;
			}
		}
		lookaheaddist += segptr->length;
		segptr = segptr->next;
	}
	return 0.0;
}

/* End of the current race */
static void
endrace(int index, tCarElt *car, tSituation *s)
{
}

/* Called before the module is unloaded */
static void
shutdown(int index)
{
}

