#include <stdio.h>
#include <conio.h>
#include <stdint.h>
#define _USE_MATH_DEFINES 
#include <math.h>
//#include "pGNUPlot.h"
#include <time.h>
#include <Windows.h>
#include "float.h"

#include<iostream>
#include<queue>
using  namespace std;

FILE *fp;
clock_t previusTime;

#define LEFT 75
#define RIGHT 77
#define UP 72
#define DOWN 80

#define SHARED_MOMORY_NAME1 "TORCS_SHARED1"
#define SHARED_MOMORY_NAME2 "TORCS_SHARED2"

#define CURVE_TYPE_RIGHT        1
#define CURVE_TYPE_LEFT         2
#define CURVE_TYPE_STRAIGHT     3

#define GEAR_FORWARD            0   // 전진 (D)
#define GEAR_BACKWARD           -1  // 후진 (R)

#define INPUT_AICAR_SIZE            20
#define INPUT_FORWARD_TRACK_SIZE    20
#define ASPHALT_FRICTION_CONSTANT   1.2
#define GRAVITY_CONSTANT        9.81

double mass = 1150;

#define NORM_PI_PI(x)               \
  do {                        \
             while ((x) > M_PI) { (x) -= 2*M_PI; }   \
			 		                           while ((x) < -M_PI) { (x) += 2*M_PI; }  \
           } while (0)

#define RADIANS_TO_DEGREES(radians) ((radians) * (180.0 / M_PI))

#define MYCAR_STEERLOCK	((21 * M_PI) / 180)

struct shared_use_st
{
	// System Value
	BOOL connected;
	int written;

	// Driving Parameters
	double toMiddle;
	double angle;
	double speed;

	// Track Parameters
	double toStart;
	double dist_track;
	double track_width;
	double track_dist_straight;
	int    track_curve_type;
	double track_forward_angles[INPUT_FORWARD_TRACK_SIZE];
	double track_forward_dists[INPUT_FORWARD_TRACK_SIZE];
	double track_current_angle;

	// Other Cars Parameters
	double dist_cars[INPUT_AICAR_SIZE];

	// Racing Info. Parameters
	double damage;
	double damage_max;
	int    total_car_num;
	int    my_rank;
	int    opponent_rank;

	// Output Values
	double steerCmd;
	double accelCmd;
	double brakeCmd;
	int    backwardCmd;
};


static int stuck = 0;
const double MAX_UNSTUCK_SPEED = 5.0;   /* [m/s] */
const double MIN_UNSTUCK_DIST = 3.0;    /* [m] */
const double MAX_UNSTUCK_ANGLE = 15.0 / 180.0*M_PI;
const double MAX_UNSTUCK_COUNT = 20.0;

const double ABS_SLIP = 0.9;                         /* [-] range [0.95..0.3] */
const double ABS_MINSPEED = 3.0;
#define MAX_SPEED_PER_METER  100.0
double previousaccel = 0.0;
static bool trytoOut = false;
double currentSectorStartDistance = -1;
double tempDistance = -1;

double currentSectorAngle = DBL_MAX;
double tempAngle = -1;

double currentCurvature = DBL_MAX;
double currentAllowSpeed = MAX_SPEED_PER_METER;

const double MYCAR_DIMENSION_X = 4.39;
const double MYCAR_DIMENSION_Y = 1.94;

const double OPP_DIMENSION_X = 4.8;
const double OPP_DIMENSION_Y = 1.8;

double myoffset = 0.0;
const double BORDER_OVERTAKE_MARGIN = 0.5;
const double OVERTAKE_OFFSET_INC = 0.3;

class trackInfo {
public:
	double dist;
	double angle;
	double curvature;
	double allowedSpeed;

	//적의 정보리스트 저장 필요
	//double oppnent_x;
	//double oppnent_x;
};


DWORD previousTime;
double speedIPrev;
double speederrorPrev;
double times;
double speedK = 0.2;
double speedTr = 0.15;
double speedTd = 0.0;
double deltaTime = 0.0;
vector<trackInfo*> TrackInfoQueue;


bool justCornerExit(shared_use_st *shared) {

	const double LOOKAHEAD_CONST = 10.0;                 /* [m] */
	const double LOOKAHEAD_FACTOR = 0.13;                /* [-] */


	/* compute target point for steering */
	double lookahead = LOOKAHEAD_CONST + shared->speed*LOOKAHEAD_FACTOR;
	double length = shared->toStart - currentSectorStartDistance;
	int index = TrackInfoQueue.size() - 1;
	bool justCornerExit = false;

	while (length < lookahead) {

		if (((trackInfo *)TrackInfoQueue[index])->curvature != 0.0) {
			justCornerExit = true;
			break;
		}

		length += ((trackInfo *)TrackInfoQueue[index])->dist - ((trackInfo *)TrackInfoQueue[index - 1])->dist;
		index--;

		if (index <= 0) {
			break;
		}
	}

	return justCornerExit;
}
double getLastCurvature() {


	trackInfo * ptrTrackinfo = (trackInfo *)TrackInfoQueue.back();

	if (ptrTrackinfo == NULL)
		return 0.0;

	return ptrTrackinfo->curvature;
}


#define OPP_IGNORE	0
#define OPP_FRONT	(1<<0)
#define OPP_BACK	(1<<1)
#define OPP_SIDE	(1<<2)
#define OPP_COLL	(1<<3)
#define OPP_LETPASS		(1<<4)
#define OPP_FRONT_FAST	(1<<5)

class Opponent {
public:
	Opponent();

	int getState() { return state; }
	void setState(int State) { state = State; }
	void clearInfo() {
		state = 0;
		catchdist = 0;
		distance = 0;
		toMiddle = 0;
		sidedist = 0;
		width = 0;
		prevToStart = 0;
	}

	double getCatchDist() { return catchdist; }
	double getToMiddle() { return toMiddle; }
	double getDistance() { return distance; }
	double getSideDist() { return sidedist; }
	double getWidth() { return width; }
	
	double getSpeed() { return speed; }
	void update(int id,shared_use_st *shared, double dist, double toMiddle);

private:
	void calcSpeed(double toStart);
	double getDistToSegStart(shared_use_st *shared, double toStart);

	double toMiddle;
	double distance;	/* approximation of the real distance */
	double speed;		/* speed in direction of the track */
	double catchdist;	/* distance needed to catch the opponent */
	double width;		/* the cars needed width on the track */
	double sidedist;		/* distance of center of gravity of the cars */
	int state;			/* state bitmask of the opponent */
	double prevToStart = 0;

	int id;
	/* constants */
	static double FRONTCOLLDIST;
	static double BACKCOLLDIST;
	static double SIDECOLLDIST;
	static double LENGTH_MARGIN;
	static double SIDE_MARGIN;
};

double Opponent::FRONTCOLLDIST = 200.0;	/* [m] distance to check for other cars */
double Opponent::BACKCOLLDIST = 50.0;	/* [m] distance to check for other cars */
double Opponent::LENGTH_MARGIN = 2.0;	/* [m] safety margin */
double Opponent::SIDE_MARGIN = 1.0;
double opponentLength = 4.8;

Opponent::Opponent() {
	state = OPP_IGNORE;
}

double deltaCalc;
double Timediff;

void Opponent::calcSpeed(double toStart)
{
	speed = (toStart - prevToStart) / Timediff;
	prevToStart = toStart;
	/*
	if (speed > MAX_SPEED_PER_METER + 100)
		speed = 0;
	
	if (speed < 0)
		speed = 0;
		*/
}

/* Compute the length to the start of the segment */
double Opponent::getDistToSegStart(shared_use_st *shared, double toStart)
{
	return shared->toStart + toStart;
}

const float TIME_MARGIN = 2.0;

void Opponent::update(int ID,shared_use_st *shared, double dist, double toMiddle)
{
	state = OPP_IGNORE;

	this->id = ID;

	if (id == 3){
		printf("");//
	}

	/* updating distance along the middle */
	distance = dist;
	this->toMiddle = toMiddle;

	/* update speed in track direction */
	Opponent::calcSpeed(shared->toStart + dist);

	//float cosa = speed / sqrt(car->_speed_x*car->_speed_x + car->_speed_y*car->_speed_y);
	//float alpha = acos(cosa);

	//width = (OPP_DIMENSION_X + OPP_DIMENSION_Y) / 2;// opponentLength;// car->_dimension_x*sin(alpha) + car->_dimension_y*cosa;
	width = (OPP_DIMENSION_Y);// opponentLength;// car->_dimension_x*sin(alpha) + car->_dimension_y*cosa;
	double SIDECOLLDIST = min(OPP_DIMENSION_X, MYCAR_DIMENSION_Y);

	/* is opponent in relevant range -50..200 m */
	if (distance > -BACKCOLLDIST && distance < FRONTCOLLDIST) {
		/* is opponent in front and slower */
		if (distance > SIDECOLLDIST && speed < shared->speed) {
			catchdist = shared->speed*distance / (shared->speed - speed);
			state |= OPP_FRONT;
			distance -= opponentLength;

			double cardist = this->toMiddle - shared->toMiddle;
			sidedist = cardist;
			cardist = fabs(cardist) - fabs(width / 2.0) - (MYCAR_DIMENSION_Y / 2.0);
			/*
		// smooth collision ? 
			double du = fabs(getSpeed() - shared->speed);
			double t_impact = 10;

			if (du>0) {
				t_impact = fabs(distance / du);
			}
			*/
			if (cardist < SIDE_MARGIN) {//&& t_impact < TIME_MARGIN) {
				state |= OPP_COLL;
				//printf ("OPP_COLL\n");
			}
			
		}
		else
			/* is opponent behind and faster */
			if (distance < -SIDECOLLDIST && speed > shared->speed) {
				catchdist = shared->speed*distance / (speed - shared->speed);
				state |= OPP_BACK;
				distance -= max(OPP_DIMENSION_X, MYCAR_DIMENSION_Y);
				distance -= LENGTH_MARGIN;
			}
		else
				/* is opponent aside */
			if (distance > -SIDECOLLDIST &&
				distance < SIDECOLLDIST) {
				sidedist = this->toMiddle - shared->toMiddle;
				state |= OPP_SIDE;
			}
		else
			// Opponent is in front and faster.
			if (distance > SIDECOLLDIST && getSpeed() > shared->speed) {
				state |= OPP_FRONT_FAST;
				//printf ("OPP_FRONT_FAST\n");
			}
	}

}

#define OPPONENT_NUM 10

Opponent opponents[OPPONENT_NUM];

void updateOpponent(shared_use_st *shared, Opponent *pOpponent) {

	int sign = 1;
	int index = 0;
	for (int i = 0; i < 19; i+=2) {
//		printf("%i opponnet dist= %f \n", i, shared->dist_cars[i]);
//		printf("%i opponnet toMiddle= %f\n", i, shared->dist_cars[i + 1]);
		if ((shared->dist_cars[i] != 0.0) && (shared->dist_cars[i] != 100.0) && (shared->dist_cars[i + 1] != 9.0))
			pOpponent[index].update(i,shared,shared->dist_cars[i], shared->dist_cars[i + 1]);
		else
			pOpponent[index].clearInfo();
		
		index++;
	}

}


void update(double curtrackDist, double curTrackAngle, double curCurvature, double curallowSpeed) {

	int  prevpushedItemIndex = -1;

	trackInfo *ptrTrackinfo = NULL;

	for (int i = 0; i < TrackInfoQueue.size(); i++){

		ptrTrackinfo = (trackInfo *)TrackInfoQueue[i];

		if (fabs(ptrTrackinfo->dist - curtrackDist) < DBL_EPSILON) {
			prevpushedItemIndex = i;
			break;
		}
	}

	if (prevpushedItemIndex != -1) {
		ptrTrackinfo->dist = curtrackDist;
		ptrTrackinfo->angle = curTrackAngle;
		ptrTrackinfo->curvature = curCurvature;
		ptrTrackinfo->allowedSpeed = curallowSpeed;
	}
	else {
		trackInfo *data = new trackInfo();

		data->dist = curtrackDist;
		data->angle = curTrackAngle;
		data->curvature = curCurvature;
		data->allowedSpeed = curallowSpeed;

		TrackInfoQueue.push_back(data);
	}

}

#if 1
/* check if the car is stuck */
bool isStuck(shared_use_st *shared)
{
	double angle = shared->angle;
	NORM_PI_PI(angle);

	//printf("shared->speed = %f \n", shared->speed);
	// angle smaller than 30 degrees?
	if (fabs(angle) > MAX_UNSTUCK_ANGLE &&
		shared->speed < MAX_UNSTUCK_SPEED &&
		fabs(shared->toMiddle) > MIN_UNSTUCK_DIST) {

		if (stuck > MAX_UNSTUCK_COUNT && shared->toMiddle*angle < 0.0) {
			//printf("stuck\n");
			return true;
		}
		else {

			//printf("stuck count is %d\n", stuck);
			stuck++;
			return false;
		}

	}
	else {
		//      printf("angle = %f\n", fabs(shared->angle));
		//      printf("speed = %f\n km/h", shared->speed*3.6);
		//      printf("toMiddle = %f\n", shared->toMiddle);

		stuck = 0;
		return false;
	}

}
#endif
inline int sign(double x) {
	return (x > 0) ? 1 : ((x < 0) ? -1 : 0);
}

#define CURRENT_TRACK_INDEX -1

double getTrackLength(shared_use_st *shared, int index){
	double length = shared->track_forward_dists[index + 1] - shared->track_forward_dists[index];
	return length;
}

double getAllowedSpeed(shared_use_st *shared, double deltaAngle, double deltaDist){
	double mu = ASPHALT_FRICTION_CONSTANT;
	double normdeltaAngle = deltaAngle;
	NORM_PI_PI(deltaAngle);
	double curvature = (deltaAngle) / deltaDist;
	curvature = fabs(curvature);
	double calcSpeed = sqrt((mu*1.5*GRAVITY_CONSTANT) / curvature);

	if (curvature == 0.0)
		return MAX_SPEED_PER_METER;

	if (isinf(calcSpeed)) {
		calcSpeed = MAX_SPEED_PER_METER;
	}

	if (isnan(calcSpeed)) {
		calcSpeed = MAX_SPEED_PER_METER;
	}

	return calcSpeed;
}

double getAllowedCurvature(shared_use_st *shared, int index){

	double first = 0.0;
	double AllowSpeed = DBL_MAX;
	double calcSpeed;

	double deltaAngle = 0.0;
	double curvature = 0.0;
	double deltadists = 0.0;

	if (index == -1) {

		deltaAngle = shared->track_forward_angles[0] - shared->track_current_angle;
	}
	else {

		deltaAngle = shared->track_forward_angles[index + 1] - shared->track_forward_angles[index];
	}

	NORM_PI_PI(deltaAngle);

	if (index == -1) {
		if (shared->track_forward_dists[0] == 0.0)
			deltadists = shared->dist_track - shared->toStart;
		else
			deltadists = (shared->track_forward_dists[0] - shared->toStart);
	}
	else {
		deltadists = (shared->track_forward_dists[index + 1] - shared->track_forward_dists[index]);

		if ((deltadists < 0) && (index - 1 >= 0)) {
			deltadists = (shared->track_forward_dists[index] - shared->track_forward_dists[index - 1]);
		}
		else if ((deltadists < 0) && (index - 1 < 0)) {
			deltadists = (shared->track_forward_dists[index + 2] - shared->track_forward_dists[index + 1]);
		}
	}

	curvature = (deltaAngle) / deltadists;

	curvature = curvature;

	return curvature;
}
double getAllowedSpeedFromCurvature(shared_use_st *shared, int index){
	double mu = ASPHALT_FRICTION_CONSTANT;
	double curvature = fabs(getAllowedCurvature(shared, index));

	double calcSpeed = sqrt((mu*1.5*GRAVITY_CONSTANT) / curvature);

	if (curvature == 0.0)
		return MAX_SPEED_PER_METER;

	if (isinf(calcSpeed)) {
		calcSpeed = MAX_SPEED_PER_METER;
	}

	if (isnan(calcSpeed)) {
		calcSpeed = MAX_SPEED_PER_METER;
	}

	return calcSpeed;
}


double getPidAccel(shared_use_st *shared, double targetspeed) {

	double speedError = targetspeed - shared->speed;
	double speedP = speedError * speedK;

	double speedI = speedIPrev + (((speedK*deltaTime) / (2 * speedTr))*(speedError));

	if (speedI > 2){
		speedI = 2;
	}

	if (speedI < -2){
		speedI = -2;
	}

	double speedD = (((speedK*speedTd) / deltaTime)*(speedError - speederrorPrev));

	speederrorPrev = speedError;
	speedIPrev = speedI;
	double accel = speedP + speedI + speedD;

	fprintf(fp, "%f, %f, %f, %f \n", deltaTime, targetspeed, shared->speed, accel);

	if (accel > 1) accel = 1;
	if (accel < 0) accel = 0;
	//  if ((accel > 0)&& (accel < 1)) accel = accel;

	printf("toStart = %f, accel = %f\n", shared->toStart, accel);
	return accel;
}

int gear = 1;

int getCurgear() {
	return gear;
}
const double ratio[] = {
	-4.0, 3.5, 2.6, 1.9, 1.54, 1.25, 1.05
};
double getCurgearRaitio(int getr) {
	return ratio[gear];
}

double getCurGearMaxSpeed() {

	double maxspeed = 0;

	if (gear == 1) {
		maxspeed = 79;
	}
	if (gear == 2) {
		maxspeed = 105;
	}
	if (gear = 3) {

		maxspeed = 134;
	}
	if (gear == 4) {

		maxspeed = 178;
	}
	if (gear == 5) {

		maxspeed = 210;
	}
	if (gear == 6) {
		maxspeed = 360;
	}
	//printf("cur gear = %d\n", gear);
	return maxspeed;

}

double getEmulatedGearRatio(double curSpeed) {

	double selratio = 0;

	if ((curSpeed*(18 / 5)) >= 0 && (curSpeed*(18 / 5)) < 79) {
		selratio = ratio[1];
		gear = 1;
	}
	if ((curSpeed*(18 / 5)) > 79 && (curSpeed*(18 / 5)) < 105) {
		selratio = ratio[2];
		gear = 2;
	}
	if ((curSpeed*(18 / 5)) > 105 && (curSpeed*(18 / 5)) < 134) {
		selratio = ratio[3];
		gear = 3;
	}
	if ((curSpeed*(18 / 5)) > 134 && (curSpeed*(18 / 5)) < 178) {
		selratio = ratio[4];
		gear = 4;
	}
	if ((curSpeed*(18 / 5)) > 178 && (curSpeed*(18 / 5)) < 210) {
		selratio = ratio[5];
		gear = 5;
	}
	if ((curSpeed*(18 / 5)) > 210) {
		gear = 6;
		selratio = ratio[6];
	}
	//printf("cur gear = %d\n", gear);
	return selratio;
}

double getDistToSegEnd(shared_use_st *shared)
{

	/*  printf("shared->toStart = %f\n ", shared->toStart);
	printf("shared->track_forward_dists[0] = %f\n", shared->track_forward_dists[0]);
	*/
	double disttrack = shared->dist_track;
	double real = shared->toStart > shared->dist_track;


	//printf("track_forward_dists[0] = %f toStart = %f \n ", shared->track_forward_dists[0], shared->toStart, disttrack);

	if (shared->track_forward_dists[0] == 0.0)
		return shared->dist_track - shared->toStart;

	return shared->track_forward_dists[0] - shared->toStart;// % shared->dist_track);
		
}
#if 1

bool isStucked = false;

double getaccel(shared_use_st *shared)
{
	/*  if ((shared->toStart  > 2760) && (shared->toStart  < 2770)) {
	return 0.5;
	}
	*/
	double allowedspeed = currentAllowSpeed;// getAllowedSpeedFromCurvature(shared, 0);

	if (allowedspeed == MAX_SPEED_PER_METER) {
				
		if (!justCornerExit(shared) && isStucked == false){
			//printf("MAX_SPEED_PER_METER\n");
			return 1.0;
		}
	}

	if (isStucked == true) {
		isStucked = false;
	}

	//return 1.0;
#if 1
	double mu = ASPHALT_FRICTION_CONSTANT;
	double currentspeedsqr = shared->speed*shared->speed;
	double maxlookaheaddist = currentspeedsqr / (2.0*mu*GRAVITY_CONSTANT);
	double lookaheaddist = getDistToSegEnd(shared);

	int index = 1;

	while (lookaheaddist < maxlookaheaddist) {
		float pallowedspeed = getAllowedSpeedFromCurvature(shared, index);

		double allowedspeedsqr = allowedspeed*allowedspeed;
		double brakedist = (currentspeedsqr - allowedspeedsqr) / (2.0*(mu*GRAVITY_CONSTANT));// +allowedspeedsqr*(CA*mu + CW)));

		if (brakedist < lookaheaddist) {

			if (pallowedspeed < allowedspeed) {
				allowedspeed = pallowedspeed;
			}
		}

		lookaheaddist += getTrackLength(shared, index);
		index++;
	}
#endif
	//printf("allowedspeed = %f km/h speed = %f km/h \n", allowedspeed * (18 / 5), shared->speed* (18 / 5));
	double delta = allowedspeed - (shared->speed + 3.0);
	double alpha = 0.5;
	double lambda = 3.0;

	if (delta>0) {
		if (delta<lambda) {
			float acc = alpha + (1 - alpha)*delta / lambda;
			//printf ("ac:%f\n", acc);
			return acc;
		}
		double curGearmaxSpeed = getCurGearMaxSpeed();

		if ((allowedspeed * (18 / 5)) - curGearmaxSpeed < 0) {
			const double radius = 0.33 / 2.0f + 0.133 * 0.75;
			double gr = getEmulatedGearRatio(curGearmaxSpeed);
			double rm = 850;
			double accel = curGearmaxSpeed / radius*gr / rm;
			//  printf("cur accel = %f\n", accel);
			if (accel > 1)
				accel = 1;

			return accel;

		}
		else {
			const double radius = 0.33 / 2.0f + 0.133 * 0.8;
			gear = getCurgear();
			double gr = ratio[gear++];
			double rm = 850;
			double accel = allowedspeed / radius*gr / rm;
			//  printf("cur accel = %f\n", accel);

			if (accel > 1)
				accel = 1;
			return accel;
		}

		//  return previousaccel + 0.1;//tanh(delta);
	}/*
	 else {
	 float acc = alpha*(1 + delta / 3.0);
	 if (acc<0) acc = 0;
	 //printf ("at:%f\n", acc);
	 return acc;
	 }*/

	else {
		const double radius = 0.33 / 2.0f + 0.133 * 0.8;
		double gr = getEmulatedGearRatio(shared->speed);
		double rm = 8500;
		double accel = allowedspeed / radius*gr / rm;
		//  printf("cur accel = %f\n", accel);

		if (accel > 1)
			accel = 1;
		return accel;
	}


#if 0
	if (allowedspeed > shared->speed + 30 * (18 / 5)) {
		double curvature = pLearn->getLastCurvature();
		return 1.0;
	}
	else {
		const double radius = 0.33 / 2.0f + 0.133 * 0.8;
		double gr = getEmulatedGearRatio(shared->speed);
		double rm = 850;
		double accel = allowedspeed / radius*gr / rm;
		//  printf("cur accel = %f\n", accel);
		return accel;
	}
	return 1.0;
#endif

}
#endif


/* Brake filter for collision avoidance */
double filterBColl(shared_use_st *shared, double brake)
{
	float currentspeedsqr = shared->speed*shared->speed;
	float mu = ASPHALT_FRICTION_CONSTANT;
	float cm = mu*GRAVITY_CONSTANT*mass;
	int i;
	int sign = 1;
	for (i = 0; i < OPPONENT_NUM; i++) {
		if (opponents[i].getState() & OPP_COLL) {

			if (i > 4)
				sign = -1;

			float allowedspeedsqr = opponents[i].getSpeed();
		
			allowedspeedsqr *= allowedspeedsqr;
			
			float brakedist = mass*(currentspeedsqr - allowedspeedsqr) / (2.0*(cm));
			if (brakedist > opponents[i].getDistance()) {
				printf("break with i = %d speed = %f tomiddle = %f mytomiddle = %f\n", i, opponents[i].getSpeed(),
					opponents[i].getToMiddle(),shared->toMiddle);
				return 1.0;
			}
		}
	}
	return brake;
}

// Reduces the brake value such that it fits the speed (more downforce -> more braking).
double filterBrakeSpeed(shared_use_st *shared, double brake)
{
	float weight = (mass)*GRAVITY_CONSTANT;
	float maxForce = weight + 84.0*84.0;
	float force = weight + (shared->speed)*(shared->speed);
	return brake*force / maxForce;
}
const float ABS_RANGE = 5.0f;

float filterABS(shared_use_st *shared,float brake)
{

	const double radius = 0.33 / 2.0f + 0.133 * 0.75;
	double wheelSpinVel = shared->speed / radius;

	if (shared->speed < ABS_MINSPEED) return brake;
	int i;
	float slip = 0.0;
	for (i = 0; i < 4; i++) {
		slip += wheelSpinVel * radius;
	}
	slip = shared->speed - slip / 4.0;

	if (slip > ABS_SLIP) {
		brake = brake - min(brake, (slip - ABS_SLIP) / ABS_RANGE);
	}
	return brake;
}


double getBrake(shared_use_st *shared)
{
	double currentspeedsqr = shared->speed*shared->speed;
	double mu = ASPHALT_FRICTION_CONSTANT;

	double maxlookaheaddist = currentspeedsqr / (2.0*mu*GRAVITY_CONSTANT);

	double lookaheaddist = getDistToSegEnd(shared);

	//printf("lookaheaddist = %f", lookaheaddist);

	double allowedspeed = currentAllowSpeed;// = getAllowedSpeedFromCurvature(shared, 0);
#if 1

	if (allowedspeed < shared->speed){
		//printf("allowedspeed < shared->speed  allowSpeed = %f speed = %f \n", allowedspeed,shared->speed);
		return min(1.0f, (shared->speed - allowedspeed) / (3.0));
		//return 1.0;
	}

#endif
	int trackIndex = 0;
#if 1
	while (lookaheaddist < maxlookaheaddist) {

		if (trackIndex > 18)
			break;
			
		allowedspeed = getAllowedSpeedFromCurvature(shared, trackIndex);

		if (allowedspeed < shared->speed) {
			double allowedspeedsqr = allowedspeed*allowedspeed;
			double brakedist = (currentspeedsqr - allowedspeedsqr) / (2.0*(mu*GRAVITY_CONSTANT));// +allowedspeedsqr*(CA*mu + CW)));

			if (brakedist > lookaheaddist) {
				return min(1.0f, (shared->speed - allowedspeed) / (3.0));
				//return 1.0;
			}

		}

		lookaheaddist += getTrackLength(shared, trackIndex);
		trackIndex++;
	}
#endif
	return 0.0;
}

const double WIDTHDIV = 3.0;                         /* [-] */

/* Hold car on the track */
double filterTrk(shared_use_st *shared, double accel)
{
	double diffangle = shared->track_forward_angles[0] - shared->track_current_angle;

	int TrackType = CURVE_TYPE_STRAIGHT;

	NORM_PI_PI(diffangle);

	if (fabs(diffangle) <= DBL_EPSILON)
		TrackType = CURVE_TYPE_STRAIGHT;
	else if (diffangle < 0)
		TrackType = CURVE_TYPE_RIGHT;
	else if (diffangle > 0)
		TrackType = CURVE_TYPE_LEFT;

	if (shared->speed < MAX_UNSTUCK_SPEED) 
		return accel;

	if (TrackType == CURVE_TYPE_STRAIGHT) {
		double tm = fabs(shared->toMiddle);
		
		double w = (shared->track_width - MYCAR_DIMENSION_Y) / 2;

		if (tm > w)
			return 0.0;
		else
			return accel;
	}
	else {
		double sign = (TrackType == CURVE_TYPE_RIGHT) ? -1 : 1;
		if (shared->toMiddle*sign > 0.0) {
			return accel;
		}
		else {
			double tm = fabs(shared->toMiddle);
			double w = shared->track_width / WIDTHDIV;
			if (tm > w)
				return 0.0;
			else
				return accel;
		}
	}
}

double TCL_status = 0.0;
const double TCL_SLIP = 2.0f;
const float TCL_RANGE = 10.0f;

float filterTCL_RWD(shared_use_st *shared)
{
	const double radius = 0.33 / 2.0f + 0.133 * 0.75;
	double wheelSpinVel = shared->speed / radius;

	return (wheelSpinVel + wheelSpinVel) * radius / 2.0;
}

double filterTCL(shared_use_st *shared,double accel)
{
	float slip = filterTCL_RWD(shared) - shared->speed;
	
	if (slip > TCL_SLIP) {
	
		accel = accel - min(accel, (slip - TCL_SLIP) / TCL_RANGE);
	}
	return accel;

}

bool isOutofTrack(shared_use_st *shared) {

	// angle smaller than 30 degrees?
	if (fabs(shared->toMiddle) > (shared->track_width / 2)) {
		return true;
	}

	double angle = fabs(shared->angle);
	NORM_PI_PI(angle);

	if (angle > (M_PI / 2))  {
		return true;
	}

	return false;
}

const double LOOKAHEAD_CONST = 17.0;                 /* [m] */
const double LOOKAHEAD_FACTOR = 0.33;                /* [-] */

/* compute target point for steering */
double getTargetPoint(shared_use_st *shared)
{
	double lookahead = LOOKAHEAD_CONST + shared->speed*LOOKAHEAD_FACTOR;
	double length = getDistToSegEnd(shared);
	int index = 0;
	while (length < lookahead) {

		length += shared->track_forward_dists[index + 1] - shared->track_forward_dists[index];
		index++;

		if (index > 17)
			break;
	}

	printf("index = %d\n", index);

	return shared->track_forward_angles[index];
}

#define SIDECOLL_MARGIN  2.0

#if 0
// Steer filter for collision avoidance.
float filterSColl(shared_use_st *shared, float steer)
{
	int i;
	float sidedist = 0.0f, fsidedist = 0.0f, minsidedist = FLT_MAX;
	Opponent *o = NULL;

	// Get the index of the nearest car (o).
	for (i = 0; i < OPPONENT_NUM; i++) {
		if (opponents[i].getState() & OPP_SIDE) {
			sidedist = opponents[i].getSideDist();
			fsidedist = fabs(sidedist);
			if (fsidedist < minsidedist) {
				minsidedist = fsidedist;
				o = &opponents[i];
			}
		}
	}

	// If there is another car handle the situation.
	if (o != NULL) {
		float d = fsidedist - o->getWidth();
		// Near, so we need to look at it.
		if (d < SIDECOLL_MARGIN) {
			/* compute angle between cars */
			
			float diffangle = atan2(o->getDistance(), o->getToMiddle());
			
			NORM_PI_PI(diffangle);
			// We are near and heading toward the car.
			if (diffangle*o->getSideDist() < 0.0f) {
				const float c = SIDECOLL_MARGIN / 2.0f;
				d = d - c;
				if (d < 0.0f) {
					d = 0.0f;
				}

				// Steer delta required to drive parallel to the opponent.
				float psteer = diffangle / MYCAR_STEERLOCK;
				myoffset = shared->toMiddle;

				// Limit myoffset to suitable limits.
				float w = shared->track_width / WIDTHDIV - BORDER_OVERTAKE_MARGIN;
				if (fabs(myoffset) > w) {
					myoffset = (myoffset > 0.0f) ? w : -w;
				}

				// On straights the car near to the middle can correct more, in turns the car inside
				// the turn does (because if you leave the track on the turn "inside" you will skid
				// back to the track.
				if (currentCurvature == 0.0) {
					if (fabs(shared->toMiddle) > fabs(o->getToMiddle())) {
						// Its me, I do correct not that much.
						psteer = steer*(d / c) + 1.5f*psteer*(1.0f - d / c);
					}
					else {
						// Its the opponent, so I correct more.
						psteer = steer*(d / c) + 2.0f*psteer*(1.0f - d / c);
					}
				}
				else {
					// Who is outside, heavy corrections are less dangerous
					// if you drive near the middle of the track.
					float outside = shared->toMiddle - o->getToMiddle();
					
					float sign = (currentCurvature > 0) ? 1.0f : -1.0f;
					
					if (outside*sign > 0.0f) {
						psteer = steer*(d / c) + 1.5f*psteer*(1.0f - d / c);
					}
					else {
						psteer = steer*(d / c) + 2.0f*psteer*(1.0f - d / c);
					}
				}

				if (psteer*steer > 0.0f && fabs(steer) > fabs(psteer)) {
					return steer;
				}
				else {
					return psteer;
				}
			}
		}
	}
	return steer;
}
#endif

const float MAX_INC_FACTOR = 5.0f;
bool overtaking = false;
const float OVERTAKE_TIME = 2.0f;
const float CENTERDIV = 0.1f;
const float DISTCUTOFF = 200.0f;

/// Compute offset to normal target point for overtaking or let pass an opponent.
float getOffset(shared_use_st *shared)
{
	int i;
	double catchdist, mincatchdist = FLT_MAX, mindist = -1000.0;
	Opponent *o = NULL;

	// Increment speed dependent.
	float incfactor = MAX_INC_FACTOR - min(fabs(shared->speed) / MAX_INC_FACTOR, (MAX_INC_FACTOR - 1.0));
	int index = -1;
	// Let overlap.
	for (i = 0; i < OPPONENT_NUM; i++) {
		if (opponents[i].getState() & OPP_LETPASS) {
			// Behind, larger distances are smaller ("more negative").
			if (opponents[i].getDistance() > mindist) {
				mindist = opponents[i].getDistance();
				o = &opponents[i];
			}
		}
	}

	//myoffset = -0.2f * car->_trkPos.seg->width;
	overtaking = false;
	if (o != NULL) {
	
		float side = shared->toMiddle - o->getToMiddle();
		float w = shared->track_width / WIDTHDIV - BORDER_OVERTAKE_MARGIN;
		if (side > 0.0) {
			if (myoffset < w) {
				myoffset += OVERTAKE_OFFSET_INC*incfactor;
			}
		}
		else {
			if (myoffset > -w) {
				myoffset -= OVERTAKE_OFFSET_INC*incfactor;
			}
		}
		//printf ("let overtake: %f\n", myoffset);
		return myoffset;
	}
	
	// Overtake.
	float time_to_overtake = OVERTAKE_TIME;

	for (i = 0; i < OPPONENT_NUM; i++) {
		if ((opponents[i].getState() & OPP_FRONT) || (opponents[i].getState() & OPP_FRONT_FAST))
		{
			catchdist = min(opponents[i].getCatchDist(), opponents[i].getDistance()*10.0);
			if (catchdist < mincatchdist) {
				mincatchdist = catchdist;
				o = &opponents[i];
			}
		}
	}
#if 0
	for (i = 0; i < OPPONENT_NUM; i++) {
		if (opponents[i].getState() & OPP_FRONT) {
			catchdist = opponents[i].getCatchDist();//MIN(opponent[i].getCatchDist(), opponent[i].getDistance()*CATCH_FACTOR);
			if (shared->speed > 0) {
				time_to_overtake = catchdist / shared->speed;
				
				if (time_to_overtake < OVERTAKE_TIME) {
					// consider overtaking if we approach in OVERTAKE_TIME
					//printf ("t_overtake: %f\n", time_to_overtake);
					if (catchdist < mincatchdist) {
						mincatchdist = catchdist;
						o = &opponents[i];
					}

				}

			}
		}
	}
#endif
	if (o != NULL) {
		overtaking = true;
		// Compute the width around the middle which we can use for overtaking.
		float w = shared->track_width / WIDTHDIV - BORDER_OVERTAKE_MARGIN;
		// Compute the opponents distance to the middle.
		float otm = -o->getToMiddle(); // .toMiddle;
		// Define the with of the middle range.
		float wm = shared->track_width*CENTERDIV;

		if (time_to_overtake >0.0f) {
			incfactor *= (1.0f + OVERTAKE_TIME) / (1.0f + time_to_overtake);
		}
		else {
			incfactor *= 2.0f;
		}

		if (otm > wm && myoffset > -w) {
			myoffset -= OVERTAKE_OFFSET_INC*incfactor;
		}
		else if (otm < -wm && myoffset < w) {
			myoffset += OVERTAKE_OFFSET_INC*incfactor;
		}
		else {
			// If the opponent is near the middle we try to move the offset toward
			// the inside of the expected turn.
			// Try to find out the characteristic of the track up to catchdist.

			float length = getDistToSegEnd(shared);
			float oldlen, seglen = length;
			float lenright = 0.0, lenleft = 0.0;
			mincatchdist = min(mincatchdist, DISTCUTOFF);

			if (fabs(currentCurvature) < DBL_EPSILON){
			}
			else if (currentCurvature < 0.0) {
				lenleft += seglen;
			}
			else if (currentCurvature > 0.0) {
				lenright += seglen;
			}
			
			shared->angle;

			int index = 0;
			float curvature = 0.0;
						
			do {
				
				curvature = getAllowedCurvature(shared, index);

				if (curvature == 0.0){
				}
				else if (curvature < 0.0) {
					lenleft += seglen;
				}
				else if (curvature > 0.0) {
					lenright += seglen;
				}
				
				index++;
				shared->angle;
				
				seglen = shared->track_forward_dists[index + 1] - shared->track_forward_dists[index];
				oldlen = length;
				length += seglen;
			} while (oldlen < mincatchdist);

			// If we are on a straight look for the next turn.
			if (lenleft == 0.0 && lenright == 0.0) {
				
				curvature = 0.0;
				
				while (curvature == 0.0) {
					
					curvature = getAllowedCurvature(shared, index);
					index++;

					if (index > 18){
						break;
					}
				}
				// Assume: left or right if not straight.
				if (curvature < 0) {
					lenleft += 10;
				}
				else {
					lenright += 10;
				}
			}

			// Because we are inside we can go to the border.
			float maxoff = (shared->track_width - MYCAR_DIMENSION_Y) / 2.0 - BORDER_OVERTAKE_MARGIN;
			if (lenleft > lenright) {
				if (myoffset < maxoff) {
					myoffset += OVERTAKE_OFFSET_INC*incfactor;
				}
			}
			else {
				if (myoffset > -maxoff) {
					myoffset -= OVERTAKE_OFFSET_INC*incfactor;
				}
			}
		}
	}
	else {
	
		// There is no opponent to overtake, so the offset goes slowly back to zero.

		float a = 1.0;

//		if (fabs(myoffset) < DBL_EPSILON)
//			return myoffset;

		if (myoffset > a*OVERTAKE_OFFSET_INC) {
			myoffset -= a*OVERTAKE_OFFSET_INC ;
			//myoffset= 0.0;

			printf("return to previous pos myoffset -= %f\n", myoffset);
		}
		else if (myoffset < -a*OVERTAKE_OFFSET_INC) {
			myoffset += a*OVERTAKE_OFFSET_INC;
			//myoffset= 0.0;
			printf("return to previous pos myoffset  += %f\n", myoffset);
		}
		else {
			printf("myoffset = 0.0\n");
			myoffset = 0.0;
		}
	}

	return myoffset;
}

// Compute the needed distance to brake.
double brakedist(shared_use_st *shared,float allowedspeed, float mu)
{
	double v1sqr = shared->speed;
	double v2sqr = allowedspeed*allowedspeed;
	return (v1sqr - v2sqr) / (2.0*mu*GRAVITY_CONSTANT);
}


int isAlone()
{
	int i;
	for (i = 0; i < OPPONENT_NUM; i++) {
		if (opponents[i].getState() & (OPP_COLL | OPP_LETPASS)) {
			return 0;	// Not alone.
		}
	}
	return 1;	// Alone.
}

int controlDriving(shared_use_st *shared){
	if (shared == NULL) return -1;

	clock_t current = clock();        // 시간설정

	Timediff = (current - previusTime) / (double)CLOCKS_PER_SEC;
	previusTime = current;

	//Input : shared_user_st
	const double SC = 1.0;
	double diff_angle = 0;
		
	if (currentSectorStartDistance == -1)
		currentAllowSpeed = MAX_SPEED_PER_METER/4;
	
	updateOpponent(shared, opponents);
	
//	if (opponents[0].getSpeed() > 0)
//		printf("opponent 0  getSpeed = %f\n", opponents[0].getSpeed());

	if (fabs(tempDistance - shared->track_forward_dists[0]) > DBL_EPSILON) {
		currentSectorStartDistance = tempDistance;
		currentSectorAngle = tempAngle;
		currentCurvature = shared->track_forward_angles[0] - currentSectorAngle / fabs(shared->track_forward_dists[0] - currentSectorStartDistance);
		
		currentAllowSpeed = getAllowedSpeed(shared, shared->track_forward_angles[0] - currentSectorAngle,
			fabs(shared->track_forward_dists[0] - currentSectorStartDistance));
	}

	tempAngle = shared->track_forward_angles[0];
	tempDistance = shared->track_forward_dists[0];
	getAllowedCurvature(shared, 0);

	update(shared->track_forward_dists[0], shared->track_forward_angles[0], fabs(getAllowedCurvature(shared, 0)), 0);

	if (isStuck(shared)) {
		diff_angle = -shared->angle;
		//NORM_PI_PI(diff_angle);

		if ((fabs(shared->speed) > 0.3) && trytoOut == false){

			//Output : 4개의 Output Cmd 값을 도출하세요.
			shared->steerCmd = diff_angle / MYCAR_STEERLOCK;
			shared->brakeCmd = 1.0;
			shared->accelCmd = 0.0;
			shared->backwardCmd = GEAR_BACKWARD;

			printf("isStuck\n");
		}
		else {
			//Output : 4개의 Output Cmd 값을 도출하세요.
			shared->steerCmd = diff_angle / MYCAR_STEERLOCK;
			shared->brakeCmd = 0.0;
			shared->accelCmd = 0.3;
			shared->backwardCmd = GEAR_BACKWARD;
			trytoOut = true;
			isStucked = true;
			printf("try to out from stuck\n");
		}

	}
	else if (isOutofTrack(shared)){
		diff_angle = shared->angle;
		//TO-DO : 알고리즘을 작성하세요.
		diff_angle -= SC*shared->toMiddle / shared->track_width;
		NORM_PI_PI(diff_angle);

		if ((fabs(shared->speed) > 0.3) && trytoOut == false){

			//Output : 4개의 Output Cmd 값을 도출하세요.
			shared->steerCmd = diff_angle / MYCAR_STEERLOCK;
			shared->brakeCmd = 1.0;
			shared->accelCmd = 0.0;
			shared->backwardCmd = GEAR_FORWARD;

			printf("isOutofTrack where = %f\n", shared->toStart);
		}
		else {
			//Output : 4개의 Output Cmd 값을 도출하세요.
			shared->steerCmd = diff_angle / MYCAR_STEERLOCK;
			shared->brakeCmd = 0.0;
			shared->accelCmd = 0.3;
			shared->backwardCmd = GEAR_FORWARD;
			trytoOut = true;

			printf("try to out from out of track\n");
		}

		printf("speed = %f", shared->speed);

	}
	else {

		trytoOut = false;
		diff_angle = shared->angle;
		//TO-DO : 알고리즘을 작성하세요.
		double offset = getOffset(shared);

		diff_angle -= (SC*shared->toMiddle + offset) / shared->track_width;
		NORM_PI_PI(diff_angle);

		//Output : 4개의 Output Cmd 값을 도출하세요.
				
		shared->steerCmd = diff_angle / MYCAR_STEERLOCK;
		shared->brakeCmd = filterABS(shared, filterBrakeSpeed(shared, filterBColl(shared, getBrake(shared))));
	
		if (shared->brakeCmd == 0.0)
			shared->accelCmd = filterTCL(shared,filterTrk(shared, getaccel(shared)));
		else
			shared->accelCmd = 0.0;
		shared->backwardCmd = GEAR_FORWARD;

	}


	//  if ((shared->toStart > 1800) && (shared->toStart < 1900))
	//shared->accelCmd = 0.2;

	//printf("shared->track_dist_straight = %f\n", shared->track_dist_straight);

	//fprintf(fp, "%f, %f\n", shared->toStart, shared->accelCmd);
	/*
	printf("shared->accel %f\n", shared->accelCmd);
	printf("shared->break %f\n", shared->brakeCmd);
	printf("shared->angle %f\n", shared->angle);
	printf("shared->steer %f\n", shared->steerCmd);
	
	if (shared->track_curve_type == CURVE_TYPE_LEFT)
	printf("CURVE_TYPE_LEFT\n");

	if (shared->track_curve_type == CURVE_TYPE_RIGHT)
	printf("CURVE_TYPE_RIGHT\n");

	if (shared->track_curve_type == CURVE_TYPE_STRAIGHT)
	printf("CURVE_TYPE_STRAIGHT\n");
	*/
	//	printf("currentSectorStartDistance = %f \n",     currentSectorStartDistance);
	//	printf("shared->track_forward_dists[0] = %f \n", shared->track_forward_dists[0]);

	previousaccel = shared->accelCmd;
	return 0;
}

void endShare(struct shared_use_st *&shared, HANDLE &hMap){
	// shared memory initialize
	if (shared != NULL) {
		UnmapViewOfFile(shared);
		shared = NULL;
	}
	if (hMap != NULL) {
		CloseHandle(hMap);
		hMap = NULL;
	}
}


int main(int argc, char **argv){
	////////////////////// set up memory sharing
	struct shared_use_st *shared = NULL;

	// try to connect to shared memory 1
	HANDLE hMap = OpenFileMappingA(FILE_MAP_ALL_ACCESS, false, SHARED_MOMORY_NAME1);
	if (hMap == NULL){
		fprintf(stderr, "Shared Memory Map open failed.\n");
		exit(EXIT_FAILURE);
	}

	shared = (struct shared_use_st*) MapViewOfFile(hMap, FILE_MAP_ALL_ACCESS, 0, 0, sizeof(struct shared_use_st));
	if (shared == NULL){
		fprintf(stderr, "Shared Memory Map open failed.\n");
		exit(EXIT_FAILURE);
	}

	// shared memory 1 is already occupied.
	if (shared->connected == true) {
		endShare(shared, hMap);
		hMap = OpenFileMappingA(FILE_MAP_ALL_ACCESS, false, SHARED_MOMORY_NAME2);
		if (hMap == NULL){
			fprintf(stderr, "Shared Memory Map open failed.\n");
			exit(EXIT_FAILURE);
		}
		shared = (struct shared_use_st*) MapViewOfFile(hMap, FILE_MAP_ALL_ACCESS, 0, 0, sizeof(struct shared_use_st));
		if (shared == NULL){
			fprintf(stderr, "Shared Memory Map open failed.\n");
			exit(EXIT_FAILURE);
		}
	}
	printf("\n********** Memory sharing started, attached at %X **********\n", shared);





	fp = fopen("C:\\Users\\shins\\Downloads\\temp\\1.dat", "wt");//"wt");

	////////////////////// DON'T TOUCH IT - Default Setting
	shared->connected = true;
	shared->written = 0;
	////////////////////// END Default Setting

	////////////////////// Initialize
	shared->steerCmd = 0.0;
	shared->accelCmd = 0.0;
	shared->brakeCmd = 0.0;
	shared->backwardCmd = GEAR_FORWARD;
	////////////////////// END Initialize
	DWORD currentTime;
	

	while (shared->connected){
		if (shared->written == 1) { // the new image data is ready to be read
			/*
			currentTime = timeGetTime();

			deltaTime = (double)(currentTime - previousTime) / 1000.0f;

			previousTime = currentTime;
			*/
			controlDriving(shared);
			shared->written = 0;
		}

		if (_kbhit()){
			char key = _getch();
			if (key == 0xE0 || key == 0)    //입력받은 값이 확장키 이면
				key = _getch();            //한번더 입력을 받는다.

			switch (key){
			case UP:
				shared->accelCmd = 1.0;
				break;
			case DOWN:
				shared->accelCmd = 0.0;
				break;
			case LEFT:
				shared->steerCmd = 1.0;
				break;
			case RIGHT:
				shared->steerCmd = -1.0;
				break;
			}

			if (key == 'q' || key == 'Q'){

				//              CpGnuplot plot("C:\\Program Files\\gnuplot\\bin\\wgnuplot.exe");

				//              plot.cmd("plot 'C:\\temp\\1.dat' with lines");
				fclose(fp);
				break;
			}

		}
	}


	trackInfo *ptrTrackinfo = NULL;

	for (int i = 0; i < TrackInfoQueue.size(); i++){

		ptrTrackinfo = (trackInfo *)TrackInfoQueue[i];
		delete ptrTrackinfo;
	}

	endShare(shared, hMap);

	return 0;
}