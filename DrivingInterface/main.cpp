#include <stdio.h>
#include <conio.h>
#include <stdint.h>
#define _USE_MATH_DEFINES 
#include <math.h>
//#include "pGNUPlot.h"
#include <time.h>
#include <Windows.h>
#include "linalg.h"
#include "float.h"
#include "learn.h"


FILE *fp;

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



#define NORM_PI_PI(x)               \
  do {                        \
           while ((x) > M_PI) { (x) -= 2*M_PI; }   \
		                           while ((x) < -M_PI) { (x) += 2*M_PI; }  \
         } while (0)

#define RADIANS_TO_DEGREES(radians) ((radians) * (180.0 / M_PI))



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

learn *pLearn = NULL;

static int stuck = 0;
const double MAX_UNSTUCK_SPEED = 5.0;   /* [m/s] */
const double MIN_UNSTUCK_DIST = 3.0;    /* [m] */
const double MAX_UNSTUCK_ANGLE = 15.0 / 180.0*M_PI;
const double MAX_UNSTUCK_COUNT = 20.0;

const double ABS_SLIP = 0.9;                         /* [-] range [0.95..0.3] */
const double ABS_MINSPEED = 3.0;
#define MAX_SPEED_PER_METER  100.0

double previousaccel = 0.0;
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
//
///* Compute the allowed speed on a segment */
//double getAllowedSpeed(shared_use_st *shared)
//{
//  if (shared->track_curve_type == CURVE_TYPE_STRAIGHT) {
//      return FLT_MAX;
//  }
//  else {
//      double mu = 1.2;
//      return sqrt(mu*9.8*shared->track_width);
//  }
//}
//

#define CURRENT_TRACK_INDEX -1

double getTrackLength(shared_use_st *shared, int index){
	double length = shared->track_forward_dists[index + 1] - shared->track_forward_dists[index];
	return length;
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

	/*
	if (deltaAngle < 0) {
	printf("bug\n");
	}*/

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

	curvature = fabs(curvature);

	return curvature;
}
double getAllowedSpeedFromCurvature(shared_use_st *shared, int index){
	double mu = ASPHALT_FRICTION_CONSTANT;
	double curvature = getAllowedCurvature(shared, index);

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


DWORD previousTime;
double speedIPrev;
double speederrorPrev;
double times;
double speedK = 0.2;
double speedTr = 0.15;
double speedTd = 0.0;
double deltaTime = 0.0;

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

double getCurGearMaxSpeed(int gear) {

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
	printf("cur gear = %d\n", gear);
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
	printf("cur gear = %d\n", gear);
	return selratio;
}

#if 0
///* compute fitting acceleration */
double getaccel(shared_use_st *shared)
{
	/*  if ((shared->toStart  > 2760) && (shared->toStart  < 2770)) {
	return 0.5;
	}
	*/

	const double radius = 0.33 / 2.0f + 0.133 * 0.8;

	double allowedspeed = getAllowedSpeedFromCurvature(shared, 0);
	//printf("allowedspeed = %f km/h speed = %f km/h \n", allowedspeed * (18 / 5), shared->speed* (18 / 5));

	if (allowedspeed > shared->speed + (20 * (18 / 5))) {
		//          double curvature = pLearn->getLastCurvature();

		return 1.0;

		//          if (curvature < 0.0001) {
		//return 1.0;
		//}
#if 0           
	else {

		double calcSpeed = sqrt((ASPHALT_FRICTION_CONSTANT*1.5*GRAVITY_CONSTANT) / curvature);

		if (curvature == 0.0)
			return MAX_SPEED_PER_METER;

		if (isinf(calcSpeed)) {
			calcSpeed = MAX_SPEED_PER_METER;
		}

		if (isnan(calcSpeed)) {
			calcSpeed = MAX_SPEED_PER_METER;
		}

		double gr = getEmulatedGearRatio(shared->speed);
		double rm = 850;
		double accel = calcSpeed / radius*gr / rm;
		//  printf("cur accel = %f\n", accel);
		return accel;
	}
#endif
	}
	else {

		double gr = getEmulatedGearRatio(shared->speed);
		double rm = 850;
		double accel = allowedspeed / radius*gr / rm;
		//  printf("cur accel = %f\n", accel);
		return accel;
	}

	return 1.0;

	/*
	if (allowedspeed * (18 / 5) < 70) {
	return 0.3f;
	}
	*/

	//return getPidAccel(shared, allowedspeed);
	//return fabs (allowedspeed - shared->speed) / MAX_SPEED_PER_METER;
	//}

}
#endif

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



	/*
	if (car->_trkPos.seg->type == TR_STR) {
	return car->_trkPos.seg->length - car->_trkPos.toStart;
	}
	else {
	return (car->_trkPos.seg->arc - car->_trkPos.toStart)*car->_trkPos.seg->radius;
	}
	*/
}
#if 1
/* compute fitting acceleration */
double getaccel(shared_use_st *shared)
{
	/*  if ((shared->toStart  > 2760) && (shared->toStart  < 2770)) {
	return 0.5;
	}
	*/
	double allowedspeed = getAllowedSpeedFromCurvature(shared, 0);

	if (allowedspeed == MAX_SPEED_PER_METER)
		return 1.0;

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
		//printf ("ac:1\n");
		//  if (fabs(1.0 - previousaccel) < 0.1)
		//      return 1.0;

		double curGearmaxSpeed = getCurGearMaxSpeed(getCurgear());

		if (allowedspeed - curGearmaxSpeed < 0) {
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

/* Antilocking filter for brakes */
double filterABS(shared_use_st *shared, double brake)
{
	if (shared->speed < ABS_MINSPEED) return brake;
	int i;
	double slip = 0.0;
	/*
	for (i = 0; i < 4; i++) {
	slip += car->_wheelSpinVel(i) * car->_wheelRadius(i) / shared->speed;
	}
	*/
	slip = MAX_SPEED_PER_METER / shared->speed;

	slip = slip / 2.0;

	if (slip < ABS_SLIP)
		brake = brake*slip;


	//printf("brake = %f", brake);

	return brake;
}


double getBrake(shared_use_st *shared)
{
	double currentspeedsqr = shared->speed*shared->speed;
	double mu = ASPHALT_FRICTION_CONSTANT;
	double mass = 1150;
	double maxlookaheaddist = currentspeedsqr / (2.0*mu*GRAVITY_CONSTANT);

	double lookaheaddist = getDistToSegEnd(shared);

	//printf("lookaheaddist = %f", lookaheaddist);

	double allowedspeed = getAllowedSpeedFromCurvature(shared, 0);
#if 1
	if (allowedspeed < shared->speed){
		//printf("allowedspeed < shared->speed  allowSpeed = %f speed = %f \n", allowedspeed,shared->speed);
		return min(1.0f, (shared->speed - allowedspeed) / (3.0));
		//return 1.0;
	}
#endif
	int trackIndex = 1;
#if 1
	while (lookaheaddist < maxlookaheaddist) {

		if (trackIndex > 18)
			break;

		//if (trackIndex == 2)
		//printf("");
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


const double WIDTHDIV = 4.0;                         /* [-] */

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

	if (shared->speed < MAX_UNSTUCK_SPEED) return accel;

	if (TrackType == CURVE_TYPE_STRAIGHT) {
		double tm = fabs(shared->toMiddle);
		double w = (shared->track_width - 4.39) / 2;

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

double getSteer(shared_use_st *shared)
{
	double targetAngle;
	float fistangle_recon = targetAngle = getTargetPoint(shared);
	printf("targetAngle = %f\n", targetAngle);
	targetAngle -= shared->track_current_angle - shared->angle;
	printf("shared->track_current_angle + shared->angle = %f\n", shared->track_current_angle + shared->angle);
	NORM_PI_PI(targetAngle);

	if (fistangle_recon < 0)
		targetAngle -= 1.0*(shared->toMiddle + 5) / (shared->track_width);
	else
		targetAngle -= 1.0*(shared->toMiddle - 5) / (shared->track_width);
	/*  if (targetAngle < 0)
	targetAngle -= 1.0*(-shared->toMiddle/2) / shared->track_width;
	else if (targetAngle > 0)
	targetAngle -= 1.0*(shared->toMiddle/2) / shared->track_width;
	*/

	printf("targetAngle = %f\n", targetAngle);
	double steer = targetAngle / ((21 * M_PI) / 180);

	printf("steer = %f\n", steer);
	return steer;
}

static bool trytoOut = false;

int controlDriving(shared_use_st *shared){
	if (shared == NULL) return -1;

	//Input : shared_user_st
	const double SC = 1.0;
	double diff_angle = 0;

	//pLearn->update(shared->track_forward_dists[0], shared->track_forward_angles[0], getAllowedCurvature(shared, 0),0);

	if (isStuck(shared)) {
		diff_angle = -shared->angle;
		//NORM_PI_PI(diff_angle);

		if ((fabs(shared->speed) > 0.3) && trytoOut == false){

			//Output : 4개의 Output Cmd 값을 도출하세요.
			shared->steerCmd = diff_angle / ((21 * M_PI) / 180);
			shared->brakeCmd = 1.0;
			shared->accelCmd = 0.0;
			shared->backwardCmd = GEAR_BACKWARD;

			printf("isStuck\n");
		}
		else {
			//Output : 4개의 Output Cmd 값을 도출하세요.
			shared->steerCmd = diff_angle / ((21 * M_PI) / 180);
			shared->brakeCmd = 0.0;
			shared->accelCmd = 0.3;
			shared->backwardCmd = GEAR_BACKWARD;
			trytoOut = true;

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
			shared->steerCmd = diff_angle / ((21 * M_PI) / 180);
			shared->brakeCmd = 1.0;
			shared->accelCmd = 0.0;
			shared->backwardCmd = GEAR_FORWARD;

			printf("isOutofTrack where = %f\n", shared->toStart);
		}
		else {
			//Output : 4개의 Output Cmd 값을 도출하세요.
			shared->steerCmd = diff_angle / ((21 * M_PI) / 180);
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
		diff_angle -= SC*shared->toMiddle / shared->track_width;
		NORM_PI_PI(diff_angle);
		/*
		printf("toMiddle = %f\n", shared->toMiddle);
		printf("track_width = %f\n", shared->track_width);
		printf("diff_angle = %f\n", RADIANS_TO_DEGREES(diff_angle));
		*/
		//Output : 4개의 Output Cmd 값을 도출하세요.
		shared->steerCmd = getSteer(shared);// diff_angle / ((21 * M_PI) / 180);
		shared->brakeCmd = filterABS(shared, getBrake(shared));
		if (shared->brakeCmd == 0.0)
			shared->accelCmd = filterTrk(shared, getaccel(shared));
		else
			shared->accelCmd = 0.0;
		shared->backwardCmd = GEAR_FORWARD;

	}


	//  if ((shared->toStart > 1800) && (shared->toStart < 1900))
	//shared->accelCmd = 0.2;

	//printf("shared->track_dist_straight = %f\n", shared->track_dist_straight);

	//fprintf(fp, "%f, %f\n", shared->toStart, shared->accelCmd);

	printf("shared->accel %f\n", shared->accelCmd);
	printf("shared->break %f\n", shared->brakeCmd);
	printf("shared->angle %f\n", shared->angle);
	printf("shared->steer %f\n", shared->steerCmd);
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
#define LEFT 75
#define RIGHT 77
#define UP 72
#define DOWN 80

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


	pLearn = new learn();

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

	endShare(shared, hMap);

	return 0;
}