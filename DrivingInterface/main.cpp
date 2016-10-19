#include <stdio.h>
#include <conio.h>
#include <stdint.h>
#define _USE_MATH_DEFINES 
#include <math.h>
//#include "pGNUPlot.h"

#include <Windows.h>
#include "linalg.h"
#include "float.h"

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

FILE *fp;

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

const float ABS_SLIP = 0.9;                         /* [-] range [0.95..0.3] */
const float ABS_MINSPEED = 3.0;
#define MAX_SPEED_PER_METER  100.0


#if 1
/* check if the car is stuck */
bool isStuck(shared_use_st *shared)
{
	float angle = shared->angle;
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
//float getAllowedSpeed(shared_use_st *shared)
//{
//  if (shared->track_curve_type == CURVE_TYPE_STRAIGHT) {
//      return FLT_MAX;
//  }
//  else {
//      float mu = 1.2;
//      return sqrt(mu*9.8*shared->track_width);
//  }
//}
//

#define CURRENT_TRACK_INDEX -1

double getTrackLength(shared_use_st *shared, int index){
	float length = shared->track_forward_dists[index + 1] - shared->track_forward_dists[index];
	return length;
}

double getAllowedSpeedFromCurvature(shared_use_st *shared, int index){

	double first = 0.0;
	double AllowSpeed = FLT_MAX;
	double calcSpeed;
	double mu = ASPHALT_FRICTION_CONSTANT;
	double deltaAngle = 0.0;
	double curvature = 0.0;
	double deltadists = 0.0;

	deltaAngle = shared->track_forward_angles[index + 1] - shared->track_forward_angles[index];

	if (deltaAngle < 0)
		printf("bug\n");


	NORM_PI_PI(deltaAngle);

	deltadists = (shared->track_forward_dists[index + 1] - shared->track_forward_dists[index]);

	if ((deltadists < 0) && (index - 1 >= 0)) {
		deltadists = (shared->track_forward_dists[index] - shared->track_forward_dists[index - 1]);
	}
	else if ((deltadists < 0) && (index - 1 < 0)) {
		deltadists = (shared->track_forward_dists[index + 2] - shared->track_forward_dists[index + 1]);
	}

	curvature = (deltaAngle) / (shared->track_forward_dists[index + 1] - shared->track_forward_dists[index]);

	curvature = fabs(curvature);

	if (fp) {
		//  fprintf(fp, "%f, %f\n", shared->track_forward_dists[index], curvature);
		//fprintf(fp, "%f, %f\n", shared->track_forward_dists[index], curvature);
		//  fprintf(fp, "deltadists = %f \n", deltadists);
		//fprintf(fp, " deltadists = %f \n", deltadists);
	}
	if (curvature == 0.0)
		calcSpeed = MAX_SPEED_PER_METER;

	else {

		calcSpeed = sqrt((mu*1.5*GRAVITY_CONSTANT) / curvature);

		if (isinf(calcSpeed)) {
			calcSpeed = MAX_SPEED_PER_METER;
		}

		if (isnan(calcSpeed)) {
			calcSpeed = MAX_SPEED_PER_METER;
		}
	}

	return calcSpeed;
}

///* compute fitting acceleration */
double getaccel(shared_use_st *shared)
{
	double allowedspeed = getAllowedSpeedFromCurvature(shared, 0);

	/*  if ((shared->toStart  > 2760) && (shared->toStart  < 2770)) {
	return 0.5;
	}
	*/
	printf("allowedspeed = %f km/h speed = %f km/h \n", allowedspeed * (18 / 5), shared->speed* (18 / 5));
	/*
	if (allowedspeed > shared->speed + 1.0) {
		return 3.0;
	}
	else {
		return 0.2;
	}*/

	return allowedspeed / shared->speed;

}

double getDistToSegEnd(shared_use_st *shared)
{

	/*  printf("shared->toStart = %f\n ", shared->toStart);
	printf("shared->track_forward_dists[0] = %f\n", shared->track_forward_dists[0]);
	*/
	return shared->track_forward_dists[0] - shared->toStart;

	/*
	if (car->_trkPos.seg->type == TR_STR) {
	return car->_trkPos.seg->length - car->_trkPos.toStart;
	}
	else {
	return (car->_trkPos.seg->arc - car->_trkPos.toStart)*car->_trkPos.seg->radius;
	}
	*/
}


/* Antilocking filter for brakes */
float filterABS(shared_use_st *shared, float brake)
{
	if (shared->speed < ABS_MINSPEED) return brake;
	int i;
	float slip = 0.0;
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
	float currentspeedsqr = shared->speed*shared->speed;
	float mu = ASPHALT_FRICTION_CONSTANT;
	float mass = 1150;
	float maxlookaheaddist = currentspeedsqr / (2.0*mu*GRAVITY_CONSTANT);
	float lookaheaddist = getDistToSegEnd(shared);

	float allowedspeed = getAllowedSpeedFromCurvature(shared, 0);
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

		allowedspeed = getAllowedSpeedFromCurvature(shared, trackIndex);

		if (allowedspeed < shared->speed) {
			float allowedspeedsqr = allowedspeed*allowedspeed;
			float brakedist = (currentspeedsqr - allowedspeedsqr) / (2.0*(mu*GRAVITY_CONSTANT));// +allowedspeedsqr*(CA*mu + CW)));

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


const float WIDTHDIV = 4.0;                         /* [-] */

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

const float LOOKAHEAD_CONST = 17.0;                 /* [m] */
const float LOOKAHEAD_FACTOR = 0.33;                /* [-] */

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
	targetAngle = getTargetPoint(shared);
	printf("targetAngle = %f\n", targetAngle);
	targetAngle -= shared->track_current_angle - shared->angle;
	printf("shared->track_current_angle + shared->angle = %f\n", shared->track_current_angle + shared->angle);
	NORM_PI_PI(targetAngle);
	targetAngle -= 1.0*shared->toMiddle / shared->track_width;
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

	getDistToSegEnd(shared);

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
		shared->steerCmd = diff_angle / ((21 * M_PI) / 180);// getSteer(shared);// diff_angle / ((21 * M_PI) / 180);
		shared->brakeCmd = filterABS(shared, getBrake(shared));
		if (shared->brakeCmd == 0.0)
			shared->accelCmd = filterTrk(shared, getaccel(shared));
		else
			shared->accelCmd = 0.0;
		shared->backwardCmd = GEAR_FORWARD;

	}

//	if ((shared->toStart > 1800) && (shared->toStart < 1900))
		//shared->accelCmd = 0.2;
	
	printf("toStart = %f, accelCmd = %f\n", shared->toStart, shared->accelCmd);

	fprintf(fp, "%f, %f\n", shared->toStart, shared->accelCmd);

	//printf("shared->accel %f\n", shared->accelCmd);

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

	while (shared->connected){
		if (shared->written == 1) { // the new image data is ready to be read
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