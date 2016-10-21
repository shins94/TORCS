#include "learn.h"


learn::learn()
{
}


learn::~learn()
{
	trackInfo *ptrTrackinfo = NULL;

	for (int i = 0; i < TrackInfoQueue.size(); i++){

		ptrTrackinfo = (trackInfo *)TrackInfoQueue[i];
		delete ptrTrackinfo;
	}
}

double learn::getLastCurvature() {




	trackInfo * ptrTrackinfo = (trackInfo *)TrackInfoQueue.back();

	if (ptrTrackinfo == NULL)
		return 0.0;

	return ptrTrackinfo->curvature;
}

void learn::update(double curtrackDist, double curTrackAngle, double curCurvature, double curallowSpeed) {

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