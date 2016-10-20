#pragma once
#include<iostream>
#include<queue>

using namespace std;

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


class learn
{


public:
	vector<trackInfo*> TrackInfoQueue;

	learn();
	~learn();
	
	double getLastCurvature();
	void update(double curtrackDist, double curTrackAngle, double curCurvature, double curallowSpeed);
};

