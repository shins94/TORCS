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

	//���� ��������Ʈ ���� �ʿ�
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

