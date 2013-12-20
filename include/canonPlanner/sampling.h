/*
 * @file sampling.h
 */
#pragma once

#include <QObject>
#include <QThread>
#include <vector>
#include "EGPlanner/searchStateImpl.h"
#include "EGPlanner/simAnn.h"

class Hand;
class GraspPlanningState;

class Body;
class SoSensor;

/**
 * @class canonSampling
 */
class CanonSampling  : public QThread {

	Q_OBJECT

protected:
	CanonSampling();

	Hand *mHand;
	GraspableBody *mObject;
	GraspPlanningState *mCurrentState;

	// Render options
	void render();

	SoSensor *mIdleSensor;
	static void sensorCallback( void* data, SoSensor* );


	std::vector<GraspPlanningState*> mSampleGrasps;

	// Sampling SA
	SimAnn* mSA;
	double mT;

	// Sampling Uniform


public:
	CanonSampling( Hand *_h );
	~CanonSampling();

	void init();

	void mainLoop();
	void setCurrentState( GraspPlanningState *gps );
	int storeGrasp( GraspPlanningState *gps );

	/** Utilities */
	int getNumGrasps() { return mSampleGrasps.size(); }
	GraspPlanningState *getGrasp( int _i );
	bool showGrasp( int _i );
	Hand *getHand( ) { return mHand; }

	/**< Sample neighbors */
	GraspPlanningState* SA_neighborState( GraspPlanningState *_s );

};
