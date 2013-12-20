/*
 * @file UniSampling.h
 */
#pragma once

#include <QObject>
#include <QThread>
#include <vector>
#include "EGPlanner/searchStateImpl.h"


class Hand;
class GraspPlanningState;

class Body;
class SoSensor;

/**
 * @class UniSampling
 */
class UniSampling  : public QThread {

	Q_OBJECT

protected:
	UniSampling();

	Hand *mHand;
	GraspableBody *mObject;
	GraspPlanningState *mCurrentState;

	// Render options
	void render();

	SoSensor *mIdleSensor;
	static void sensorCallback( void* data, SoSensor* );

	std::vector<GraspPlanningState*> mSampleGrasps;

public:
	UniSampling( Hand *_h );
	~UniSampling();

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
	GraspPlanningState* sampleNeighbor( GraspPlanningState *_s );
	void variablePosition( PositionState *_p );
	void variablePosture( PostureState *_q );
	void simpleNeighbor( SearchVariable *_var );
	double uniformDistribution( double _a, double _b );
	Quaternion uniformQuaternion();
};
