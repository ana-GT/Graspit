/*
 * @file SASampling.h
 * @brief Sampling used by Simulated Annealer
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
 * @class SASampling
 */
class SASampling  : public QThread {

	Q_OBJECT

protected:
	SASampling();

	Hand *mHand;
	GraspableBody *mObject;
	GraspPlanningState *mCurrentState;

	// Render options
	void render();

	SoSensor *mIdleSensor;
	static void sensorCallback( void* data, SoSensor* );


	std::vector<GraspPlanningState*> mSampleGrasps;

public:
	SASampling( Hand *_h );
	~SASampling();

	void init();
	void reset();

	void mainLoop();
	void setCurrentState( GraspPlanningState *gps );
	int storeGrasp( GraspPlanningState *gps );

	/** Utilities */
	int getNumGrasps() { return mSampleGrasps.size(); }

	GraspPlanningState *getGrasp( int _i );
	bool showGrasp( int _i );
	Hand *getHand( ) { return mHand; }

	/**< Sample neighbors */
	GraspPlanningState* sampleNeighbor( GraspPlanningState *_s,
					    double _T=1.0e6 );
	void variableNeighbor( VariableSet *_set,
			       double _T );
	double neighborDistribution( double _T );
};
