/**
 * @file canonPlanner.h
 * @author A. Huaman Q.
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
 * @class CanonPlanner
 */
class CanonPlanner : public QThread {

	Q_OBJECT

protected:
	CanonPlanner() {}

	Hand *mHand;
	GraspableBody *mObject;

	GraspPlanningState *mCurrentState;

	double sMaxTime;
    int sMaxTransSteps;
    int sMaxRotSteps;
    double sDx;
    double sRotStep;

	GraspPlanningState *mTargetState;

	// Render options
	int mRenderType, mRenderCount;
	// A decision is made whether to put in a redraw request to the scene graph
	void render();

	bool checkTerminationConditions();

	// For single-threaded operation
	SoSensor *mIdleSensor;
	static void sensorCallback( void *data, SoSensor*);

    std::vector<GraspPlanningState*> mBaseGrasps;
    std::vector< std::vector<GraspPlanningState*> > mSampleGrasps;


 public:

    CanonPlanner( Hand *_h );
    ~CanonPlanner();

	void init();
    bool readBaseGraspFile( std::string _filename );
    int addBaseGrasp( GraspPlanningState* _gps );
    int addSampleGrasp( int _i, GraspPlanningState* _gps );
    int addSampleGrasp( int _i, transf _T );
    bool makeGraspValid( int _i );
    bool closeSampleGrasps( int _i );
	void mainLoop();

    PostureStateDOF* mOpenPosture;

    /** Utilities */
    GraspPlanningState *getBaseGrasp( int _i );
    GraspPlanningState *getSampleGrasp( int _i, int _j );
    bool showBaseGrasp( int _i );
    bool showSampleGrasp( int _i, int _j );
    int getNumBaseGrasps() { return mBaseGrasps.size(); }
    int getNumSampleGrasps( int _i ) { return mSampleGrasps[_i].size(); }
    void setRenderType( RenderType _r ) { mRenderType = _r; }
    Hand *getHand() { return mHand; }


};
