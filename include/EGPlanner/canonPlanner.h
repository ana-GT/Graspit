/**
 * @file canonPlanner.h
 * @author A. Huaman Q.
 */
#pragma once

#include <vector>
#include <list>
#include "egPlanner.h"
#include "EGPlanner/searchStateImpl.h"

class Hand;
class GraspPlanningState;
class SimAnn;

class Body;
class SoSensor;
class SearchEnergy;


/**
 * @class CanonPlanner
 */
class CanonPlanner : public EGPlanner {

 public:

    CanonPlanner( Hand *_h );
    ~CanonPlanner();
    virtual PlannerType getType() { return PLANNER_GT; } // Just to put a name
    void setAnnealingParameters( AnnealingType _y );
    virtual bool initialized();
    virtual void setModelState( const GraspPlanningState *_modelState );

    // graspSys *********************
    void setObject( GraspableBody* _o );
    int addBaseGrasp( GraspPlanningState* _gps );
    int addSampleGrasp( int _i, transf _T );
    bool makeGraspValid( int _i );
    position rotation;
    Quaternion translation;

    std::vector<GraspPlanningState*> mBaseGrasps;
    std::vector< std::vector<GraspPlanningState*> > mSampleGrasps;
    PostureStateDOF* mOpenPosture;
    GraspableBody *mObject;

    int sMaxMoveSteps;
    double sDx;
    // graspSys *********************


 protected:
    CanonPlanner() {};
    void mainLoop();
    void resetParameters();

    SimAnn *mSimAnn;

};
