/**
 * @file canonPlanner.cpp
 */
#include "simAnnPlanner.h"
#include "searchState.h"
#include "searchEnergy.h"
#include "simAnn.h"



#include "canonPlanner.h"

#include "debug.h"



/**
 * @function CanonPlanner
 * @brief Constructor
 */
CanonPlanner::CanonPlanner( Hand *_h ) {

    mHand = _h;
    init();
    mEnergyCalculator = new SearchEnergy();
    mSimAnn = new SimAnn();

	// Set the openHand position
	mOpenPosture = new PostureStateDOF( _h );
	std::vector<double> openPose(8);
	openPose[0] = +0; openPose[1] = -1.5708;
	openPose[2] = -1.5708; openPose[3] = -1.5708;
	openPose[4] = -1.5708; openPose[5] = +0;
	openPose[6] = -1.5708; openPose[7] = -1.5708;
	mOpenPosture->readFromArray(openPose);
}

/**
 * @function ~CanonPlanner
 * @brief Destructor
 */
CanonPlanner::~CanonPlanner() {

    if (mSimAnn ) { delete mSimAnn; }

}

/**
 * @function setAnnealingParameters
 * @brief 
 */
void CanonPlanner::setAnnealingParameters( AnnealingType _y ) {

    if( isActive() ) {
	DBGA("Stop planner before setting annealing parameters");
	return;
    }

    mSimAnn->setParameters(_y);
}

/**
 * @function
 * @brief
 */
bool CanonPlanner::initialized() {

    if( !mCurrentState ) { return false; }
    return true;

}

/**
 * @function setModelState
 * @brief Set current state of the environment
 */
void CanonPlanner::setModelState( const GraspPlanningState *_modelState ) {

    if( isActive() ) {
	DBGA("Cannot change model state while planner is running");
	return;
    }

    if( mCurrentState ) {
	delete mCurrentState;
    }

    mCurrentState = new GraspPlanningState( _modelState );
    mCurrentState->setEnergy( 1.0e5 );

    mCurrentState->changeHand( mHand, true );

    if( mTargetState && 
	( mTargetState->readPosition()->getType() != mCurrentState->readPosition()->getType() ||
	  mTargetState->readPosture()->getType() != mCurrentState->readPosture()->getType() ) ) {
	delete mTargetState;
	mTargetState = NULL;
    }

    if( !mTargetState ) {
	mTargetState = new GraspPlanningState( mCurrentState );
	mTargetState->reset();
	mInputType = INPUT_NONE;
    }

    invalidateReset();

}


/**
 * @function mainLoop
 * @brief
 */
void CanonPlanner::mainLoop() {

    GraspPlanningState *input = NULL;
    if( processInput() ) {
	input = mTargetState;
    }

    // Call sim ann
    SimAnn::Result result = mSimAnn->iterate( mCurrentState,
					      mEnergyCalculator,
					      input );
    if( result == SimAnn::FAIL ) {
	DBGP("Sim ann failed");
	return;
    }
    
    // Put result in list
}

/**
 * @function
 * @brief
 */
void CanonPlanner::resetParameters() {
   
}

/**
 * @function addBaseGrasp
 * @brief Add a grasp to the base grasp list. Returns the current number of base grasps
 */
int CanonPlanner::addBaseGrasp( GraspPlanningState* _gps ) {

    mBaseGrasps.push_back( _gps );
    return mBaseGrasps.size();
}

/**
 * @function makeGraspValid
 */
bool CanonPlanner::makeGraspValid( int _i ) {

	if( _i >= mBaseGrasps.size() || _i < 0 ) {
		std::cout << "Number of base grasps: "<< mBaseGrasps.size() << std::endl;
		return false;
	}


}
