/**
 * @file canonPlanner.cpp
 */
#include "simAnnPlanner.h"
#include "searchState.h"
#include "searchEnergy.h"
#include "simAnn.h"


#include "worldElement.h"
#include "robot.h"
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

	sMaxMoveSteps = 20;
	sDx = 10; // 1 cm = 0.01*1000
}

/**
 * @function ~CanonPlanner
 * @brief Destructor
 */
CanonPlanner::~CanonPlanner() {

    if (mSimAnn ) { delete mSimAnn; }

}

/**
 * @function setObject
 * @brief Set object to grasp . Not sure if it is needed
 */
void CanonPlanner::setObject( GraspableBody* _o ) {
	mObject = _o;
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
    mSampleGrasps.resize( mBaseGrasps.size() );
    return mBaseGrasps.size();
}

/**
 * @function makeGraspValid
 * @brief Move grasp along the normal and rotate around the axis
 */
bool CanonPlanner::makeGraspValid( int _i ) {

	if( _i >= mBaseGrasps.size() || _i < 0 ) {
		std::cout << "Number of base grasps: "<< mBaseGrasps.size() << std::endl;
		return false;
	}


	// Set the end effector to the corresponding grasp configuration
	printf("Executing base grasp \n");
	mBaseGrasps[_i]->execute();

	// Get the normal
	transf Tnow = mBaseGrasps[_i]->getPosition()->getCoreTran();
	mat3 R = Tnow.affine();
	vec3 N( R.element(0,2), R.element(1,2), R.element(2,2));
	transf Tmove;
	printf("N: %f %f %f \n", N[0], N[1], N[2]);

	Tmove = Tnow;
	vec3 trans = Tnow.translation();
	vec3 tf;
	vec3 t1;
	for( int j = 0; j < sMaxMoveSteps; ++j ) {
		t1[0] = N[0]*sDx*j;
		t1[1] = N[1]*sDx*j;
		t1[2] = N[2]*sDx*j;
		tf = trans - t1;
		Tmove.set( Tnow.rotation(), tf );
		vec3 moveT = Tmove.translation();
		printf("Move TF to: %f %f %f \n", moveT[0], moveT[1], moveT[2]);
		CollisionReport contactReport;
		if( mHand->setTo( Tmove, &contactReport ) == true ) { printf(" No collision yeah! \n"); }
			addSampleGrasp( _i, Tmove );
		//}


	}

	return true;
}

/**
 * @function addSampleGrasp
 */
int CanonPlanner::addSampleGrasp( int _i, transf _T ) {

	if( _i >= mSampleGrasps.size() || _i < 0 ) {
		std::cout << "Base grasps have"<<mSampleGrasps.size() <<" of size"<< std::endl;
		return -1;
	}

	GraspPlanningState* gps;
	gps = new GraspPlanningState( mHand );
	gps->setObject( mObject );
	gps->setPositionType( SPACE_COMPLETE );
	gps->setPostureType( POSE_DOF );
	gps->setRefTran( mObject->getTran() );
	gps->reset();
	gps->getPosition()->setTran( _T );
	gps->getPosture()->copyValuesFrom( mOpenPosture );

	mSampleGrasps[_i].push_back( gps );

	return mSampleGrasps[_i].size();
}
