/**
 * @file sampling.cpp
 */
#include "world.h"
#include "robot.h"
#include "grasp.h"
#include "sampling.h"

/**
 * @function CanonSampling
 * @brief Constructor
 */
CanonSampling::CanonSampling() {

}

/**
 * @function CanonSampling
 * @brief Constructor
 */
void CanonSampling::render() {

}

/**
 * @function CanonSampling
 * @brief Constructor
 */
void CanonSampling::sensorCallback( void* data, SoSensor* ) {

}

/**
 * @function CanonSampling
 * @brief Constructor
 */
CanonSampling::CanonSampling( Hand *_h ) {

	mCurrentState = NULL;

	mHand = _h;
	mObject = mHand->getGrasp()->getObject();
	init();
}

/**
 * @function CanonSampling
 * @brief Constructor
 */
CanonSampling::~CanonSampling() {

}

/**
 * @function CanonSampling
 * @brief Constructor
 */
void CanonSampling::mainLoop() {

}

/**
 * @function init
 */
void CanonSampling::init() {
	mSA = new SimAnn();
	mSA->setParameters( ANNEAL_DEFAULT );
	mSA->reset();
	mT = 1.0e6; // NO IDEA WHAT THIS IS FOR

	mSampleGrasps.resize(0);

}
/**
 * @function setCurrentState
 */
void CanonSampling::setCurrentState( GraspPlanningState *gps ) {
	if (mCurrentState) delete mCurrentState;
	mCurrentState = new GraspPlanningState( gps );
}

/**
 * @function storeGrasp
 */
int CanonSampling::storeGrasp( GraspPlanningState *gps ) {

	mSampleGrasps.push_back( gps );
	return mSampleGrasps.size();
}

/**
 * @function CanonSampling
 * @brief Constructor
 */
GraspPlanningState *CanonSampling::getGrasp( int _i ) {
	if( _i < 0 || _i >= mSampleGrasps.size() ) {
		std::cout << "Grasp index out of scope. Size of sample grasps is: "<<mSampleGrasps.size() << std::endl;
		return NULL;
	}

	return mSampleGrasps[_i];
}

/**
 * @function CanonSampling
 * @brief Constructor
 */
bool CanonSampling::showGrasp( int _i ) {

	GraspPlanningState *gps = getGrasp(_i);
	if( gps != NULL ) {
		gps->execute();
	}
}

/**
 * @function SA_neighborState
 */
GraspPlanningState* CanonSampling::SA_neighborState( GraspPlanningState *_s ) {

	return mSA->stateNeighbor( _s, mT, NULL );
}
