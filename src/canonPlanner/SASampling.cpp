/**
 * @file SASampling.cpp
 */
#include "world.h"
#include "robot.h"
#include "grasp.h"
#include "SASampling.h"

/**
 * @function SASampling
 * @brief Constructor
 */
SASampling::SASampling() {

}

/**
 * @function SASampling
 * @brief render
 */
void SASampling::render() {

}

/**
 * @function sensorCallback
 * @brief sensorCallback
 */
void SASampling::sensorCallback( void* data, SoSensor* ) {

}

/**
 * @function SASampling
 * @brief Constructor
 */
SASampling::SASampling( Hand *_h ) {

	mHand = _h;
	mObject = mHand->getGrasp()->getObject();
	init();
}

/**
 * @function SASampling
 * @brief Destructor
 */
SASampling::~SASampling() {
	reset();
}

/**
 * @function SASampling
 * @brief mainLoop
 */
void SASampling::mainLoop() {

}

/**
 * @function init
 */
void SASampling::init() {

    mSampleGrasps.resize(0);
    mCurrentState = NULL;
    // Initialize random seed
    srand( time(NULL) );
}

/**
 * @function reset
 * @brief Erases all stored grasps
 */
void SASampling::reset() {

	while( !mSampleGrasps.empty() ) {
		delete( mSampleGrasps.back() );
		mSampleGrasps.pop_back();
	}
}

/**
 * @function setCurrentState
 */
void SASampling::setCurrentState( GraspPlanningState *gps ) {

	if (mCurrentState) delete mCurrentState;
	mCurrentState = new GraspPlanningState( gps );
}

/**
 * @function storeGrasp
 */
int SASampling::storeGrasp( GraspPlanningState *gps ) {

	mSampleGrasps.push_back( gps );
	return mSampleGrasps.size();
}

/**
 * @function getGrasp
 * @brief Retrieve a grasp from the stored list
 */
GraspPlanningState *SASampling::getGrasp( int _i ) {
	if( _i < 0 || _i >= mSampleGrasps.size() ) {
		std::cout << "Grasp index out of scope. Size of sample grasps is: "<<mSampleGrasps.size() << std::endl;
		return NULL;
	}

	return mSampleGrasps[_i];
}

/**
 * @function showGrasp
 * @brief Render grasp from the list
 */
bool SASampling::showGrasp( int _i ) {

	GraspPlanningState *gps = getGrasp(_i);
	if( gps != NULL ) {
	    gps->execute();
	}
}

/**
 * @function SA_neighborState
 */
GraspPlanningState* SASampling::sampleNeighbor( GraspPlanningState *_s, double _T ) {

    GraspPlanningState* sn = new GraspPlanningState(_s);
    double x,y,z;
    x = sn->getPosition()->getVariable(0)->getValue();
    y = sn->getPosition()->getVariable(1)->getValue();
    z = sn->getPosition()->getVariable(2)->getValue();

    variableNeighbor( sn->getPosition(), _T );
    variableNeighbor( sn->getPosture(), _T );

    sn->getPosition()->getVariable(0)->setValue(x);
    sn->getPosition()->getVariable(1)->setValue(y);
    sn->getPosition()->getVariable(2)->setValue(z);


    return sn;
}

/**
 * @function variableNeighbor
 */
void SASampling::variableNeighbor( VariableSet *_set,
				   double _T ) {

    SearchVariable *var;
    double v;
    int iter;

    for( int i = 0; i < _set->getNumVariables(); ++i ) {
	var = _set->getVariable(i);
	if( var->isFixed() ) { continue; }
	
	v = var->mMaxVal + 1.0;
	iter = 0;
	while( v > var->mMaxVal || v < var->mMinVal ) {
	    iter++;
	    v = var->getValue() + neighborDistribution(_T)*var->mMaxJump;
	    if( iter == 100 ) {
		break;
	    }
	} // end while

	if( iter > 10 ) { std::cout << "Wasn't able to get a value between the limits" << std::endl; }
	var->setValue(v);

    } // end for

}


/**
 * @function neighborDistribution
 */
double SASampling::neighborDistribution( double _T ) {

    double y;
    // Start with uniform distribution
    double u = ((double)rand())/RAND_MAX;
    double v = fabs( 2.0*u - 1.0);

    // Ingber's probability distribution
    y = _T*( pow(1.0+1.0/_T, v) - 1 );

    if( u < 0.5 ) y = -1.0*y;
    return y;
}
