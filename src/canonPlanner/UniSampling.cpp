/**
 * @file UniSampling.cpp
 */
#include "world.h"
#include "robot.h"
#include "grasp.h"
#include "UniSampling.h"

/**
 * @function UniSampling
 * @brief Constructor
 */
UniSampling::UniSampling() {

}

/**
 * @function render
 * @brief Rendea?
 */
void UniSampling::render() {

}

/**
 * @function sensorCallback
 * @brief Calls an iteration while window is still interactive
 */
void UniSampling::sensorCallback( void* data, SoSensor* ) {

}

/**
 * @function UniSampling
 * @brief Constructor (el firme :) )
 */
UniSampling::UniSampling( Hand *_h ) {

	mCurrentState = NULL;
	mHand = _h;
	mObject = mHand->getGrasp()->getObject();
	init();
}

/**
 * @function ~UniSampling
 * @brief Destructor
 */
UniSampling::~UniSampling() {

}

/**
 * @function mainLoop
 * @brief Execute an iteration
 */
void UniSampling::mainLoop() {

}

/**
 * @function init
 */
void UniSampling::init() {

    mSampleGrasps.resize(0);
    mCurrentState = NULL;
    srand( time(NULL) );
}
/**
 * @function setCurrentState
 */
void UniSampling::setCurrentState( GraspPlanningState *gps ) {
	if (mCurrentState) delete mCurrentState;
	mCurrentState = new GraspPlanningState( gps );
}

/**
 * @function storeGrasp
 */
int UniSampling::storeGrasp( GraspPlanningState *gps ) {

	mSampleGrasps.push_back( gps );
	return mSampleGrasps.size();
}

/**
 * @function getGrasp
 * @brief Retrieve grasp from the stored list
 */
GraspPlanningState *UniSampling::getGrasp( int _i ) {
	if( _i < 0 || _i >= mSampleGrasps.size() ) {
		std::cout << "Grasp index out of scope. Size of sample grasps is: "<<mSampleGrasps.size() << std::endl;
		return NULL;
	}

	return mSampleGrasps[_i];
}

/**
 * @function showGrasp
 * @brief What else do you expect? It shows the grasp from the list! :)
 */
bool UniSampling::showGrasp( int _i ) {

	GraspPlanningState *gps = getGrasp(_i);
	if( gps != NULL ) {
		gps->execute();
	}
}

/**
 * @function sampleNeighbor
 */
GraspPlanningState* UniSampling::sampleNeighbor( GraspPlanningState *_s ) {
    GraspPlanningState* sn = new GraspPlanningState( _s );
    variablePosition( sn->getPosition() );
    variablePosture( sn->getPosture() );
    return sn;
}

/**
 * @function variablePosition 
 * @brief By now only with position
 */
void UniSampling::variablePosition( PositionState *_p ) {

    // For translation
    //simpleNeighbor( _p->getVariable(0) ); // Tx
    //simpleNeighbor( _p->getVariable(1) ); // Ty
    //simpleNeighbor( _p->getVariable(2) ); // Tz

    // For rotation
    Quaternion p = Quaternion( _p->getVariable(3)->getValue() , _p->getVariable(4)->getValue(),
    							_p->getVariable(5)->getValue(), _p->getVariable(6)->getValue() );
    Quaternion q = this->uniformQuaternion();

    Quaternion r = Quaternion::Slerp( 0.1, p, q);

    _p->getVariable(3)->setValue( r.w );
    _p->getVariable(4)->setValue( r.x );
    _p->getVariable(5)->setValue( r.y );
    _p->getVariable(6)->setValue( r.z );
}

/**
 * @function variablePosture
 */
void UniSampling::variablePosture( PostureState *_q ) {

    for( int i = 0; i < _q->getNumVariables(); ++i ) {
	simpleNeighbor( _q->getVariable(i) );
    }
 
}

/**
 * @function simpleNeighbor
 */
void UniSampling::simpleNeighbor( SearchVariable *_var ) {
    double v;
    int iter;

	if( _var->isFixed() ) { return; }
	
	v = _var->mMaxVal + 1.0;
	iter = 0;
	while( v > _var->mMaxVal || v < _var->mMinVal ) {
	    iter++;
	    v = _var->getValue() + uniformDistribution(-1.0, 1.0)*_var->mMaxJump;
	    if( iter >= 100 ) {
		break;
	    }
	} // end while

	if( iter >= 100 ) { std::cout << "Wasn't able to get a value between the limits" << std::endl; }
	_var->setValue(v);

}

/**
 * @function uniformDistribution
 */
double UniSampling::uniformDistribution( double _a, double _b ) {

    double u = ((double)rand()) / RAND_MAX;
    double v = _a + u*( _b - _a );

    return v;
}

/**
 * @function uniformQuaternion
 */
Quaternion UniSampling::uniformQuaternion() {

	double s = ((double)rand())/RAND_MAX;

	double gamma1; double gamma2;
	double theta1; double theta2;
	double w; double x; double y; double z;

	gamma1 = sqrt(1 - s);
	gamma2 = sqrt(s);

	theta1 = 2*M_PI*((double)rand())/RAND_MAX;
	theta2 = 2*M_PI*((double)rand())/RAND_MAX;
	w = cos(theta2)*gamma2;
	x = sin(theta1)*gamma1;
	y = cos(theta1)*gamma1;
	z = sin(theta2)*gamma2;

	return Quaternion(w,x,y,z);
}
