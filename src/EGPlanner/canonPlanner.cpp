/**
 * @file canonPlanner.cpp
 * @author A. Huaman Quispe
 */
#include "searchState.h"
#include "grasp.h"
#include "worldElement.h"
#include "robot.h"
#include "world.h"

#include "debug.h"

#include <fstream>
#include <iostream>
#include <sstream>
#include <time.h>

#include "canonPlanner.h"
#include "matvec3D.h"

/**
 * @function CanonPlanner
 * @brief Constructor
 */
CanonPlanner::CanonPlanner( Hand *_h ) {

	// Set hand and object (probably the latter is not needed)
    mHand = _h;
	mObject = mHand->getGrasp()->getObject();

	init();

	// Set the openHand position
	mOpenPosture = new PostureStateDOF( _h );
	std::vector<double> openPose(8);
	openPose[0] = +0; openPose[1] = -1.5708;
	openPose[2] = -1.5708; openPose[3] = -1.5708;
	openPose[4] = -1.5708; openPose[5] = +0;
	openPose[6] = -1.5708; openPose[7] = -1.5708;
	mOpenPosture->readFromArray(openPose);

	sMaxTransSteps = 20;
	sMaxRotSteps = 6;
	sDx = 10; // 1 cm = 0.01*1000
	sRotStep = 20.0*M_PI / 180.0;
}

/**
 * @function ~CanonPlanner
 * @brief Destructor
 */
CanonPlanner::~CanonPlanner() {
}

/**
 * @function init
 */
void CanonPlanner::init() {

}

/**
 * @function readBaseGraspFile
 * @brief Read base grasps from a text file (with SI metric system)
 */
bool CanonPlanner::readBaseGraspFile( std::string _filename ) {

    std::ifstream input;
    std::string line;

    double px, py, pz;

    input.open( _filename.c_str(), std::ios::out );
    std::getline( input, line );

    std::cout << "Grasp object name: "<< line << std::endl;

    for( int i = 0; i < 6; ++i ) {

    	GraspPlanningState *gps;
    	vec3 pos; Quaternion rot;
    	vec3 z0(0,0,1);
    	vec3 n;

    	gps = new GraspPlanningState( mHand );
    	gps->setObject( mObject );

    	std::getline( input, line );
    	std::stringstream(line) >> px >> py >> pz;
    	std::getline( input, line );
    	std::stringstream(line) >> n[0] >> n[1] >> n[2];

    	pos = vec3( px*1000, py*1000, pz*1000 ); // to milimeters

    	// Wrist position + orientation
    	gps->setPositionType( SPACE_COMPLETE );
    	gps->setPostureType( POSE_DOF );
    	gps->setRefTran( mObject->getTran() );
    	gps->reset();

    	// Set initial rotation
    	vec3 nn = normalise(n);
    	double dot = z0 % nn;
    	double angle = acos( dot );
    	vec3 cross = z0 * nn;
    	vec3 axis = normalise( cross );

    	rot = Quaternion( angle, axis );

    	// Set position in grasp planning state
    	gps->getPosition()->setTran( transf(rot,pos) );
    	gps->getPosture()->copyValuesFrom( mOpenPosture );
    	addBaseGrasp( gps );

    }

	return true;
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
 * @function addSampleGrasp
 */
int CanonPlanner::addSampleGrasp( int _i, GraspPlanningState* _gps ) {

	if( _i >= mSampleGrasps.size() || _i < 0 ) {
		std::cout << "Sample grasps only have "<< mSampleGrasps.size() << " groups "<< std::endl;
		return -1;
	}

	mSampleGrasps[_i].push_back( _gps);
	return mSampleGrasps[_i].size();
}

/**
 * @function addSampleGrasp
 */
int CanonPlanner::addSampleGrasp( int _i, transf _T ) {

	if( _i >= mSampleGrasps.size() || _i < 0 ) {
		std::cout << "Base grasps have"<<mSampleGrasps.size() <<" of size"<< std::endl;
		return -1;
	}

	// Do not directly do gps( base ) or it will modify base :S
	GraspPlanningState* gps = new GraspPlanningState( mHand );
	gps->copyFrom( mBaseGrasps[_i] );
	gps->getPosition()->setTran( _T );

	mSampleGrasps[_i].push_back( gps );

	return mSampleGrasps[_i].size();
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
	mBaseGrasps[_i]->execute();

	// Get the normal
	transf Tbase = mBaseGrasps[_i]->getPosition()->getCoreTran();
	vec3 N = (Tbase.affine()).row(2);

	transf Tsample = Tbase;
	vec3 trans = Tbase.translation();
	double ang;
	transf trans2;

	for( int j = 0; j < sMaxTransSteps; ++j ) {

		Tsample.set( Tbase.rotation(), trans - N*sDx*j );

		for( int k = 0; k < sMaxRotSteps; ++k ) {
			ang = ( M_PI / sMaxRotSteps )*k;
			trans2 = Tsample*rotate_transf( ang, N );

			if( mHand->setTo( trans2*mBaseGrasps[_i]->getRefTran() ) == true ) {
				addSampleGrasp( _i, trans2 );
			}
		}

	} // end translation for

	if( mSampleGrasps[_i].size() > 0 ) { return true; }
	else { return false; }
}

/***
 * @function closeSampleGrasps
 */
bool CanonPlanner::closeSampleGrasps( int _i ) {

	// Proximal: 1,3,6
	int proximal[3] = {1,3,6};
	int distal[3] = {2,4,7};

	bool renderIt = false;
	bool stopAtContact = false;

	double *desiredSteps =  new double[ mHand->getNumDOF()];
	for( int k = 0; k < mHand->getNumDOF(); ++k ) {  desiredSteps[k] = sRotStep; }
	double *desiredVals = new double[ mHand->getNumDOF()];

	for( int j = 0; j < mSampleGrasps[_i].size(); ++j ) {
		GraspPlanningState* gps = mSampleGrasps[_i][j];

		// Set hand to position
		gps->execute();

		// Proximal
		gps->getPosture()->getHandDOF( desiredVals );

		desiredVals[ proximal[0] ] = mHand->getDOF( proximal[0] )->getMax();
		desiredVals[ proximal[1] ] = mHand->getDOF( proximal[1] )->getMax();
		desiredVals[ proximal[2] ] = mHand->getDOF( proximal[2] )->getMax();

		//mHand->moveDOFToContacts( desiredVals, desiredSteps, stopAtContact, renderIt );
		double 	*dof = new double[ mHand->getNumDOF()];
		gps->getPosture()->getHandDOF(dof);
		gps->getPosture()->storeHandDOF(dof);


	}

	return true;
}


/**
 * @function mainLoop
 * @brief
 */
void CanonPlanner::mainLoop() {

	clock_t ts, tf;
	double dt;

	ts = clock();
	for( int i = 0; i < mBaseGrasps.size(); ++i ) {
		std::cout<<"Making grasp ["<<i<<"] valid"<<std::endl;
		makeGraspValid(i);
		std::cout << "Valid samples ["<<i<<"]: "<< mSampleGrasps[i].size() << std::endl;
	}
	tf = clock();
	dt = (double)(tf - ts)/CLOCKS_PER_SEC;

	std::cout << "Time for making grasps valid: "<< dt << std::endl;

	for( int i = 0; i < mBaseGrasps.size(); ++i ) {
		ts = clock();
		std::cout<<"Making sample grasps ["<<i<<"] close"<<std::endl;
		closeSampleGrasps(i);
		tf = clock();
		dt = (double)(tf - ts)/CLOCKS_PER_SEC;
		std::cout << "Time in closing "<<i<<": "<<dt<< std::endl;
	}

	std::cout << "Done with main loop "<< std::endl;
}


/////////////////// UTILITIES ////////////////////////////

/**
 * @function getBaseGrasp
 * @brief Returns base grasp
 */
GraspPlanningState *CanonPlanner::getBaseGrasp( int _i ) {

	if( _i >= mBaseGrasps.size()  || _i < 0 || mBaseGrasps.size() == 0 ) {
		std::cout << "Base index incorrect. BaseGrasps size:"<< mBaseGrasps.size() << std::endl;
		return NULL;
	}
	return mBaseGrasps[_i];
}

/**
 * @function getSampleGrasp
 * @brief Returns sample grasp
 */
GraspPlanningState *CanonPlanner::getSampleGrasp( int _i, int _j ) {
	if( _i >= mBaseGrasps.size() || _i < 0 ) {
		return NULL;
	}
	if(_j >= mSampleGrasps[_i].size() || _j < 0 || mSampleGrasps[_i].size() == 0 ) {
		std::cout << "Sample index incorrect. SampleGrasps size:"<< mSampleGrasps[_i].size() << std::endl;
		return NULL;
	}

	return mSampleGrasps[_i][_j];
}

/**
 * @function showBaseGrasp
 * @brief Renders base grasp _i
 */
bool CanonPlanner::showBaseGrasp( int _i ) {
	const GraspPlanningState* gps = getBaseGrasp( _i );
	if( gps == NULL ) { return false; }

	gps->execute();
	return true;
}

/**
 * @function showSampleGrasp
 * @brief Renders sample grasp [_i][_j]
 */
bool CanonPlanner::showSampleGrasp( int _i, int _j ) {
	const GraspPlanningState* gps = getSampleGrasp( _i, _j );
	if( gps == NULL ) { return false; }

	gps->execute();
	return true;
}


/**
 * @function render
 * @brief
 */
void CanonPlanner::render() {

}

/**
 * @function checkTerminationConditions
 * @brief
 */
bool CanonPlanner::checkTerminationConditions() {
	return false;
}

/**
 * @function sensorCallback
 * @brief
 */
void CanonPlanner::sensorCallback( void *data, SoSensor*) {

}


