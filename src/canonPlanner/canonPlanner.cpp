/**
 * @file canonPlanner.cpp
 * @author A. Huaman Quispe
 */
#include "searchState.h"
#include "grasp.h"
#include "worldElement.h"
#include "robot.h"
#include "world.h"
#include "bBox.h"

#include "debug.h"

#include <fstream>
#include <iostream>
#include <sstream>
#include <time.h>

#include "canonPlanner.h"
#include "matvec3D.h"

#include "canonPlanner/simAnnPlusPlanner.h"
#include "EGPlanner/simAnnPlanner.h"

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
  *
 */
void CanonPlanner::pausePlanner() {
	mPlanner->pausePlanner();
}

/**
 * @function getBoundingBox
 */
void CanonPlanner::getBoundingBox() {
	std::vector<BoundingBox> bvs;
	mHand->getWorld()->getBvs( mObject, 0, &bvs );
	std::cout << "Size of vector bounding box:"<<bvs.size() << std::endl;
	std::cout << "Bounding box of object: Transf is: \n"<< bvs[0].getTran() << std::endl;
	std::cout << "Bounding box of object: Half size: \n"<< bvs[0].halfSize << std::endl;
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
 * @function generateBaseGrasps
 */
void CanonPlanner::generateBaseGrasps() {

	std::vector<BoundingBox> bvs;
	mHand->getWorld()->getBvs( mObject, 0, &bvs );

	transf Tobj_box = bvs[0].getTran();
	mat3 Robj_box = Tobj_box.affine();

	transf Tw_obj = mObject->getTran();

	vec3 boxDim = bvs[0].halfSize;
	std::vector<vec3> canAxis(6);
	double offset = 100;
	canAxis[0] = vec3( offset + boxDim[0], 0, 0);
	canAxis[1] = vec3( -(offset + boxDim[0]), 0, 0);
	canAxis[2] = vec3( 0, offset + boxDim[1], 0);
	canAxis[3] = vec3( 0, -(offset + boxDim[1]), 0);
	canAxis[4] = vec3( 0, 0, offset + boxDim[2]);
	canAxis[5] = vec3( 0, 0, -(offset + boxDim[2]));

	// Set positions
	std::vector<vec3> p_obj_hand(6);

	for( int i = 0; i < 6; ++i ) {
		p_obj_hand[i] = Robj_box*canAxis[i] + Tobj_box.translation();
	}

	// Set rotations
	std::vector<mat3> r_obj_hand(6);
	r_obj_hand[0] = ( ( rotate_transf(-M_PI / 2.0, vec3::Y) ).affine() )*Robj_box.inverse();
	r_obj_hand[1] = ( ( rotate_transf(M_PI / 2.0, vec3::Y) ).affine() )*Robj_box.inverse();
	r_obj_hand[2] = ( ( rotate_transf(M_PI / 2.0, vec3::X) ).affine() )*Robj_box.inverse();
	r_obj_hand[3] = ( ( rotate_transf(-M_PI / 2.0, vec3::X) ).affine() )*Robj_box.inverse();
	r_obj_hand[4] = ( ( rotate_transf(M_PI, vec3::Y) ).affine() )*Robj_box.inverse();
	r_obj_hand[5] = Robj_box.inverse();

	// Generate base grasps
	for( int i = 0; i < 6; ++i ) {

    	GraspPlanningState *gps = new GraspPlanningState( mHand );
    	gps->setObject( mObject );

    	// Wrist position + orientation
    	gps->setPositionType( SPACE_COMPLETE );
    	gps->setPostureType( POSE_DOF );
    	gps->setRefTran( mObject->getTran() );
    	gps->reset();

		gps->getPosition()->setTran( transf( r_obj_hand[i], p_obj_hand[i] ) );
    	gps->getPosture()->copyValuesFrom( mOpenPosture );

    	addBaseGrasp( gps );
	}

}

/***
 * @function plannerUpdate
 */
void CanonPlanner::plannerUpdate() {

}
void CanonPlanner::plannerComplete() {

  std::cout << "Stopped planner with "<< mPlanner->getCurrentStep()<<" steps"<< std::endl;
  std::cout << " Running time: "<< mPlanner->getRunningTime() << std::endl;
  for( int i = 0; i < mPlanner->getListSize(); ++i ) {
    mSampleGrasps[mCounter].push_back( new GraspPlanningState( mPlanner->getGrasp(i)) );  
  }

  // Put limits back to normal
  if( isMin[mCounter] == true ) {
    mBaseGrasps[mCounter]->getPosition()->getVariable( QString( varName[mCounter].c_str() ) )->mMaxVal = 
      mStoredGps->getPosition()->getVariable( QString( varName[mCounter].c_str() ) )->mMaxVal; 
  } else {
    mBaseGrasps[mCounter]->getPosition()->getVariable( QString( varName[mCounter].c_str() ) )->mMinVal = 
      mStoredGps->getPosition()->getVariable( QString( varName[mCounter].c_str() ) )->mMinVal; ;  
  }

}

/**
 * @function mainLoop
 * @brief
 */
void CanonPlanner::mainLoop() {

  // Store important information
  mStoredGps = new GraspPlanningState( mHand );
  varLim.resize(6); varName.resize(6); isMin.resize(6);



  // 1. Generate base grasps
  generateBaseGrasps();
  
  int maxIndX, maxIndY, maxIndZ;
  int minIndX, minIndY, minIndZ;
  double maxX, maxY, maxZ;
  double minX, minY, minZ;
  
  maxIndX = 0; maxIndY = 0; maxIndZ = 0;
  minIndX = 0; minIndY = 0; minIndZ = 0;
  
  maxX = mBaseGrasps[maxIndX]->getPosition()->getVariable("Tx")->getValue();
  minX = mBaseGrasps[minIndX]->getPosition()->getVariable("Tx")->getValue();
  maxY = mBaseGrasps[maxIndX]->getPosition()->getVariable("Ty")->getValue();
  minY = mBaseGrasps[minIndX]->getPosition()->getVariable("Ty")->getValue();
  maxZ = mBaseGrasps[maxIndX]->getPosition()->getVariable("Tz")->getValue();
  minZ = mBaseGrasps[minIndX]->getPosition()->getVariable("Tz")->getValue();
  double eval;
  
  for( int i = 1; i < 6; ++i ) {
    eval = mBaseGrasps[i]->getPosition()->getVariable("Tx")->getValue();
    if(  eval > maxX ) { maxIndX = i; maxX = eval; }
    if( eval < minX ) { minIndX = i; minX = eval; }
    
    eval = mBaseGrasps[i]->getPosition()->getVariable("Ty")->getValue();
    if(  eval > maxY ) { maxIndY = i; maxY = eval; }
    if( eval < minY ) { minIndY = i; minY = eval; }
    
    eval = mBaseGrasps[i]->getPosition()->getVariable("Tz")->getValue();
    if(  eval > maxZ ) { maxIndZ = i; maxZ = eval; }
    if( eval < minZ ) { minIndZ = i; minZ = eval; }
  }
  
  //isMin varName varLim
  isMin[maxIndX] = false; varName[maxIndX] = "Tx"; varLim[maxIndX] = maxX;
  isMin[minIndX] = true; varName[minIndX] = "Tx"; varLim[minIndX] = minX;

  isMin[maxIndY] = false; varName[maxIndY] = "Ty"; varLim[maxIndY] = maxY;
  isMin[minIndY] = true; varName[minIndY] = "Ty"; varLim[minIndY] = minY;

  isMin[maxIndZ] = false; varName[maxIndZ] = "Tz"; varLim[maxIndZ] = maxZ;
  isMin[minIndZ] = true; varName[minIndZ] = "Tz"; varLim[minIndZ] = minZ;

  std::cout << "Max X: "<< maxIndX << std::endl; 
  std::cout << "Min X: "<< minIndX << std::endl; 

  std::cout << "Max Y: "<< maxIndY << std::endl; 
  std::cout << "Min Y: "<< minIndY << std::endl; 
  
  std::cout << "Max Z: "<< maxIndZ << std::endl; 
  std::cout << "Min Z: "<< minIndZ << std::endl; 


	// 2. Generate annealing search in localized area
	mCounter = 3;
	
	mPlanner = new SimAnnPlanner( mHand );
	//mBaseGrasps[mCounter]->getPosition()->getVariable( QString("Tx") )->mMaxJump = 50; // 1cm
	//mBaseGrasps[mCounter]->getPosition()->getVariable( QString("Ty") )->mMaxJump = 50; // 1cm
	//mBaseGrasps[mCounter]->getPosition()->getVariable( QString("Tz") )->mMaxJump = 50; // 1cm

	// Modify the limit so the serch is only done in the area relevant
	if( isMin[mCounter] == true ) {
	  mBaseGrasps[mCounter]->getPosition()->getVariable( QString( varName[mCounter].c_str() ) )->mMaxVal = 0; 
	} else {
	  mBaseGrasps[mCounter]->getPosition()->getVariable( QString( varName[mCounter].c_str() ) )->mMinVal = 0;  
	}

	((SimAnnPlanner*) mPlanner)->setModelState( mBaseGrasps[mCounter] );
	
	mPlanner->setEnergyType( ENERGY_CONTACT );
	mPlanner->setContactType( CONTACT_PRESET );
	mPlanner->resetPlanner();
	mPlanner->startPlanner();
	
	QObject::connect(mPlanner,SIGNAL(update()),this,SLOT(plannerUpdate()));
	QObject::connect(mPlanner,SIGNAL(complete()),this,SLOT(plannerComplete()));

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


