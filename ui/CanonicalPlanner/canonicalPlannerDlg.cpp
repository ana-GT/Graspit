/**
 * @Authors: A. Huaman
 */


#include "canonicalPlannerDlg.h"

#include <QLineEdit>
#include <QFileDialog>
#include <QValidator>

#include "robot.h" /**< Contains Hand */
#include "gws.h"
#include "mainWindow.h"
#include "graspitGUI.h"
#include "grasp_manager.h"
#include "grasp_tester.h"
#include "grasp_planner.h"
#include "grasp.h"
#include "world.h"
#include "ivmgr.h"
#include "quality.h"
#include "mytools.h"

#include "EGPlanner/searchState.h"

#include "simAnnPlanner.h"
#include "EGPlanner/canonPlanner.h"

#include <fstream>
#include <iostream>
#include <sstream>

void CanonicalPlannerDlg::init() {

}

/*!
  Deletes the grasp_manager.
*/
void CanonicalPlannerDlg::destroy() 
{
    if (masterFile.isOpen()) masterFile.close();
}


/**
 *  @function exitButton_clicked
 */
void CanonicalPlannerDlg::exitButton_clicked() {
    QDialog::accept();
}

/**
 * @function plannerStart_clicked
 */
void CanonicalPlannerDlg::plannerStart_clicked() {

	printf("Set start planner \n");
	for( int i = 0; i < mPlanner->mBaseGrasps.size(); ++i ) {
		printf("Making grasp %d valid \n", i);
		mPlanner->makeGraspValid(i);
		printf(" Number of grasps valid for %d: %d \n", i, mPlanner->mSampleGrasps[i].size() );
	}
	printf("End planner \n");

}

/**
 * @function readFile_clicked()
 */
void CanonicalPlannerDlg::readFile_clicked() {

	mPlanner = new CanonPlanner( mHand );
	mPlanner->setObject( mObject );
    QString fn( QFileDialog::getOpenFileName( this,
					      QString(),
					      "/home/ana/Code/manipulation/graspTypes/data",
					      "Grasp files (*.txt)") );
    if( fn.isEmpty() ) { return; }

    //fileName = fn;
    std::cout << "Filename :"<< fn.toStdString() << std::endl;
    
    std::ifstream input;
    std::string line;
    
    double px, py, pz;
    double nx, ny, nz;

    input.open( fn.toStdString().c_str(), std::ios::out );
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
	std::stringstream(line) >> pos[0] >> pos[1] >> pos[2];
	std::getline( input, line );
	std::stringstream(line) >> n[0] >> n[1] >> n[2];

	// Set position based on the read input
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

	// Set position in gps
	transf Tf( rot, pos );
	gps->getPosition()->setTran( Tf );
	gps->getPosture()->copyValuesFrom( mPlanner->mOpenPosture );
	mPlanner->addBaseGrasp( gps );

    }

    std::cout << "Finished reading "<< mPlanner->mBaseGrasps.size() << std::endl;
}


/**
 * @function baseBox_valueChanged
 */
void CanonicalPlannerDlg::baseBox_valueChanged(int _i ) {

	if( _i < 0 || _i >= mPlanner->mBaseGrasps.size() ) {
		std::cout << "Index exceeds the number of base grasps YOU IDIOT!"<< std::endl;
		return;
	}

	mCurrentBaseIndex = _i;
	mPlanner->mBaseGrasps[mCurrentBaseIndex]->execute();
	return;
}

/**
 * @function sampleBox_valueChanged
 */
void CanonicalPlannerDlg::sampleBox_valueChanged(int _i ) {

	if( _i < 0 || _i >= (mPlanner->mSampleGrasps[mCurrentBaseIndex]).size() ) {
		std::cout << "Index exceeds the number of base grasps"<< std::endl;
		return;
	}

	mCurrentSampleIndex = _i;
	if( ! mPlanner->mSampleGrasps[mCurrentBaseIndex][mCurrentSampleIndex]->execute() ) {
		printf("Execution false!!! \n");
	}

	// Get hand
	transf T;
	mat3 r;
	vec3 t; t[0] = 300;
	T.set( r, t);

	transf ana = mPlanner->mSampleGrasps[mCurrentBaseIndex][mCurrentSampleIndex]->getPosition()->getCoreTran();
	std::cout << "My trans: "<<ana << std::endl;
	std::cout << "The 3900: "<<T << std::endl;
	mHand->setTran(ana );
	return;
}


/**
 * @function setMembers
 */
void CanonicalPlannerDlg::setMembers( Hand *_h, GraspableBody *_b ) {

  mPlanner = NULL;
  mHand = _h;
  mObject = _b;
  mHand->getGrasp()->setObjectNoUpdate( mObject );
  mHand->getGrasp()->setGravity( false );

  mHandObjectState = new GraspPlanningState( mHand );
  mHandObjectState->setObject( mObject );
  mHandObjectState->setPositionType( SPACE_AXIS_ANGLE );
  mHandObjectState->setRefTran(mObject->getTran());
  mHandObjectState->reset();

}


