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

    // Create planner according to the type selected
    SimAnnPlanner* mSP;
    mSP = new SimAnnPlanner( mHand );

    // Init planner
    mSP->setModelState( mHandObjectState );

    // Set planner settings
    mSP->setEnergyType( ENERGY_CONTACT );

    // Contact type
    mSP->setContactType( CONTACT_PRESET );

    // Number of steps
    mSP->setMaxSteps( 70000 );

    // Reset parameters (count)
    mSP->resetPlanner();

    // Check status
    int status = mSP->getState();
    if( status == READY ) { 
	printf("Planner ready to run \n");
	mSP->startPlanner();
	printf("Before mainLoop \n");
	//mSP->mainLoop();
	printf("After main loop \n");
	printf("Finished planner!! \n");
    }
    else { printf("Planner in a status other than ready \n"); }



    printf("End clicked \n");


}

/**
 * @function readFile_clicked()
 */
void CanonicalPlannerDlg::readFile_clicked() {

	mPlanner = new CanonPlanner( mHand );

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
 * @function showBaseGrasps_clicked
 * @brief Show each of the 6 base grasps once per second
 */
void CanonicalPlannerDlg::showBaseGrasps_clicked() {

		int index = indiceBox->value();
		if( index < 0 && index >= mPlanner->mBaseGrasps.size() ) {
			std::cout << "Index exceeds the number of base grasps"<< std::endl;
		}
		mPlanner->mBaseGrasps[index]->execute();

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


/**
 * @function printInfo
 */
void CanonicalPlannerDlg::printInfo() {
    printf("Print \n");
    /*
  assert( mWorld->getCurrentHand() );
  printf(" Num hand dof: %d \n", mWorld->getCurrentHand()->getNumDOF() );
 // Get grasp
 Grasp* g; g = mWorld->getCurrentHand()->getGrasp();	
  if( g == NULL ) { printf("No grasp! \n"); }
  else { 
	 printf("Grasp! \n"); 

     // Close it, in case it was not closed
  	mWorld->getCurrentHand()->autoGrasp(true);
  	mWorld->updateGrasps();
     // Create a GWS 
     GWS* gws; gws = g->addGWS("L1 Norm");
    g->update(Grasp::ALL_DIMENSIONS);
    bool isForceClosure = gws->forceClosure;
    if( isForceClosure == false ) { printf("[BAD] It is not force closure! \n"); }
    else { printf("[GOOD] It is force closure! \n");}
 }
  */
}
