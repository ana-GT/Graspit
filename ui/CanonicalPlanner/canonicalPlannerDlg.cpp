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

void CanonicalPlannerDlg::init()
{

}

/*!
  Deletes the grasp_manager.
*/
void CanonicalPlannerDlg::destroy() 
{
    if (masterFile.isOpen()) masterFile.close();
}


/**
 *  @function CanonicalPlannerDlg::exitButton_clicked
 */
void CanonicalPlannerDlg::exitButton_clicked() {

}

/*!
  Calls on the grasp_manager to show the next planned grasp.
*/
void CanonicalPlannerDlg::readGraspFile() 
{    
 printf("Read grasp file! \n");
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
