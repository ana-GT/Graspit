//######################################################################
//
// Authors: A. Huaman
//
// $Id: 
//
//######################################################################

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

/*!
  First this creates a new grasp_manager and gets the default planning and
  testing parameters.  Then it sets up number validators for the text entry
  boxes.  Finally it populates the quality measure comboBox with the names
  of the currently defined quality measures for this grasp.  If there are
  no quality measures defined, the generate button is disabled, but the user
  can add QM's by pressing the new button in this dialog box.
*/
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


/*!
  Calls on the grasp_manager to show the next planned grasp.
*/
void CanonicalPlannerDlg::readGraspFile() 
{    
 printf("Read grasp file! \n");
}

/**
 * @function printInfo
 */
void CanonicalPlannerDlg::printInfo() {

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
}


void CanonicalPlanner::setMembers( Hand *_h, GraspableBody *_b ) {
}
