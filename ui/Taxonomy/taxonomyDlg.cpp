//######################################################################
//
// Authors: A. Huaman
//
// $Id: 
//
//######################################################################

#include "taxonomyDlg.h"

#include <QLineEdit>
#include <QFileDialog>
#include <QValidator>

#include "robot.h" /**< Contains Hand */
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
void TaxonomyDlg::init()
{

}

/*!
  Deletes the grasp_manager.
*/
void TaxonomyDlg::destroy() 
{
    if (masterFile.isOpen()) masterFile.close();
}


/*!
  Calls on the grasp_manager to show the next planned grasp.
*/
void TaxonomyDlg::readGraspFile() 
{    
 printf("Read grasp file!!!!!!!!!1 \n");
}

/**
 * @function printInfo
 */
void TaxonomyDlg::printInfo() {

  assert( mWorld->getCurrentHand() );
  printf(" Num hand dof: %d \n", mWorld->getCurrentHand()->getNumDOF() );

}
