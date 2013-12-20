/**
 * @file canonicalPlannerDlg.cpp
 * @Authors: A. Huaman
 */


#include "samplingDlg.h"

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
#include "canonPlanner/sampling.h"


/**
 * @function init
 */
void SamplingDlg::init() {

}

/**
 * @function destroy
 * @brief Deletes the grasp_manager.
 */
void SamplingDlg::destroy()  {

	if (masterFile.isOpen()) masterFile.close();
}


/**
 *  @function exitButton_clicked
 */
void SamplingDlg::exitButton_clicked() {
    QDialog::accept();
}

/**
 * @function sampleSAButton_clicked
 */
void SamplingDlg::sampleSAButton_clicked() {
	printf("Sample SA CLICKED! \n");
	mSampling->setCurrentState( mHandObjectState );

	PositionState* ps = mHandObjectState->getPosition();
	for( int i = 0; i < ps->getNumVariables(); ++i ) {
		std::cout << "Variable name: "<< ps->getVariable(i)->getName().toStdString() << std::endl;
		std::cout << "Minimum: "<< ps->getVariable(i)->mMinVal << std::endl;
		std::cout << "Maximum: "<< ps->getVariable(i)->mMaxVal << std::endl;
	}


	for( int i = 0; i < 30; ++i ) {
	mSampling->storeGrasp( mSampling->SA_neighborState( mHandObjectState ) );
	}

	std::cout << "Generated samples fine with SA" << std::endl;

	return;
}

/**
 * @function sampleSASpinBox_valueChanged
 */
void SamplingDlg::sampleSASpinBox_valueChanged(int _i ) {

	if( mSampling == NULL ) {
		std::cout << "No sampler yet. Generate samples first."<< std::endl;
		return;
	}

	sampleSASpinBox->setRange(0, mSampling->getNumGrasps() - 1 );
	mSampling->showGrasp( _i );
	return;

}

/**
 * @function plannerStart_clicked
 * @brief Make grasps valid, close them and store them
 */
void SamplingDlg::samplingStart_clicked() {

    //	mPlanner->mainLoop();
}


/**
 * @function baseBox_valueChanged
 */
void SamplingDlg::baseBox_valueChanged(int _i ) {
    /*
	if( mPlanner == NULL ) {
		std::cout << "No planner yet. Load base grasps first."<< std::endl;
		return;
	}

	baseBox->setRange(0, mPlanner->getNumBaseGrasps() - 1 );
	sampleBox->setRange(0, mPlanner->getNumSampleGrasps(baseBox->value()) - 1 );
	mPlanner->showBaseGrasp( baseBox->value() );*/
	return;
}

/**
 * @function sampleBox_valueChanged
 */
void SamplingDlg::sampleBox_valueChanged(int _j ) {
    /*
	if( mPlanner == NULL ) {
		std::cout << "No planner yet. Load base grasps first."<< std::endl;
		return;
	}

	sampleBox->setRange(0, mPlanner->getNumSampleGrasps(baseBox->value()) - 1 );
	mPlanner->showSampleGrasp( baseBox->value(), _j );
    */
}


/**
 * @function setMembers
 */
void SamplingDlg::setMembers( Hand *_h, GraspableBody *_b ) {

  mSampling = NULL;
  mHand = _h;
  mObject = _b;
  mHand->getGrasp()->setObjectNoUpdate( mObject );
  mHand->getGrasp()->setGravity( false );

  mHandObjectState = new GraspPlanningState( mHand );
  mHandObjectState->setObject( mObject );
  mHandObjectState->setPositionType( SPACE_COMPLETE );
  mHandObjectState->setRefTran(mObject->getTran());
  mHandObjectState->reset();

  mSampling = new CanonSampling( mHand );

}


