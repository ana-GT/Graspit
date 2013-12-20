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
#include "canonPlanner/SASampling.h"
#include "canonPlanner/UniSampling.h"

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

	mSASampling->setCurrentState( mHandObjectState );

	PositionState* ps = mHandObjectState->getPosition();
	for( int i = 0; i < ps->getNumVariables(); ++i ) {
		std::cout << "Variable name: "<< ps->getVariable(i)->getName().toStdString() << std::endl;
		std::cout << "Minimum: "<< ps->getVariable(i)->mMinVal << std::endl;
		std::cout << "Maximum: "<< ps->getVariable(i)->mMaxVal << std::endl;
	}


	for( int i = 0; i < 30; ++i ) {
	    mSASampling->storeGrasp( mSASampling->sampleNeighbor( mHandObjectState ) );
	}

	std::cout << "Generated samples fine with SA" << std::endl;
	
	return;
}


/**
 * @function sampleUniButton_clicked
 */
void SamplingDlg::sampleUniButton_clicked() {

	mUniSampling->setCurrentState( mHandObjectState );

	PositionState* ps = mHandObjectState->getPosition();
	for( int i = 0; i < ps->getNumVariables(); ++i ) {
		std::cout << "Variable name: "<< ps->getVariable(i)->getName().toStdString() << std::endl;
		std::cout << "Minimum: "<< ps->getVariable(i)->mMinVal << std::endl;
		std::cout << "Maximum: "<< ps->getVariable(i)->mMaxVal << std::endl;
	}


	for( int i = 0; i < 30; ++i ) {
	mUniSampling->storeGrasp( mUniSampling->sampleNeighbor( mHandObjectState ) );
	}

	std::cout << "Generated samples fine with Uni" << std::endl;

	return;
}


/**
 * @function sampleSASpinBox_valueChanged
 */
void SamplingDlg::sampleSASpinBox_valueChanged(int _i ) {

	if( mSASampling == NULL ) {
		std::cout << "No sampler yet. Generate samples first."<< std::endl;
		return;
	}

	sampleSASpinBox->setRange(0, mSASampling->getNumGrasps() - 1 );
	mSASampling->showGrasp( _i );
	return;

}

/**
 * @function sampleUniSpinBox_valueChanged
 */
void SamplingDlg::sampleUniSpinBox_valueChanged(int _i ) {

	if( mUniSampling == NULL ) {
	    std::cout << "No sampler yet. Generate samples first."<< std::endl;
	    return;
	}

	sampleUniSpinBox->setRange(0, mUniSampling->getNumGrasps() - 1 );
	mUniSampling->showGrasp( _i );
	return;
}


/**
 * @function setMembers
 */
void SamplingDlg::setMembers( Hand *_h, GraspableBody *_b ) {

  mSASampling = NULL;
  mUniSampling = NULL;

  mHand = _h;
  mObject = _b;

  mHand->getGrasp()->setObjectNoUpdate( mObject );
  mHand->getGrasp()->setGravity( false );

  mHandObjectState = new GraspPlanningState( mHand );
  mHandObjectState->setObject( mObject );
  mHandObjectState->setPositionType( SPACE_COMPLETE );
  mHandObjectState->setRefTran(mObject->getTran());
  mHandObjectState->reset();
  mHandObjectState->getPosition()->setTran( mHand->getTran() );

  mSASampling = new SASampling( mHand );
  mUniSampling = new UniSampling( mHand );
}


