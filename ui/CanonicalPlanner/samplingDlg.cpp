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

	mSASampling->reset();
	mSASampling->setCurrentState( mInitState );

	PositionState* ps = mInitState->getPosition();
	for( int i = 0; i < ps->getNumVariables(); ++i ) {
		std::cout << "Variable: "<< ps->getVariable(i)->getName().toStdString() << "Min: "<< ps->getVariable(i)->mMinVal << " and Max: "<< ps->getVariable(i)->mMaxVal << std::endl;
	}

	PostureState* qs = mInitState->getPosture();
	for( int i = 0; i < qs->getNumVariables(); ++i ) {
		std::cout << "Variable: "<< qs->getVariable(i)->getName().toStdString() <<"Min: "<< qs->getVariable(i)->mMinVal << " and max: "<< qs->getVariable(i)->mMaxVal <<std::endl;
	}


	for( int i = 0; i < 30; ++i ) {
	    mSASampling->storeGrasp( mSASampling->sampleNeighbor( mInitState ) );
	}

	std::cout << "Generated samples fine with SA" << std::endl;
	
	return;
}


/**
 * @function sampleUniButton_clicked
 */
void SamplingDlg::sampleUniButton_clicked() {

	mUniSampling->reset();
	mUniSampling->setCurrentState( mInitState );

	PositionState* ps = mInitState->getPosition();
	for( int i = 0; i < ps->getNumVariables(); ++i ) {
		std::cout << "Variable name: "<< ps->getVariable(i)->getName().toStdString() << std::endl;
		std::cout << "Minimum: "<< ps->getVariable(i)->mMinVal << std::endl;
		std::cout << "Maximum: "<< ps->getVariable(i)->mMaxVal << std::endl;
	}


	for( int i = 0; i < 30; ++i ) {
	mUniSampling->storeGrasp( mUniSampling->sampleNeighbor( mInitState ) );
	}

	std::cout << "Generated samples fine with Uni" << std::endl;

	return;
}

/**
 * @function rotationTypeBox_activated
 */
void SamplingDlg::rotationTypeBox_activated( int _i ) {

	// Quaternion (see constructor for order)
	if( _i == 0 ) {
		transf T = mInitState->getPosition()->getCoreTran();
		mInitState->setPositionType( SPACE_COMPLETE );
		mInitState->getPosition()->setTran(T);
	}
	// Angle axis
	else if( _i == 1 ) {
		transf T = mInitState->getPosition()->getCoreTran();
		mInitState->setPositionType( SPACE_AXIS_ANGLE );
		mInitState->getPosition()->setTran(T);

	}
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

  mInitState = new GraspPlanningState( mHand );
  mInitState->setObject( mObject );

  mInitState->setPositionType( SPACE_COMPLETE );
  mInitState->setPostureType( POSE_EIGEN );
  mInitState->setRefTran(mObject->getTran());

  // Set all values to zero
  mInitState->reset();
  // Set Tx,Ty,Tz, Qw,Qx,Qy,Qz
  mInitState->getPosition()->setTran( mHand->getTran() );

  mSASampling = new SASampling( mHand );
  mUniSampling = new UniSampling( mHand );
}


