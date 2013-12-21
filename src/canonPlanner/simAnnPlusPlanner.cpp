//######################################################################
//
// GraspIt!
// Copyright (C) 2002-2009  Columbia University in the City of New York.
// All rights reserved.
//
// GraspIt! is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// GraspIt! is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with GraspIt!.  If not, see <http://www.gnu.org/licenses/>.
//
// Author(s): Matei T. Ciocarlie
//
// $Id: simAnnPlanner.cpp,v 1.16 2009/05/07 19:57:26 cmatei Exp $
//
//######################################################################

#include "simAnnPlusPlanner.h"

#include "searchState.h"
#include "searchEnergy.h"
#include "simAnnPlus.h"

//#define GRASPITDBG
#include "debug.h"

//! How many of the best states are buffered. Should be a parameter
#define BEST_LIST_SIZE 20
//! Two states within this distance of each other are considered to be in the same neighborhood
#define DISTANCE_THRESHOLD 0.3

/**
 * @function SimAnnPlusPlanner
 * @brief Constructor
 */
SimAnnPlusPlanner::SimAnnPlusPlanner( Hand *h ) {
	mHand = h;
	init();
	mEnergyCalculator = new SearchEnergy();
	mSimAnnPlus = new SimAnnPlus();
	//mSimAnn->writeResults(true);
}

/**
 * @function ~SimAnnPlusPlanner
 * @brief Destructor
 */
SimAnnPlusPlanner::~SimAnnPlusPlanner(){
	if (mSimAnnPlus) delete mSimAnnPlus;
}

/**
 * @function setAnnealingParameters
 * @brief Set parameters for the annealing search
 */
void SimAnnPlusPlanner::setAnnealingParameters(AnnealingType y) {
	if (isActive()) {
		DBGA("Stop planner before setting ann parameters");
		return;
	}
	mSimAnnPlus->setParameters(y);
}

/**
 * @function resetParameters
 * @brief Set current step to zero and reset the simulated annealing search
 */
void SimAnnPlusPlanner::resetParameters() {
	EGPlanner::resetParameters();
	mSimAnnPlus->reset();
	mCurrentStep = mSimAnnPlus->getCurrentStep();
	mCurrentState->setEnergy(1.0e8);
}

/**
 * @function initialized
 * @brief True if there is an initial state set, false otherwise
 */
bool SimAnnPlusPlanner::initialized() {
	if (!mCurrentState) return false;
	return true;
}

/**
 * @function setModelState
 * @brief Set current state and some other things that I don't get just yet
 */
void SimAnnPlusPlanner::setModelState(const GraspPlanningState *modelState) {
	if (isActive()) {
		DBGA("Can not change model state while planner is running");
		return;
	}

	if (mCurrentState) delete mCurrentState;
	mCurrentState = new GraspPlanningState(modelState);
	mCurrentState->setEnergy(1.0e5);
	//my hand might be a clone
	mCurrentState->changeHand(mHand, true);

	if (mTargetState && (mTargetState->readPosition()->getType() != mCurrentState->readPosition()->getType() ||
						 mTargetState->readPosture()->getType() != mCurrentState->readPosture()->getType() ) ) {
		delete mTargetState; mTargetState = NULL;
    }
	if (!mTargetState) {
		mTargetState = new GraspPlanningState(mCurrentState);
		mTargetState->reset();
		mInputType = INPUT_NONE;
	}
	invalidateReset();
}

/**
 * @function mainLoop
 * @brief Iterate function for single-thread
 */
void SimAnnPlusPlanner::mainLoop() {
	GraspPlanningState *input = NULL;
	if ( processInput() ) {
		input = mTargetState;
	}

	//call sim ann
	SimAnnPlus::Result result = mSimAnnPlus->iterate(mCurrentState, mEnergyCalculator, input);
	if ( result == SimAnnPlus::FAIL) {
		DBGP("Sim ann failed");
		return;
	}
	DBGP("Sim Ann success");

	//put result in list if there's room or it's better than the worst solution so far
	double worstEnergy;
	if ((int)mBestList.size() < BEST_LIST_SIZE) worstEnergy = 1.0e5;
	else worstEnergy = mBestList.back()->getEnergy();
	if (result == SimAnnPlus::JUMP && mCurrentState->getEnergy() < worstEnergy) {
		GraspPlanningState *insertState = new GraspPlanningState(mCurrentState);
		//but check if a similar solution is already in there
		if (!addToListOfUniqueSolutions(insertState,&mBestList,0.2)) {
			delete insertState;
		} else {
			mBestList.sort(GraspPlanningState::compareStates);
			while ((int)mBestList.size() > BEST_LIST_SIZE) {
				delete(mBestList.back());
				mBestList.pop_back();
			}
		}
	}
	render();
	mCurrentStep = mSimAnnPlus->getCurrentStep();
	if (mCurrentStep % 100 == 0 && !mMultiThread) emit update();
	if (mMaxSteps == 200) {DBGP("Child at " << mCurrentStep << " steps");}
}



/**
 * @function checkTerminationConditions
 * @brief Check if planner should finish (max Steps /time reached) or if it is already finished
 */
bool SimAnnPlusPlanner::checkTerminationConditions() {

	if (!isActive()) return true;
	bool termination = false;
	//max steps equal to -1 means run forever
	if (mMaxSteps != -1 && mCurrentStep >= mMaxSteps){ 
		if (!mRepeat) {
			pausePlanner();
			termination = true;
		} else {
			resetParameters();
		}
		if (!mMultiThread) {
			emit update();
		}
	} else if (mMaxTime != -1 ) {
		//check time limit
		//for now exceeding the time limit simply kills it for good
		if (getRunningTime() > mMaxTime) {
			termination = true;
			pausePlanner();
		}
	}
	if (termination) {
		emit complete();
	}
	return termination;
}
