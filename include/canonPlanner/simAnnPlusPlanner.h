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
// $Id: simAnnPlanner.h,v 1.12 2009/05/07 19:57:46 cmatei Exp $
//
//######################################################################

#pragma once 

#include <vector>
#include <list>

#include "QObject"
#include "search.h"
#include "egPlanner.h"

class Hand;
class Body;
class GraspPlanningState;
class SoSensor;
class SearchEnergy;
class SimAnnPlus;

/**
 * @class SimAnnPlusPlanner
 */
class SimAnnPlusPlanner : public EGPlanner
{
protected:
	//! The instance that is used to do simulated annealing
	SimAnnPlus *mSimAnnPlus;
	SimAnnPlusPlanner(){}
	//! Calls a simulated annealing step and buffers the best solutions
	void mainLoop();
	//! Also resets the simulated annealer
	virtual void resetParameters();
public:
	//! Also initializes the simulated annealer
	SimAnnPlusPlanner(Hand *h);
	~SimAnnPlusPlanner();
	virtual PlannerType getType(){return PLANNER_SIM_ANN;}
	virtual void pausePlanner();
	void setAnnealingParameters(AnnealingType y);

	virtual bool checkTerminationConditions();

	virtual bool initialized();
	virtual void setModelState(const GraspPlanningState *modelState);
};
