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
// $Id: simAnn.cpp,v 1.30 2009/05/07 19:57:26 cmatei Exp $
//
//######################################################################

#include "simAnnPlus.h"

#include <time.h>

#include "searchState.h"
#include "searchEnergy.h"

//#define GRASPITDBG
#include "debug.h"

#define TINY 1.0e-7

SimAnnPlus::SimAnnPlus() {

    setParameters(ANNEAL_DEFAULT);
    mWriteResults = false;
    mFile = NULL;
    
    if (mWriteResults) {
    } else {
	mFile = NULL;
    }
    mTotalSteps = 0;
}

SimAnnPlus::~SimAnnPlus() {
    if (mWriteResults) fclose(mFile);
}

void SimAnnPlus::writeResults(bool w) {

    if (w) {
	if (mWriteResults) {
	    DBGA("Sim ann already writing");
	    return;
	}
	mFile = fopen("simAnnPlus.txt","a");
	mWriteResults = true;
    } else {
	if (mWriteResults) {
	    fclose(mFile);
	    mFile = NULL;
	} else {
	    DBGA("Sim Ann Plus was not writing");
	}
	mWriteResults = false;
    }
}

void SimAnnPlus::setParameters(AnnealingType type) {

    switch (type) {
    case ANNEAL_DEFAULT:
	//these are the default parameters that experimentation shows work best over 8 dimensions
	//identical schedule for error and neighbors
	//error is supposed to be distance between fingers and object, so raw order of 1-100 mm
	YC = 7.0;
	HC = 7.0;
	YDIMS = 8;
	HDIMS = 8;
	NBR_ADJ = 1.0;
	ERR_ADJ = 1.0e-6;
	DEF_T0 = 1.0e6;
	DEF_K0 = 30000;
	break;
    case ANNEAL_LOOP:
	//for looping we use a slightly shorter interval
	YC = 7.0;
	HC = 7.0;
	YDIMS = 8;
	HDIMS = 8;
	NBR_ADJ = 1.0;
	ERR_ADJ = 1.0e-6;
	DEF_T0 = 1.0e6;
	DEF_K0 = 35000;
	break;
    case ANNEAL_MODIFIED:
	//different version which doesn't really work better than default
	YC = 2.38;
	YDIMS = 8;
	HC = 2.38;
	HDIMS = 8;
	NBR_ADJ = 1.0e-3;
	ERR_ADJ = 1.0e-1;
	DEF_T0 = 1.0e3;
	DEF_K0 = 0;
	break;
    case ANNEAL_STRICT:
	DBGP("Strict Sim Ann parameters!");
	//this are for searches that ONLY look at grasp quality measure
	YC = 0.72; 
	HC = 0.22;
	YDIMS = 2;
	HDIMS = 2;
	NBR_ADJ = 1.0;
	ERR_ADJ = 1.0; //meant to work with ENERGY_STRICT_AUTOGRASP which multiplies eps. gq by 1.0e2
	DEF_T0 = 10.0;
	DEF_K0 = 0; 
	break;
    case ANNEAL_ONLINE:
	//parameters for on-line search. Supposed to work really fast and in fewer dimensions
	//YC = 0.54;
	YC = 0.24;
	HC = 0.54;
	//YDIMS = 3;
	YDIMS = 2;
	HDIMS = 3;
	NBR_ADJ = 1.0;
	ERR_ADJ = 1.0e-2;
	DEF_T0 = 1.0e1;
	DEF_K0 = 100;
	break;
    default:
	fprintf(stderr,"Unknown Annealing params requested, using default!\n");
	setParameters(ANNEAL_DEFAULT);
			break;
    }
}

/**
 * @function reset
 */
void SimAnnPlus::reset() {
    srand( (unsigned)time(NULL) );
    mCurrentStep = DEF_K0;
    mT0 = DEF_T0;
}

/*! The caller passes it a HandObjectState and a calculator that can be used
	to compute the quality (or in annealing terms "energy") of a HandObjectState. 
	This function computes the next state in the annealing schedule.

	See SimAnn::Result declaration for possible return values.
*/
SimAnnPlus::Result SimAnnPlus::iterate(GraspPlanningState *currentState, SearchEnergy *energyCalculator, GraspPlanningState *targetState) {
	//using different cooling constants for probs and neighbors
	double T = cooling(mT0, YC, mCurrentStep, YDIMS);

	//attempt to compute a neighbor of the current state
	GraspPlanningState* newState;
	double energy; bool legal = false;
	int attempts = 0; int maxAttempts = 10;
	DBGP("Ngbr gen loop");
	while (!legal && attempts <= maxAttempts) {
		newState = stateNeighbor(currentState, T * NBR_ADJ, targetState);
		DBGP("Analyze state...");
		energyCalculator->analyzeState( legal, energy, newState );
		DBGP("Analysis done.");
		if (!legal) delete newState;
		attempts++;
	}

	if (!legal) {
		DBGP("Failed to compute a legal neighbor");
		//we have failed to compute a legal neighbor. 
		//whether the SimAnn should advance a step and cool down the temperature even when it fails to compute a
		//legal neighbor is debatable. Might be more interactive (especially for the online planner) if it does.
		//mCurrentStep += 1;
		return FAIL;
	}

	//we have a neighbor. Now we decide if we jump to it.
	DBGP("Legal neighbor computed; energy: " << energy )
	newState->setEnergy(energy);
	newState->setLegal(true);
	newState->setItNumber(mCurrentStep);

	//using different cooling constants for probs and neighbors
	T = cooling(mT0, HC, mCurrentStep, HDIMS);

	double P = prob(ERR_ADJ*currentState->getEnergy(), ERR_ADJ*newState->getEnergy(), T);
	double U = ((double)rand()) / RAND_MAX;
	Result r = KEEP;
	if ( P > U ) {
		DBGP("Jump performed");
		currentState->copyFrom(newState);
		r = JUMP;
	} else{
		DBGP("Jump rejected");
	}

	mCurrentStep+=1; mTotalSteps+=1;
	DBGP("Main iteration done.")
	delete newState;

	if (mWriteResults && mCurrentStep % 2 == 0 ) {
		assert(mFile);
		fprintf(mFile,"%ld %d %f %f %f %f\n",mCurrentStep,mTotalSteps,T,currentState->getEnergy(),
										 currentState->readPosition()->readVariable("Tx"),
										 targetState->readPosition()->readVariable("Tx"));
		//currentState->writeToFile(mFile);
	}

	return r;
}

/* ------------------------------ NEIGHBOR GENERATION FUNCTION -------------------------- */

GraspPlanningState* SimAnnPlus::stateNeighbor( GraspPlanningState *s, 
					       double T, 
					       GraspPlanningState *t ) {
	GraspPlanningState *sn = new GraspPlanningState(s);
	
	// For position	    
	variablePosition( sn->getPosition(), T, NULL );
	variableNeighbor( sn->getPosture(), T, NULL );
	return sn;
}

/**
 * @function variablePosition 
 */
void SimAnnPlus::variablePosition( VariableSet* _p, double _T,
				   VariableSet* _q ) {

    for( int i = 0; i < 3; ++i ) {
	variableOne( _p->getVariable(i), _T );
    }

    variableQuaternion( _p->getVariable(3),
			_p->getVariable(4),
			_p->getVariable(5),
			_p->getVariable(6),
			_T );
}

/**
 * @function variableQuaternion
 */
void SimAnnPlus::variableQuaternion( SearchVariable *_w,
				     SearchVariable *_x,
				     SearchVariable *_y,
				     SearchVariable *_z,
				     double _T) {

    Quaternion p = Quaternion( _w->getValue(), _x->getValue(),
			       _y->getValue(), _z->getValue() );
    // Quaternion q = this->uniformQuaternion();
    /*
    Quaternion r; r = Quaternion::Slerp( 0.1, p, q );
    //Quaternion r; r = Quaternion::Slerp( neighborDistribution2(_T), p, q );
    */
    
    // Generate a  random axis
    double theta = M_PI*((double)rand())/RAND_MAX;
    double phi =  -M_PI*(-1 + 2*((double)rand())/RAND_MAX );

    double xt = sin(theta)*cos(phi);
    double yt = sin(theta)*sin(phi);
    double zt = cos(theta);

    double ang = (M_PI / 2.0)*neighborDistribution(_T);
    double cs = cos(ang/2.0);
    double ss = sin(ang/2.0);
    Quaternion q = Quaternion( cs, ss*xt, ss*yt, ss*zt );
    Quaternion r; r = p*q;
    

    _w->setValue( r.w );
    _x->setValue( r.x );
    _y->setValue( r.y );
    _z->setValue( r.z );

}

Quaternion SimAnnPlus::uniformQuaternion() {

	double s = ((double)rand())/RAND_MAX;

	double gamma1; double gamma2;
	double theta1; double theta2;
	double w; double x; double y; double z;

	gamma1 = sqrt(1 - s);
	gamma2 = sqrt(s);

	theta1 = 2*M_PI*((double)rand())/RAND_MAX;
	theta2 = 2*M_PI*((double)rand())/RAND_MAX;
	w = cos(theta2)*gamma2;
	x = sin(theta1)*gamma1;
	y = cos(theta1)*gamma1;
	z = sin(theta2)*gamma2;

	return Quaternion(w,x,y,z);

}

/**
 * @function variableOne
 */
void SimAnnPlus::variableOne( SearchVariable *_var, double _T ) {

	double v,tv,conf;

	if ( _var->isFixed() ) { return; }

	v = _var->mMaxVal + 1.0; //start off illegal
	int loop = 0;
	while ( v > _var->mMaxVal || v < _var->mMinVal ) {
	    loop++;
	    v = _var->getValue() + neighborDistribution(_T) * _var->mMaxJump;

	    if (_var->isCircular()) {
		DBGP("Circular variable! " << _var->getName());
		if ( v > _var->mMaxVal) v -= _var->getRange();
		else if ( v < _var->mMinVal) v += _var->getRange();
	    }
	    if ( v > _var->mMaxVal && v - _var->mMaxVal < TINY) v = _var->mMaxVal;
	    if ( v < _var->mMinVal && v - _var->mMinVal > -TINY) v = _var->mMinVal;
	    
	    if (loop == 100) {
		DBGA("value: " << _var->getValue() << " Mj: " << _var->mMaxJump);
		DBGA("min val: " << _var->mMinVal << " max val: " << _var->mMaxVal);
		break;
	    }
	}
	if (loop > 10) DBGA("Neighbor gen loops: " << loop);

	_var->setValue(v);
}


void SimAnnPlus::variableNeighbor( VariableSet *set, 
				   double T, 
				   VariableSet *target ) {
	SearchVariable *var;
	double v,tv,conf;
	for (int i=0; i<set->getNumVariables(); i++) {
	    var = set->getVariable(i);
		if ( var->isFixed() ) continue;
		v = var->mMaxVal + 1.0; //start off illegal
		int loop = 0;
		while ( v>var->mMaxVal || v < var->mMinVal ) {
		    loop++;
		    //we have no target value; use regular sim ann neighbor distribution
		    v = var->getValue() + neighborDistribution(T) * var->mMaxJump;

		    if (var->isCircular()) {
			DBGP("Circular variable! " << var->getName());
			if ( v > var->mMaxVal) v -= var->getRange();
			else if ( v < var->mMinVal) v += var->getRange();
		    }
		    if ( v > var->mMaxVal && v - var->mMaxVal < TINY) v = var->mMaxVal;
		    if ( v < var->mMinVal && v - var->mMinVal > -TINY) v = var->mMinVal;
		    
		    if (loop == 100) {
			DBGA("value: " << var->getValue() << " Mj: " << var->mMaxJump);
			DBGA("min val: " << var->mMinVal << " max val: " << var->mMaxVal);
			if (target->getVariable(i)->isFixed()) {
			    DBGA("Target: " << tv << "; Nbr: " << biasedNeighborDistribution(T,tv - var->getValue(),conf));
			}
			break;
		    }
		}
		if (loop > 10) DBGA("Neighbor gen loops: " << loop);
		var->setValue(v);
	}
}

/* ------------------- SCHEDULING FUNCTIONS -------------------------*/
double SimAnnPlus::cooling(double t0, double c, int k, int d)
{
	double t;
	// Cauchy cooling schedule
	// return t0 / (double)k;

	//Ingber cooling schedule
	t = t0 * exp ( -c * (double)pow((double)k, 1.0/d) );
	return t;
}

double SimAnnPlus::prob(double e_old, double e_new, double T)
{
	if (e_new < e_old) return 1.0;
	return pow( M_E , (e_old - e_new) / T);
}

double SimAnnPlus::neighborDistribution(double T) {	
	double y;

	//start with uniform distribution
	double u = ((double)rand()) / RAND_MAX;
	double v = fabs( 2.0 * u - 1.0);

	//Ingber's prob. distribution
	y = T * ( pow( 1.0+1.0/T , v ) - 1 );
	if ( u<0.5) y = -1.0 * y;

	return y;
}

double SimAnnPlus::neighborDistribution2(double T) {	
	double y;

	//start with uniform distribution
	double u = ((double)rand()) / RAND_MAX;
	double v = fabs( 2.0 * u - 1.0);

	//Ingber's prob. distribution
	y = T * ( pow( 1.0+1.0/T , v ) - 1 );
	//if ( u<0.5) y = -1.0 * y;

	return y;
}

double  SimAnnPlus::neighborInverse(double T, double y) {
	double u = log(1+fabs(y)/T)/log(1+1.0/T);
	if ( y<0) u = -1.0 * u;
	return u;
}

double SimAnnPlus::biasedNeighborDistribution(double T, double in, double conf) {

    double u1,u2,u;
    double sigma_sq,mean;
    
    //we get the confidence as a linear value between 0 and 1
    
    //if we don't invert the nbr fct first, a variance of 0.1 seems to be "average confidence"
    //convert to a log so that we exploit mainly the area between 0 and 0.1
	//this also inverts it, mapping conf = 1 (max confidence) to sigma = 0 (always produce a 0)
	//if (conf < 0.01) sigma_sq = 1;
	//else sigma_sq = -log( conf ) / 10;	

	//for the case where we use the inverse neighbor, it's best to set the variance more conservatively
	//which means that most play is actually between 0.1 and 1
	//so all we gotta do is invert the confidence
	sigma_sq = 1.0 - conf;

	DBGP("In: " << in << " conf: " << conf << " simga_sq: " << sigma_sq);
	
	//the target value we get here is normalized and capped to -1..1
	//we set the mean of the gaussian to the value that the neighbor function would map to our target value
	mean = neighborInverse(T,in);
	//alternatively, we could just set it directly to the target and let the neighbor fctn map it to 
	//something depending on the temperature, like this:
	//mean = in;
	//but this means that even for infinitely strict target (confidence = 1.0), we can never reach it exactly
	//as the nbr fctn will map it to smaller and smaller steps.

	//compute a normal distribution, centered at input and clamped btw -1 and 1
	u = 2;
	double loops = 0;
	while ( u>1 || u<-1) {
		//start with uniform distribution
		u1 = ((double)rand()) / RAND_MAX;
		u2 = ((double)rand()) / RAND_MAX;

		//turn it into a normal distribution with the Box-Muller transform
		u = sqrt( -2 * log(u1) ) * cos(2*M_PI*u2);
		
		//set variance based on confidence and mean based on input
		u = mean + sqrt(sigma_sq) * u;
		loops++;
	}
	//just check that we're not spending too much time here
	if (loops > 20) DBGA("Biased distribution loops:  " << loops);

	//use it to generate Ingber neighbor
	double y = T * ( pow( 1.0+1.0/T , fabs(u) ) - 1 );
	if ( u < 0) y = -1.0 * y;
	DBGP("u: " << u << " y: " << y << " loops: " << loops);

	return y;
}