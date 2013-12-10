//######################################################################
//
// Author : A. Huaman Quispe
// $Id: 
//
//######################################################################

/*! \file
  \brief Implements the PlannerDlg, the grasp planner dialog box. 
*/

//Added by qt3to4:
#include <QTextStream>
#include <QDialog>
#include <QFile>
#include "ui_canonicalPlannerDlg.h"

class World;
class GraspPlanningState;
class GraspableBody;
class Hand;
//class CanonPlanner;

/**
 * @class CanonicalPlannerDlg
 * @brief Creates and controls the grasp planner dialog box.
*/

class  CanonicalPlannerDlg : public QDialog, 
  public Ui::CanonicalPlannerDlgUI
{
    Q_OBJECT
      private:
  QTextStream stream;
  QFile masterFile;
  
  void init();
  void destroy() ;
  
 public:
 CanonicalPlannerDlg(QWidget *parent = 0) : QDialog(parent) {
    setupUi(this);
    init();
  }
  ~CanonicalPlannerDlg(){destroy();}
  
  void setWorld( World* _world ) { mWorld = _world; }
  void printInfo();
  
  World* mWorld;
  
  public slots:  
  void setMembers( Hand *_h, GraspableBody *_b );
  void readGraspFile();
  
 private:
  GraspPlanningState *mHandObjectState;
  GraspableBody *mObject;
  Hand *mHand;
  int mDisplayState;
  //CanonPlanner *mPlanner;

};
