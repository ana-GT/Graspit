/**
 * @file canonicalPlannerDlg.h
 * @brief Implements the PlannerDlg, the grasp planner dialog box. 
 */
#pragma once

//Added by qt3to4:
#include <QTextStream>
#include <QDialog>
#include <QFile>
#include "ui_canonicalPlannerDlg.h"

class World;
class GraspPlanningState;
class GraspableBody;
class Hand;
class CanonPlanner;
//class CanonPlanner;

/**
 * @class CanonicalPlannerDlg
 * @brief Creates and controls the grasp planner dialog box.
 */
class  CanonicalPlannerDlg : public QDialog, 
  public Ui::CanonicalPlannerDlgUI {

  Q_OBJECT
    
    private:
  QTextStream stream;
  QFile masterFile;
  
  GraspPlanningState *mHandObjectState;
  GraspableBody *mObject;
  Hand *mHand;
  int mDisplayState;
  CanonPlanner *mPlanner;

  void init();
  void destroy();
  void startPlanner();
  void stopPlanner();


  int mCurrentBaseIndex;
  int mCurrentSampleIndex;

 public:
 CanonicalPlannerDlg(QWidget *parent = 0) : QDialog(parent) {
    setupUi(this);
    init();
  }
  ~CanonicalPlannerDlg(){destroy();}
    
  
  public slots:  
  void exitButton_clicked();
  void plannerStart_clicked();
  void readFile_clicked();
  void baseBox_valueChanged(int);
  void sampleBox_valueChanged(int);
  void setMembers( Hand *_h, GraspableBody *_b );

};
