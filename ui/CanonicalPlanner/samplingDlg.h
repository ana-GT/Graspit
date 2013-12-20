/**
 * @file samplingDlg.h
 * @brief Simple Sampling interface 
 */
#pragma once

//Added by qt3to4:
#include <QTextStream>
#include <QDialog>
#include <QFile>
#include "ui_samplingDlg.h"

class World;
class GraspPlanningState;
class GraspableBody;
class Hand;
class SASampling;
class UniSampling;

/**
 * @class SamplingDlg
 * @brief Creates and controls the grasp planner dialog box.
 */
class  SamplingDlg : public QDialog, 
  public Ui::SamplingDlgUI {

  Q_OBJECT
    
    private:
  QTextStream stream;
  QFile masterFile;
  
  GraspPlanningState *mHandObjectState;
  GraspableBody *mObject;
  Hand *mHand;
  int mDisplayState;

  SASampling *mSASampling;
  UniSampling *mUniSampling;

  void init();
  void destroy();
  void startSampling();
  void stopSampling();


 public:
 SamplingDlg(QWidget *parent = 0) : QDialog(parent) {
    setupUi(this);
    init();
  }
  ~SamplingDlg(){destroy();}
    
  
  public slots:  
  void exitButton_clicked();
  void sampleSAButton_clicked();
  void sampleUniButton_clicked();

  void sampleSASpinBox_valueChanged(int);
  void sampleUniSpinBox_valueChanged(int);

  void setMembers( Hand *_h, GraspableBody *_b );

};
