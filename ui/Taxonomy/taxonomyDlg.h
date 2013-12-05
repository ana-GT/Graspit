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
#include "ui_taxonomyDlg.h"

class World;

/*! \class PlannerDlg
  \brief Creates and controls the grasp planner dialog box.
*/

class TaxonomyDlg : public QDialog, public Ui::TaxonomyDlgUI
{
    Q_OBJECT
	private:
    QTextStream stream;
    QFile masterFile;
    
    void init();
    void destroy() ;

 public:
 TaxonomyDlg(QWidget *parent = 0) : QDialog(parent) {
      setupUi(this);
      init();
    }
    ~TaxonomyDlg(){destroy();}

    void setWorld( World* _world ) { mWorld = _world; }
    void printInfo();

    World* mWorld;
    
    public slots:    
    void readGraspFile();
    
};
