/**
 * @file tester.cpp
 */

#include <Inventor/Qt/SoQt.h>
#include <qapplication.h>
#include <iostream>

// Customized

#include <mainWindow.h>
#include <world.h>
#include <ivmgr.h>

#include <SoComplexShape.h>
#include <SoArrow.h>
#include <SoTorquePointer.h>


/**
 * @function main
 */
int main(int argc, char **argv) {

  QApplication app( argc, argv );
  
  MainWindow* mainWindow;
  IVmgr* ivmgr;
  
  mainWindow = new MainWindow;
  SoQt::init( mainWindow->mWindow );
  
  // Initialize my Inventor additions
  SoComplexShape::initClass();
  SoArrow::initClass();
  SoTorquePointer::initClass();
 

  ivmgr = new IVmgr( (QWidget*)mainWindow->mUI->viewerHolder, "my_IVmgr" );
  ivmgr->getViewer()->getWidget()->setFocusPolicy( Qt::StrongFocus );
  ivmgr->emptyWorld();

  app.setMainWidget( mainWindow->mWindow );
  mainWindow->setMainWorld( ivmgr->getWorld() );
 

  
  printf("Loading world \n");
  ivmgr->getWorld()->load( argv[1] );
  printf("After loading world \n");
  return 0;
}

