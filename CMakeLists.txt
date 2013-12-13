# **************************
# CMake for GraspIt
# **************************
cmake_minimum_required( VERSION 2.8 )
project( Graspit )
message( STATUS "Generate makefile/project file for Graspit" )

set( Graspit_MAJOR_VERSION "1" )
set( Graspit_MINOR_VERSION "0" )
set( PKG_DES "Ana's Graspit version" )
set( Graspit_VERSION "${Graspit_MAJOR_VERSION}.${Graspit_MINOR_VERSION}" )
set( PKG_EXTERNAL_DEPS "Qt4" )

# Options
set( CMAKE_INSTALL_PREFIX "/usr/local" )

# ******************************************
# Generate the GraspIt! CMake Config file
# ******************************************
file( GLOB cmake_mods "cmake/Modules/*.cmake" )
configure_file( ${CMAKE_SOURCE_DIR}/cmake/FindGraspItExt.cmake.in ${CMAKE_BINARY_DIR}/cmake/FindGraspItExt.cmake @ONLY )
configure_file( ${CMAKE_SOURCE_DIR}/cmake/GraspItConfig.cmake.in ${CMAKE_BINARY_DIR}/cmake/GraspItConfig.cmake @ONLY )
install( FILES ${CMAKE_BINARY_DIR}/cmake/FindGraspItExt.cmake DESTINATION share/GraspIt )
install( FILES ${CMAKE_BINARY_DIR}/cmake/GraspItConfig.cmake DESTINATION share/GraspIt )


set( PC_CONFIG_IN ${CMAKE_SOURCE_DIR}/cmake/graspit.pc.in )
set( PC_CONFIG_OUT ${CMAKE_BINARY_DIR}/cmake/graspit.pc )
configure_file( ${PC_CONFIG_IN} ${PC_CONFIG_OUT} @only )
install( FILES ${PC_CONFIG_OUT} DESTINATION lib/pkgconfig )

# Find important packages
set( CMAKE_MODULE_PATH "${CMAKE_BINARY_DIR}/cmake" )
find_package( GraspItExt REQUIRED )
include_directories( ${GraspItExt_INCLUDE_DIRS} )

# ***************************************************

# Set for all Qt4-generated files (MOC, resources)
include_directories( ${CMAKE_CURRENT_BINARY_DIR} )
include_directories( ${CMAKE_BINARY_DIR} )

# ********************************
# GraspIt SOURCE CODE 
# ********************************

# Include directories
include_directories(   src 
  src/Collision 
  include 
  include/math 
  include/Planner 
  include/EGPlanner  
  ui 
  ui/Planner 
  ui/EGPlanner
  ui/CanonicalPlanner )


# Header files
set( graspIt_headers ${graspIt_headers}
  include/bBox.h 
  include/bbox_inl.h 
  include/contact.h 
  include/contactSetting.h 
  include/debug.h 
  include/dof.h 
  include/dynamics.h 
  include/eigenGrasp.h 
  include/gloveInterface.h 
  include/graspRecord.h 
  include/gws.h 
  include/gwsprojection.h 
  include/jacobian.h 
  include/joint.h 
  include/kinematicChain.h 
  include/lmiOptimizer.h 
  include/material.h 
  include/matvec3D.h 
  include/matvecIO.h 
  include/maxdet.h 
  include/mytools.h 
  include/profiling.h 
  include/puma560.h 
  include/qhull_mutex.h 
  include/quality.h 
  include/plugin.h 
  include/SoArrow.h 
  include/SoComplexShape.h 
  include/SoTorquePointer.h 
  include/scanSimulator.h 
  include/timer_calls.h 
  include/triangle_inl.h 
  include/triangle.h 
  include/worldElementFactory.h 
  include/graspitGUI.h 
  include/graspitApp.h 
  include/dynJoint.h 
  include/arch.h 
  include/math/matrix.h 
  src/Collision/collisionInterface.h 
  src/Collision/collisionStructures.h 
  include/Planner/grasp_visualization.h 
  include/Planner/grasp_preshape.h 
  include/Planner/grasp_presenter.h 
  include/Planner/grasp_planner.h 
  include/Planner/grasp_manager.h 
  include/Planner/grasp_grasps.h 
  include/Planner/grasp_directions.h 
  include/Planner/grasp_coordinates.h 
  include/EGPlanner/search.h 
  include/EGPlanner/searchState.h 
  include/EGPlanner/searchStateImpl.h 
  include/EGPlanner/onLinePlanner.h 
  include/EGPlanner/simAnnPlanner.h 
  include/EGPlanner/timeTest.h 
  include/EGPlanner/graspTesterThread.h 
  include/EGPlanner/onLineGraspInterface.h 
  include/EGPlanner/listPlanner.h 
  include/FitParabola.h 
  include/shadow.h )

# MOC Files
set( graspIt_OBJECT_HEADERS ${graspIt_OBJECT_HEADERS}
  include/barrett.h
  include/body.h
  include/grasp.h
  include/graspitServer.h
  include/humanHand.h
  include/ivmgr.h
  include/m7.h
  include/m7tool.h
  include/mcGrip.h
  include/pr2Gripper.h
  include/robonaut.h
  include/robot.h
  include/robotiq.h
  include/worldElement.h
  include/world.h
  include/EGPlanner/egPlanner.h
  include/EGPlanner/canonPlanner.h
  include/EGPlanner/guidedPlanner.h 
  include/EGPlanner/loopPlanner.h
  include/EGPlanner/searchEnergy.h
  include/EGPlanner/simAnn.h
  include/Planner/grasp_tester.h )

# Source files
set( graspIt_sources ${graspIt_sources} 
  src/arch.cpp 
  src/barrett.cpp 
  src/bBox.cpp 
  src/body.cpp 
  src/contact.cpp 
  src/contactSetting.cpp 
  src/dof.cpp 
  src/dynamics.cpp 
  src/dynJoint.cpp 
  src/eigenGrasp.cpp 
  src/gloveInterface.cpp 
  src/grasp.cpp 
  src/graspitGUI.cpp 
  src/graspitServer.cpp 
  src/graspitApp.cpp 
  src/graspRecord.cpp 
  src/gws.cpp 
  src/gwsprojection.cpp 
  src/humanHand.cpp 
  src/ivmgr.cpp 
  src/jacobian.cpp 
  src/joint.cpp 
  src/kinematicChain.cpp 
  src/lmiOptimizer.cpp 
#  src/main.cpp 
  src/material.cpp 
  src/matvec.cpp 
  src/matvecIO.cpp 
  src/maxdet_src.cpp 
  src/mcGrip.cpp 
  src/mytools.cpp 
  src/profiling.cpp 
  src/pr2Gripper.cpp 
  src/m7.cpp 
  src/m7tool.cpp 
  src/plugin.cpp 
  src/puma560.cpp 
  src/quality.cpp 
  src/robonaut.cpp 
  src/robot.cpp 
  src/robotiq.cpp 
  src/scanSimulator.cpp 
  src/shadow.cpp 
  src/SoArrow.cpp 
  src/SoComplexShape.cpp 
  src/SoTorquePointer.cpp 	
  src/triangle.cpp 
  src/world.cpp 
  src/worldElement.cpp 
  src/worldElementFactory.cpp 
  src/math/matrix.cpp 
  src/Collision/collisionInterface.cpp 
  src/Planner/grasp_visualization.cpp 
  src/Planner/grasp_tester.cpp 
  src/Planner/grasp_preshape.cpp 
  src/Planner/grasp_presenter.cpp 
  src/Planner/grasp_planner.cpp 
  src/Planner/grasp_manager.cpp 
  src/Planner/grasp_grasps.cpp 
  src/Planner/grasp_directions.cpp 
  src/Planner/grasp_coordinates.cpp 
  src/EGPlanner/simAnn.cpp 
  src/EGPlanner/searchState.cpp 
  src/EGPlanner/searchStateImpl.cpp 
  src/EGPlanner/searchEnergy.cpp 
  src/EGPlanner/onLinePlanner.cpp 
  src/EGPlanner/egPlanner.cpp 
  src/EGPlanner/simAnnPlanner.cpp 
  src/EGPlanner/guidedPlanner.cpp 
  src/EGPlanner/loopPlanner.cpp 
  src/EGPlanner/timeTest.cpp 
  src/EGPlanner/graspTesterThread.cpp 
  src/EGPlanner/onLineGraspInterface.cpp 
  src/EGPlanner/listPlanner.cpp
  src/EGPlanner/canonPlanner.cpp )


#***************************************
# Collision Interface implementation
# **************************************
add_definitions( -DGRASPIT_COLLISION )

include_directories( src/Collision/Graspit )

set( graspIt_headers ${graspIt_headers} 
  src/Collision/Graspit/collisionModel.h 
  src/Collision/Graspit/collisionAlgorithms.h 
  src/Collision/Graspit/collisionAlgorithms_inl.h 
  src/Collision/Graspit/graspitCollision.h )

set( graspIt_sources ${graspIt_sources}
  src/Collision/Graspit/collisionModel.cpp 
  src/Collision/Graspit/collisionAlgorithms.cpp 
  src/Collision/Graspit/graspitCollision.cpp )


# *******************************************
# User Interface: Main Window and Dialogs
# *******************************************

# Include directories
include_directories( ui 
  ui/Planner 
  ui/EGPlanner
  ui/CanonicalPlanner )

# Header files
set( graspIt_headers ${graspIt_headers}
  ui/archBuilderDlg.h )

# MOC Files
set( graspIt_OBJECT_HEADERS ${graspIt_OBJECT_HEADERS}
  ui/barrettHandDlg.h 
  ui/bodyPropDlg.h 
  ui/contactExaminerDlg.h 
  ui/eigenGraspDlg.h 
  ui/gfoDlg.h 
  ui/gloveCalibrationDlg.h 
  ui/graspCaptureDlg.h 
  ui/gwsProjDlg.h 
  ui/mainWindow.h 
  ui/qmDlg.h 
  ui/settingsDlg.h
  ui/EGPlanner/compliantPlannerDlg.h
  ui/EGPlanner/egPlannerDlg.h 
  ui/Planner/plannerdlg.h
  ui/CanonicalPlanner/canonicalPlannerDlg.h )  

# Source files
set( graspIt_sources ${graspIt_sources}
  ui/mainWindow.cpp 
  ui/archBuilderDlg.cpp 
  ui/barrettHandDlg.cpp 
  ui/bodyPropDlg.cpp 
  ui/contactExaminerDlg.cpp 
  ui/eigenGraspDlg.cpp 
  ui/gfoDlg.cpp 
  ui/gloveCalibrationDlg.cpp 
  ui/graspCaptureDlg.cpp 
  ui/gwsProjDlg.cpp 
  ui/qmDlg.cpp 
  ui/settingsDlg.cpp 
  ui/Planner/plannerdlg.cpp 
  ui/EGPlanner/egPlannerDlg.cpp 
  ui/EGPlanner/compliantPlannerDlg.cpp
  ui/CanonicalPlanner/canonicalPlannerDlg.cpp )


# UI files
set( QT_FORMS ${QT_FORMS}
  ui/mainWindow.ui 
  ui/about.ui 
  ui/archBuilderDlg.ui 
  ui/barrettHandDlg.ui 
  ui/bodyPropDlg.ui 
  ui/contactExaminerDlg.ui 
  ui/eigenGraspDlg.ui 
  ui/gfoDlg.ui 
  ui/gloveCalibrationDlg.ui 
  ui/graspCaptureDlg.ui 
  ui/gwsProjDlgBase.ui 
  ui/qmDlg.ui 
  ui/qualityIndicator.ui 
  ui/settingsDlg.ui 
  ui/Planner/plannerdlg.ui 
  ui/EGPlanner/egPlannerDlg.ui 
  ui/EGPlanner/compliantPlannerDlg.ui
  ui/CanonicalPlanner/canonicalPlannerDlg.ui )

# **********************************
# Optimizer 
# **********************************
include_directories( src/optimizer )

# MOC files  
set( graspIt_OBJECT_HEADERS ${graspIt_OBJECT_HEADERS}
  src/optimizer/optimizerDlg.h )  

# Source files
set( graspIt_sources ${graspIt_sources}
  src/optimizer/optimizerDlg.cpp )

# UI files
set( QT_FORMS ${QT_FORMS} 
  src/optimizer/optimizerDlg.ui )


# *************************
# Qt4 file processing
# *************************
QT4_ADD_RESOURCES( graspIt_RESOURCES ./src/resources.qrc )
set_source_files_properties( ${graspIt_RESOURCES} PROPERTIES GENERATED ON ) 
QT4_WRAP_UI( graspIt_FORMS_HEADERS ${QT_FORMS} ) 
QT4_WRAP_CPP( graspIt_MOC_HEADERS ${graspIt_OBJECT_HEADERS} )
MESSAGe( "resources: ${graspIt_RESOURCES}")

add_executable( graspIt src/main.cpp ${graspIt_headers} ${graspIt_sources} ${graspIt_MOC_HEADERS} ${graspIt_FORMS_HEADERS} ${graspIt_RESOURCES} )  
target_link_libraries( graspIt ${GraspItExt_LIBRARIES} )
set_target_properties( graspIt PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

add_executable( tester apps/tester.cpp ${graspIt_headers} ${graspIt_sources} ${graspIt_MOC_HEADERS} ${graspIt_FORMS_HEADERS} ${graspIt_RESOURCES} )  
target_link_libraries( tester ${GraspItExt_LIBRARIES} )
set_target_properties( tester PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

add_library( graspIt_basic ${graspIt_headers} ${graspIt_sources} ${graspIt_RESOURCES} )
target_link_libraries( graspIt_basic ${GraspItExt_LIBRARIES} )
set_target_properties( graspIt_basic PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib )




