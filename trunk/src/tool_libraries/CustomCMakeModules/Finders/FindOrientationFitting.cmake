FIND_INIT2(OrientationFitting "OrientationFitting/OrientationFitting" "OrientationFitting" "OrientationFitting" "OrientationFitting"  )
FIND_EXECUTABLE(OrientationFitting OrientationFitting)
FIND_DEPENDENCIES(OrientationFitting "GeneralAlgorithms;OSG;QT;BOOST;utils;IMU;QuatUtils")
FIND_FINISH(OrientationFitting)