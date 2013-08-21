BEGIN_PROJECT("executable")

SET_PRIVATE_HEADERS (
	XSENSTStickVisualizer.h
	VICONTStickVisualizer.h
	PointsCloudTStick.h
	PATTStick.h
	CustomQOSGWidget.h
	3PointsTStick.h
	ITStick.h
	TStickDrawHelpers.h
	MainWindow.h
	IMUWidget.h
	XSENSDockWidget.h
	VICONDockWidget.h
	FitDataDock.h
	FittingWidget.h
	FittingWizzard.h
	OrientationInput.h
)

SET_SOURCE_FILES (
	XSENSTStickVisualizer.cpp
	VICONTStickVisualizer.cpp
	PointsCloudTStick.cpp
	PATTStick.cpp
	CustomQOSGWidget.cpp
	3PointsTStick.cpp
	TStickDrawHelpers.cpp
	MainWindow.cpp
	IMUWidget.cpp
	main.cpp
	XSENSDockWidget.cpp
	VICONDockWidget.cpp
	FitDataDock.cpp
	FittingWidget.cpp
	FittingWizzard.cpp
	OrientationInput.cpp
)

SET_MOC_FILES(
	MainWindow.h	
	IMUWidget.h
	XSENSDockWidget.h
	VICONDockWidget.h
	FitDataDock.h
	FittingWidget.h
	FittingWizzard.h
	OrientationInput.h
)

SET_UI_FILES(
	MainWindow.ui	
	IMUWidget.ui
	FittingWidget.ui
	FittingWizzard.ui
	OrientationInput.ui
)

END_PROJECT()