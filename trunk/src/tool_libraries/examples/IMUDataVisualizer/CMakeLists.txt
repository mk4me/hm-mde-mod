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
	main.cpp
)

SET_MOC_FILES(
	MainWindow.h	
)

SET_UI_FILES(
	MainWindow.ui	
)

END_PROJECT()