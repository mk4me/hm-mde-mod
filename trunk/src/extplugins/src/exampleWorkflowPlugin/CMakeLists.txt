BEGIN_PLUGIN_PROJECT("dynamic")

SET_PRIVATE_HEADERS (
	Plugin.h
	exampleIntParser.h
	exampleIntProcessorStatistic.h
	exampleIntProcessorFilter.h
	exampleIntWidgetProcessorFilterConfiguration.h
	exampleIntProcessorFlopSignMul.h
	exampleIntVisualizer.h
	exampleIntVisualizerStatistics.h
	examplePins.h
	exampleIntRemoteSource.h
	IntsModel.h
	PCH.h
)

SET_PUBLIC_HEADERS (
	exampleIntStatistics.h
)

SET_SOURCE_FILES (
	Plugin.cpp
	exampleIntParser.cpp
	exampleIntProcessorStatistic.cpp
	exampleIntProcessorFilter.cpp
	exampleIntWidgetProcessorFilterConfiguration.cpp
	exampleIntProcessorFlopSignMul.cpp
	exampleIntVisualizer.cpp
	exampleIntRemoteSource.cpp
	examplePins.cpp
	exampleIntVisualizerStatistics.cpp
	IntsModel.cpp
	PCH.cpp
)

SET_MOC_FILES(
	exampleIntWidgetProcessorFilterConfiguration.h
	exampleIntVisualizerStatistics.h
	exampleIntRemoteSource.h
	IntsModel.h
 )

SET_UI_FILES(
	exampleIntWidgetProcessorFilterConfiguration.ui
)


SET_PRECOMPILED_HEADER(PCH.h PCH.cpp)

END_PLUGIN_PROJECT()