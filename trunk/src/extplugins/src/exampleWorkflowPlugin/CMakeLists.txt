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
	examplePins.cpp
	exampleIntVisualizerStatistics.cpp
	PCH.cpp
)

SET_MOC_FILES(
	exampleIntWidgetProcessorFilterConfiguration.h
	exampleIntVisualizerStatistics.h
 )

SET_UI_FILES(
	exampleIntWidgetProcessorFilterConfiguration.ui
)


SET_PRECOMPILED_HEADER(PCH.h PCH.cpp)

GENERATE_PLUGIN_ARTIFACTS()

END_PLUGIN_PROJECT()