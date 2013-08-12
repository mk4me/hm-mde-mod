#include "PCH.h"
#include "Plugin.h"
#include "exampleIntParser.h"
#include "exampleIntVisualizer.h"
#include "exampleIntVisualizerStatistics.h"
#include "exampleIntProcessorFilter.h"
#include "exampleIntProcessorFlopSignMul.h"
#include "exampleIntProcessorStatistic.h"
#include "exampleIntRemoteSource.h"

CORE_PLUGIN_BEGIN("ExampleWorkflowPlugin", core::UID::GenerateUniqueID("{07F0085C-B1EF-4D2F-8281-785D5EA5086F}"))
CORE_PLUGIN_ADD_PARSER(ExampleIntParser)
CORE_PLUGIN_ADD_VISUALIZER(ExampleIntVisualizer)
CORE_PLUGIN_ADD_VISUALIZER(ExampleIntVisualizerStatistics)
VDF_SERVICE_BEGIN(ExampleWorkflowService, "{880E9240-FCE3-4E93-B7C3-FC58A785B9A2}")
	VDF_ADD_DATA_PROCESSOR_SIMPLE(ExampleIntProccesorFlopSignMul, core::UID::GenerateUniqueID("{20F17BB2-32BF-40FA-946A-35AEB301144B}"))
	VDF_ADD_DATA_PROCESSOR_SIMPLE(ExampleIntProccesorStatistic, core::UID::GenerateUniqueID("{40FB529B-9DFD-4A8F-B4F5-8C840CD75BBD}"))
	VDF_ADD_DATA_PROCESSOR_SIMPLE(ExampleIntProccesorFilter, core::UID::GenerateUniqueID("{232CF2AF-E13C-45AD-91D2-F378F988B2BE}"))
    VDF_ADD_DATA_SOURCE_SIMPLE(ExampleIntRemoteSource, core::UID::GenerateUniqueID("{753BAA74-FDA2-4F19-B654-F4CB9F360547}"))
VDF_SERVICE_END(ExampleWorkflowService)
CORE_PLUGIN_ADD_OBJECT_WRAPPER(Ints)
CORE_PLUGIN_ADD_OBJECT_WRAPPER(ExampleIntStatistics)
CORE_PLUGIN_END

