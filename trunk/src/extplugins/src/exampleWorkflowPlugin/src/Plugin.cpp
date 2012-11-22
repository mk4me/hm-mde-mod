#include "PCH.h"
#include "Plugin.h"
#include "exampleIntParser.h"
#include "exampleIntVisualizer.h"
#include "exampleIntVisualizerStatistics.h"
#include "exampleIntProcessorFilter.h"
#include "exampleIntProcessorFlopSignMul.h"
#include "exampleIntProcessorStatistic.h"

CORE_PLUGIN_BEGIN("ExampleWorkflowPlugin", UID::GenerateUniqueID("{07F0085C-B1EF-4D2F-8281-785D5EA5086F}"))
CORE_PLUGIN_ADD_PARSER(ExampleIntParser)
CORE_PLUGIN_ADD_VISUALIZER(ExampleIntVisualizer)
CORE_PLUGIN_ADD_VISUALIZER(ExampleIntVisualizerStatistics)
CORE_PLUGIN_ADD_DATA_PROCESSOR(ExampleIntProccesorFilter)
CORE_PLUGIN_ADD_DATA_PROCESSOR(ExampleIntProccesorFlopSignMul)
CORE_PLUGIN_ADD_DATA_PROCESSOR(ExampleIntProccesorStatistic)
CORE_PLUGIN_ADD_OBJECT_WRAPPER(Ints)
CORE_PLUGIN_ADD_OBJECT_WRAPPER(ExampleIntStatistics)
CORE_PLUGIN_END
