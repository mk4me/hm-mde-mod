#include <core/Plugin.h>
#include <c3dMarkers/C3DMarkersChannels.h>
#include "C3DMarkersParser.h"


CORE_PLUGIN_BEGIN("c3dMarkers", UID::GenerateUniqueID("{07596453-BF9F-4CFE-88BC-B374538206A2}"))
	CORE_PLUGIN_ADD_PARSER(C3DMarkersParser);
	CORE_PLUGIN_ADD_OBJECT_WRAPPER(AllMarkersCollection);
CORE_PLUGIN_END