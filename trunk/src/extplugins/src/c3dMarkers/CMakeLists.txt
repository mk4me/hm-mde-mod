BEGIN_PLUGIN_PROJECT("dynamic")

SET_PRIVATE_HEADERS(
	C3DMarkersPlugin.h
	C3DMarkersParser.h
	C3DMarkersPCH.h
)

SET_PUBLIC_HEADERS(
	C3DMarkersChannels.h
)

SET_SOURCE_FILES(
	C3DMarkersPlugin.cpp
	C3DMarkersParser.cpp
	C3DMarkersPCH.cpp
)

SET_PRECOMPILED_HEADER(C3DMarkersPCH.h C3DMarkersPCH.cpp)

GENERATE_PLUGIN_ARTIFACTS()

END_PLUGIN_PROJECT()