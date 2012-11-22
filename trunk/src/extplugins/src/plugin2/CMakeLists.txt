BEGIN_PROJECT("dynamic")

SET_PRIVATE_HEADERS(
	PluginPrivate.h
)

SET_PUBLIC_HEADERS(
	PluginPublic.h
)

SET_SOURCE_FILES(
	Plugin.cpp
)

GENERATE_PLUGIN_ARTIFACTS()

END_PROJECT()
