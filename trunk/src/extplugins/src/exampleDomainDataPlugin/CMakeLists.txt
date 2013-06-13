BEGIN_PLUGIN_PROJECT("dynamic")

SET_PRIVATE_HEADERS(
	exampleScalarChannelProcessor.h
	exampleScalarConfigurationWidget.h
)

SET_SOURCE_FILES(
	Plugin.cpp
	exampleScalarChannelProcessor.cpp
	exampleScalarConfigurationWidget.cpp
)

SET_MOC_FILES(
	exampleScalarConfigurationWidget.h
)

SET_UI_FILES(
	exampleScalarConfigurationWidget.ui
)

END_PLUGIN_PROJECT()
