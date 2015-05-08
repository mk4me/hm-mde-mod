BEGIN_PLUGIN_PROJECT("dynamic")

SET_PRIVATE_HEADERS(
	Plugin.h
	CQuatIO.h
	calibWidget.h
	filter_lib/lib_main.h
	filter_lib/IOrientationFilter.h
	filter_lib/internal/CAQKfFilter.h
	filter_lib/internal/CInstFilter.h
	filter_lib/internal/JS_BaseElements.h
	filter_lib/internal/JS_DCMatrix.h
	filter_lib/internal/JS_FiMatrix.h
	filter_lib/internal/JS_Quaternion.h
	filter_lib/internal/JS_StateVector.h
	filter_lib/internal/Utils.h
)

SET_SOURCE_FILES(
	Plugin.cpp
	CQuatIO.cpp	
	calibWidget.cpp
	filter_lib/lib_main.cpp
	filter_lib/internal/CAQKfFilter.cpp
	filter_lib/internal/CInstFilter.cpp
	filter_lib/internal/JS_BaseElements.cpp
	filter_lib/internal/JS_DCMatrix.cpp
	filter_lib/internal/JS_FiMatrix.cpp
	filter_lib/internal/JS_Quaternion.cpp
	filter_lib/internal/JS_StateVector.cpp
	filter_lib/internal/Utils.cpp
)

SET_UI_FILES(
	calib.ui
)

SET_MOC_FILES(
	calibWidget.h
)

END_PLUGIN_PROJECT()