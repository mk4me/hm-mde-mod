BEGIN_PLUGIN_PROJECT("dynamic")

SET_SOURCE_FILES(
	QuaternionCompressorProcessor.h
	QuaternionCompressorProcessor.cpp
	QuaternionDenoiseProcessor.h
	QuaternionDenoiseProcessor.cpp
	EulerNoiseAdderProcessor.h
	EulerNoiseAdderProcessor.cpp
	QuatLiftingSchemeTProcessor.h
	QuaternionExpProcessor.h
	QuaternionExpProcessor.cpp
	QuaternionLogProcessor.h
	QuaternionLogProcessor.cpp
	QuaternionEulerConverter.h
	QuaternionEulerConverter.cpp
	Plugin.cpp
	Types.h
	DegreesToRadians.h
	DegreesToRadians.cpp
	QuaternionToAngleAxis.h
	QuaternionToAngleAxis.cpp
	SkeletonCompressorProcessor.h
	SkeletonCompressorProcessor.cpp
	QuatCompression.h
	QuatCompression.cpp
	QuatCommon.h
	QuatCommon.cpp
	QuaternionDistanceProcessor.h
	QuaternionDistanceProcessor.cpp
)

SET_MOC_FILES(
	EulerNoiseAdderProcessor.h
	QuaternionDenoiseProcessor.h
)

END_PLUGIN_PROJECT()