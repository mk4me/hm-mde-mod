BEGIN_PROJECT("static")

SET_PUBLIC_HEADERS(
	QuatUtils.h
	Vec8d.h
	DualQuat.h
	DualQuatUtils.h
	QuaternionInterpolators.h
	QuaternionCompressor.h
)

SET_SOURCE_FILES(
	QuatUtils.cpp
	DualQuat.cpp
	DualQuatUtils.cpp
	QuaternionCompressor.cpp
)

END_PROJECT()