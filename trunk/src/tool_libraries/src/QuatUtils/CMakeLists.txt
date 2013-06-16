BEGIN_PROJECT("static")

SET_PUBLIC_HEADERS(
	QuatUtils.h
	Vec8d.h
	DualQuat.h
	DualQuatUtils.h
	QuaternionInterpolators.h
)

SET_SOURCE_FILES(
	QuatUtils.cpp
	DualQuat.cpp
	DualQuatUtils.cpp
)

END_PROJECT()