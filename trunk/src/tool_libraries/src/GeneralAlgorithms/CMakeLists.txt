BEGIN_PROJECT("static")

SET_PUBLIC_HEADERS(
	CombinationGenerator/CombinationGenerator.h
	KalmanFilters/IKalmanFilter.h
	KalmanFilters/KalmanFilter.h
	KalmanFilters/IExtendedKalmanFilter.h
	KalmanFilters/EKF.h
	KalmanFilters/IUnscentedKalmanFilter.h
	KalmanFilters/UKF.h
	UnscentedTransform/UnscentedTransform.h
	DTW/DTW.h
	LiftingScheme/LiftingSchemeT.h
	LiftingScheme/IndexHelperPolicy.h
)

SET_SOURCE_FILES(
	CombinationGenerator.cpp
	DTW.cpp
)

END_PROJECT()