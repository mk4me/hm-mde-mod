BEGIN_PROJECT("static")

SET_PUBLIC_HEADERS(
	Algorithms/OrientationFitting.h
	Data/IMUDataSample.h
	Data/VICONDataSample.h
	Data/XSENSDataSample.h
	OrientationTestingFramework/CSVTestResultSerializer.h
	OrientationTestingFramework/IIMUOrientationEstimator.h
	OrientationTestingFramework/TestingFramework.h
	OrientationTestingFramework/ITestResultSerializer.h
	OrientationTestingFramework/MJ_UKF_OrientationEstimator.h
	Parsers/FileCSVParser.h
	Parsers/ICSVParser.h
	Parsers/IMUDataReader.h
	Parsers/StreamCSVParser.h
	Parsers/VICONDataReader.h
	Parsers/XSENSDataReader.h
	Parsers/UniversalDataReader.h
)

SET_SOURCE_FILES(
	OrientationFitting.cpp
	CSVTestResultSerializer.cpp
	FileCSVParser.cpp
	TestingFramework.cpp
	MJ_UKF_OrientationEstimator.cpp
	StreamCSVParser.cpp
	VICONDataReader.cpp
	VICONDataSample.cpp
	XSENSDataReader.cpp
	IMUDataReader.cpp
)

END_PROJECT()