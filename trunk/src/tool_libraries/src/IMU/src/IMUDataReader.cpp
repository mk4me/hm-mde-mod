#include <IMU/Parsers/IMUDataReader.h>
#include <IMU/Data/IMUDataSample.h>
#include <IMU/Parsers/FileCSVParser.h>
#include <boost/bind.hpp>

using namespace IMU;

IMUDataReader::IMUDataReader(const std::string & path) : parser(new FileCSVParser(path, true)),
	dataReader(parser.get(), DataReader::Extractor(boost::bind(&IMUDataReader::convert, _1, _2)), 11)
{
	parser->skipNext(1);
}

IMUDataReader::~IMUDataReader()
{
	
}

const IMUDataReader::ResultType IMUDataReader::readNextSample(IMUDataSample & imuDataSample)
{
	return dataReader.readNextValue(imuDataSample);
}

const ICSVParser::ConversionResultType IMUDataReader::convert(const ICSVParser::LineData & lineData, IMUDataSample & imuDataSample)
{

	ICSVParser::ConversionResultType ret = ICSVParser::CONVERSION_OK;

	// identyfikator czasu
	IMUDataSample::TimeIDType timeID = 0;
	IMUDataSample::TimeIDType sequenceID = 0;
	Vec3 accelerometer;
	Vec3 magnetometer;
	Vec3 gyroscope;

	if(ICSVParser::convert(lineData, 0, timeID) == ICSVParser::CONVERSION_OK &&
		ICSVParser::convert(lineData, 1, sequenceID) == ICSVParser::CONVERSION_OK &&
		ICSVParser::convert(lineData, 2, accelerometer.x()) == ICSVParser::CONVERSION_OK &&
		ICSVParser::convert(lineData, 3, accelerometer.y()) == ICSVParser::CONVERSION_OK &&
		ICSVParser::convert(lineData, 4, accelerometer.z()) == ICSVParser::CONVERSION_OK &&
		ICSVParser::convert(lineData, 5, magnetometer.x()) == ICSVParser::CONVERSION_OK &&
		ICSVParser::convert(lineData, 6, magnetometer.y()) == ICSVParser::CONVERSION_OK &&
		ICSVParser::convert(lineData, 7, magnetometer.z()) == ICSVParser::CONVERSION_OK &&
		ICSVParser::convert(lineData, 8, gyroscope.x()) == ICSVParser::CONVERSION_OK &&
		ICSVParser::convert(lineData, 9, gyroscope.y()) == ICSVParser::CONVERSION_OK &&
		ICSVParser::convert(lineData, 10,gyroscope.z()) == ICSVParser::CONVERSION_OK) {


		imuDataSample.setTimeID(timeID);
		imuDataSample.setSequenceID(sequenceID);
		imuDataSample.setAccelerometerSample(accelerometer);
		imuDataSample.setMagnetometerSample(magnetometer);
		imuDataSample.setGyroscopeSample(gyroscope);
	}else{
		ret = ICSVParser::CONVERSION_FAILED;
	}

	return ret;
}