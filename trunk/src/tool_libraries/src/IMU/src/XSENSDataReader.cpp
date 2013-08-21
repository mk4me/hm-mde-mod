#include <IMU/Parsers/XSENSDataReader.h>
#include <IMU/Data/XSENSDataSample.h>
#include <IMU/Parsers/FileCSVParser.h>
#include <boost/bind.hpp>

using namespace IMU;

XSENSDataReader::XSENSDataReader(const std::string & path) : parser(new FileCSVParser(path, true, '\t')),
	dataReader(parser.get(), DataReader::Extractor(boost::bind(&XSENSDataReader::convert, _1, _2)), 13)
{
	parser->skipNext(5);
}

XSENSDataReader::~XSENSDataReader()
{

}

const XSENSDataReader::ResultType XSENSDataReader::readNextSample(XSENSDataSample & xsensDataSample)
{
	return dataReader.readNextValue(xsensDataSample);
}

const ICSVParser::ConversionResultType XSENSDataReader::convert(const ICSVParser::LineData & lineData, XSENSDataSample & xsensDataSample)
{

	ICSVParser::ConversionResultType ret = ICSVParser::CONVERSION_OK;

	// identyfikator czasu
	XSENSDataSample::TimeIDType timeID = 0;
	Vec3 accelerometer;
	Vec3 magnetometer;
	Vec3 gyroscope;
	Vec3 orientation;

	if(ICSVParser::convert(lineData, 0, timeID) == ICSVParser::CONVERSION_OK &&
		ICSVParser::convert(lineData, 1, accelerometer.x()) == ICSVParser::CONVERSION_OK &&
		ICSVParser::convert(lineData, 2, accelerometer.y()) == ICSVParser::CONVERSION_OK &&
		ICSVParser::convert(lineData, 3, accelerometer.z()) == ICSVParser::CONVERSION_OK &&
		ICSVParser::convert(lineData, 4, gyroscope.x()) == ICSVParser::CONVERSION_OK &&
		ICSVParser::convert(lineData, 5, gyroscope.y()) == ICSVParser::CONVERSION_OK &&
		ICSVParser::convert(lineData, 6,gyroscope.z()) == ICSVParser::CONVERSION_OK &&
		ICSVParser::convert(lineData, 7, magnetometer.x()) == ICSVParser::CONVERSION_OK &&
		ICSVParser::convert(lineData, 8, magnetometer.y()) == ICSVParser::CONVERSION_OK &&
		ICSVParser::convert(lineData, 9, magnetometer.z()) == ICSVParser::CONVERSION_OK &&
		ICSVParser::convert(lineData, 10, orientation.x()) == ICSVParser::CONVERSION_OK &&
		ICSVParser::convert(lineData, 11, orientation.y()) == ICSVParser::CONVERSION_OK &&
		ICSVParser::convert(lineData, 12, orientation.z()) == ICSVParser::CONVERSION_OK){


			xsensDataSample.setTimeID(timeID);
			xsensDataSample.setAccelerometerSample(accelerometer);
			xsensDataSample.setMagnetometerSample(magnetometer);
			xsensDataSample.setGyroscopeSample(gyroscope);
			xsensDataSample.setEstimatedOrientationSample(orientation);
	}else{
		ret = ICSVParser::CONVERSION_FAILED;
	}

	return ret;
}