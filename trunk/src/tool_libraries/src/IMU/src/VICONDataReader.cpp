#include <IMU/Parsers/VICONDataReader.h>
#include <IMU/Data/VICONDataSample.h>
#include <IMU/Parsers/FileCSVParser.h>
#include <boost/bind.hpp>

using namespace IMU;

VICONDataReader::VICONDataReader(const std::string & path) : parser(new FileCSVParser(path, true, ',')),
	dataReader(parser.get(), DataReader::Extractor(boost::bind(&VICONDataReader::convert, _1, _2)), 19)
{
	parser->skipNext(5);
}

VICONDataReader::~VICONDataReader()
{

}

const VICONDataReader::ResultType VICONDataReader::readNextSample(VICONDataSample & viconDataSample)
{
	return dataReader.readNextValue(viconDataSample);
}

const ICSVParser::ConversionResultType VICONDataReader::convert(const ICSVParser::LineData & lineData, VICONDataSample & viconDataSample)
{

	ICSVParser::ConversionResultType ret = ICSVParser::CONVERSION_OK;

	// identyfikator czasu
	VICONDataSample::TimeIDType timeID = 0;	
	Vec3 m1;
	Vec3 m2;
	Vec3 m3;
	Vec3 m4;
	Vec3 m5;
	Vec3 m6;
	
	
	if(ICSVParser::convert(lineData, 0, timeID) == ICSVParser::CONVERSION_OK &&		
		ICSVParser::convert(lineData, 2, m1.x()) == ICSVParser::CONVERSION_OK &&
		ICSVParser::convert(lineData, 3, m1.y()) == ICSVParser::CONVERSION_OK &&
		ICSVParser::convert(lineData, 4, m1.z()) == ICSVParser::CONVERSION_OK &&
		ICSVParser::convert(lineData, 5, m2.x()) == ICSVParser::CONVERSION_OK &&
		ICSVParser::convert(lineData, 6, m2.y()) == ICSVParser::CONVERSION_OK &&
		ICSVParser::convert(lineData, 7, m2.z()) == ICSVParser::CONVERSION_OK &&
		ICSVParser::convert(lineData, 8, m3.x()) == ICSVParser::CONVERSION_OK &&
		ICSVParser::convert(lineData, 9, m3.y()) == ICSVParser::CONVERSION_OK &&
		ICSVParser::convert(lineData, 10, m3.z()) == ICSVParser::CONVERSION_OK &&
		ICSVParser::convert(lineData, 11, m4.x()) == ICSVParser::CONVERSION_OK &&
		ICSVParser::convert(lineData, 12, m4.y()) == ICSVParser::CONVERSION_OK &&
		ICSVParser::convert(lineData, 13, m4.z()) == ICSVParser::CONVERSION_OK &&
		ICSVParser::convert(lineData, 14, m5.x()) == ICSVParser::CONVERSION_OK &&
		ICSVParser::convert(lineData, 15, m5.y()) == ICSVParser::CONVERSION_OK &&
		ICSVParser::convert(lineData, 16, m5.z()) == ICSVParser::CONVERSION_OK &&
		ICSVParser::convert(lineData, 17, m6.x()) == ICSVParser::CONVERSION_OK &&
		ICSVParser::convert(lineData, 18, m6.y()) == ICSVParser::CONVERSION_OK &&
		ICSVParser::convert(lineData, 19, m6.z()) == ICSVParser::CONVERSION_OK) {
		

	/*
	if(ICSVParser::convert(lineData, 0, timeID) == ICSVParser::CONVERSION_OK &&		
		ICSVParser::convert(lineData, 2, m1.x()) == ICSVParser::CONVERSION_OK &&
		ICSVParser::convert(lineData, 3, m1.z()) == ICSVParser::CONVERSION_OK &&
		ICSVParser::convert(lineData, 4, m1.y()) == ICSVParser::CONVERSION_OK &&
		ICSVParser::convert(lineData, 5, m2.x()) == ICSVParser::CONVERSION_OK &&
		ICSVParser::convert(lineData, 6, m2.z()) == ICSVParser::CONVERSION_OK &&
		ICSVParser::convert(lineData, 7, m2.y()) == ICSVParser::CONVERSION_OK &&
		ICSVParser::convert(lineData, 8, m3.x()) == ICSVParser::CONVERSION_OK &&
		ICSVParser::convert(lineData, 9, m3.z()) == ICSVParser::CONVERSION_OK &&
		ICSVParser::convert(lineData, 10, m3.y()) == ICSVParser::CONVERSION_OK &&
		ICSVParser::convert(lineData, 11, m4.x()) == ICSVParser::CONVERSION_OK &&
		ICSVParser::convert(lineData, 12, m4.z()) == ICSVParser::CONVERSION_OK &&
		ICSVParser::convert(lineData, 13, m4.y()) == ICSVParser::CONVERSION_OK &&
		ICSVParser::convert(lineData, 14, m5.x()) == ICSVParser::CONVERSION_OK &&
		ICSVParser::convert(lineData, 15, m5.z()) == ICSVParser::CONVERSION_OK &&
		ICSVParser::convert(lineData, 16, m5.y()) == ICSVParser::CONVERSION_OK &&
		ICSVParser::convert(lineData, 17, m6.x()) == ICSVParser::CONVERSION_OK &&
		ICSVParser::convert(lineData, 18, m6.z()) == ICSVParser::CONVERSION_OK &&
		ICSVParser::convert(lineData, 19, m6.y()) == ICSVParser::CONVERSION_OK) {*/


			viconDataSample.setTimeID(timeID);
			viconDataSample.setPositionM1(m1);
			viconDataSample.setPositionM2(m2);
			viconDataSample.setPositionM3(m3);
			viconDataSample.setPositionM4(m4);
			viconDataSample.setPositionM5(m5);
			viconDataSample.setPositionM6(m6);			
	}else{
		ret = ICSVParser::CONVERSION_FAILED;
	}

	return ret;
}