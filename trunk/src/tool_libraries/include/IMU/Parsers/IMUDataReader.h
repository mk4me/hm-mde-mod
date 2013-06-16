/********************************************************************
    created:  2013/05/11
    created:  11:5:2013   17:10
    filename: IMUDataReader.h
    author:   Mateusz Janiak
    
    purpose:  Klasa czytaj�ca dane z IMU z dedykowanego formatu.
				Format ten oparty jest na standardzie CSV. Pierwsza
				linijka zawiera opisy kolumn. Pliki powinny
				mie� rozszerzenie ".imu" dla odr�nienia si� od
				innych danych.
*********************************************************************/
#ifndef HEADER_GUARD___IMUDATAREADER_H__
#define HEADER_GUARD___IMUDATAREADER_H__

#include <boost/shared_ptr.hpp>
#include <string>
#include <fstream>
#include <IMU/Parsers/ICSVParser.h>
#include <IMU/Parsers/UniversalDataReader.h>

namespace IMU {

class IMUDataSample;
class FileCSVParser;

class IMUDataReader
{
private:
	//! Typ universalnego parsera danych
	typedef UniversalDataReader<IMUDataSample> DataReader;

public:
	//! Typ statusu operacji
	typedef DataReader::ResultType ResultType;

public:
	//! \param path �cie�ka do pliku z danymi IMU
	//! \param Czy pomija� pierwsz� linijk� z nag��wkami?
	IMUDataReader(const std::string & path);

	//! Destruktor
	~IMUDataReader();

	//! \param imuDataSample [out] Kolejna pr�bka danych
	//! \return Stan odczytu
	const ResultType readNextSample(IMUDataSample & imuDataSample);

private:
	//! Funkcja wypakowuje pr�bke imu z parsowanych p�l
	//! \param lineData Parsowane pola z CSV
	//! \param imuDataSample [out] Pr�bka danych IMU odczytana z CSV
	static const ICSVParser::ConversionResultType convert(const ICSVParser::LineData & lineData, IMUDataSample & imuDataSample);

private:
	//! Parser csv
	boost::shared_ptr<FileCSVParser> parser;
	//! 
	DataReader dataReader;
};

}

#endif	//	HEADER_GUARD___IMUDATAREADER_H__
