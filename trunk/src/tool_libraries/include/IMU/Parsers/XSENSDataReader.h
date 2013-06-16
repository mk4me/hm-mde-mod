/********************************************************************
    created:  2013/05/11
    created:  11:5:2013   17:10
    filename: XSENSDataReader.h
    author:   Mateusz Janiak
    
    purpose:  Klasa czytaj�ca dane z XSENS z dedykowanego formatu.
				Format ten oparty jest na standardzie CSV. Pierwsze
				5 wierszy zawiera opisy kolumn i pomiaru. Pliki powinny
				mie� rozszerzenie ".xsens" dla odr�nienia si� od
				innych danych.
*********************************************************************/
#ifndef HEADER_GUARD___XSENSDATAREADER_H__
#define HEADER_GUARD___XSENSDATAREADER_H__

#include <boost/shared_ptr.hpp>
#include <string>
#include <fstream>
#include <IMU/Parsers/ICSVParser.h>
#include <IMU/Parsers/UniversalDataReader.h>

namespace IMU {

class XSENSDataSample;
class FileCSVParser;

class XSENSDataReader
{
private:
	//! Typ universalnego parsera danych
	typedef UniversalDataReader<XSENSDataSample> DataReader;

public:
	//! Typ statusu operacji
	typedef DataReader::ResultType ResultType;

public:
	//! \param path �cie�ka do pliku z danymi XSENS
	//! \param Czy pomija� pierwsz� linijk� z nag��wkami?
	XSENSDataReader(const std::string & path);

	//! Destruktor
	~XSENSDataReader();

	//! \param XSENSDataSample [out] Kolejna pr�bka danych
	//! \return Stan odczytu
	const ResultType readNextSample(XSENSDataSample & xsensDataSample);

private:
	//! Funkcja wypakowuje pr�bke XSENS z parsowanych p�l
	//! \param lineData Parsowane pola z CSV
	//! \param XSENSDataSample [out] Pr�bka danych XSENS odczytana z CSV
	static const ICSVParser::ConversionResultType convert(const ICSVParser::LineData & lineData, XSENSDataSample & xsensDataSample);

private:
	//! Parser csv
	boost::shared_ptr<FileCSVParser> parser;
	//! 
	DataReader dataReader;
};

}

#endif	//	HEADER_GUARD___XSENSDATAREADER_H__
