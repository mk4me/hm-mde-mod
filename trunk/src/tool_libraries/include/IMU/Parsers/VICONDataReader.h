/********************************************************************
    created:  2013/05/11
    created:  11:5:2013   17:10
    filename: VICONDataReader.h
    author:   Mateusz Janiak
    
    purpose:  Klasa czytaj�ca dane z VICON z dedykowanego formatu.
				Format ten oparty jest na standardzie CSV. Pierwsze
				5 linijek mo�e zawiera� opisy kolumn i danych. Pliki powinny
				mie� rozszerzenie np. ".ref" dla odr�nienia si� od
				innych danych.
*********************************************************************/
#ifndef HEADER_GUARD___VICONDATAREADER_H__
#define HEADER_GUARD___VICONDATAREADER_H__

#include <boost/shared_ptr.hpp>
#include <string>
#include <fstream>
#include <IMU/Parsers/ICSVParser.h>
#include <IMU/Parsers/UniversalDataReader.h>

namespace IMU {

class VICONDataSample;
class FileCSVParser;

class VICONDataReader
{
private:
	//! Typ universalnego parsera danych
	typedef UniversalDataReader<VICONDataSample> DataReader;

public:
	//! Typ statusu operacji
	typedef DataReader::ResultType ResultType;

public:
	//! \param path �cie�ka do pliku z danymi VICON
	//! \param Czy pomija� pierwsz� linijk� z nag��wkami?
	VICONDataReader(const std::string & path);

	//! Destruktor
	~VICONDataReader();

	//! \param VICONDataSample [out] Kolejna pr�bka danych
	//! \return Stan odczytu
	const ResultType readNextSample(VICONDataSample & VICONDataSample);

private:
	//! Funkcja wypakowuje pr�bke VICON z parsowanych p�l
	//! \param lineData Parsowane pola z CSV
	//! \param viconDataSample [out] Pr�bka danych VICON odczytana z CSV
	static const ICSVParser::ConversionResultType convert(const ICSVParser::LineData & lineData, VICONDataSample & viconDataSample);

private:
	//! Parser csv
	boost::shared_ptr<FileCSVParser> parser;
	//! 
	DataReader dataReader;
};

}

#endif	//	HEADER_GUARD___VICONDATAREADER_H__
