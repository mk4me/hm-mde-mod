/********************************************************************
    created:  2013/05/11
    created:  11:5:2013   17:10
    filename: VICONDataReader.h
    author:   Mateusz Janiak
    
    purpose:  Klasa czytaj¹ca dane z VICON z dedykowanego formatu.
				Format ten oparty jest na standardzie CSV. Pierwsze
				5 linijek mo¿e zawieraæ opisy kolumn i danych. Pliki powinny
				mieæ rozszerzenie np. ".ref" dla odró¿nienia siê od
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
	//! \param path Œcie¿ka do pliku z danymi VICON
	//! \param Czy pomijaæ pierwsz¹ linijkê z nag³ówkami?
	VICONDataReader(const std::string & path);

	//! Destruktor
	~VICONDataReader();

	//! \param VICONDataSample [out] Kolejna próbka danych
	//! \return Stan odczytu
	const ResultType readNextSample(VICONDataSample & VICONDataSample);

private:
	//! Funkcja wypakowuje próbke VICON z parsowanych pól
	//! \param lineData Parsowane pola z CSV
	//! \param viconDataSample [out] Próbka danych VICON odczytana z CSV
	static const ICSVParser::ConversionResultType convert(const ICSVParser::LineData & lineData, VICONDataSample & viconDataSample);

private:
	//! Parser csv
	boost::shared_ptr<FileCSVParser> parser;
	//! 
	DataReader dataReader;
};

}

#endif	//	HEADER_GUARD___VICONDATAREADER_H__
