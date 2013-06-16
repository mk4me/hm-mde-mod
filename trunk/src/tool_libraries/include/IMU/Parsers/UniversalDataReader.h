/********************************************************************
    created:  2013/05/11
    created:  11:5:2013   17:10
    filename: UniversalDataReader.h
    author:   Mateusz Janiak
    
    purpose:  Klasa czytaj�ca dane z Universal z dedykowanego formatu.
				Format ten oparty jest na standardzie CSV. Pierwsza
				linijka zawiera opisy kolumn. Pliki powinny
				mie� rozszerzenie ".Universal" dla odr�nienia si� od
				innych danych.
*********************************************************************/
#ifndef HEADER_GUARD___UniversalDATAREADER_H__
#define HEADER_GUARD___UniversalDATAREADER_H__

#include <IMU/Parsers/ICSVParser.h>
#include <boost/function.hpp>
#include <string>

namespace IMU {

template<typename T>
class UniversalDataReader
{
public:
	//! Stan odczytu danych
	enum ResultType {
		RESULT_OK,			//! Odczyt OK
		IO_ERROR,			//! B��d I/O
		FORMAT_ERROR,		//! B��dny format danych
		CONVERSION_ERROR,	//! B��d konwersji danych
		DATA_FINISHED		//! Koniec danych
	};

	//! Typ funktora rozpakowuj�cego dane CSV jednej linii do zadanego typu
	//! Nie powinien ustawia�/modyfikowa� danych wej�ciowych je�li konwersja nie mo�e by� przeprowadzona poprawnie
	typedef boost::function<const ICSVParser::ConversionResultType(const ICSVParser::LineData &, T &)> Extractor;	

public:
	//! \param parser Parser CSV dostarczaj�cy danych z kolejnych wierszy
	//! \param extractor Funktor wyci�gaj�cy zadany tym z danych wiersza
	//! \param minimalFieldsCount Minimalna ilo�� p�l jakie powinny znale�� si� w wierszu dla poprawnego wypakowywania
	UniversalDataReader(ICSVParser * parser, const Extractor & extractor, ICSVParser::LineData::size_type minimalFieldsCount);	

	//! Destruktor
	~UniversalDataReader();

	//! \param UniversalDataSample [out] Kolejna pr�bka danych
	//! \return Stan odczytu
	const ResultType readNextValue(T & value);

private:
	//! Funkcja mapuje efekt parsowania pliku CSV do efektu parsowania pr�bki Universal
	static const ResultType mapParserResult(const ICSVParser::ReadResultType res);

private:
	//! Parser csv
	ICSVParser * parser_;
	//! Wynik ostatniego parsowania
	ResultType lastResult_;
	//! Minimalna ilo�� p� w wierszu potrzebna do wypakowywania danych
	const ICSVParser::LineData::size_type minimalFieldsCount_;
	//! Funktor relaizuj�cy przepakowanie danych wiersza do zadanego formatu
	Extractor extractor_;
};




//! ------------------------- Implementacja ---------------------------------

template<typename T>
UniversalDataReader<T>::UniversalDataReader(ICSVParser * parser, const Extractor & extractor, ICSVParser::LineData::size_type minimalFieldsCount)
	: parser_(parser), extractor_(extractor), minimalFieldsCount_(minimalFieldsCount), lastResult_(RESULT_OK)
{
	if(parser_ == nullptr){
		throw std::invalid_argument("Null pointer for CSV parser");
	}

	if(extractor_.empty() == true){
		throw std::invalid_argument("Empty data extractor");
	}
}

template<typename T>
UniversalDataReader<T>::~UniversalDataReader()
{

}

template<typename T>
const typename UniversalDataReader<T>::ResultType UniversalDataReader<T>::readNextValue(T & value)
{
	if(lastResult_ == RESULT_OK){					

		ICSVParser::LineData lineData;
		lastResult_ = mapParserResult(parser_->readNext(lineData));
		if(lastResult_ == RESULT_OK){
			if(lineData.size() >= minimalFieldsCount_){			
				if(extractor_(lineData, value) != ICSVParser::CONVERSION_OK){
					lastResult_ = CONVERSION_ERROR;
				}
			}else{
				lastResult_ = FORMAT_ERROR;
			}
		}
	}

	return lastResult_;
}

template<typename T>
const typename UniversalDataReader<T>::ResultType UniversalDataReader<T>::mapParserResult(const ICSVParser::ReadResultType res)
{
	ResultType ret = RESULT_OK;

	switch(res){

	case ICSVParser::READ_FINISHED:
		ret = DATA_FINISHED;
		break;

	case ICSVParser::READ_FORMAT_ERROR:
		ret = FORMAT_ERROR;
		break;

	case ICSVParser::READ_IO_ERROR:
		ret = IO_ERROR;
		break;
	}

	return ret;
}

}

#endif	//	HEADER_GUARD___UniversalDATAREADER_H__
