/********************************************************************
    created:  2013/05/12
    created:  12:5:2013   12:10
    filename: ICSVParser.h
    author:   Mateusz Janiak
    
    purpose:  Interfejs parsera CSV
*********************************************************************/
#ifndef HEADER_GUARD___ICSVPARSER_H__
#define HEADER_GUARD___ICSVPARSER_H__

#include <string>
#include <vector>
#include <boost/lexical_cast.hpp>

namespace IMU {

class ICSVParser
{
public:
	//! Typ danych opisuj¹cy elementy jednej linijki
	typedef std::vector<std::string> LineData;
	//! Typ opisuj¹cy stan odczytu
	enum ReadResultType {
		READ_OK,			//! Odczyt przebieg³ pomyœlnie
		READ_IO_ERROR,		//! B³¹d operacji I/O (obs³ugi strumienia)
		READ_FORMAT_ERROR,	//! B³êdny format danych (weryfikacja zak³adanej iloœci tokenów siê nie powiod³a
		READ_FINISHED,		//! Zakoñczono odczyt danych - nie ma wiêcej do odczytu
	};
	//! Typ opisuj¹cy stan konwersji danych
	enum ConversionResultType {
		CONVERSION_OK,			//! Konwersja danych sie uda³a
		CONVERSION_FAILED,		//! Konwersja danych siê nie powiod³a		
		CONVERSION_OUT_OF_RANGE,//! Konwersja wykracza poza zakres danych
		CONVERSION_ERROR		//! B³¹d konwersji
	};

public:
	//! Destruktor wirtualny
	virtual ~ICSVParser() {}
	//! \param lineData [out] Dane zapisane w kolejnej linii
	//! \return Stan odczytu danych
	virtual const ReadResultType readNext(LineData & lineData) = 0;
	//! \para n Iloœæ linii do opuszczenia
	virtual const ReadResultType skipNext(const LineData::size_type n) = 0;
	//! \tparam Typ do jakiego chcemy rzutowaæ odczytane dane
	//! \param lineData Odczytana linia danych
	//! \param idx Indeks danych które chcemy konwertowaæ
	//! \param data [out] Dane aktualizowane rzutowaniem
	//! \return Stan konwersji danych
	template<typename T>
	static ConversionResultType convert(const LineData & lineData, const LineData::size_type idx, T & data);
};




//! -------------------------------- Implementacja --------------------------

template<typename T>
ICSVParser::ConversionResultType ICSVParser::convert(const LineData & lineData, const LineData::size_type idx, T & data)
{
	if(idx > lineData.size()){
		return CONVERSION_OUT_OF_RANGE;
	}

	ConversionResultType ret = CONVERSION_OK;

	try{
		data = boost::lexical_cast<T>(lineData[idx]);
	}catch(boost::bad_lexical_cast &){
		ret = CONVERSION_FAILED;
	}catch(...){
		ret = CONVERSION_ERROR;
	}

	return ret;
}

}

#endif	//	HEADER_GUARD___ICSVPARSER_H__
