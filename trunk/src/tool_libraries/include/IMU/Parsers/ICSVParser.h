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
	//! Typ danych opisuj�cy elementy jednej linijki
	typedef std::vector<std::string> LineData;
	//! Typ opisuj�cy stan odczytu
	enum ReadResultType {
		READ_OK,			//! Odczyt przebieg� pomy�lnie
		READ_IO_ERROR,		//! B��d operacji I/O (obs�ugi strumienia)
		READ_FORMAT_ERROR,	//! B��dny format danych (weryfikacja zak�adanej ilo�ci token�w si� nie powiod�a
		READ_FINISHED,		//! Zako�czono odczyt danych - nie ma wi�cej do odczytu
	};
	//! Typ opisuj�cy stan konwersji danych
	enum ConversionResultType {
		CONVERSION_OK,			//! Konwersja danych sie uda�a
		CONVERSION_FAILED,		//! Konwersja danych si� nie powiod�a		
		CONVERSION_OUT_OF_RANGE,//! Konwersja wykracza poza zakres danych
		CONVERSION_ERROR		//! B��d konwersji
	};

public:
	//! Destruktor wirtualny
	virtual ~ICSVParser() {}
	//! \param lineData [out] Dane zapisane w kolejnej linii
	//! \return Stan odczytu danych
	virtual const ReadResultType readNext(LineData & lineData) = 0;
	//! \para n Ilo�� linii do opuszczenia
	virtual const ReadResultType skipNext(const LineData::size_type n) = 0;
	//! \tparam Typ do jakiego chcemy rzutowa� odczytane dane
	//! \param lineData Odczytana linia danych
	//! \param idx Indeks danych kt�re chcemy konwertowa�
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
