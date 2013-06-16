/********************************************************************
    created:  2013/05/12
    created:  12:5:2013   8:56
    filename: StreamCSVParser.h
    author:   Mateusz Janiak
    
    purpose:  CSV stream parser. Accepts input stream and custom delimiter, escape sign and quota sign.				
*********************************************************************/
#ifndef HEADER_GUARD___CSVPARSER_H__
#define HEADER_GUARD___CSVPARSER_H__

#include <IMU/Parsers/ICSVParser.h>
#include <istream>

namespace IMU {

class StreamCSVParser : public ICSVParser
{
public:	
	//! \param stream Strumieñ wejœciowy do parsowania
	//! \param delimiter Znak rozdzielaj¹cy dane
	//! \param escape Znak ucieczki
	//! \param quota Znak nawiasu
	StreamCSVParser(std::istream * stream, const char delimiter = ';', const char escape = '\\', const char quota = '\"');
	//! Destruktor
	virtual ~StreamCSVParser();
	//! \param lineData [out] Dane zapisane w kolejnej linii
	//! \return Stan odczytu danych
	virtual const ReadResultType readNext(LineData & lineData);
	//! \para n Iloœæ linii do opuszczenia
	virtual const ReadResultType skipNext(const LineData::size_type n = 1);

private:
	//! Aktualizuje stan strumienia dla kolejnych operacji
	void updateStreamStatus();

private:
	//! Strumieñ danych
	std::istream * stream_;
	//! Ostatni rezultat odczytu
	ReadResultType lastResult_;
	//! Znak rozdzielaj¹cy dane
	const char delimiter_;
	//! Znak ucieczki
	const char escape_;
	//! Znak nawiasu, porcji danych
	const char quota_;
};

}

#endif	//	HEADER_GUARD___CSVPARSER_H__
