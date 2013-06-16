/********************************************************************
    created:  2013/05/12
    created:  12:5:2013   12:38
    filename: FileCSVParser.h
    author:   Mateusz Janiak
    
    purpose:  File CSV parser. For optimization offers file caching in
				memory.
*********************************************************************/
#ifndef HEADER_GUARD___FILECSVPARSER_H__
#define HEADER_GUARD___FILECSVPARSER_H__

#include <IMU/Parsers/StreamCSVParser.h>
#include <fstream>
#include <boost/shared_ptr.hpp>

namespace IMU {

class FileCSVParser : public ICSVParser
{
private:
	//! Typ smart pointera do strumienia danych
	typedef boost::shared_ptr<std::istream> StreamPtr;
	//! Typ inteligetnego wskaŸnika do parsera strumienia CSV
	typedef boost::shared_ptr<StreamCSVParser> StreamParserPtr;
	//! Typ inteligetnego wskaŸnika do strumienia plikowego
	typedef boost::shared_ptr<std::ifstream> FileStreamPtr;

public:

	//! \param path Œcie¿ka do pliku z danymi
	//! \param cache Czy wcxzytaæ ca³y plik do pamiêci?
	//! \param delimiter Znak rozdzielaj¹cy dane
	//! \param escape Znak ucieczki
	//! \param quota Znak nawiasu
	FileCSVParser(const std::string & path, const bool cache = false, const char delimiter = ';', const char escape = '\\', const char quota = '\"');
	//! Destruktor
	virtual ~FileCSVParser();
	//! \param lineData [out] Dane zapisane w kolejnej linii
	//! \return Stan odczytu danych
	virtual const ReadResultType readNext(LineData & lineData);
	//! \para n Iloœæ linii do opuszczenia
	virtual const ReadResultType skipNext(const LineData::size_type n);

private:
	//! Opcjonalny cache
	StreamPtr stream_;
	//! Parser strumieni CSV
	StreamParserPtr streamParser_;
	//! Ostatni stan odczytu
	ReadResultType lastResult_;
};

}

#endif	//	HEADER_GUARD___FILECSVPARSER_H__
