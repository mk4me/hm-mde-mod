/********************************************************************
    created:  2013/06/05
    created:  5:6:2013   19:26
    filename: CSVTestResultSerializer.h
    author:   Mateusz Janiak
    
    purpose:  Klasa eksportuj¹ca wyniki testów do pliku CSV
*********************************************************************/
#ifndef HEADER_GUARD___CSVTESTRESULTSERIALIZER_H__
#define HEADER_GUARD___CSVTESTRESULTSERIALIZER_H__

#include <IMU/OrientationTestingFramework/ITestResultSerializer.h>

namespace IMU {

class CSVTestResultSerializer : public ITestResultSerializer
{
public:
	//! Domyœlny konstruktor
	CSVTestResultSerializer();
	//! Konstruktor
	//! \param path Plik do którego zapisujemy/z którego czytamy wyniki
	CSVTestResultSerializer(const std::string & path);
	//! Destruktor
	virtual ~CSVTestResultSerializer();

	//! \return Œcie¿ka do pliku z danymi
	const std::string & path() const;
	//! \param path Œcie¿ka do pliku z danymi
	void setPath(const std::string & path);

	//! \param results Wyniki testu estymatorów orientacji dla IMU
	virtual void serialize(const TestingFramework::EstimationResults & results,
		const TestingFramework::OrientationData & orientation);
	//! \param results [out] Wyniki testu estymatorów orientacji dla IMU
	virtual void deserialize(TestingFramework::EstimationResults & results);

private:
	//! Waliduje œcie¿kê do pliku
	void validatePath() const;

private:
	//! Œcie¿ka do pliku z danymi
	std::string path_;
};

}

#endif	//	HEADER_GUARD___CSVTESTRESULTSERIALIZER_H__
