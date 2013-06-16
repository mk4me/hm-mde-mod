/********************************************************************
    created:  2013/06/05
    created:  5:6:2013   19:26
    filename: CSVTestResultSerializer.h
    author:   Mateusz Janiak
    
    purpose:  Klasa eksportuj�ca wyniki test�w do pliku CSV
*********************************************************************/
#ifndef HEADER_GUARD___CSVTESTRESULTSERIALIZER_H__
#define HEADER_GUARD___CSVTESTRESULTSERIALIZER_H__

#include <IMU/OrientationTestingFramework/ITestResultSerializer.h>

namespace IMU {

class CSVTestResultSerializer : public ITestResultSerializer
{
public:
	//! Domy�lny konstruktor
	CSVTestResultSerializer();
	//! Konstruktor
	//! \param path Plik do kt�rego zapisujemy/z kt�rego czytamy wyniki
	CSVTestResultSerializer(const std::string & path);
	//! Destruktor
	virtual ~CSVTestResultSerializer();

	//! \return �cie�ka do pliku z danymi
	const std::string & path() const;
	//! \param path �cie�ka do pliku z danymi
	void setPath(const std::string & path);

	//! \param results Wyniki testu estymator�w orientacji dla IMU
	virtual void serialize(const TestingFramework::EstimationResults & results,
		const TestingFramework::OrientationData & orientation);
	//! \param results [out] Wyniki testu estymator�w orientacji dla IMU
	virtual void deserialize(TestingFramework::EstimationResults & results);

private:
	//! Waliduje �cie�k� do pliku
	void validatePath() const;

private:
	//! �cie�ka do pliku z danymi
	std::string path_;
};

}

#endif	//	HEADER_GUARD___CSVTESTRESULTSERIALIZER_H__
