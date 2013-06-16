#include <IMU/OrientationTestingFramework/CSVTestResultSerializer.h>
#include <boost/filesystem.hpp>
#include <stdexcept>
#include <fstream>

using namespace IMU;

CSVTestResultSerializer::CSVTestResultSerializer()
{

}

CSVTestResultSerializer::CSVTestResultSerializer(const std::string & path)
	:path_(path)
{

}

CSVTestResultSerializer::~CSVTestResultSerializer()
{

}

const std::string & CSVTestResultSerializer::path() const
{
	return path_;
}

void CSVTestResultSerializer::setPath(const std::string & path)
{
	path_ = path;
}

void CSVTestResultSerializer::serialize(const TestingFramework::EstimationResults & results,
	const TestingFramework::OrientationData & orientation)
{
	std::ofstream outFile(path_);
	if(outFile.is_open() == false){
		throw std::runtime_error("Error opening file for writing");
	}

	//nag³ówek danych referencyjnych
	outFile << "Reference_roll;Reference_pitch;Reference_yaw;";

	//teraz generujemy kolumny nag³ówkowe estymatorów
	for(auto it = results.begin(); it != results.end(); ++it){
		auto name = it->first;
		for(unsigned int i = 0; i < name.size(); ++i){
			if(name[i] == ' '){
				name[i] = '_';
			}
		}

		outFile << name + "_roll;" << name + "_pitch;" << name + "_yaw;";
	}

	outFile << std::endl;

	auto i = results.begin()->second.results.size();

	//teraz generujemy dane
	for(unsigned int j = 0; j < i; ++j ){

		auto d = orientation[j];

		outFile << d.x() << ";" << d.y() << ";" << d.z() << ";";

		for(auto it = results.begin(); it != results.end(); ++it){
			auto r = it->second.results[j];
			outFile << r.x() << ";" << r.y() << ";" << r.z() << ";";
		}
		
		outFile << std::endl;
	}

	//konczymy zapis i zamykamy plik
	outFile.flush();
	outFile.close();
}

void CSVTestResultSerializer::deserialize(TestingFramework::EstimationResults & results)
{
	validatePath();
	//TODO
	//do zrobienia w razie koniecznosci
	throw std::runtime_error("Not implemented yet");
}

void CSVTestResultSerializer::validatePath() const
{
	if(path_.empty() == true){
		throw std::runtime_error("Empty file path");
	}

	if(boost::filesystem::exists(path_.c_str()) == false){
		throw std::runtime_error("Given path not exist");
	}
}