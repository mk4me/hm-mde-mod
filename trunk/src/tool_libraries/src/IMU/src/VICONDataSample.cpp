#include <IMU/Data/VICONDataSample.h>
#include <Eigen/Dense>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <QuatUtils/QuatUtils.h>
#include <GeneralAlgorithms/CombinationGenerator/CombinationGenerator.h>

using namespace IMU;

//! Typ generatora kombinacji dla tr�jk�t�w
typedef GeneralAlgorithms::TCombinationGenerator<3,6> CGeneratorType;
//! Typ agreguj�cy kombinacje
typedef boost::array<CGeneratorType::Combination, CGeneratorType::total - 5> Combinations;

VICONDataSample::VICONDataSample(TimeIDType timeID, const Vec3 & positionM1,	const Vec3 & positionM2,
	const Vec3 & positionM3, const Vec3 & positionM4, const Vec3 & positionM5,
	const Vec3 & positionM6) : timeID_(timeID)
{
	markerPositions_[0] = positionM1;
	markerPositions_[1] = positionM2;
	markerPositions_[2] = positionM3;
	markerPositions_[3] = positionM4;
	markerPositions_[4] = positionM5;
	markerPositions_[5] = positionM6;
}

VICONDataSample::VICONDataSample(const VICONDataSample & ds)
	: timeID_(ds.timeID_), markerPositions_(ds.markerPositions_)
{

}

VICONDataSample::VICONDataSample() : timeID_(0)
{
	markerPositions_[0] = Vec3::Zero();
	markerPositions_[1] = Vec3::Zero();
	markerPositions_[2] = Vec3::Zero();
	markerPositions_[3] = Vec3::Zero();
	markerPositions_[4] = Vec3::Zero();
	markerPositions_[5] = Vec3::Zero();
}

VICONDataSample::~VICONDataSample()
{

}

//generuje wszystkie trojkaty jakie moge utworzy�, pomijam punkty wspo�liniowe
const Combinations generateCombinations()
{
	std::vector<CGeneratorType::Combination> ret;
	ret.reserve(Combinations::size());

	//wszystkie kombinacje punkt�w na "r�d�ce" w kszta�cie T
	CGeneratorType cg;
	while(cg.hasMore()){
		ret.push_back(cg.getNext());
	}

	//usuwam punkty wsp�liniowe
	{
		CGeneratorType::Combination c;
		c[0] = 0;
		c[1] = 1;
		c[2] = 2;		
		ret.erase(std::remove(ret.begin(), ret.end(), c), ret.end());

		c[2] = 3;
		ret.erase(std::remove(ret.begin(), ret.end(), c), ret.end());

		c[1] = 2;
		ret.erase(std::remove(ret.begin(), ret.end(), c), ret.end());

		c[0] = 1;		
		ret.erase(std::remove(ret.begin(), ret.end(), c), ret.end());

		c[0] = 0;
		c[1] = 4;
		c[2] = 5;
		ret.erase(std::remove(ret.begin(), ret.end(), c), ret.end());
	}

	Combinations retFinall;
	
	for(unsigned int i = 0; i < retFinall.size(); ++i){
		retFinall[i] = ret[i];
	}

	return retFinall;
}

void VICONDataSample::getAxis(const VICONDataSample & viconSample,
	Vec3 & xAxis, Vec3 & yAxis, Vec3 & zAxis)
{
	using namespace boost::accumulators;
	typedef accumulator_set<Vec3::Scalar, stats<tag::mean>> NormalAccumulator;

	const static Combinations combinations(generateCombinations());

	//osie r�ki przyjmuj� nast�puj�co:
	//d� litery T w kierunku daszka to o� Y, daszek z lewej do prawej to o� X
	//o� Z wg prawoskr�tnego uk�adu dla 2 wymienionych wektor�w
	//dla dostarczonego obrazka markery M5 M1 M6 opisuj� w tej kolejno�ci kierunek osi X
	//markery M4 M3 M2 M1 opisuj� w tej kolejno�ci kierunek osi Y
	//iloczyn X i Y w prawoskr�tnym uk�adzie generuje nam o� Z

	boost::array<Vec3, 6> points;
	points[0] = viconSample.positionM1();
	points[1] = viconSample.positionM2();
	points[2] = viconSample.positionM3();
	points[3] = viconSample.positionM4();
	points[4] = viconSample.positionM5();
	points[5] = viconSample.positionM6();

	//o� Y "r�d�ki", d�u�sza pionowa kreska litery T, w kierunku daszka
	{
		NormalAccumulator yAxisX;
		NormalAccumulator yAxisY;
		NormalAccumulator yAxisZ;

		for(unsigned int i = 0; i < 4; ++i){
			for(unsigned int j = i+1; j < 4; ++j){
				Vec3 yAxisSample = points[i] - points[j];
				//Vec3 yAxisSample = points[j] - points[i];
				yAxisSample.normalize();
				yAxisX(yAxisSample.x());
				yAxisY(yAxisSample.y());
				yAxisZ(yAxisSample.z());
			}
		}

		yAxis.x() = mean(yAxisX);
		yAxis.y() = mean(yAxisY);
		yAxis.z() = mean(yAxisZ);
	}

	yAxis.normalize();

	//o� Z "r�d�ki" - najbogatsza w kombinacje punkt�w (bazuje na iloczynie wektorowym)
	{
		NormalAccumulator zAxisX;
		NormalAccumulator zAxisY;
		NormalAccumulator zAxisZ;

		for(auto it = combinations.begin(); it != combinations.end(); ++it){

			auto c = *it;

			Vec3 a(Vec3::Zero());
			Vec3 b(Vec3::Zero());

			if(c[2] == 5 && c[1] == 4){

				a = points[5] - points[4];
				b = points[4] - points[c[0]];
				

			}else if(c[2] == 5){

				a = points[5] - points[c[1]];
				b = points[c[0]] - points[c[1]];

			}else if(c[2] == 4){

				a = points[c[0]] - points[c[1]];
				b = points[4] - points[c[1]];

			}

			Vec3 res = a.cross(b);
			res.normalize();

			zAxisX(res.x());
			zAxisY(res.y());
			zAxisZ(res.z());

		}

		zAxis.x() = mean(zAxisX);
		zAxis.y() = mean(zAxisY);
		zAxis.z() = mean(zAxisZ);
	}

	zAxis.normalize();

	//o� X "r�d�ki" - daszek litery T od lewa do prawa (marker M6 po lewej,
	// M5 po prawej)
	// ta o� jako najubo�sza b�dzie wyznaczana z iloczynu wektorowego osi Y i Z
	xAxis = yAxis.cross(zAxis);
}

const Vec3 VICONDataSample::estimateOrientation(const VICONDataSample & viconSample)
{
	Vec3 xAxis;
	Vec3 yAxis;
	Vec3 zAxis;

	getAxis(viconSample, xAxis, yAxis, zAxis);

	return osg::QuatUtils::axisToEuler<Vec3, Vec3>(xAxis, yAxis, zAxis);
}