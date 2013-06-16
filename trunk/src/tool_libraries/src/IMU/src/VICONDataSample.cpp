#include <IMU/Data/VICONDataSample.h>
#include <Eigen/Dense>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <QuatUtils/QuatUtils.h>
//#include <GeneralAlgorithms/CombinationGenerator/CombinationGenerator.h>

using namespace IMU;

//! Typ generatora kombinacji dla trójk¹tów
//typedef GeneralAlgorithms::TCombinationGenerator<3,6> CGeneratorType;
//! Typ agreguj¹cy kombinacje
//typedef boost::array<CGeneratorType::Combination, CGeneratorType::total - 5> Combinations;

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

//generuje wszystkie trojkaty jakie moge utworzyæ, pomijam punkty wspo³liniowe
//const Combinations generateCombinations()
//{
//	std::vector<CGeneratorType::Combination> ret;
//	ret.reserve(Combinations::size());
//
//	//wszystkie kombinacje punktów na "ró¿d¿ce" w kszta³cie T
//	CGeneratorType cg;
//	while(cg.hasMore()){
//		ret.push_back(cg.getNext());
//	}
//
//	//usuwam punkty wspó³liniowe
//	{
//		CGeneratorType::Combination c;
//		c[0] = 0;
//		c[1] = 1;
//		c[2] = 2;		
//		ret.erase(std::remove(ret.begin(), ret.end(), c), ret.end());
//
//		c[2] = 3;
//		ret.erase(std::remove(ret.begin(), ret.end(), c), ret.end());
//
//		c[1] = 2;
//		ret.erase(std::remove(ret.begin(), ret.end(), c), ret.end());
//
//		c[0] = 1;		
//		ret.erase(std::remove(ret.begin(), ret.end(), c), ret.end());
//
//		c[0] = 0;
//		c[1] = 4;
//		c[2] = 5;
//		ret.erase(std::remove(ret.begin(), ret.end(), c), ret.end());
//	}
//
//	Combinations retFinall;
//	
//	for(unsigned int i = 0; i < retFinall.size(); ++i){
//		retFinall[i] = ret[i];
//	}
//
//	return retFinall;
//}

const VICONDataSample::Vec3 VICONDataSample::estimateOrientation(const VICONDataSample & viconSample)
{
	using namespace boost::accumulators;
	typedef accumulator_set<Vec3::Scalar, stats<tag::mean>> NormalAccumulator;

	typedef Eigen::Quaternion<double> Quat;

	//osie ró¿ki przyjmujê nastêpuj¹co:
	//dó³ litery T w kierunku daszka to oœ Y, daszek z lewej do prawej to oœ X
	//oœ Z wg prawoskrêtnego uk³adu dla 2 wymienionych wektorów
	//dla dostarczonego obrazka markery M5 M1 M6 opisuj¹ w tej kolejnoœci kierunek osi X
	//markery M4 M3 M2 M1 opisuj¹ w tej kolejnoœci kierunek osi Y
	//iloczyn X i Y w prawoskrêtnym uk³adzie generuje nam oœ Z

	boost::array<Vec3, 6> points;
	points[0] = viconSample.positionM1();
	points[1] = viconSample.positionM2();
	points[2] = viconSample.positionM3();
	points[3] = viconSample.positionM4();
	points[4] = viconSample.positionM5();
	points[5] = viconSample.positionM6();

	//oœ Y "ró¿d¿ki", d³u¿sza pionowa kreska litery T, w kierunku daszka
	Vec3 yAxis;
	{	
		NormalAccumulator yAxisX;
		NormalAccumulator yAxisY;
		NormalAccumulator yAxisZ;

		for(unsigned int i = 0; i < 4; ++i){
			for(unsigned int j = i+1; j < 4; ++j){
				Vec3 yAxisSample = points[i] - points[j];
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

	//oœ X "ró¿d¿ki" - daszek litery T od lewa do prawa (marker M6 po lewej,
	// M5 po prawej)
	Vec3 xAxis;
	{	
		NormalAccumulator xAxisX;
		NormalAccumulator xAxisY;
		NormalAccumulator xAxisZ;

		Vec3 xAxisSample = points[4] - points[0];
		xAxisSample.normalize();
		xAxisX(xAxisSample.x());
		xAxisY(xAxisSample.y());
		xAxisZ(xAxisSample.z());

		xAxisSample = points[4] - points[5];
		xAxisSample.normalize();
		xAxisX(xAxisSample.x());
		xAxisY(xAxisSample.y());
		xAxisZ(xAxisSample.z());

		xAxisSample = points[0] - points[5];
		xAxisSample.normalize();
		xAxisX(xAxisSample.x());
		xAxisY(xAxisSample.y());
		xAxisZ(xAxisSample.z());
			

		xAxis.x() = mean(xAxisX);
		xAxis.y() = mean(xAxisY);
		xAxis.z() = mean(xAxisZ);
	}

	xAxis.normalize();

	//oœ Z "ró¿d¿ki"
	Vec3 zAxis = xAxis.cross(yAxis);
	zAxis.normalize();

	Quat q1(Quat::FromTwoVectors(Vec3(1.0,0.0,0.0), xAxis));	

	Vec3 tmpYAxis = q1._transformVector(yAxis);

	Quat q2(Quat::FromTwoVectors(tmpYAxis, yAxis));

	Quat quat = q1 * q2;

	Vec3 ret;

	osg::QuatUtils::quaterionToEuler(quat.x(), quat.y(), quat.z(), quat.w(),
		ret.x(), ret.y(), ret.z());

	return ret;
}