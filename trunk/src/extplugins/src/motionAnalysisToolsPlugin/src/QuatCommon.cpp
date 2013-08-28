#include "QuatCommon.h"
#include <fstream>
#include <QuatUtils/QuatUtils.h>

const QuatUtils::QuatLiftingScheme::Data QuatCommon::convert(
	const VectorChannelReaderInterfaceConstPtr vectorChannel)
{
	QuatUtils::QuatLiftingScheme::Data ret;
	ret.reserve(vectorChannel->size());

	for(unsigned int i = 0; i < vectorChannel->size(); ++i){
		auto euler = vectorChannel->value(i);
		osg::Vec3 radEuler(osg::DegreesToRadians(euler.x()),
			osg::DegreesToRadians(euler.y()),
			osg::DegreesToRadians(euler.z()));

		ret.push_back(osg::QuatUtils::eulerToQuaternion
			<osg::Quat, osg::Vec3>(radEuler));
	}

	return ret;
}

const std::vector<osg::Vec3> QuatCommon::convert(
	const QuatUtils::QuatLiftingScheme::Data & quatChannel)
{
	std::vector<osg::Vec3> ret;
	ret.reserve(quatChannel.size());

	for(unsigned int i = 0; i < quatChannel.size(); ++i){
		auto euler = osg::QuatUtils::quaterionToEuler
			<osg::Quat, osg::Vec3>(quatChannel[i]);

		osg::Vec3 degEuler(osg::RadiansToDegrees(euler.x()),
			osg::RadiansToDegrees(euler.y()),
			osg::RadiansToDegrees(euler.z()));

		ret.push_back(degEuler);
	}

	return ret;
}


const double QuatCommon::dist(const QuatUtils::QuatLiftingScheme::Data & src,
	const QuatUtils::QuatLiftingScheme::Data & dest)
{
	double ret = 0;
	
	auto s = std::min(src.size(), dest.size());

	for(unsigned int i = 0; i < s; ++i){
		
		ret += std::fabs(std::acos(std::max<double>(
			std::min<double>((dest[i] / src[i]).w(), 1.0), -1.0)));

	}

	return ret;
}


void QuatCommon::exportData(const std::string & path,
	const VectorChannelReaderInterfaceConstPtr & src,
	const VectorChannelReaderInterfaceConstPtr & dest)
{
	std::ofstream out(path);

	auto s = std::min(src->size(), dest->size());

	for(unsigned int i = 0; i < s; ++i){
		const auto & s = src->value(i);
		const auto & d = dest->value(i);
		out << s.x() << " " << s.y() << " " << s.z()
			<< " " << d.x() << " " << d.y() << " " << d.z() << std::endl;
	}

	out.flush();
	out.close();
}