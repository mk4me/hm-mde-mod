/********************************************************************
    created:  2013/08/24
    created:  24:8:2013   12:16
    filename: QuatCommon.h
    author:   Mateusz Janiak
    
    purpose:  
*********************************************************************/
#ifndef HEADER_GUARD_MOTION_ANALYSIS__QUATCOMMON_H__
#define HEADER_GUARD_MOTION_ANALYSIS__QUATCOMMON_H__

#include "Types.h"

class QuatCommon
{
public:

	static const QuatUtils::QuatLiftingScheme::Data convert(
		const VectorChannelReaderInterfaceConstPtr vectorChannel);

	static const std::vector<osg::Vec3> convert(
		const QuatUtils::QuatLiftingScheme::Data & quatChannel);


	static const double dist(const QuatUtils::QuatLiftingScheme::Data & src,
		const QuatUtils::QuatLiftingScheme::Data & dest);


	static void exportData(const std::string & path,
		const VectorChannelReaderInterfaceConstPtr & src,
		const VectorChannelReaderInterfaceConstPtr & dest);
};


#endif	//	HEADER_GUARD_MOTION_ANALYSIS__QUATCOMMON_H__
