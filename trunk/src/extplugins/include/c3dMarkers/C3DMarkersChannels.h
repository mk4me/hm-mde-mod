/********************************************************************
	created:	2011/10/10
	created:	10:10:2011   11:06
	filename: 	C3DChannels.h
	author:		Wojciech Kniec
	
	purpose:	
*********************************************************************/

#ifndef HEADER_GUARD_C3DMARKERS__C3DCHANNELS_H__
#define HEADER_GUARD_C3DMARKERS__C3DCHANNELS_H__

#include <utils/SmartPtr.h>
#include <utils/DataChannel.h>
#include <utils/DataChannelCollection.h>
#include <utils/ObjectWrapper.h>
#include <osg/Vec3>
#include <osg/Math>
#include <plugins/c3d/C3DCollections.h>
#include <c3dlib/C3DParser.h>
#include <utils/PtrPolicyStd.h>

class AllMarkersCollection : public c3dlib::MarkerCollection
{};
typedef boost::shared_ptr<AllMarkersCollection> AllMarkersCollectionPtr;
typedef boost::shared_ptr<const AllMarkersCollection> AllMarkersCollectionConstPtr;

DEFINE_WRAPPER(AllMarkersCollection, utils::PtrPolicyStd, utils::ClonePolicyCopyConstructor);

#endif

