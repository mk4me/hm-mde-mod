/********************************************************************
    created:  2013/08/21
    created:  21:8:2013   14:12
    filename: Types.h
    author:   Mateusz Janiak
    
    purpose:  
*********************************************************************/
#ifndef HEADER_GUARD_MOTION_ANALYSIS__TYPES_H__
#define HEADER_GUARD_MOTION_ANALYSIS__TYPES_H__

#include <corelib/BaseDataTypes.h>
#include <plugins/dfElements/DFPins.h>
#include <plugins/c3d/C3DChannels.h>
#include <plugins/kinematic/Wrappers.h>
#include <QuatUtils/QuaternionCompressor.h>
#include <QuatUtils/QuaternionInterpolators.h>
#include <plugins/newVdf/UniversalOutputPin.h>
#include <utils/PtrPolicyStd.h>

typedef vdf::UniversalOutputPinT<kinematic::JointAnglesCollection> JointAnglesCollectionOutputPin;
typedef vdf::UniversalInputPinT<kinematic::JointAnglesCollection> JointAnglesCollectionInputPin;

DEFINE_WRAPPER(kinematic::JointAngleChannel, utils::PtrPolicyStd, utils::ClonePolicyVirtualCloneMethod);

typedef vdf::UniversalOutputPinT<kinematic::JointAngleChannel> JointAnglesOutputPin;
typedef vdf::UniversalInputPinT<kinematic::JointAngleChannel> JointAnglesInputPin;

DEFINE_WRAPPER(QuatUtils::QuatLiftingCompressor::CompressedSignal, utils::PtrPolicyStd, utils::ClonePolicyCopyConstructor);

DEFINE_WRAPPER(utils::ConstObjectsList, utils::PtrPolicyStd, utils::ClonePolicyCopyConstructor);

typedef vdf::UniversalOutputPinT<QuatUtils::QuatLiftingCompressor::CompressedSignal> QuatCompressedSignalOutputPin;
typedef vdf::UniversalInputPinT<QuatUtils::QuatLiftingCompressor::CompressedSignal> QuatCompressedSignalInputPin;
		
typedef vdf::UniversalOutputPinT<kinematic::JointAnglesCollection> JointAnglesCollectionOutputPin;
typedef vdf::UniversalInputPinT<kinematic::JointAnglesCollection> JointAnglesCollectionInputPin;
		
typedef vdf::UniversalOutputPinT<utils::ConstObjectsList> ObjectsCollectionOutputPin;
typedef vdf::UniversalInputPinT<utils::ConstObjectsList> ObjectsCollectionInputPin;

typedef LiftingScheme::InterpolatorLiftingSchemeT<osg::Quat, QuatUtils::QuatLinHaarInterpolator> LinearHaarLS;
typedef LiftingScheme::InterpolatorLiftingSchemeT<osg::Quat, QuatUtils::QuatHaarInterpolator> QuatHarrLS;
typedef LiftingScheme::InterpolatorLiftingSchemeT<osg::Quat, QuatUtils::QuatLerpInterpolator> QuatLerpLS;
typedef LiftingScheme::InterpolatorLiftingSchemeT<osg::Quat, QuatUtils::QuatSlerpInterpolator> QuatSlerpLS;
typedef QuatUtils::PseudoQuatTangentLiftingScheme<> PseudoTangentSpaceLS;


#endif	//	HEADER_GUARD_MOTION_ANALYSIS__TYPES_H__
