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

typedef UniversalOutputPinT<kinematic::JointAnglesCollection> JointAnglesCollectionOutputPin;
typedef UniversalInputPinT<kinematic::JointAnglesCollection> JointAnglesCollectionInputPin;

DEFINE_WRAPPER(kinematic::JointAngleChannel, utils::PtrPolicyBoost, utils::ClonePolicyVirtualCloneMethod);

typedef UniversalOutputPinT<kinematic::JointAngleChannel> JointAnglesOutputPin;
typedef UniversalInputPinT<kinematic::JointAngleChannel> JointAnglesInputPin;

DEFINE_WRAPPER(QuatUtils::QuatLiftingCompressor::CompressedSignal, utils::PtrPolicyBoost, utils::ClonePolicyCopyConstructor);

DEFINE_WRAPPER(core::ConstObjectsList, utils::PtrPolicyBoost, utils::ClonePolicyCopyConstructor);

typedef UniversalOutputPinT<QuatUtils::QuatLiftingCompressor::CompressedSignal> QuatCompressedSignalOutputPin;
typedef UniversalInputPinT<QuatUtils::QuatLiftingCompressor::CompressedSignal> QuatCompressedSignalInputPin;

typedef UniversalOutputPinT<kinematic::JointAnglesCollection> JointAnglesCollectionOutputPin;
typedef UniversalInputPinT<kinematic::JointAnglesCollection> JointAnglesCollectionInputPin;

typedef UniversalOutputPinT<core::ConstObjectsList> ObjectsCollectionOutputPin;
typedef UniversalInputPinT<core::ConstObjectsList> ObjectsCollectionInputPin;

typedef LiftingScheme::InterpolatorLiftingSchemeT<osg::Quat, QuatUtils::QuatLinHaarInterpolator> LinearHaarLS;
typedef LiftingScheme::InterpolatorLiftingSchemeT<osg::Quat, QuatUtils::QuatHaarInterpolator> QuatHarrLS;
typedef LiftingScheme::InterpolatorLiftingSchemeT<osg::Quat, QuatUtils::QuatLerpInterpolator> QuatLerpLS;
typedef LiftingScheme::InterpolatorLiftingSchemeT<osg::Quat, QuatUtils::QuatSlerpInterpolator> QuatSlerpLS;
typedef QuatUtils::PseudoQuatTangentLiftingScheme<> PseudoTangentSpaceLS;


#endif	//	HEADER_GUARD_MOTION_ANALYSIS__TYPES_H__
