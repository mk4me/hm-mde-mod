#include <corelib/IPlugin.h>
#include <plugins/newVdf/IDataFlowProvider.h>
#include "EulerNoiseAdderProcessor.h"
#include "QuaternionCompressorProcessor.h"
#include "QuaternionDenoiseProcessor.h"
#include "QuaternionEulerConverter.h"
#include "QuaternionExpProcessor.h"
#include "QuaternionLogProcessor.h"
#include "QuatLiftingSchemeTProcessor.h"
#include "SkeletonCompressorProcessor.h"
#include "DegreesToRadians.h"
#include "QuaternionToAngleAxis.h"
#include <plugins/newVdf/IDataFlowProvider.h>
#include <GeneralAlgorithms/LiftingScheme/LiftingSchemeT.h>
#include <QuatUtils/QuaternionInterpolators.h>
#include <plugins/dfElements/DFSources.h>
#include "QuatCompression.h"

typedef TransformProcessor<true, LinearHaarLS> LinHaarF;
typedef TransformProcessor<false, LinearHaarLS> LinHaarB;

typedef TransformProcessor<true, QuatHarrLS> QuatHaarF;
typedef TransformProcessor<false, QuatHarrLS> QuatHaarB;

typedef TransformProcessor<true, QuatLerpLS> QuatLerpF;
typedef TransformProcessor<false, QuatLerpLS> QuatLerpB;

typedef TransformProcessor<true, QuatSlerpLS> QuatSlerpF;
typedef TransformProcessor<false, QuatSlerpLS> QuatSlerpB;

typedef TransformProcessor<true, PseudoTangentSpaceLS> PseudoQuatLiftF;
typedef TransformProcessor<false, PseudoTangentSpaceLS> PseudoQuatLiftB;

//rozpoczêscie definicji pluginu - podajemy jego nazwe i unikalny identyfikator
CORE_PLUGIN_BEGIN("MotionDataAnalysisPlugin", core::UID::GenerateUniqueID("{07F0084C-B1EA-4D2F-8281-785D5EA5086F}"))

	CORE_PLUGIN_ADD_OBJECT_WRAPPER(kinematic::JointAngleChannel);

	CORE_PLUGIN_ADD_OBJECT_WRAPPER(QuatUtils::QuatLiftingCompressor::CompressedSignal);

	CORE_PLUGIN_ADD_OBJECT_WRAPPER(utils::ConstObjectsList);

	// dodajemy element przetwarzajacy - klase tego elementu, musi on mieæ domyœlny konstruktor
	// Wszystkie tego typu elementy musz¹ znaleŸæ siê wewn¹trz serwisu. 
	VDF_SERVICE_BEGIN(MotionVDFElementsService, "{C45743E2-4D41-4B73-AE72-91E979FD0E8C}")

		//VDF_ADD_DATA_SOURCE(
		//UniversalSource<kinematic::JointAnglesCollection>, 
		//	"Skeleton source",
		//	"{03FCCADA-5814-4B7B-BC60-2E07CBBF15FB}",
		//	QIcon(":/dfElements/icons/source.png"));

		VDF_ADD_DATA_SOURCE(
			SkeletonSource,
			"Skeleton Custom Source",
			"{CEBCAA8B-FD4C-4862-BC52-FA287160DC23}",
			QIcon());

		VDF_ADD_DATA_PROCESSOR(
			MotionAnalysisTests,
			"Motion Analysis Tests",
			"{CEBCAA8E-FD4C-4852-BC52-F0287160DC23}",
			QIcon());

		/*
		VDF_ADD_DATA_PROCESSOR(
			EulerNoiseAdderProcessor,
			"Euler Angles Noise Adder",
			"{CEBCAA8E-FD4C-4862-BC52-FA287160DC23}",
			QIcon());

		VDF_ADD_DATA_PROCESSOR(
			QuaternionDenoiseProcessor,
			"Quaternion De noising Processor",
			"{CEBCAAAE-FD4C-4862-BC52-FA287160DC23}",
			QIcon());

		VDF_ADD_DATA_PROCESSOR(
			QuaternionToEulerConverter,
			"Quaternion To Euler Angles Converter",
			"{CEBCAAAA-FD4C-4862-BC52-FA287160DC23}",
			QIcon());

		VDF_ADD_DATA_PROCESSOR(
			EulerToQuaternionConverter,
			"Euler To Quaternion Angles Converter",
			"{1EBCAAAA-FD4C-4862-BC52-FA287160DC23}",
			QIcon());

		VDF_ADD_DATA_PROCESSOR(
			QuaternionExpProcessor,
			"Quaternion Exponent",
			"{CEBAAAAA-FD4C-4862-BC52-FA287160DC23}",
			QIcon());

		VDF_ADD_DATA_PROCESSOR(
			QuaternionLogProcessor,
			"Quaternion Logarithm",
			"{CECAAAAA-FD4C-4862-BC52-FA287160DC23}",
			QIcon());

		VDF_ADD_DATA_PROCESSOR(
			QuaternionProcessorT<LinHaarF>,
			"Linear Haar Forward Transform",
			"{CECAAAAA-BD4C-4862-BC52-FA287160DC23}",
			QIcon());

		VDF_ADD_DATA_PROCESSOR(
			QuaternionProcessorT<LinHaarB>,
			"Linear Haar Backward Transform",
			"{CECAAAAA-BD4C-4862-1C52-FA287160DC23}",
			QIcon());

		VDF_ADD_DATA_PROCESSOR(
			QuaternionProcessorT<QuatHaarF>,
			"Quat Haar Forward Transform",
			"{CECAAAAA-BD4C-4861-BC52-FA287160DC23}",
			QIcon());

		VDF_ADD_DATA_PROCESSOR(
			QuaternionProcessorT<QuatHaarB>,
			"Quat Haar Backward Transform",
			"{CECAAAAA-BD4C-4862-BC52-FA287160DC2A}",
			QIcon());

		VDF_ADD_DATA_PROCESSOR(
			QuaternionProcessorT<QuatLerpF>,
			"Quat Lerp Forward Transform",
			"{CECAAAAA-BDAC-4861-BC52-FA287160DC23}",
			QIcon());

		VDF_ADD_DATA_PROCESSOR(
			QuaternionProcessorT<QuatLerpB>,
			"Quat Lerp Backward Transform",
			"{CECAAAAA-BA4C-4862-BC52-FA287160DC2A}",
			QIcon());

		VDF_ADD_DATA_PROCESSOR(
			QuaternionProcessorT<QuatSlerpF>,
			"Quat Slerp Forward Transform",
			"{CECAAEEA-BDAC-4861-BC52-FA287160DC23}",
			QIcon());

		VDF_ADD_DATA_PROCESSOR(
			QuaternionProcessorT<QuatSlerpB>,
			"Quat Slerp Backward Transform",
			"{CECAFFAA-BA4C-4862-BC52-FA287160DC2A}",
			QIcon());

		VDF_ADD_DATA_PROCESSOR(
			QuaternionProcessorT<PseudoQuatLiftF>,
			"Tangent Space Quat Forward Transform",
			"{CECAAEEA-BDAC-4861-BA52-FA287160DC23}",
			QIcon());

		VDF_ADD_DATA_PROCESSOR(
			QuaternionProcessorT<PseudoQuatLiftB>,
			"Tangent Space Quat Backward Transform",
			"{CECAFFAA-BA4C-4862-BA52-FA287160DC2A}",
			QIcon());

		VDF_ADD_DATA_PROCESSOR(
			EulerRadiansToDegreesConverter,
			"Euler Angles Radians To Degrees Converter",
			"{CECAFFAA-BA4C-4862-BA52-FA287160DC20}",
			QIcon());

		VDF_ADD_DATA_PROCESSOR(
			EulerDegreesToRadiansConverter,
			"Euler Angles Degrees To Radians Converter",
			"{CECAFFAA-BA4C-4862-BA52-FA286160DC20}",
			QIcon());

		VDF_ADD_DATA_PROCESSOR(
			ScalarDegreesToRadiansConverter,
			"Scalar Degrees To Radians Converter",
			"{CECAFFA5-BA4C-4862-BA52-FA286160DC20}",
			QIcon());

		VDF_ADD_DATA_PROCESSOR(
			ScalarRadiansToDegreesConverter,
			"Scalar Radians To Degrees Converter",
			"{CECAFFAA-1A4C-4862-BA52-FA287160DC20}",
			QIcon());

		VDF_ADD_DATA_PROCESSOR(
			QuaternionToAngleAxisConverter,
			"Quaternion to Axis-Angle splitter",
			"{CECAFFAA-1A4C-4862-BA52-FA287160AC20}",
			QIcon());

		VDF_ADD_DATA_PROCESSOR(
			QuaternionSignalCompressor,
			"Quat Lifting Coefficients Compressor",
			"{CECAFBAA-1A4C-4862-BA52-FA287160DC20}",
			QIcon());

		VDF_ADD_DATA_PROCESSOR(
			QuaternionSignalDecompressor,
			"Quat Lifting Coefficients Decompressor",
			"{CECAFFAA-1A4A-4862-BA52-FA287160AC20}",
			QIcon());

		VDF_ADD_DATA_PROCESSOR(
			SkeletonCompressor<QuatHarrLS>,
			"Quat Haar Skeleton Compressor-Decompressor",
			"{B4C1FFAA-1A4A-4862-BA52-FA284160AC20}",
			QIcon());

		VDF_ADD_DATA_PROCESSOR(
			SkeletonCompressor<QuatLerpLS>,
			"Quat Lerp Skeleton Compressor-Decompressor",
			"{B3C1FFAA-1A4A-4862-BA52-FA284160AC20}",
			QIcon());

		VDF_ADD_DATA_PROCESSOR(
			SkeletonCompressor<QuatSlerpLS>,
			"Quat Slerp Skeleton Compressor-Decompressor",
			"{B2C1FFAA-1A4A-4862-BA52-FA284160AC20}",
			QIcon());

		VDF_ADD_DATA_PROCESSOR(
			SkeletonCompressor<PseudoTangentSpaceLS>,
			"Pseudo Tangent space Skeleton Compressor-Decompressor",
			"{B1C1FFAA-1A4A-4862-BA52-FA284160AC20}",
			QIcon());
			*/

		

	VDF_SERVICE_END(MotionVDFElementsService)

	//konczymy definicje pluginu
CORE_PLUGIN_END
