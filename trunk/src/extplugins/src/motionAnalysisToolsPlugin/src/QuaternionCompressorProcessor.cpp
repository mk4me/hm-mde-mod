#include "QuaternionCompressorProcessor.h"

QuaternionSignalCompressor::QuaternionSignalCompressor()
{
	inPinA = new JointAnglesInputPin(this);
	outPinA = new QuatCompressedSignalOutputPin(this);
	addInputPin(inPinA);
	addOutputPin(outPinA);
}

QuaternionSignalCompressor::~QuaternionSignalCompressor()
{

}

void QuaternionSignalCompressor::process()
{
	auto inQuatData = inPinA->getValue();

	//kopiuje go - przygotowuje sobie porcje danych wyjsciowych

	utils::shared_ptr<QuatUtils::QuatLiftingCompressor::CompressedSignal> outData(new QuatUtils::QuatLiftingCompressor::CompressedSignal);
	//kinematic::JointAngleChannel(inQuatData->getSamplesPerSecond()));
	//outQuatData->setName("Forward lifting scheme");
	//outQuatData->setTimeBaseUnit(inQuatData->getTimeBaseUnit());
	//outQuatData->setValueBaseUnit(inQuatData->getValueBaseUnit());

	QuatUtils::QuatLiftingScheme::Data lsData;
	auto size = LiftingScheme::LiftingSchemeUtils::floorPowerOfTwo(inQuatData->size());
	auto resolutionsCount = LiftingScheme::LiftingSchemeUtils::ilog2(size);

	//modyfikuje dane ktore bêdê zaraz udostepnia³ na wyjœciu	
	for(kinematic::JointAngleChannel::size_type i = 0; i < size; ++i){
		lsData.push_back(inQuatData->value(i));
	}

	QuatUtils::QuatLiftingCompressor::CompressionSettings settings;
	const int toRemove = 3;

	int idx = 0;
	int res = resolutionsCount-1;

	while(idx++ < toRemove && res > 0){
		settings.insert(res--);
	}

	(*outData) = QuatUtils::QuatLiftingCompressor::compress(lsData, settings);

	//zapisujê zmodyfikowane dane do udostêpniena
	outPinA->setValue(outData);
}

void QuaternionSignalCompressor::reset()
{

}


QuaternionSignalDecompressor::QuaternionSignalDecompressor()
{
	outPinA = new JointAnglesOutputPin(this);
	inPinA = new QuatCompressedSignalInputPin(this);
	addInputPin(inPinA);
	addOutputPin(outPinA);
}

QuaternionSignalDecompressor::~QuaternionSignalDecompressor()
{

}

void QuaternionSignalDecompressor::process()
{
	auto inData = inPinA->getValue();

	//kopiuje go - przygotowuje sobie porcje danych wyjsciowych

	kinematic::JointAngleChannelPtr outQuatData(new kinematic::JointAngleChannel(100));
	outQuatData->setName("Decompressed signal");
	outQuatData->setTimeBaseUnit("s");
	outQuatData->setValueBaseUnit("");

	auto res = QuatUtils::QuatLiftingCompressor::decompress(*inData);

	//modyfikuje dane ktore bêdê zaraz udostepnia³ na wyjœciu	
	for(kinematic::JointAngleChannel::size_type i = 0; i < res.size(); ++i){
		outQuatData->addPoint(res[i]);
	}

	//zapisujê zmodyfikowane dane do udostêpniena
	outPinA->setValue(outQuatData);
}

void QuaternionSignalDecompressor::reset()
{

}