#include <QuatUtils/QuaternionCompressor.h>

using namespace QuatUtils;

const QuatLiftingCompressor::CompressedSignal QuatLiftingCompressor::compress(const QuatLiftingScheme::Data & data,
	const CompressionSettings & settings)
{

	if(data.empty() == true){
		throw std::invalid_argument("Empty data to compress");
	}

	QuatLiftingScheme::Data::size_type elementsCount = LiftingScheme::LiftingSchemeUtils::floorPowerOfTwo(data.size());
	QuatLiftingScheme::Data::size_type levelsCount = LiftingScheme::LiftingSchemeUtils::ilog2(elementsCount);

	CompressedSignal ret;
	ret.globalAverage = data.front();
	ret.sourceResolutions = levelsCount;

	const auto endIT = settings.end();

	for(unsigned int i = 0; i < levelsCount; ++i){

		if(settings.find(i) == endIT){
			unsigned int start = std::pow(2.0, (int)i);
			unsigned int end = start << 1;

			QuatLiftingScheme::Data resData;
			resData.reserve(start);

			for( ; start < end; ++start){
				resData.push_back(data[start]);
			}

			ret.compressedData.insert(CompressedCoefficients::value_type(i, resData));
		}

	}

	return ret;
}

const QuatLiftingScheme::Data QuatLiftingCompressor::decompress(const CompressedSignal & data)
{
	QuatLiftingScheme::Data ret;

	const unsigned int signalLevels = data.sourceResolutions;

	ret.reserve(std::pow(2.0, (int)signalLevels));

	ret.push_back(data.globalAverage);
	
	for(unsigned int i = 0; i <= signalLevels; ++i){
		
		auto it = data.compressedData.find(i);
		if(it != data.compressedData.end()){
			ret.insert(ret.end(), it->second.begin(), it->second.end());
		}else{
			const unsigned int count = std::pow(2.0, (int)i);
			for(unsigned int j = 0; j < count; ++j){
				ret.push_back(osg::Quat(0.0, 0.0, 0.0, 1.0));
			}
		}
	}

	return ret;

}