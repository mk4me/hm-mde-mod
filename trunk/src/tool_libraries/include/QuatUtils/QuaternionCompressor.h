/********************************************************************
    created:  2013/08/21
    created:  21:8:2013   11:23
    filename: QuaternionCompressor.h
    author:   Mateusz Janiak
    
    purpose:  
*********************************************************************/
#ifndef HEADER_GUARD_QuatCompression__QUATERNIONCOMPRESSOR_H__
#define HEADER_GUARD_QuatCompression__QUATERNIONCOMPRESSOR_H__

#include <QuatUtils/QuaternionInterpolators.h>
#include <set>
#include <map>


namespace QuatUtils
{
		
	class QuatLiftingCompressor
	{
	public:

		//! Typ zbioru opisuj�cego rozdzielczo�ci do usuni�cia
		typedef std::set<unsigned int> CompressionSettings;
		//! Typ mapy z pozostawionymi rozdzielczo�ciami
		typedef std::map<unsigned int, QuatLiftingScheme::Data> CompressedCoefficients;
		//! Typ opisujacy skompresowane dane - pocz�tkowa warto�� �rednia + skompresowane detale
		struct CompressedSignal
		{
			osg::Quat globalAverage;
			CompressedCoefficients compressedData;
			unsigned int sourceResolutions;
		};

		//! \param data Dane po dekompozycji schematem liftingu
		//! \para settings Ustawienia kompresji - kt�re poziomy odrzucamy
		static const CompressedSignal compress(const QuatLiftingScheme::Data & data,
			const CompressionSettings & settings);

		//! \param data Dane skompresowane do dekompresji
		static const QuatLiftingScheme::Data decompress(const CompressedSignal & data);

	};

}


#endif	//	HEADER_GUARD_QuatCompression__QUATERNIONCOMPRESSOR_H__
