/********************************************************************
    created:  2013/06/15
    created:  15:6:2013   15:59
    filename: OrientationFitting.h
    author:   Mateusz Janiak
    
    purpose:  Klasa znajduj¹ca najlepsze dopasowanie pomiêdzy dwoma uk³adami
				wspó³rzêdnych dla zadanych próbek. Próbki obu sygna³ów mog¹
				byæ przesuniête w czasie. Otrzymujemy wartoœc przesuniêcia
				+ obót jednego uk³adu w drugi - relacjê pomiêdzy nimi
*********************************************************************/
#ifndef HEADER_GUARD___ORIENTATIONFITTING_H__
#define HEADER_GUARD___ORIENTATIONFITTING_H__

#include <Eigen/Dense>
#include <vector>

namespace IMU {

	class OrientationFitting
	{
	public:
		//! Typ kwatenrionu rotacji jednego uk³adu w drugi
		typedef Eigen::Quaternion<double> Quat;

	public:

		//! Metoda szuka obszarów nak³adaj¹cych sie w dwóch sygna³ach wraz
		//! z ich wzajemnym obrotem
		//! \tparam ItA Iterator pierwszej serii orientacji
		//! \tparam ItB Iterator drugiej serii orientacji
		//! \param startA
		//! \param endA
		//! \param startB
		//! \param endB
		//! \param[out] rotation
		//! \param[out] offset
		//! \param step
		//! \param skip
		template<class ItA, class ItB>
		static void overlap(const ItA startA, const ItA endA, ItB startB,
			const ItB endB, Quat & rotation,
			unsigned long int & offset, const unsigned long int step = 1,
			const unsigned long int skip = 0);	

		//! Metoda dopasowuje najlepszy obrót pomiêdzy uk³adami
		//! Zakres jednego musi mieœciæ siê w zakresie drugiego
		//! \tparam ItA Iterator pierwszej serii orientacji
		//! \tparam ItB Iterator drugiej serii orientacji
		//! \param startA
		//! \param endA
		//! \param startB
		template<class ItA, class ItB>
		static void fitOrientation(const ItA startA, const ItA endA,
			ItB startB, const unsigned long int range, Quat & rotation,
			unsigned long int & offset);

	private:

		//! Metoda zwraca sumê k¹tów dla ró¿nic kwaternionów
		template<class ItA, class ItB>
		static const double difference(ItA startA, const ItA endA, ItB startB,
			const Quat & rotation);

	};




	//! --------------------------------- Implementacja ------------------------

	template<class ItA, class ItB>
	void OrientationFitting::overlap(const ItA startA, const ItA endA, ItB startB,
		const ItB endB, Quat & rotation,
		unsigned long int & offset, const unsigned long int step,
		const unsigned long int skip){

		if(step == 0){
			throw std::runtime_error("Wrong step value");
		}

		if(startA == endA || startB == endB){
			throw std::runtime_error("One of data series empty");
		}

		unsigned long int sizeB = std::distance(startB, endB);		

		if(skip >= sizeB){
			throw std::runtime_error("Skip value greater than provided range");
		}

		unsigned long int sizeA = std::distance(startA, endA);

		if(sizeA > sizeB){
			throw std::runtime_error("Fitting range greater than data to fit");
		}

		auto sB = startB;
		std::advance(sB, skip);

		auto eA = endA;

		if((sizeB - skip) < sizeA){
			eA = startA;
			std::advance(eA, sizeB - skip);
		}

		offset = skip;
		rotation = *startA * (*sB).inverse();

		double minCost = difference(startA, eA, startB, rotation);

		if(eA == endA){
			const unsigned long int totalSteps = (sizeB - skip - sizeA) / step;
			unsigned long int currentStep = 0;

			while(currentStep++ < totalSteps){
				
				std::advance(sB, step);

				auto rot = *startA * (*sB).inverse();

				double cost = difference(startA, endA, sB, rot);

				if(cost < minCost){
					minCost = cost;
					rotation = rot;
					offset = skip + currentStep * step;
				}
			}			
		}
	}

	template<class ItA, class ItB>
	static void OrientationFitting::fitOrientation(const ItA startA, const ItA endA,
		ItB startB, const unsigned long int range, Quat & rotation, unsigned long int & offset)
	{

		double minError = std::numeric_limits<double>::max();

		for(unsigned int i = 0; i < range; ++i, ++startB){

			auto sB = startB;
			auto r = *startA * (*sB).inverse();

			std::vector<Quat> differences;

			for(auto sA = startA; sA != endA; ++sA, ++sB){
				differences.push_back(Quat(*sA * (*sB * r).inverse()));
			}

			Quat adjustmentQuat(1.0, 0.0, 0.0, 0.0);

			osg::QuatUtils::average(differences.begin(), differences.end(), adjustmentQuat);

			r *= adjustmentQuat;

			double error = difference(startA, endA, startB, r);

			if(error < minError){
				minError = error;
				rotation = r;
				offset = i;
			}
		}
	}

	template<class ItA, class ItB>
	const double OrientationFitting::difference(ItA startA, const ItA endA, ItB startB,
		const Quat & rotation)
	{
		double ret = 0.0;

		for( ; startA != endA; ++startA, ++startB){
			auto dif = *startA * (*startB * rotation).inverse();
			ret += std::acos(dif.w());
		}

		return ret;
	}
}

#endif	//	HEADER_GUARD___ORIENTATIONFITTING_H__
