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

#include <IMU/Data/Types.h>
#include <Eigen/Dense>
#include <vector>
#include <limits>
#include <QuatUtils/QuatUtils.h>
#include <iterator>

#ifdef max
#undef max
#endif

#ifdef min
#undef min
#endif

namespace IMU {

	class OrientationFitting
	{
	public:

		//! Metoda szuka obszarów nak³adaj¹cych sie w dwóch sygna³ach wraz
		//! z ich wzajemnym obrotem
		//! \tparam ItA Iterator pierwszej serii orientacji
		//! \tparam ItB Iterator drugiej serii orientacji
		//! \param startA Iterator pocz¹tku pierwszej serii orientacji
		//! \param endA Iterator koñca pierwszej serii orientacji
		//! \param startB Iterator pocz¹tku drugiej serii orientacji
		//! \param endB Iterator koñca drugiej serii orientacji
		//! \param[out] rotation Rotacja pomiêdzy seri¹ orientacji A i B dopasowuj¹c¹ B do A
		//! \param[out] offset Offset o jaki trzeba przesun¹æ B wzglêdem A aby dopasowaæ dane
		//! \param step Krok z jakim przesuwamy dane serii B wzglêdem danych serii A
		//! \param skip Iloœæ opuszczonych danych w serii A
		template<class ItA, class ItB>
		static void overlap(const ItA startA, const ItA endA, ItB startB,
			const ItB endB, Quat & rotation,
			unsigned long int & offset, const unsigned long int step = 1,
			const unsigned long int skip = 0, const unsigned int dx = 20);
	};




	//! --------------------------------- Implementacja ------------------------
	
	

	static Quat localRotation(const Quat & src,
		const Quat & dest)
	{		
		Vec3 orientA = osg::QuatUtils::quaterionToEuler<Quat, Vec3>(src);
		Vec3 orientB = osg::QuatUtils::quaterionToEuler<Quat, Vec3>(dest);	

		orientB -= orientA;

		osg::QuatUtils::clampEuler(orientB);

		return osg::QuatUtils::eulerToQuaternion<Quat, Vec3>(orientB);
	}

	static Quat localRotationM(const Quat & src,
		const Quat & dest)
	{		
		Eigen::Matrix3d rot = src.matrix().colPivHouseholderQr().solve(dest.matrix());
		return Quat(rot);
	}

	static Quat __localRotationM(const Quat & src,
		const Quat & dest)
	{
		Eigen::Matrix3d srcM;

		Eigen::Vector3d x = src._transformVector(Eigen::Vector3d::UnitX());
		Eigen::Vector3d y = src._transformVector(Eigen::Vector3d::UnitY());
		Eigen::Vector3d z = src._transformVector(Eigen::Vector3d::UnitZ());

		srcM(0,0) = x.x();
		srcM(1,0) = x.y();
		srcM(2,0) = x.z();
		srcM(0,1) = y.x();
		srcM(1,1) = y.y();
		srcM(2,1) = y.z();
		srcM(0,2) = z.x();
		srcM(1,2) = z.y();
		srcM(2,2) = z.z();

		Eigen::Matrix3d destM;

		x = dest._transformVector(Eigen::Vector3d::UnitX());
		y = dest._transformVector(Eigen::Vector3d::UnitY());
		z = dest._transformVector(Eigen::Vector3d::UnitZ());

		destM(0,0) = x.x();
		destM(1,0) = x.y();
		destM(2,0) = x.z();
		destM(0,1) = y.x();
		destM(1,1) = y.y();
		destM(2,1) = y.z();
		destM(0,2) = z.x();
		destM(1,2) = z.y();
		destM(2,2) = z.z();

		Eigen::Matrix3d rot = srcM.colPivHouseholderQr().solve(destM);

		//Eigen::Quaterniond q(rot);



		return Quat(rot);
	}

	template<class ItA, class ItB>
	void OrientationFitting::overlap(const ItA startA, const ItA endA, ItB startB,
		const ItB endB, Quat & rotation,
		unsigned long int & offset, const unsigned long int step,
		const unsigned long int skip, const unsigned int dx){		

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

		auto size = sizeB - skip;

		if((sizeB - skip) < sizeA){
			eA = startA;
			std::advance(eA, size);
		}else{
			size = sizeA;
		}

		offset = skip;
		rotation.setIdentity();

		double minCost = std::numeric_limits<double>::max();
		auto currentA = startA;
		auto currentB = sB;
		
		auto locRotation = *currentB * (*currentA).inverse();		


		double cost = 0;
		//double cost = osg::QuatUtils::quatsDifference(startA, eA, startB, endB, locRotation);
		if(cost < minCost){
			minCost = cost;
			rotation = locRotation;
		}
		

		const auto subsetepsCount = size / dx;
		/*
		//teraz kolejna petla sprawdzaj¹ca czy wewn¹trz nie ma lepszego dopasowania
		for(unsigned int i = 0; i < subsetepsCount - 1; ++i){			
			std::advance(currentA, dx);
			std::advance(currentB, dx);

			//locRotation = *currentB * (*currentA).inverse();
			auto locRotation = getLocalRotationM(*currentA, *currentB);

			cost = difference(startA, eA, startB, locRotation);
			if(cost < minCost){
				minCost = cost;
				rotation = locRotation;
			}
		}
		}*/

		if(eA == endA){
			const unsigned long int totalSteps = (sizeB - skip - sizeA) / step;
			unsigned long int currentStep = 0;

			while(currentStep++ < totalSteps){
				
				std::advance(sB, step);

				currentA = startA;
				currentB = sB;
				
				locRotation = *currentB * (*currentA).inverse();

				//cost = osg::QuatUtils::quatsDifference(startA, endA, sB, endB, locRotation);
				if(cost < minCost){
					minCost = cost;
					rotation = locRotation;
					offset = skip + currentStep * step;
				}

				/*
				//teraz kolejna petla sprawdzaj¹ca czy wewn¹trz nie ma lepszego dopasowania
				for(unsigned int i = 0; i < subsetepsCount - 1; ++i){			
					std::advance(currentA, dx);
					std::advance(currentB, dx);

					//locRotation = *currentB * (*currentA).inverse();
					auto locRotation = getLocalRotationM(*currentA, *currentB);

					cost = difference(startA, endA, sB, locRotation);
					if(cost < minCost){
						minCost = cost;
						rotation = locRotation;
						offset = skip + currentStep * step;
					}
				}
				}*/
			}
		}
	}
}

#endif	//	HEADER_GUARD___ORIENTATIONFITTING_H__
