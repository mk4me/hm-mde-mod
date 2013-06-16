/********************************************************************
    created:  2013/04/05
    created:  5:4:2013   15:03
    filename: UnscentedTransform.h
    author:   Mateusz Janiak
    
    purpose:  
*********************************************************************/
#ifndef HEADER_GUARD___UNSCENTEDTRANSFORM_H__
#define HEADER_GUARD___UNSCENTEDTRANSFORM_H__

#include <cmath>
#include <boost/array.hpp>

namespace GeneralAlgorithms {

namespace UnscentedTransforms
{
	//! Struktura opisuj¹ca punkt sigma - stan + waga
	//! \tparam State Typ opisuj¹cy stan procesu
	template<typename State>
	struct SigmaPoint
	{
		//! Stan
		State point;
		//! Waga
		double weight;
	};

	//! Struktura pomocnicza definiuj¹ca tablice punktów sigma
	//! \tparam State Typ opisuj¹cy stan procesu
	//! \tparam SigmaPointsSize Iloœæ punktów sigma
	template<typename State, unsigned int SigmaPointsSize>
	struct SigmaPoints
	{
		//! Typ tablicy punktów sigma
		typedef boost::array<SigmaPoint<State>, SigmaPointsSize> type;
	};

	//! Struktura pomocnicza przy wyznaczaniu iloœci punktów sigma dla
	//! Scaled Unscented Transform
	//! \tparam StateSize Rozmiar wektora stanu procesu
	template<unsigned int StateSize>
	struct ScaledUnscentedTransformSigmaPointsSizeHelper
	{
		//! Iloœæ punktów sigma
		static const unsigned int size = 2 * StateSize + 1;
	};

	//! Wzorzec klasy generuj¹cej punkty sigma dla Scaled Unscented Transform
	//! http://www.google.pl/url?sa=t&rct=j&q=&esrc=s&source=web&cd=1&cad=rja&ved=0CDYQFjAA&url=http%3A%2F%2Fwww.cs.unc.edu%2F~welch%2Fkalman%2Fmedia%2Fpdf%2FACC02-IEEE1357.PDF&ei=oUFfUbjPFoWB4gS4u4GQBg&usg=AFQjCNFmnkpM_cu6l8OwuEZODc5tZuES7A&sig2=2QjcKPftqtnLA377MR2zWg&bvm=bv.44770516,d.bGE
	//! \tparam StateSize Rozmiar wektora stanu procesu
	template<unsigned int StateSize>
	class ScaledUnscentedTransformSigmaPointsGenerator
	{
	public:
		//! Iloœæ punktów sigma do generacji
		static const unsigned int SigmaPointsSize = ScaledUnscentedTransformSigmaPointsSizeHelper<StateSize>::size;

	private:
		//! Parametr 
		double k_;		
		//! Parametr 
		double a_;	
		//! Parametr
		double b_;

		//! Pomocnicza
		double mul;
		//! Pomocnicza
		double weight;
		//! Pomocnicza
		double weight0;

	private:

		//! Odœwia¿a zmienne pomocnicze przy zmianie parametrów
		void updateHelpers()
		{
			const double a2Inv = 1.0 / std::pow(a_, 2.0);						
			mul =  a_ * std::sqrt(StateSize + k_);			
			weight = a2Inv / (2.0 * (StateSize + k_));			
			weight0 = k_ * a2Inv / (StateSize + k_) + 1.0 - a2Inv;
		}

	public:
		//! Publiczny konstruktor
		ScaledUnscentedTransformSigmaPointsGenerator(double k = 3.0 - StateSize, double a = 1.0, double b = 2.0)
			: k_(k), a_(a), b_(b)
		{
			updateHelpers();
		}

		const double k() const
		{
			return k_;
		}

		void k(const double kk)
		{
			k_ = kk;
			updateHelpers();
		}

		const double a() const
		{
			return a_;
		}

		void a(const double aa)
		{
			a_ = aa;
			updateHelpers();
		}

		const double b() const
		{
			return b_;
		}

		void b(const double bb)
		{
			b_ = bb;
			updateHelpers();
		}

		//! Metoda generuj¹ca punkty sigma
		template<typename CovarianceMatrix, typename StateType>
		void generateSigmaPoints(const StateType & mean, const CovarianceMatrix & sqrtCovarianceMatrix, typename SigmaPoints<StateType, SigmaPointsSize>::type & sigmaPoints)
		{
			sigmaPoints[0].weight = weight0;
			sigmaPoints[0].point = mean;

			for(unsigned int i = 0; i < StateSize; ++i){
				sigmaPoints[1 + i].weight = sigmaPoints[SigmaPointsSize - 1 - i].weight = weight;
				sigmaPoints[1 + i].point = mean + mul * sqrtCovarianceMatrix.col(i);
				sigmaPoints[SigmaPointsSize - 1 - i].point = mean - mul * sqrtCovarianceMatrix.col(i);
			}
		}
	};
}

}

#endif	//	HEADER_GUARD___UNSCENTEDTRANSFORM_H__
