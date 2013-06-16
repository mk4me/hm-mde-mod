/********************************************************************
    created:  2013/06/15
    created:  15:6:2013   12:16
    filename: DTW.h
    author:   Mateusz Janiak
    
    purpose:  Klasa realizuj�ca algorytm Dynamic Time Warping w podstawowej
				wersji
*********************************************************************/
#ifndef HEADER_GUARD___DTW_H__
#define HEADER_GUARD___DTW_H__

#include <Eigen/Dense>
#include <vector>
#include <algorithm>
#include <limits>

namespace GeneralAlgorithms {

	class DynamicTimeWarping
	{
	public:
		//! Macierz koszu dla DTW
		typedef Eigen::MatrixXd CostMatrix;
		//! �cie�ka w macierzy kosztu
		typedef std::vector<std::pair<CostMatrix::Index, CostMatrix::Index>> Path;

	public:
		//! \tparam ItA Iterator pierwszej serii danych
		//! \tparam ItB Iterator drugiej serii danych
		//! \tparam CostMeasure Miara kosztu przej�cia pomiedzy punktami serii danych
		//! \param startA Iterator pocz�tka pierwszej serii danych
		//! \param endA Iterator ko�ca pierwszej serii danych
		//! \param startB Iterator pocz�tka drugiej serii danych
		//! \param endB Iterator ko�ca drugiej serii danych		
		//! \param distanceMeasure Obiekt wyliczaj�cy odleg�o�� pomi�dzy punktami serii danych
		//! \return Wynik DTW - macierz koszt�w przej�cia
		template<class ItA, class ItB, class CostMeasure>
		static const CostMatrix dtw( ItA startA, ItA endA, ItB startB, ItB endB,
			const CostMeasure & costMeasure);

		//! \param costMatrix Macierz koszt�w dla wyliczonej dtw
		//! \return Najmniejszy koszt przej�cia w macierzy
		static const double minimalCost(const CostMatrix & costMatrix);
		//! \param costMatrix Macierz koszt�w dla wyliczonej dtw
		//! \param path �cie�ka o najmniejszym koszcie
		//! \return Najmniejszy koszt przej�cia w macierzy
		static const double minimalCost(const CostMatrix & costMatrix,
			const Path & path);

		//! \param costMatrix Macierz koszt�w dla wyliczonej dtw
		//! \param[out] path �cie�ka o najmniejszym koszcie
		static void shortestPath(const CostMatrix & costMatrix,
			Path & path);
	};


	//! -------------------------- Implementacja -----------------------------------------


	template<class ItA, class ItB, class CostMeasure>
	const Eigen::MatrixXd DynamicTimeWarping::dtw( ItA startA, const ItA endA, ItB startB, const ItB endB,
		const CostMeasure & costMeasure)
	{
		if(startA == endA || startB == endB){
			throw std::runtime_error("One of signals is empty");
		}		

		auto ret = Eigen::MatrixXd(std::distance(startA, endA),
									std::distance(startB, endB));

		//ret.fill(std::numeric_limits::infinity<double>());
		ret.col(0).fill(std::numeric_limits::infinity<double>());
		ret.row(0).fill(std::numeric_limits::infinity<double>());
		ret(0,0) = 0.0;

		++startA;
		++startB;

		for(Eigen::MatrixXd::Index idxA = 1; startA != endA; ++startA, idxA+=1){
			for(Eigen::MatrixXd::Index idxB = 1; startB != endB; ++startB; idxB+=1){

				auto cost = costMeasure.cost(*startA, *startB);
				ret(idxA, idxB) = cost + std::min(std::min(	ret(idxA - 1, idxB), // insertion
															ret(idxA  , idxB - 1)), // deletion   
												ret(idxA - 1, idxB - 1));    // match
			}
		}

		return ret;
	}

}

#endif	//	HEADER_GUARD___DTW_H__
