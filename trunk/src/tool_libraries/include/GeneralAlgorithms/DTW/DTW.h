/********************************************************************
    created:  2013/06/15
    created:  15:6:2013   12:16
    filename: DTW.h
    author:   Mateusz Janiak
    
    purpose:  Klasa realizuj¹ca algorytm Dynamic Time Warping w podstawowej
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
		//! Œcie¿ka w macierzy kosztu
		typedef std::vector<std::pair<CostMatrix::Index, CostMatrix::Index>> Path;

	public:
		//! \tparam ItA Iterator pierwszej serii danych
		//! \tparam ItB Iterator drugiej serii danych
		//! \tparam CostMeasure Miara kosztu przejœcia pomiedzy punktami serii danych
		//! \param startA Iterator pocz¹tka pierwszej serii danych
		//! \param endA Iterator koñca pierwszej serii danych
		//! \param startB Iterator pocz¹tka drugiej serii danych
		//! \param endB Iterator koñca drugiej serii danych		
		//! \param distanceMeasure Obiekt wyliczaj¹cy odleg³oœæ pomiêdzy punktami serii danych
		//! \return Wynik DTW - macierz kosztów przejœcia
		template<class ItA, class ItB, class CostMeasure>
		static const CostMatrix dtw( ItA startA, ItA endA, ItB startB, ItB endB,
			const CostMeasure & costMeasure);

		//! \param costMatrix Macierz kosztów dla wyliczonej dtw
		//! \return Najmniejszy koszt przejœcia w macierzy
		static const double minimalCost(const CostMatrix & costMatrix);
		//! \param costMatrix Macierz kosztów dla wyliczonej dtw
		//! \param path Œcie¿ka o najmniejszym koszcie
		//! \return Najmniejszy koszt przejœcia w macierzy
		static const double minimalCost(const CostMatrix & costMatrix,
			const Path & path);

		//! \param costMatrix Macierz kosztów dla wyliczonej dtw
		//! \param[out] path Œcie¿ka o najmniejszym koszcie
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
