#include <GeneralAlgorithms/DTW/DTW.h>

using namespace GeneralAlgorithms;

const double DynamicTimeWarping::minimalCost(const Eigen::MatrixXd & costMatrix)
{
	double ret = 0.0;

	Eigen::MatrixXd::Index i = 1;
	Eigen::MatrixXd::Index j = 1;

	while(i < costMatrix.rows() && j < costMatrix.cols()){

		if(costMatrix(i, j) < costMatrix(i - 1, j)){
			if(costMatrix(i, j) < costMatrix(i, j - 1)){
				ret += costMatrix(i, j);
				i+=1;
				j+=1;
			}else if(costMatrix(i - 1, j) < costMatrix(i, j - 1)){
				ret += costMatrix(i - 1, j);
				j+=1;
			}else{
				ret += costMatrix(i, j - 1);
				i+=1;
			}
		}else if(costMatrix(i - 1, j) < costMatrix(i, j - 1)){
			ret += costMatrix(i - 1, j);
			j+=1;				
		}else {
			ret += costMatrix(i, j - 1);
			i+=1;
		}
	}

	return ret;
}

void DynamicTimeWarping::shortestPath(const CostMatrix & costMatrix,
	Path & path)
{
	CostMatrix::Index i = 1;
	CostMatrix::Index j = 1;

	while(i < costMatrix.rows() && j < costMatrix.cols()){

		if(costMatrix(i, j) < costMatrix(i - 1, j)){
			if(costMatrix(i, j) < costMatrix(i, j - 1)){
				path.push_back(std::pair<CostMatrix::Index, CostMatrix::Index>(i, j));				
				i+=1;
				j+=1;
			}else if(costMatrix(i - 1, j) < costMatrix(i, j - 1)){
				path.push_back(std::pair<CostMatrix::Index, CostMatrix::Index>(i - 1, j));				
				j+=1;
			}else{
				path.push_back(std::pair<CostMatrix::Index, CostMatrix::Index>(i, j - 1));				
				i+=1;
			}
		}else if(costMatrix(i - 1, j) < costMatrix(i, j - 1)){
			path.push_back(std::pair<CostMatrix::Index, CostMatrix::Index>(i - 1, j));			
			j+=1;				
		}else {
			path.push_back(std::pair<CostMatrix::Index, CostMatrix::Index>(i, j - 1));			
			i+=1;
		}
	}	
}

const double DynamicTimeWarping::minimalCost(const CostMatrix & costMatrix,
	const Path & path)
{
	double cost = 0.0;

	for(auto it = path.begin(); it != path.end(); ++it){
		cost += costMatrix((*it).first, (*it).second);
	}

	return cost;
}