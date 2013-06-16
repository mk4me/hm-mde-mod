#include <GeneralAlgorithms/CombinationGenerator/CombinationGenerator.h>
#include <stdexcept>

using namespace GeneralAlgorithms;

CombinationGenerator::CombinationGenerator(unsigned int r, unsigned int n) : n(n), r(r), nMinusR(n-r), rMinusOne(r -1), total(getFactorial(n) / (getFactorial(r) * getFactorial(nMinusR)))  {
	if (r > n) {
		throw std::runtime_error("Invalid combination arguments");
	}
	if (n < 1) {
		throw std::runtime_error("Invalid combination arguments");
	}

	a.reserve(r);
	a.resize(r);
	reset ();
}
//------
// Reset
//------
void CombinationGenerator::reset () {
	for (unsigned int i = 0; i < r; i++) {
		a[i] = i;
	}
	numLeft = total;
}
//------------------------------------------------
// Return number of combinations not yet generated
//------------------------------------------------
const unsigned int CombinationGenerator::getNumLeft() const {
	return numLeft;
}
//-----------------------------
// Are there more combinations?
//-----------------------------
const bool CombinationGenerator::hasMore() const {
	return numLeft != 0;
}

//--------------------------------------------------------
// Generate next combination (algorithm from Rosen p. 286)
//--------------------------------------------------------
const CombinationGenerator::Combination & CombinationGenerator::getNext () {
	if (numLeft == total) {
		--numLeft;
		return a;
	}
	unsigned int i = rMinusOne;
	while (a[i] == nMinusR + i) {
		i--;
	}
	a[i] = a[i] + 1;
	for (unsigned int j = i + 1; j < r; j++) {
		a[j] = a[i] + j - i;
	}
	--numLeft;
	return a;
}

const unsigned int CombinationGenerator::getFactorial(unsigned int n) {
	unsigned int fact = 1;
	for (unsigned int i = n; i > 1; i--) {
		fact *= i;
	}
	return fact;
}