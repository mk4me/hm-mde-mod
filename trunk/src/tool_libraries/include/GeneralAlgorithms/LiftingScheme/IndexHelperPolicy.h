/********************************************************************
    created:  2013/06/14
    created:  14:6:2013   10:46
    filename: BorderedLiftingSchemeT.h
    author:   Mateusz Janiak
    
    purpose:  
*********************************************************************/
#ifndef HEADER_GUARD_ALGO__BORDEREDLIFTINGSCHEMET_H__
#define HEADER_GUARD_ALGO__BORDEREDLIFTINGSCHEMET_H__

class IndexResolver
{
public:
	typedef unsigned long int ulint;
	typedef unsigned long int lint;
};

//! Klasa pomocnicza przy generowaniu indeksów próbek
//! Zak³ada okresowoœæ sygna³u - dla próbki mniejszej od 0 generuje próbkê od koñca przedzia³u maj¹c na uwadze d³ugoœæ przedzia³u (modulo)
//! dla próbki wiêkszej od size - 1 generuje próbkê od pocz¹tku przedzia³u maj¹c na uwadze d³ugoœæ przedzia³u (modulo)
class PeriodicIndexResolver : public IndexResolver
{
public:
    inline static unsigned int indexUnderflow(const lint idx, const ulint size)
    {
        BOOST_ASSERT(size > 0);
        BOOST_ASSERT(idx < 0);
        return size - std::abs(idx) % size;
    }

    inline static unsigned int indexOverflow(const lint idx, const ulint size)
    {
        BOOST_ASSERT(size > 0);
        BOOST_ASSERT(idx >= size);
        return idx % size;
    }
};

//! Klasa pomocnicza przy generowaniu indeksów próbek
//! Dla indeksów mniejszych od 0 zwraca 0 a dla wiêkszych lub równych size zwraca size - 1
class BorderIndexResolver : public IndexResolver
{
public:
    inline static unsigned int indexUnderflow(const lint idx, const ulint size)
    {
        BOOST_ASSERT(size > 0);
        BOOST_ASSERT(idx < 0);
        return 0;
    }

    inline static unsigned int indexOverflow(const lint idx, const ulint size)
    {
        BOOST_ASSERT(size > 0);
        BOOST_ASSERT(idx >= size);
        return size - 1;
    }
};

template<class IndexResolver = PeriodicIndexResolver>
class LiftingSchemeIndexHelper
{
private:
	typedef IndexResolver IndexResolverType;

public:

	inline static IndexResolver::ulint index(const IndexResolver::lint idx,
		const IndexResolver::ulint size)
	{
		IndexResolver::ulint ret = 0;
		if(idx < 0){
			ret = IndexResolverType::indexUnderflow(idx, size);
		}else if(idx >= size){
			ret = IndexResolverType::indexOverflow(idx, size);
		}

		BOOST_ASSERT((ret >= 0 && ret < size), "B³edny indeks");

		return ret;
	}
};

template <class T, class IndexResolver = PeriodicIndexResolver>
class LiftingTransformWithPreUpdate : public LiftingTransformWithIndexHelper<T, IndexResolver>
{
public:
    virtual void forwardStep( Data& vec, const uint N )
    {
        BOOST_ASSERT((isPowerOfTwo(N) && N <= vec.size()));
        split( vec, N );
        preUpdate(vec, N, Forward);
        predict( vec, N, Forward );
        update( vec, N, Forward );
    } // forwardStep

protected:

    virtual void preUpdate(Data& vec, const uint N, const TransDirection direction) = 0;
};


template <class T, class Interpolator, class IndexResolver = PeriodicIndexResolver>
class LiftTrans : public LiftingTransformWithIndexHelper<T, IndexResolver>, public Interpolator {

protected:
	virtual void predict( Data& vec, const uint N, const TransDirection direction ){
		Interpolator::interpolate(vec, N, direction);
	}

	virtual void update( Data& vec, const uint N, const TransDirection direction ){
		Interpolator::update(vec, N, direction);
	}

};

#endif	//	HEADER_GUARD_ALGO__BORDEREDLIFTINGSCHEMET_H__
