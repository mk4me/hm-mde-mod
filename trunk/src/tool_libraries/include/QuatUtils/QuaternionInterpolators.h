#ifndef _QUATINTERPOLATORS_H_
#define _QUATINTERPOLATORS_H_

#include <GeneralAlgorithms/LiftingScheme/LiftingSchemeT.h>
#include <QuatUtils/QuatUtils.h>
#include <osg/Vec3>
#include <boost/assert.hpp>

namespace QuatUtils
{

typedef osg::Quat Quat;

typedef LiftingScheme::LiftingSchemeT<Quat> QuatLiftingScheme;
typedef LiftingScheme::LiftingSchemeT<osg::Vec3> Vec3LiftingScheme;

typedef LiftingScheme::IndexResolver::ulint uint;

template<class IndexHelper>
class QuatLinHaarInterpolator
{
public:
	
	static void interpolate(QuatLiftingScheme::Data & vec, const QuatLiftingScheme::size_type N,
		 const LiftingScheme::LiftingSchemeUtils::TransDirection direction){
		
		const uint half = N >> 1;

		for(uint i = 0; i < half; i++){
			const uint j = half + i;
			if(direction == LiftingScheme::LiftingSchemeUtils::TransDirection::Forward){
				vec[j] -= vec[i];
			}else if(direction == LiftingScheme::LiftingSchemeUtils::TransDirection::Inverse){
				vec[j] += vec[i];
			}

			vec[j] = osg::QuatUtils::normalize(vec[j]);
		}
	}

	static void update(QuatLiftingScheme::Data & vec, const QuatLiftingScheme::size_type N,
		 const LiftingScheme::LiftingSchemeUtils::TransDirection direction){
		
		const uint half = N >> 1;

		for(uint i = 0; i < half; i++){
			const uint j = half + i;
            const Quat updateValue(vec[j]/2);
			if(direction == LiftingScheme::LiftingSchemeUtils::Forward){
				vec[i] += updateValue;
			}else if(direction == LiftingScheme::LiftingSchemeUtils::Inverse){
				vec[i] -= updateValue;
			}

			vec[i] = osg::QuatUtils::normalize(vec[i]);
		}
	}
};

template<class IndexHelper>
class QuatHaarInterpolator
{
public:
	static  void interpolate(QuatLiftingScheme::Data & vec, const QuatLiftingScheme::size_type N,
		const LiftingScheme::LiftingSchemeUtils::TransDirection direction){
		
		const uint half = N >> 1;

		for(uint i = 0; i < half; i++){
			const uint j = half + i;
			if(direction == LiftingScheme::LiftingSchemeUtils::Forward){
				vec[j] /= vec[i];
			}else if(direction == LiftingScheme::LiftingSchemeUtils::Inverse){
				vec[j] *= vec[i];
			}

			//vec[j] = osg::QuatUtils::normalize(vec[j]);
		}
	}

	static void update(QuatLiftingScheme::Data & vec, const QuatLiftingScheme::size_type N,
		const LiftingScheme::LiftingSchemeUtils::TransDirection direction){
		
		const uint half = N >> 1;

		for(uint i = 0; i < half; i++){
			const uint j = half + i;
			if(direction == LiftingScheme::LiftingSchemeUtils::Forward){
				//vec[i] = osg::QuatUtils::pow(osg::QuatUtils::normalize(vec[j]), 0.5) * vec[i];
				vec[i] = osg::QuatUtils::pow(vec[j], 0.5) * vec[i];
			}else if(direction == LiftingScheme::LiftingSchemeUtils::Inverse){
				//vec[i] = osg::QuatUtils::pow(osg::QuatUtils::normalize(vec[j]), 0.5).inverse() * vec[i];
				vec[i] = osg::QuatUtils::pow(vec[j], 0.5).inverse() * vec[i];
			}

			//vec[i] = osg::QuatUtils::normalize(vec[i]);
		}
	}
};

template<class IndexHelper>
class QuatLerpInterpolator
{
public:
	static void interpolate(QuatLiftingScheme::Data & vec, const QuatLiftingScheme::size_type N,
		const LiftingScheme::LiftingSchemeUtils::TransDirection direction){
        
		const uint half = N >> 1;

        for(uint i = 0; i < half; i++){
            const uint j = half + i;
            const osg::Quat predValue((vec[i] + vec[IndexHelper::index(i+1,half)])/2);
            if(direction == LiftingScheme::LiftingSchemeUtils::Forward){
                vec[j] -= predValue;
            }else if(direction == LiftingScheme::LiftingSchemeUtils::Inverse){
                vec[j] += predValue;
            }

            vec[j] = osg::QuatUtils::normalize(vec[j]);
        }
    }

	static void update(QuatLiftingScheme::Data & vec, const QuatLiftingScheme::size_type N,
		const LiftingScheme::LiftingSchemeUtils::TransDirection direction){
        
		const uint half = N >> 1;
		const uint lessHalf = half - 1;

        for(uint i = 0; i < half; i++){
            const uint j = half + i;
            const osg::Quat updateValue((vec[j] + vec[IndexHelper::index(lessHalf + i, N, half)])/4);
            if(direction == LiftingScheme::LiftingSchemeUtils::Forward){
                vec[i] += updateValue;
            }else if(direction == LiftingScheme::LiftingSchemeUtils::Inverse){
                vec[i] -= updateValue;
            }

            vec[i] = osg::QuatUtils::normalize(vec[i]);
        }
    }
};

template<class IndexHelper>
class QuatSlerpInterpolator
{
public:
	static void interpolate(QuatLiftingScheme::Data & vec, const QuatLiftingScheme::size_type N,
		const LiftingScheme::LiftingSchemeUtils::TransDirection direction){
        
		const uint half = N >> 1;

        for(uint i = 0; i < half; i++){
            const uint j = half + i;
            osg::Quat predValue;
            predValue.slerp(0.5, vec[i], vec[IndexHelper::index(i+1,half)]);
            if(direction == LiftingScheme::LiftingSchemeUtils::Forward){
                vec[j] = predValue.inverse() * vec[j];
            }else if(direction == LiftingScheme::LiftingSchemeUtils::Inverse){
                vec[j] = predValue * vec[j];
            }

            //vec[j] = osg::QuatUtils::normalize(vec[j]);
        }
    }

	static void update(QuatLiftingScheme::Data & vec, const QuatLiftingScheme::size_type N,
		const LiftingScheme::LiftingSchemeUtils::TransDirection direction){
        
		const uint half = N >> 1;
		const uint lessHalf = half - 1;

        for(uint i = 0; i < half; i++){
            const uint j = half + i;
            osg::Quat updateVal;
            updateVal.slerp(0.5, vec[IndexHelper::index(lessHalf + i, N, half)], vec[j]);
            //updateVal = osg::QuatUtils::normalize(updateVal);
            if(direction == LiftingScheme::LiftingSchemeUtils::Forward){
                vec[i] = vec[i] * osg::QuatUtils::pow(updateVal, 0.5);
            }else if(direction == LiftingScheme::LiftingSchemeUtils::Inverse){
                vec[i] = vec[i] * osg::QuatUtils::pow(updateVal, -0.5);
            }

            //vec[i] = osg::QuatUtils::normalize(vec[i]);
        }
    }
};

template<class IndexHelper>
class TangentSpaceQuatInterpolator
{
public:
	
	static void interpolate(Vec3LiftingScheme::Data & vec, const Vec3LiftingScheme::size_type N,
		const LiftingScheme::LiftingSchemeUtils::TransDirection direction){
	
		const uint half = N >> 1;

		for (uint i = 0; i < half; i++) {
			const uint j = i + half;
			const osg::Vec3d predictVal = vec[IndexHelper::index(i+1,half)] * 0.5625 + vec[IndexHelper::index(i+2,half)] * 0.5625 - vec[i] * 0.0625 - vec[IndexHelper::index(i+3,half)] * 0.0625;

			if (direction == LiftingScheme::LiftingSchemeUtils::Forward) {
				vec[j] -= predictVal;
			}
			else if (direction == LiftingScheme::LiftingSchemeUtils::Inverse) {
				vec[j] += predictVal;
			}
		}
	}

	static void update(Vec3LiftingScheme::Data & vec, const Vec3LiftingScheme::size_type N,
		const LiftingScheme::LiftingSchemeUtils::TransDirection direction){
	
		const uint half = N >> 1;

		for(uint i = 0; i < half; i++){
			const uint j = i + half;
			const osg::Vec3d updateVal = vec[j] / 2.0;

			if (direction == LiftingScheme::LiftingSchemeUtils::Forward) {
				vec[i] += updateVal;
			}
			else if (direction == LiftingScheme::LiftingSchemeUtils::Inverse) {
				vec[i] -= updateVal;
			}
		}
	}
};

template<class IR = LiftingScheme::PeriodicIndexResolver>
class PseudoQuatTangentLiftingScheme : public QuatLiftingScheme
{
private:

	typedef LiftingScheme::InterpolatorLiftingSchemeT<osg::Vec3,
		TangentSpaceQuatInterpolator, IR> TangentSpaceLiftingScheme;

protected:

	static void quatToTangentSpace(const Data & src, Vec3LiftingScheme::Data & dest)
	{
		for(auto it = src.begin(); it != src.end(); it++){
			dest.push_back(osg::QuatUtils::log(*it).asVec3());
		}
	}

	static void tangentSpaceToQuat(const Vec3LiftingScheme::Data & src, Data & dest)
	{
		for(int i = 0; i < src.size(); i++){
			dest[i] = osg::QuatUtils::exp(osg::Quat(src[i].x(), src[i].y(), src[i].z(), 0));
		}
	}

	virtual void predict( Data& vec, const size_type N,
		const LiftingScheme::LiftingSchemeUtils::TransDirection direction )
	{

	}

	virtual void update( Data& vec, const size_type N,
		const LiftingScheme::LiftingSchemeUtils::TransDirection direction )
	{

	}

public:

	virtual void forwardTrans( Data& vec, const uint N )
	{
		
		TangentSpaceLiftingScheme tangentLifting;
		TangentSpaceLiftingScheme::Data data;

		quatToTangentSpace(vec, data);

		tangentLifting.forwardTrans(data, N);

		tangentSpaceToQuat(data, vec);
	}

	virtual void inverseTrans( Data& vec, const uint N )
	{
		TangentSpaceLiftingScheme tangentLifting;
		TangentSpaceLiftingScheme::Data data;

		quatToTangentSpace(vec, data);

		tangentLifting.inverseTrans(data, N);

		tangentSpaceToQuat(data, vec);
	}


};


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template<class IndexHelper>
class QuatQuadInterpolator
{
public:
	void interpolate(QuatLiftingScheme::Data & vec, const QuatLiftingScheme::size_type N,
		const LiftingScheme::LiftingSchemeUtils::TransDirection direction){
		uint half = N >> 1;
		
		vec[half] /= osg::QuatUtils::squad(vec[0], vec[0], osg::QuatUtils::spline(vec[0],vec[0],vec[1]), 
			osg::QuatUtils::spline(vec[0],vec[1],vec[2]), 0.5);
		
		for(uint i = 1; i < half - 2; i++){
			uint j = half + i;
			osg::Quat predValue(osg::QuatUtils::squad(vec[i-1], vec[i], osg::QuatUtils::spline(vec[i-1],vec[i],vec[i+1]), 
				osg::QuatUtils::spline(vec[i],vec[i+1],vec[i+2]), 0.5));
			if(direction == LiftQuat::Forward){
				vec[j] /= predValue;
			}else if(direction == LiftQuat::Inverse){
				vec[j] *= predValue;
			}
		}

		osg::Quat predValueA(osg::QuatUtils::squad(vec[N-2], vec[N-1], osg::QuatUtils::spline(vec[N-3],vec[N-2],vec[N-1]), 
			osg::QuatUtils::spline(vec[N-2],vec[N-1],vec[N-1]), 0.5));

		osg::Quat predValueB(osg::QuatUtils::squad(vec[N-1], vec[N-1], osg::QuatUtils::spline(vec[N-2],vec[N-1],vec[N-1]), 
			osg::QuatUtils::spline(vec[N-1],vec[N-1],vec[N-1]), 0.5));

		if(direction == LiftQuat::Forward){
			vec[N-2] /= predValueA;
			vec[N-1] /= predValueB;
		}else if(direction == LiftQuat::Inverse){
			vec[N-2] *= predValueA;
			vec[N-1] *= predValueB;
		}

	}

	void update(QuatLiftingScheme::Data & vec, const QuatLiftingScheme::size_type N,
		const LiftingScheme::LiftingSchemeUtils::TransDirection direction){

	}
};

template<class IndexHelper>
class QuatBezierInterpolator
{
public:
	void interpolate(QuatLiftingScheme::Data & vec, const QuatLiftingScheme::size_type N,
		const LiftingScheme::LiftingSchemeUtils::TransDirection direction){
		uint half = N >> 1;
		
		vec[half] /= osg::QuatUtils::bezier(vec[0], vec[0], osg::QuatUtils::spline(vec[0],vec[0],vec[1]), 
			osg::QuatUtils::spline(vec[0],vec[1],vec[2]), 0.5);

		for(uint i = 1; i < half - 2; i++){
			uint j = half + i;
			osg::Quat predValue(osg::QuatUtils::bezier(vec[i-1], vec[i], osg::QuatUtils::spline(vec[i-1],vec[i],vec[i+1]), 
				osg::QuatUtils::spline(vec[i],vec[i+1],vec[i+2]), 0.5));
			if(direction == LiftQuat::Forward){
				vec[j] /= predValue;
			}else if(direction == LiftQuat::Inverse){
				vec[j] *= predValue;
			}
		}

		osg::Quat predValueA(osg::QuatUtils::bezier(vec[N-2], vec[N-1], osg::QuatUtils::spline(vec[N-3],vec[N-2],vec[N-1]), 
			osg::QuatUtils::spline(vec[N-2],vec[N-1],vec[N-1]), 0.5));

		osg::Quat predValueB(osg::QuatUtils::bezier(vec[N-1], vec[N-1], osg::QuatUtils::spline(vec[N-2],vec[N-1],vec[N-1]), 
			osg::QuatUtils::spline(vec[N-1],vec[N-1],vec[N-1]), 0.5));

		if(direction == LiftQuat::Forward){
			vec[N-2] /= predValueA;
			vec[N-1] /= predValueB;
		}else if(direction == LiftQuat::Inverse){
			vec[N-2] *= predValueA;
			vec[N-1] *= predValueB;
		}
	}

	void update(QuatLiftingScheme::Data & vec, const QuatLiftingScheme::size_type N,
		const LiftingScheme::LiftingSchemeUtils::TransDirection direction){

	}
};

}

#endif

