#ifndef _QUATINTERPOLATORS_H_
#define _QUATINTERPOLATORS_H_

#include "DualQuatUtils.h"
#include "liftbase.h"
#include <boost/assert.hpp>

class QuatLinHaarInterpolator
{
public:
	 void interpolate(LiftQuat::Data & vec, const LiftQuat::size_type N, const LiftQuat::TransDirection direction){
		uint half = N >> 1;

		for(uint i = 0; i < half; i++){
			uint j = half + i;
			if(direction == LiftQuat::Forward){
				vec[j] -= vec[i];
			}else if(direction == LiftQuat::Inverse){
				vec[j] += vec[i];
			}

			vec[j] = osg::DualQuatUtils::normalize(vec[j]);
		}
	}

	 void update(LiftQuat::Data & vec, const LiftQuat::size_type N, const LiftQuat::TransDirection direction){
		uint half = N >> 1;

		for(uint i = 0; i < half; i++){
			uint j = half + i;
            osg::DualQuat updateValue(vec[j]/2);
			if(direction == LiftQuat::Forward){
				vec[i] += updateValue;
			}else if(direction == LiftQuat::Inverse){
				vec[i] -= updateValue;
			}

			vec[i] = osg::DualQuatUtils::normalize(vec[i]);
		}
	}
};

class QuatLinHaarInterpolatorSlerpAverage
{
public:
     void interpolate(LiftQuat::Data & vec, const LiftQuat::size_type N, const LiftQuat::TransDirection direction){
        uint half = N >> 1;

        for(uint i = 0; i < half; i++){
            uint j = half + i;
            if(direction == LiftQuat::Forward){
                vec[j] -= vec[i];
            }else if(direction == LiftQuat::Inverse){
                vec[j] += vec[i];
            }

            vec[j] = osg::DualQuatUtils::normalize(vec[j]);
        }
    }

     void update(LiftQuat::Data & vec, const LiftQuat::size_type N, const LiftQuat::TransDirection direction){
        uint half = N >> 1;

        for(uint i = 0; i < half; i++){
            uint j = half + i;
            if(direction == LiftQuat::Forward){
                osg::DualQuat updateValue(vec[j] + vec[i]);
                vec[i] = osg::DualQuatUtils::slerp(vec[i], updateValue, 0.5);
            }else if(direction == LiftQuat::Inverse){
                osg::DualQuat updateValue(vec[j] - vec[i]);
                vec[i] = osg::DualQuatUtils::pow(osg::DualQuatUtils::normalize(updateValue * vec[i]), 2.0);
            }

            vec[i] = osg::DualQuatUtils::normalize(vec[i]);
        }
    }
};

class QuatHaarInterpolator
{
public:
	 void interpolate(LiftQuat::Data & vec, const LiftQuat::size_type N, const LiftQuat::TransDirection direction){
		uint half = N >> 1;

		for(uint i = 0; i < half; i++){
			uint j = half + i;
			if(direction == LiftQuat::Forward){
				vec[j] /= vec[i];
			}else if(direction == LiftQuat::Inverse){
				vec[j] *= vec[i];
			}

			vec[j] = osg::DualQuatUtils::normalize(vec[j]);
		}
	}

	 void update(LiftQuat::Data & vec, const LiftQuat::size_type N, const LiftQuat::TransDirection direction){
		uint half = N >> 1;

		for(uint i = 0; i < half; i++){
			uint j = half + i;
			if(direction == LiftQuat::Forward){
				vec[i] = osg::DualQuatUtils::pow(osg::DualQuatUtils::normalize(vec[j]), 0.5) * vec[i];
			}else if(direction == LiftQuat::Inverse){
				vec[i] = osg::DualQuatUtils::pow(osg::DualQuatUtils::normalize(vec[j]), 0.5).inverse() * vec[i];
			}

			vec[i] = osg::DualQuatUtils::normalize(vec[i]);
		}
	}
};

class QuatLerpInterpolator
{
public:
     void interpolate(LiftQuat::Data & vec, const LiftQuat::size_type N, const LiftQuat::TransDirection direction){
        uint half = N >> 1;

        for(uint i = 0; i < half; i++){
            uint j = half + i;
            osg::DualQuat predValue((vec[i] + vec[(i+1)%half])/2);
            if(direction == LiftQuat::Forward){
                vec[j] -= predValue;
            }else if(direction == LiftQuat::Inverse){
                vec[j] += predValue;
            }

            vec[j] = osg::DualQuatUtils::normalize(vec[j]);
        }
    }

     void update(LiftQuat::Data & vec, const LiftQuat::size_type N, const LiftQuat::TransDirection direction){
        uint half = N >> 1;

        for(uint i = 0; i < half; i++){
            uint j = half + i;
            osg::DualQuat updateValue((vec[j] + vec[half + ((j-1) % half)])/4);
            if(direction == LiftQuat::Forward){
                vec[i] += updateValue;
            }else if(direction == LiftQuat::Inverse){
                vec[i] -= updateValue;
            }

            vec[i] = osg::DualQuatUtils::normalize(vec[i]);
        }
    }
};

class QuatLerpInterpolatorSlerpAvarage
{
public:
     void interpolate(LiftQuat::Data & vec, const LiftQuat::size_type N, const LiftQuat::TransDirection direction){
        uint half = N >> 1;

        for(uint i = 0; i < half - 1; i++){
            uint j = half + i;
            osg::DualQuat predValue((vec[i] + vec[(i+1)%half])/2);
            if(direction == LiftQuat::Forward){
                vec[j] -= predValue;
            }else if(direction == LiftQuat::Inverse){
                vec[j] += predValue;
            }

            vec[j] = osg::DualQuatUtils::normalize(vec[j]);
        }
    }

     void update(LiftQuat::Data & vec, const LiftQuat::size_type N, const LiftQuat::TransDirection direction){
        uint half = N >> 1;

        for(uint i = 0; i < half; i++){
            uint j = half + i;
            osg::DualQuat updateValue(vec[j] + (vec[i] + vec[(half + i-1)%half]) / 2.0);
            if(direction == LiftQuat::Forward){ 
                vec[i] = osg::DualQuatUtils::pow(osg::DualQuatUtils::normalize(updateValue * vec[i]), 0.5);
            }else if(direction == LiftQuat::Inverse){
  //              BOOST_ASSERT((false));
            }

            vec[i] = osg::DualQuatUtils::normalize(vec[i]);
        }
    }
};

class QuatLerpQuatInterpolator
{
public:
     void interpolate(LiftQuat::Data & vec, const LiftQuat::size_type N, const LiftQuat::TransDirection direction){
        uint half = N >> 1;
        
        for(uint i = 0; i < half; i++){
            uint j = half + i;
            osg::DualQuat predValue(osg::DualQuatUtils::pow(osg::DualQuatUtils::normalize(vec[(i + 1)%half] * vec[i]), 0.5));
            if(direction == LiftQuat::Forward){
                vec[j] /= predValue;
            }else if(direction == LiftQuat::Inverse){
                vec[j] *= predValue;
            }

            vec[j] = osg::DualQuatUtils::normalize(vec[j]);
        }
    }

     void update(LiftQuat::Data & vec, const LiftQuat::size_type N, const LiftQuat::TransDirection direction){
        uint half = N >> 1;

        for(uint i = 0; i < half; i++){
            uint j = half + i;
            osg::DualQuat updateVal(osg::DualQuatUtils::pow(osg::DualQuatUtils::normalize(vec[j] * vec[half + ((j-1) % half)]), 0.25));
            if(direction == LiftQuat::Forward){
                vec[i] = osg::DualQuatUtils::pow(updateVal * vec[i], 0.5);
            }else if(direction == LiftQuat::Inverse){
                vec[i] = osg::DualQuatUtils::pow(vec[i], 2.0) / updateVal;
            }

            vec[i] = osg::DualQuatUtils::normalize(vec[i]);
        }
    }
};

class QuatSlerpInterpolator
{
public:
     void interpolate(LiftQuat::Data & vec, const LiftQuat::size_type N, const LiftQuat::TransDirection direction){
        uint half = N >> 1;

        for(uint i = 0; i < half; i++){
            uint j = half + i;
            osg::DualQuat predValue;
            predValue.slerp(0.5, vec[i], vec[(i+1)%half]);
            if(direction == LiftQuat::Forward){
                vec[j] = predValue.inverse() * vec[j];
            }else if(direction == LiftQuat::Inverse){
                vec[j] = predValue * vec[j];
            }

            vec[j] = osg::DualQuatUtils::normalize(vec[j]);
        }
    }

     void update(LiftQuat::Data & vec, const LiftQuat::size_type N, const LiftQuat::TransDirection direction){
        uint half = N >> 1;

        for(uint i = 0; i < half; i++){
            uint j = half + i;
            osg::DualQuat updateVal;
            updateVal.slerp(0.5, vec[half + ((j-1) % half)], vec[j]);
            updateVal = osg::DualQuatUtils::normalize(updateVal);
            if(direction == LiftQuat::Forward){
                vec[i] = vec[i] * osg::DualQuatUtils::pow(updateVal, 0.5);
            }else if(direction == LiftQuat::Inverse){
                vec[i] = vec[i] * osg::DualQuatUtils::pow(updateVal, -0.5);
            }

            vec[i] = osg::DualQuatUtils::normalize(vec[i]);
        }
    }
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


class QuatQuadInterpolator
{
public:
	 void interpolate(LiftQuat::Data & vec, const LiftQuat::size_type N, const LiftQuat::TransDirection direction){
		uint half = N >> 1;
		
		vec[half] /= osg::DualQuatUtils::squad(vec[0], vec[0], osg::DualQuatUtils::spline(vec[0],vec[0],vec[1]), 
			osg::DualQuatUtils::spline(vec[0],vec[1],vec[2]), 0.5);
		
		for(uint i = 1; i < half - 2; i++){
			uint j = half + i;
			osg::DualQuat predValue(osg::DualQuatUtils::squad(vec[i-1], vec[i], osg::DualQuatUtils::spline(vec[i-1],vec[i],vec[i+1]), 
				osg::DualQuatUtils::spline(vec[i],vec[i+1],vec[i+2]), 0.5));
			if(direction == LiftQuat::Forward){
				vec[j] /= predValue;
			}else if(direction == LiftQuat::Inverse){
				vec[j] *= predValue;
			}
		}

		osg::DualQuat predValueA(osg::DualQuatUtils::squad(vec[N-2], vec[N-1], osg::DualQuatUtils::spline(vec[N-3],vec[N-2],vec[N-1]), 
			osg::DualQuatUtils::spline(vec[N-2],vec[N-1],vec[N-1]), 0.5));

		osg::DualQuat predValueB(osg::DualQuatUtils::squad(vec[N-1], vec[N-1], osg::DualQuatUtils::spline(vec[N-2],vec[N-1],vec[N-1]), 
			osg::DualQuatUtils::spline(vec[N-1],vec[N-1],vec[N-1]), 0.5));

		if(direction == LiftQuat::Forward){
			vec[N-2] /= predValueA;
			vec[N-1] /= predValueB;
		}else if(direction == LiftQuat::Inverse){
			vec[N-2] *= predValueA;
			vec[N-1] *= predValueB;
		}

	}

	 void update(LiftQuat::Data & vec, const LiftQuat::size_type N, const LiftQuat::TransDirection direction){

	}
};


class QuatBezierInterpolator
{
public:
	 void interpolate(LiftQuat::Data & vec, const LiftQuat::size_type N, const LiftQuat::TransDirection direction){
		uint half = N >> 1;
		
		vec[half] /= osg::DualQuatUtils::bezier(vec[0], vec[0], osg::DualQuatUtils::spline(vec[0],vec[0],vec[1]), 
			osg::DualQuatUtils::spline(vec[0],vec[1],vec[2]), 0.5);

		for(uint i = 1; i < half - 2; i++){
			uint j = half + i;
			osg::DualQuat predValue(osg::DualQuatUtils::bezier(vec[i-1], vec[i], osg::DualQuatUtils::spline(vec[i-1],vec[i],vec[i+1]), 
				osg::DualQuatUtils::spline(vec[i],vec[i+1],vec[i+2]), 0.5));
			if(direction == LiftQuat::Forward){
				vec[j] /= predValue;
			}else if(direction == LiftQuat::Inverse){
				vec[j] *= predValue;
			}
		}

		osg::DualQuat predValueA(osg::DualQuatUtils::bezier(vec[N-2], vec[N-1], osg::DualQuatUtils::spline(vec[N-3],vec[N-2],vec[N-1]), 
			osg::DualQuatUtils::spline(vec[N-2],vec[N-1],vec[N-1]), 0.5));

		osg::DualQuat predValueB(osg::DualQuatUtils::bezier(vec[N-1], vec[N-1], osg::DualQuatUtils::spline(vec[N-2],vec[N-1],vec[N-1]), 
			osg::DualQuatUtils::spline(vec[N-1],vec[N-1],vec[N-1]), 0.5));

		if(direction == LiftQuat::Forward){
			vec[N-2] /= predValueA;
			vec[N-1] /= predValueB;
		}else if(direction == LiftQuat::Inverse){
			vec[N-2] *= predValueA;
			vec[N-1] *= predValueB;
		}
	}

	 void update(LiftQuat::Data & vec, const LiftQuat::size_type N, const LiftQuat::TransDirection direction){

	}
};

#endif

