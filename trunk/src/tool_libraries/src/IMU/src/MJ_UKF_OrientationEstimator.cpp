#include <IMU/OrientationTestingFramework/MJ_UKF_OrientationEstimator.h>
#include <Eigen/Eigen>

using namespace IMU;

MJ_UKF_OrientationEstimator::~MJ_UKF_OrientationEstimator() {}

MJ_UKF_OrientationEstimator::MJ_UKF_OrientationEstimator(const ProcessCovarianceMatrix & processNoise,
	const MeasurementCovarianceMatrix & measurementsNoise)
	: processNoiseMatrix_(processNoise), measurementsNoiseMatrix_(measurementsNoise),
	processCovarianceMatrix_(ProcessCovarianceMatrix::Zero()), dt_(0.01)
{
	state_.angularVelocities = Vec3::Zero();
	state_.orientation = Quat(1.0, 1.0, 1.0, 1.0);
}

MJ_UKF_OrientationEstimator::MJ_UKF_OrientationEstimator(const ProcessCovarianceMatrix & processNoise,
	const MeasurementCovarianceMatrix & measurementsNoise,
	const StateType & initialState) : processNoiseMatrix_(processNoise),
	measurementsNoiseMatrix_(measurementsNoise), processCovarianceMatrix_(ProcessCovarianceMatrix::Zero()),
	state_(initialState), dt_(0.01)
{

}

void MJ_UKF_OrientationEstimator::reset()
{
	state_.angularVelocities = Vec3::Zero();
	state_.orientation = Quat(1.0, 1.0, 1.0, 1.0);
	processCovarianceMatrix_ = ProcessCovarianceMatrix::Zero();
}

void MJ_UKF_OrientationEstimator::estimate(const IMUDataSample & sample, Vec3 & orientation)
{
	//mno¿nik dla punktów sigma
	const static double sigmaMul = std::sqrt(2.0 * StateSize);

	//punkty sigma
	//suma szumu i niepewnoœci procesu
	const ProcessCovarianceMatrix PQ = processCovarianceMatrix_ + processNoiseMatrix_;
	//generujemy punkty sigma ze œredni¹ 0 i kowariancj¹ PQ
	//wyznaczamy pierwiastek macierzy - najprawdopodobniej dekompozycja Choleskiego
	const ProcessCovarianceMatrix sqrtPQ = Eigen::SelfAdjointEigenSolver<ProcessCovarianceMatrix>(PQ).operatorSqrt();
	//wrzucam estymowany stan do punktów sigma
	SigmaPoints sigmaPoints;
	sigmaPoints[0] = state_;	

	//kwaternion orientacji
	Quat stateOrientation(getOrientation(state_));
	// wektor prêdkoœci k¹towych
	Vec3 stateAngularVelocity(getAngularVelocities(state_));
	//generujê pozosta³e
	for(unsigned int i = 0; i < StateSize; ++i){
		//punkty sigma 6cio wymiarowe (orientacja + prêdkoœc k¹towa)
		const auto p1 = sigmaMul * sqrtPQ.col(i);
		const auto p2 = -p1;

		//transformacja do stanu z 6cio wymiarowych punktów sigma
		updateAngularVelocities(sigmaPoints[1 + i], stateAngularVelocity + Vec3(p1[3], p1[4], p1[5]));
		updateOrientation(sigmaPoints[1 + i], stateOrientation * convert(Vec3(p1[0], p1[1], p1[2])));// * orientationChange(getAngularVelocities(sigmaPoints[1 + i]), dt_));

		updateAngularVelocities(sigmaPoints[SigmaPointsSize - 1 - i], stateAngularVelocity + Vec3(p2[3], p2[4], p2[5]));
		updateOrientation(sigmaPoints[SigmaPointsSize - 1 - i], stateOrientation * convert(Vec3(p2[0], p2[1], p2[2])));// * orientationChange(getAngularVelocities(sigmaPoints[SigmaPointsSize - 1 - i]), dt_));
	}

	ProjectedMeasurements projectedMeasurements;
	MeasurementsVector meanMeasurement = MeasurementsVector::Zero();
	Vec3 meanAngularVelocity = Vec3::Zero();

	//projekcja punktów sigma przez model procesu, nowe stany apriori
	//projekcja pomiarów
	for(unsigned int i = 0; i < SigmaPointsSize; ++i){

		//projekcja stanu
		sigmaPoints[i] = processModel(sigmaPoints[i], ProcessCovarianceMatrix::Zero(), dt_);

		auto av = getAngularVelocities(sigmaPoints[i]);
		auto o = getOrientation(sigmaPoints[i]);

		meanAngularVelocity += av;

		//projekcja prêdkoœci k¹towej
		{
			projectedMeasurements[i][0] = av[0];
			projectedMeasurements[i][1] = av[1];
			projectedMeasurements[i][2] = av[2];
		}

		//projekcja przyspieszenia
		{
			Quat acc = o * Quat(0.0, sample.accelerometerSample().x(),
				sample.accelerometerSample().y(),
				sample.accelerometerSample().z()
				) * o.inverse();

			projectedMeasurements[i][3] = acc.x();
			projectedMeasurements[i][4] = acc.y();
			projectedMeasurements[i][5] = acc.z();
		}

		//projekcja magnetometru
		{
			Quat mag = o * Quat(0.0, sample.magnetometerSample().x(),
				sample.magnetometerSample().y(),
				sample.magnetometerSample().z()
				) * o.inverse();

			projectedMeasurements[i][6] = mag.x();
			projectedMeasurements[i][7] = mag.y();
			projectedMeasurements[i][8] = mag.z();
		}

		meanMeasurement += projectedMeasurements[i];
	}

	//estymacja pomiaru apriori - potrzebna do obliczenia innowacji
	meanMeasurement /= SigmaPointsSize;
	//estymacja prêdkoœci k¹towej apriori dla stanu procesu
	meanAngularVelocity /= SigmaPointsSize;

	updateAngularVelocities(state_, meanAngularVelocity);
	//teraz musze estymowaæ œredni¹ orientacjê
	//nie jest to proste - estymuje œredni kwaternion opisuj¹cy orientacjê
	//minimalizuje wartoœæ k¹ta kwaternionów ró¿nicy pomiêdzy œredni¹ a próbkami
	//QTEM?
	
	OrientationErrors orientationErrors;
	{
		Quat orientationMean = stateOrientation;
		Quat adjustmentValue(0.5,0.5,0.5,0.5);
		
		unsigned int i = 0;
		//while(((adjustmentValue.w() < 0.9995 && adjustmentValue.w() > 0.0005) || i < 10) && i++ < 1000){
		while(adjustmentValue.w() < 0.9995 && adjustmentValue.w() > 0.0005 && i++ < 1000){

			Vec3 mean = Vec3::Zero();

			for(unsigned int i = 0; i < SigmaPointsSize; ++i){
				orientationErrors[i] = getOrientation(sigmaPoints[i]) * orientationMean.inverse();

				double angle = 2.0 * std::acos(orientationErrors[i].w());

				if(angle > 0.0001){
					mean += Vec3(orientationErrors[i].x(),
						orientationErrors[i].y(),
						orientationErrors[i].z()
						) * (angle / std::sin(angle));
				}
			}

			mean /= SigmaPointsSize;

			if(mean.norm() > 0.0001){
				adjustmentValue = convert(mean);
				orientationMean *= adjustmentValue;
			}else{
				adjustmentValue = Quat(1.0, 0.0, 0.0, 0.0);
			}
		}

		updateOrientation(state_, orientationMean);
	}
	
	//kowariancja stanu apriori
	ProcessCovarianceMatrix Pk = ProcessCovarianceMatrix::Zero();
	{
		for(unsigned int i = 0; i < SigmaPointsSize; ++i){
			Vec3 diff = getAngularVelocities(sigmaPoints[i]) - stateAngularVelocity;
			VectorStateType s;
			s[0] = orientationErrors[0].x();
			s[1] = orientationErrors[0].y();
			s[2] = orientationErrors[0].z();
			s[3] = diff.x();
			s[4] = diff.y();
			s[5] = diff.z();

			Pk += s * s.transpose();
		}

		Pk /= SigmaPointsSize;
	}

	//kowariancja estymacji pomiarów
	MeasurementCovarianceMatrix Pzz = MeasurementCovarianceMatrix::Zero();
	{
		for(unsigned int i = 0; i < SigmaPointsSize; ++i){

			MeasurementsVector m = projectedMeasurements[i] - meanMeasurement;

			Pzz += m * m.transpose();
		}

		Pzz /= SigmaPointsSize;
	}

	//kowariancja innowacji
	MeasurementCovarianceMatrix Pvv = Pzz + measurementsNoiseMatrix_;

	//crosss corelation
	KalmanGainMatrix Pxz = KalmanGainMatrix::Zero();
	{
		for(unsigned int i = 0; i < SigmaPointsSize; ++i){

			Vec3 diff = getAngularVelocities(sigmaPoints[i]) - stateAngularVelocity;
			VectorStateType s;
			s[0] = orientationErrors[i].x();
			s[1] = orientationErrors[i].y();
			s[2] = orientationErrors[i].z();
			s[3] = diff.x();
			s[4] = diff.y();
			s[5] = diff.z();

			MeasurementsVector m = projectedMeasurements[i] - meanMeasurement;

			Pxz += s * m.transpose();			
		}

		Pxz /= SigmaPointsSize;
	}

	//wzmocnienie kalmana
	KalmanGainMatrix kalmanGain = Pxz * Pvv.inverse();

	//aktualizacja stanu
	VectorStateType s;
	s[0] = state_.orientation.x();
	s[1] = state_.orientation.y();
	s[2] = state_.orientation.z();
	s[3] = state_.angularVelocities.x();
	s[4] = state_.angularVelocities.y();
	s[5] = state_.angularVelocities.z();


	//innowacja pomiarów
	MeasurementsVector vcc;
	vcc[0] = sample.gyroscopeSample().x();
	vcc[1] = sample.gyroscopeSample().y();
	vcc[2] = sample.gyroscopeSample().z();
	vcc[3] = sample.accelerometerSample().x();
	vcc[4] = sample.accelerometerSample().y();
	vcc[5] = sample.accelerometerSample().z();
	vcc[6] = sample.magnetometerSample().x();
	vcc[7] = sample.magnetometerSample().y();
	vcc[8] = sample.magnetometerSample().z();

	vcc -= meanMeasurement;

	s += kalmanGain * vcc;

	orientation.x() = s[0];
	orientation.y() = s[1];
	orientation.z() = s[2];

	updateOrientation(state_, convert(orientation));
	updateAngularVelocities(state_, Vec3(s[3], s[4], s[5]));	

	//orientation.normalize();

	orientation = quatToYawPitchRoll(getOrientation(state_));
}

const std::string MJ_UKF_OrientationEstimator::name() const
{
	return std::string("UKF IMU Orientation estimator");
}

const std::string MJ_UKF_OrientationEstimator::author() const
{
	return std::string("Mateusz Janiak");
}

const IMU::Quat MJ_UKF_OrientationEstimator::orientationChange(const IMU::Vec3 & angularVelocity, const double dt)
{	
	const double l = angularVelocity.norm();
	if(l > 0.0001){
		const double a2 = l * dt / 2.0;
		const Vec3 axis = angularVelocity * (std::sin(a2) / angularVelocity.norm());
		return Quat(std::cos(a2), axis.x(), axis.y(), axis.z());
	}

	return Quat(1.0, 0.0, 0.0, 0.0);
}

const IMU::Quat MJ_UKF_OrientationEstimator::convert(const IMU::Vec3 & vec)
{
	auto n = vec.norm();
	if(n > 0.0001){
		const double a2 = n / 2.0;		
		const Vec3 axis = vec * (std::sin(a2) / n);
		return Quat(std::cos(a2), axis.x(), axis.y(), axis.z());
	}

	return IMU::Quat(1.0, 0.0, 0.0, 0.0);
}

const MJ_UKF_OrientationEstimator::StateType MJ_UKF_OrientationEstimator::processModel(const StateType & previousState,
	const ProcessCovarianceMatrix & processNoiseMatrix, const double dt)
{
	StateType ret;
	updateOrientation(ret, getOrientation(previousState) * convert(
		Vec3(
		processNoiseMatrix(0,0),
		processNoiseMatrix(1,1),
		processNoiseMatrix(2,2)
		)
		) * orientationChange(getAngularVelocities(previousState), dt));

	updateAngularVelocities(ret, getAngularVelocities(previousState) + Vec3(
		processNoiseMatrix(3,3),
		processNoiseMatrix(4,4),
		processNoiseMatrix(5,5)
		));

	return ret;
}

const IMU::Quat MJ_UKF_OrientationEstimator::getOrientation(const StateType & state)
{
	//return Quat(state[3], state[0], state[1], state[2]);
	return state.orientation;
}

const IMU::Vec3 MJ_UKF_OrientationEstimator::getAngularVelocities(const StateType & state)
{
	//return Vec3(state[4], state[5], state[6]);
	return state.angularVelocities;
}

void MJ_UKF_OrientationEstimator::updateOrientation(StateType & state, const IMU::Quat & orientation)
{
	/*
	state[0] = orientation.x();
	state[1] = orientation.y();
	state[2] = orientation.z();
	state[3] = orientation.w();
	*/
	state.orientation = orientation;
}

void MJ_UKF_OrientationEstimator::updateAngularVelocities(StateType & state, const IMU::Vec3 & angularVelocities)
{
	/*
	state[4] = angularVelocities.x();
	state[5] = angularVelocities.y();
	state[6] = angularVelocities.z();
	*/

	state.angularVelocities = angularVelocities;
}

IMU::Vec3 MJ_UKF_OrientationEstimator::quatToYawPitchRoll(const IMU::Quat & quat)
{
	Vec3 ret;

	ret.x() = std::atan2(2.0 * (quat.w() * quat.x() + quat.y() * quat.z()), 1.0 - 2.0 * (std::pow(quat.x(), 2.0) + std::pow(quat.y(), 2.0)));
	ret.y() = std::asin(2.0 * (quat.w() * quat.y() - quat.z() * quat.x()));
	ret.z() = std::atan2(2.0 * (quat.w() * quat.z() + quat.x() * quat.y()), 1.0 - 2.0 * (std::pow(quat.y(), 2.0) + std::pow(quat.z(), 2.0)));

	return ret;
}
