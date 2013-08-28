#include "QuatCompression.h"
#include <corelib/IJob.h>
#include <corelib/IJobManager.h>
#include <threading/SynchronizationPolicies.h>
#include <corelib/PluginCommon.h>
#include <boost/lexical_cast.hpp>
#include <boost/random.hpp>
#include <QuatUtils/QuatUtils.h>
#include <QuatUtils/QuaternionInterpolators.h>
#include "QuatCommon.h"
#include <boost/bimap.hpp>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <corelib/Filesystem.h>

class CompoundJob
{

private:

	class InnerJob : public core::IRunnable
	{
	public:
		InnerJob(CompoundJob * cj, core::IRunnablePtr job) : cj(cj), job(job)
		{

		}

		virtual ~InnerJob() {}

		virtual void run()
		{
			job->run();
			cj->finalizeJob();
		}

	private:
		CompoundJob * cj;
		core::IRunnablePtr job;
	};

	friend class InnerJob;

public:
	CompoundJob(core::IJobManager * jm) : jm(jm), counter(0)
	{

	}

	~CompoundJob()
	{
		counter = jobs.size();
		if(counter > 0){

			wait.lock();

			auto jm = plugin::getJobManager();

			for(auto it = jobs.begin(); it != jobs.end(); ++it){
				jm->addJob("MotionAnalysisProcessor", "Tests", core::IRunnablePtr(new InnerJob(this, *it)));
			}

			jobs.swap(std::list<utils::IRunnablePtr>());

			utils::ScopedLock<utils::StrictSyncPolicy> lock(wait);
		}
	}

	void addJob(core::IRunnablePtr job)
	{
		jobs.push_back(job);
	}

private:

	void finalizeJob()
	{
		utils::ScopedLock<utils::StrictSyncPolicy> lock(synch);
		--counter;
		if(counter == 0){
			wait.unlock();
		}
	}

private:
	std::list<utils::IRunnablePtr> jobs;
	utils::StrictSyncPolicy wait;
	utils::StrictSyncPolicy synch;
	unsigned int counter;
	core::IJobManager * jm;
};


VectorChannelPtr createNoisyChannel(VectorChannelReaderInterfaceConstPtr input, const double noise)
{
	boost::random::mt19937 rng;
	boost::random::uniform_real_distribution<double> eulerNoiseGenerator( -noise, noise);

	//kopiuje go - przygotowuje sobie porcje danych wyjsciowych
	VectorChannelPtr retData(new VectorChannel(input->getSamplesPerSecond()));
	retData->setName("Noisy channel " + input->getName() + ": " + boost::lexical_cast<std::string>(noise));
	retData->setTimeBaseUnit(input->getTimeBaseUnit());
	retData->setValueBaseUnit(input->getValueBaseUnit());

	//modyfikuje dane ktore bêdê zaraz udostepnia³ na wyjœciu	
	for(VectorChannel::size_type i = 0; i < input->size(); ++i){
		//zaszumianie

		auto val = input->value(i) + osg::Vec3(
			eulerNoiseGenerator(rng),
			eulerNoiseGenerator(rng),
			eulerNoiseGenerator(rng));

		val.x() = osg::QuatUtils::clamp<double>(val.x(), -180.0, 180.0);
		val.y() = osg::QuatUtils::clamp<double>(val.y(), -180.0, 180.0);
		val.z() = osg::QuatUtils::clamp<double>(val.z(), -180.0, 180.0);

		retData->addPoint( val );
	}

	return retData;
}

MotionAnalysisTests::MotionAnalysisTests()
{
	inPinA = new VectorInputPin(this);
	outPinA = new ObjectsCollectionOutputPin(this);
	addInputPin(inPinA);
	addOutputPin(outPinA);
}

MotionAnalysisTests::~MotionAnalysisTests()
{

}

typedef boost::bimap<std::string, core::shared_ptr<QuatUtils::QuatLiftingScheme>> LiftingSchemes;

struct TestData
{
	double noise;
	double distance;
	QuatUtils::QuatLiftingScheme::Data quatData;
	VectorChannelReaderInterfaceConstPtr eulerData;
};

typedef boost::array<TestData, 4> QuaternionTestData;
typedef boost::array<QuatUtils::QuatLiftingScheme::Data, 4> QuaternionTestResults;
typedef std::map<core::shared_ptr<QuatUtils::QuatLiftingScheme>, QuaternionTestResults> LiftingResults;

struct ExperimentResult
{
public:
	std::string name;
	core::shared_ptr<QuatUtils::QuatLiftingScheme> ls;
	VectorChannelReaderInterfaceConstPtr eulerInput;
	QuatUtils::QuatLiftingScheme::Data quatReference;
	QuatUtils::QuatLiftingScheme::Data quatInput;
	QuatUtils::QuatLiftingScheme::Data forward;
	QuatUtils::QuatLiftingScheme::Data modifiedForward;
	QuatUtils::QuatLiftingScheme::Data backward;
	VectorChannelPtr convertedOut;
	double distInput;
	double distReference;
	unsigned int size;

public:

	void updateResults()
	{
		ls->inverseTrans(backward, size);
		distInput = QuatCommon::dist(backward, quatInput);
		distReference = QuatCommon::dist(backward, quatReference);
		convertedOut = VectorChannelPtr(new VectorChannel(eulerInput->getSamplesPerSecond()));
		convertedOut->setName(name);
		convertedOut->setTimeBaseUnit(eulerInput->getTimeBaseUnit());
		convertedOut->setValueBaseUnit(eulerInput->getValueBaseUnit());
		convertedOut->setTimeScaleFactor(eulerInput->getTimeScaleFactor());
		convertedOut->setValueScaleFactor(eulerInput->getValueScaleFactor());

		for(auto it = backward.begin(); it != backward.end(); ++it){
			auto res = osg::QuatUtils::quaterionToEuler
				<osg::Quat, osg::Vec3>(*it);

			res.x() = osg::RadiansToDegrees(res.x());
			res.y() = osg::RadiansToDegrees(res.y());
			res.z() = osg::RadiansToDegrees(res.z());

			convertedOut->addPoint(res);
		}

		std::string fileName = name;
		for(unsigned int i = 0; i < fileName.size(); ++i){
			if(fileName[i] == ' '){
				fileName[i] = '_';
			}
		}

		fileName += ".dat";

		if(core::Filesystem::pathExists(plugin::getPluginPath()) == false){
			core::Filesystem::createDirectory(plugin::getPluginPath());
		}

		QuatCommon::exportData((plugin::getPluginPath() / fileName).string(),
			eulerInput, convertedOut);
	}
};

typedef std::list<ExperimentResult> ExperimentsResults;

void jobCreateNoisyChannel(TestData * out, VectorChannelReaderInterfaceConstPtr input,
	const double noise)
{
	out->noise = noise;
	out->eulerData = createNoisyChannel(input, noise);
}

void jobConvertChannel(QuaternionTestData * out, const unsigned int idx)
{
	(*out)[idx].quatData = QuatCommon::convert((*out)[idx].eulerData);
}

void jobComputeForwardTransforms(core::shared_ptr<QuatUtils::QuatLiftingScheme> ls,
	LiftingResults * out, const QuaternionTestData * input, const unsigned int size)
{
	static utils::StrictSyncPolicy synch;

	QuaternionTestResults forwardRes;

	for(unsigned int i = 0; i < input->size(); ++i){
		forwardRes[i] = (*input)[i].quatData;
		ls->forwardTrans(forwardRes[i], size);
	}

	utils::ScopedLock<utils::StrictSyncPolicy> lock(synch);
	out->insert(LiftingResults::value_type(ls, forwardRes));
}

void jobComputeInverseTransform(ExperimentResult * result)
{
	result->backward = result->forward;
	result->updateResults();
}

void jobDenoiseChannel(ExperimentResult * result, const double threshold)
{
	result->modifiedForward = result->forward;
	osg::QuatUtils::denoise(result->modifiedForward.begin(), result->modifiedForward.end(), threshold);
	result->backward = result->modifiedForward;
	result->updateResults();
}

void jobCompressChannel(ExperimentResult * result,
	const QuatUtils::QuatLiftingCompressor::CompressionSettings & settings)
{
	result->modifiedForward = result->forward;
	auto res = QuatUtils::QuatLiftingCompressor::compress(result->modifiedForward, settings);
	result->backward = result->modifiedForward = QuatUtils::QuatLiftingCompressor::decompress(res);
	result->updateResults();
}

void quatToTangentSpace(const QuatUtils::QuatLiftingScheme::Data & src, QuatUtils::Vec3LiftingScheme::Data & dest)
{
	for(int i = 0; i < src.size(); i++){
		dest[i] = osg::QuatUtils::log(src[i]).asVec3();
	}
}

void tangentSpaceToQuat(const QuatUtils::Vec3LiftingScheme::Data & src, QuatUtils::QuatLiftingScheme::Data & dest)
{
	for(int i = 0; i < src.size(); i++){
		dest[i] = osg::QuatUtils::normalize(osg::QuatUtils::exp(osg::Quat(src[i].x(), src[i].y(), src[i].z(), 0)));
	}
}

core::ObjectWrapperPtr createWrapper(const QuatUtils::Vec3LiftingScheme::Data & src,
	VectorChannelReaderInterfaceConstPtr prototype, const std::string & name)
{		
	auto newChannel = VectorChannelPtr(new VectorChannel(prototype->getSamplesPerSecond()));
	newChannel->setName(name);
	newChannel->setTimeBaseUnit(prototype->getTimeBaseUnit());
	newChannel->setValueBaseUnit("");
	newChannel->setTimeScaleFactor(prototype->getTimeScaleFactor());
	newChannel->setValueScaleFactor(prototype->getValueScaleFactor());

	for(auto it = src.begin(); it != src.end(); ++it){
		newChannel->addPoint(*it);
	}

	auto ow = core::ObjectWrapper::create<VectorChannel::Interface>();
	ow->set(core::dynamic_pointer_cast<VectorChannel::Interface>(newChannel));
	(*ow)["core/label"] = name;

	return ow;
}

core::ObjectWrapperPtr createWrapper(const QuatUtils::QuatLiftingScheme::Data & data,
	VectorChannelReaderInterfaceConstPtr prototype, const std::string & name)
{
	auto newChannel = VectorChannelPtr(new VectorChannel(prototype->getSamplesPerSecond()));
	newChannel->setName(name);
	newChannel->setTimeBaseUnit(prototype->getTimeBaseUnit());
	newChannel->setValueBaseUnit(prototype->getValueBaseUnit());
	newChannel->setTimeScaleFactor(prototype->getTimeScaleFactor());
	newChannel->setValueScaleFactor(prototype->getValueScaleFactor());

	for(auto it = data.begin(); it != data.end(); ++it){
		
		auto res = osg::QuatUtils::quaterionToEuler
			<osg::Quat, osg::Vec3>(*it);

		res.x() = osg::RadiansToDegrees(res.x());
		res.y() = osg::RadiansToDegrees(res.y());
		res.z() = osg::RadiansToDegrees(res.z());
		
		newChannel->addPoint(res);
	}

	auto ow = core::ObjectWrapper::create<VectorChannel::Interface>();
	ow->set(core::dynamic_pointer_cast<VectorChannel::Interface>(newChannel));
	(*ow)["core/label"] = name;

	return ow;
}

void MotionAnalysisTests::process()
{
	//oryginalne dane
	auto inputData = inPinA->getValue();
	const auto size = LiftingScheme::LiftingSchemeUtils::floorPowerOfTwo(inputData->size());

	//tworzê dane z szumem
	QuaternionTestData testData;
	testData[0].eulerData = inputData;
	testData[0].noise = 0.0;
	testData[0].noise = 0.0;

	{
		CompoundJob cj(plugin::getJobManager());

		cj.addJob(utils::IRunnablePtr(new utils::FunctorRunnable(boost::bind(&jobCreateNoisyChannel, &(testData[1]), inputData, 0.5))));
		cj.addJob(utils::IRunnablePtr(new utils::FunctorRunnable(boost::bind(&jobCreateNoisyChannel, &(testData[2]), inputData, 2.0))));
		cj.addJob(utils::IRunnablePtr(new utils::FunctorRunnable(boost::bind(&jobCreateNoisyChannel, &(testData[3]), inputData, 5.0))));
	}

	//konwertujê na sygna³ kwaternionowy obs³ugiwany przez schematy liftingu
	{
		CompoundJob cj(plugin::getJobManager());

		cj.addJob(utils::IRunnablePtr(new utils::FunctorRunnable(boost::bind(&jobConvertChannel, &testData, 0))));
		cj.addJob(utils::IRunnablePtr(new utils::FunctorRunnable(boost::bind(&jobConvertChannel, &testData, 1))));
		cj.addJob(utils::IRunnablePtr(new utils::FunctorRunnable(boost::bind(&jobConvertChannel, &testData, 2))));
		cj.addJob(utils::IRunnablePtr(new utils::FunctorRunnable(boost::bind(&jobConvertChannel, &testData, 3))));
	}

	//licze odleglosci zaszumionych od oryginau
	testData[0].distance = 0.0;
	for(unsigned int i = 1; i < 4; ++i){
		testData[i].distance = QuatCommon::dist(testData[i].quatData, testData[0].quatData);
	}

	//tworze mape schematów liftingu i ich opisu do póŸniejszego uruchamiania
	LiftingSchemes liftingSchemes;
	liftingSchemes.insert(LiftingSchemes::value_type("LinearHaar", core::shared_ptr<QuatUtils::QuatLiftingScheme>(new LinearHaarLS)));
	liftingSchemes.insert(LiftingSchemes::value_type("QuatHaar", core::shared_ptr<QuatUtils::QuatLiftingScheme>(new QuatHarrLS)));
	liftingSchemes.insert(LiftingSchemes::value_type("Lerp", core::shared_ptr<QuatUtils::QuatLiftingScheme>(new QuatLerpLS)));
	liftingSchemes.insert(LiftingSchemes::value_type("Slerp", core::shared_ptr<QuatUtils::QuatLiftingScheme>(new QuatSlerpLS)));
	//liftingSchemes.insert(LiftingSchemes::value_type("TangentSpace", core::shared_ptr<QuatUtils::QuatLiftingScheme>(new PseudoTangentSpaceLS)));

	//wyliczam forward transform dla zbiorów
	LiftingResults forwardResults;
	{
		CompoundJob cj(plugin::getJobManager());

		for(auto it = liftingSchemes.left.begin(); it != liftingSchemes.left.end(); ++it){
			cj.addJob(utils::IRunnablePtr(new utils::FunctorRunnable(boost::bind(&jobComputeForwardTransforms, it->get_right(),
			&forwardResults, &testData, size))));
		}
	}

	//teraz faktyczne zadania
	std::list<core::shared_ptr<ExperimentResult>> results;

	core::shared_ptr<core::ConstObjectsList> outputData(new core::ConstObjectsList);

	{		
		CompoundJob cj(plugin::getJobManager());
		{
			//tylko odwrotna
			{
				for(auto it = liftingSchemes.begin(); it != liftingSchemes.end(); ++it){
					auto experiment = core::shared_ptr<ExperimentResult>(new ExperimentResult);
					experiment->name = "Inverse transform " + it->get_left();
					experiment->eulerInput = inputData;
					experiment->quatInput = testData[0].quatData;
					experiment->quatReference = testData[0].quatData;
					experiment->size = size;
					experiment->ls = it->get_right();
				
					auto fIT = forwardResults.find(it->get_right());
				
					experiment->forward = fIT->second[0];
					results.push_back(experiment);

					cj.addJob(utils::IRunnablePtr(new utils::FunctorRunnable(boost::bind(&jobComputeInverseTransform, experiment.get()))));
				}
			}


			LiftingSchemes quatLiftingSchemes = liftingSchemes;
			quatLiftingSchemes.left.erase("LinearHaar");
			quatLiftingSchemes.left.erase("Lerp");

			//odszumianie - 3 poziomy dla 3 liftingów i 3 szumów
			{
				std::set<double> denoiseThresholds;
				denoiseThresholds.insert(0.001);
				denoiseThresholds.insert(0.01);
				denoiseThresholds.insert(0.05);
				denoiseThresholds.insert(0.2);
				denoiseThresholds.insert(0.5);
				denoiseThresholds.insert(2);
				denoiseThresholds.insert(5);

				for(auto it = quatLiftingSchemes.begin(); it != quatLiftingSchemes.end(); ++it){
					for(unsigned int i = 1; i < 4; ++i){
						for(auto tIT = denoiseThresholds.begin(); tIT != denoiseThresholds.end(); ++tIT){
							auto experiment = core::shared_ptr<ExperimentResult>(new ExperimentResult);
							experiment->name = "Denoise-Transform " + it->get_left() + " n " +
								boost::lexical_cast<std::string>(testData[i].noise) + " t " +
								boost::lexical_cast<std::string>(*tIT);

							experiment->eulerInput = testData[i].eulerData;
							experiment->quatInput = testData[i].quatData;
							experiment->quatReference = testData[0].quatData;
							experiment->size = size;
							experiment->ls = it->get_right();

							auto fIT = forwardResults.find(it->get_right());

							experiment->forward = fIT->second[i];
							results.push_back(experiment);
							cj.addJob(utils::IRunnablePtr(new utils::FunctorRunnable(boost::bind(&jobDenoiseChannel, experiment.get(), osg::DegreesToRadians(*tIT)))));
						}
					}
				}
			}

			// kompresja - 3 warianty dla oryginalnych danych
			{
				const auto sizeLog2 = LiftingScheme::LiftingSchemeUtils::ilog2(size);

				std::set<QuatUtils::QuatLiftingCompressor::CompressionSettings> settingsSet;
				{
					auto res = sizeLog2-1;
					QuatUtils::QuatLiftingCompressor::CompressionSettings ss;
					ss.insert(res--);
					settingsSet.insert(ss);
					ss.insert(res--);
					settingsSet.insert(ss);
					ss.insert(res--);
					settingsSet.insert(ss);
				}

				for(auto it = quatLiftingSchemes.begin(); it != quatLiftingSchemes.end(); ++it){
					for(auto sIT = settingsSet.begin(); sIT != settingsSet.end(); ++sIT){
						auto experiment = core::shared_ptr<ExperimentResult>(new ExperimentResult);
						experiment->name = "Compression " + it->get_left() + " levels ";

						for(auto s = (*sIT).begin(); s != (*sIT).end(); ++s){
							experiment->name += boost::lexical_cast<std::string>(*s) + "-";
						}

						experiment->name.erase(experiment->name.size() - 1, 1);
						experiment->size = size;
						experiment->ls = it->get_right();

						experiment->eulerInput = inputData;
						experiment->quatInput = testData[0].quatData;
						experiment->quatReference = testData[0].quatData;

						auto fIT = forwardResults.find(it->get_right());

						experiment->forward = fIT->second[0];
						results.push_back(experiment);
						cj.addJob(utils::IRunnablePtr(new utils::FunctorRunnable(boost::bind(&jobCompressChannel, experiment.get(), *sIT))));
					}
				}
			}

			//komprescja - szkielet
			{

			}
		}

		typedef LiftingScheme::InterpolatorLiftingSchemeT<osg::Vec3,
			QuatUtils::TangentSpaceQuatInterpolator> TangentSpaceLiftingScheme;

		struct TangentData
		{
			QuatUtils::Vec3LiftingScheme::Data data;
			QuatUtils::Vec3LiftingScheme::Data forward;
		};

		//przygotowuje dane - przechodze do przestrzeni stycznej
		boost::array<TangentData, 4> tangentSpaceData;

		TangentSpaceLiftingScheme ls;

		//konwertujemy i zapisujemy
		for(unsigned int i = 0; i < testData.size(); ++i){
			tangentSpaceData[i].data.resize(testData[i].quatData.size());
			quatToTangentSpace(testData[i].quatData, tangentSpaceData[i].data);
			auto ow = createWrapper(tangentSpaceData[i].data, inputData, "TangentSpace data - n " + boost::lexical_cast<std::string>(testData[i].noise));
			outputData->push_back(ow);
			tangentSpaceData[i].forward = tangentSpaceData[i].data;
			ls.forwardTrans(tangentSpaceData[i].forward, size);
			ow = createWrapper(tangentSpaceData[i].forward, inputData, "Details TangentSpace data - n " + boost::lexical_cast<std::string>(testData[i].noise));
			outputData->push_back(ow);

			//teraz do kwaternionow
			QuatUtils::QuatLiftingScheme::Data quat;
			quat.resize(tangentSpaceData[i].forward.size());
			tangentSpaceToQuat(tangentSpaceData[i].forward, quat);

			ow = createWrapper(quat, inputData, "Quat Details TangentSpace data - n " + boost::lexical_cast<std::string>(testData[i].noise));
			outputData->push_back(ow);
		}

		//teraz odwracamy
		{
			QuatUtils::Vec3LiftingScheme::Data backward = tangentSpaceData[0].forward;
			ls.inverseTrans(backward, size);
			auto ow = createWrapper(backward, inputData, "Inverse TangentSpace");
			outputData->push_back(ow);

			//teraz do kwaternionow
			QuatUtils::QuatLiftingScheme::Data quat;
			quat.resize(backward.size());
			tangentSpaceToQuat(backward, quat);

			const double dist = QuatCommon::dist(quat, testData[0].quatData);

			ow = createWrapper(quat, inputData, "Quat Inverse TangentSpace - dist " + boost::lexical_cast<std::string>(dist));
			outputData->push_back(ow);
		}

		//teraz usuwamy szum
		{

		}

		//kompresujemy
		{

		}
	}

	for(unsigned int i = 1; i < 4; ++i){
		auto ow = core::ObjectWrapper::create<VectorChannel::Interface>();
		ow->set(core::const_pointer_cast<VectorChannel::Interface>(testData[i].eulerData));
		(*ow)["core/label"] = "Noisy data: " + boost::lexical_cast<std::string>(testData[i].noise)
			+ " dist: " + boost::lexical_cast<std::string>(testData[i].distance);
		outputData->push_back(ow);
	}

	for(auto it = results.begin(); it != results.end(); ++it){

		//dane wlasciwe
		auto result = *it;
		auto ow = core::ObjectWrapper::create<VectorChannel::Interface>();
		ow->set(VectorChannelReaderInterfacePtr(result->convertedOut));

		(*ow)["core/label"] = result->name + " distInput: " + boost::lexical_cast<std::string>(result->distInput)
			+ " distReference: " + boost::lexical_cast<std::string>(result->distReference);

		outputData->push_back(ow);

		//detale
		ow = core::ObjectWrapper::create<VectorChannel::Interface>();
		auto convertedOut = VectorChannelPtr(new VectorChannel(result->convertedOut->getSamplesPerSecond()));
		convertedOut->setName("Details " + result->name);
		convertedOut->setTimeBaseUnit(result->convertedOut->getTimeBaseUnit());
		convertedOut->setValueBaseUnit(result->convertedOut->getValueBaseUnit());
		convertedOut->setTimeScaleFactor(result->convertedOut->getTimeScaleFactor());
		convertedOut->setValueScaleFactor(result->convertedOut->getValueScaleFactor());

		for(unsigned int i = 0; i < result->forward.size(); ++i){
			auto res = osg::QuatUtils::quaterionToEuler
				<osg::Quat, osg::Vec3>(result->forward[i]);

			res.x() = osg::RadiansToDegrees(res.x());
			res.y() = osg::RadiansToDegrees(res.y());
			res.z() = osg::RadiansToDegrees(res.z());

			convertedOut->addPoint(res);
		}

		ow->set(VectorChannelReaderInterfacePtr(convertedOut));
		(*ow)["core/label"] = convertedOut->getName();
		outputData->push_back(ow);

		if((*it)->modifiedForward.empty() == false){
			ow = core::ObjectWrapper::create<VectorChannel::Interface>();
			auto convertedOut = VectorChannelPtr(new VectorChannel(result->convertedOut->getSamplesPerSecond()));
			convertedOut->setName("Modified details " + result->name);
			convertedOut->setTimeBaseUnit(result->convertedOut->getTimeBaseUnit());
			convertedOut->setValueBaseUnit(result->convertedOut->getValueBaseUnit());
			convertedOut->setTimeScaleFactor(result->convertedOut->getTimeScaleFactor());
			convertedOut->setValueScaleFactor(result->convertedOut->getValueScaleFactor());

			for(unsigned int i = 0; i < result->modifiedForward.size(); ++i){
				auto res = osg::QuatUtils::quaterionToEuler
					<osg::Quat, osg::Vec3>(result->modifiedForward[i]);

				res.x() = osg::RadiansToDegrees(res.x());
				res.y() = osg::RadiansToDegrees(res.y());
				res.z() = osg::RadiansToDegrees(res.z());

				convertedOut->addPoint(res);
			}

			ow->set(VectorChannelReaderInterfacePtr(convertedOut));
			(*ow)["core/label"] = convertedOut->getName();
			outputData->push_back(ow);
		}
	}

	outPinA->setValue(outputData);
}

void MotionAnalysisTests::reset()
{

}