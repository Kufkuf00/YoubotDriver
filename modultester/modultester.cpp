#include "adapters.hpp"
#include "Manager.hpp"
#include "MTaskRawConstantJointSpeed.hpp"
#include "Time.hpp"
#include "Logger.hpp"
#include "MTaskCommutation.hpp"
#include "MTaskCalibration.hpp"
#include "MTaskZeroCurrent.hpp"
#include "MTaskStop.hpp"
#include "MTaskJointPozition.hpp"
#include "MTaskFreeDrive.hpp"


using namespace youbot;

EtherCATMaster::Ptr center;

int main(int argc, char* argv[])
{
	std::string configpath = std::string(CONFIG_FOLDER) + "youBotArmConfig_fromKeisler.json";
	//youBotArmConfig_fromfactory.json");
	//youBotArmConfig_fromMoveIt.json");
	//youBotArmConfig_fromKeisler.json");

	Manager modul(configpath,true);
	modul.StartThreadAndInitialize();
	//wait till conig and auto task finish
	{
		auto start = std::chrono::steady_clock::now();
		bool finished;
		do {
			SLEEP_MILLISEC(10);
			if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start).count() > 100)
			{
				throw std::runtime_error("Overtime during initialization");
			}
			auto status = modul.GetStatus();
			finished = (status.motion == MTask::STOPPED)
				&& (status.manipulatorStatus.IsCalibrated())
				&& (status.manipulatorStatus.IsCommutationInitialized());

		} while (!finished);
	}

	for (size_t i = 0; i < 15; i++)
	{
		log(Log::info, "CALIBRATION ENDED");
	}

	{
		Eigen::VectorXd q(5);
		//		q << 0.5, 0.5, 0.5, 0.5, 0.5;
		//q << 1, 1, 1, 1, 1;
		q << 1, 0, 1, -1, 1;
		//q << 0.17, 0.17, 0.17, 0.17, 0.17;
		//q = q * 0;
		//q << 0, 0, 0, 0, 0;
		MTask::Ptr JointTask = std::make_shared<MTaskJointPozition>(q);
		modul.NewManipulatorTask(JointTask,100);
		auto start = std::chrono::steady_clock::now();
		do {
			modul.GetStatus().LogStatus();
			SLEEP_MILLISEC(10);
		} while (std::chrono::duration_cast<std::chrono::seconds>(
			std::chrono::steady_clock::now() - start).count() < 10);
		int i = 0;
		while (i<200)
		{
			log(Log::info, "........................................");
			i++;
		}
	}

	 
	 {
		MTask::Ptr JointTask = std::make_shared<MTaskFreeDrive>();
		modul.NewManipulatorTask(JointTask, 100);
		auto start = std::chrono::steady_clock::now();
		do {
			modul.GetStatus().LogStatus();
			SLEEP_MILLISEC(10);
		} while (std::chrono::duration_cast<std::chrono::seconds>(
			std::chrono::steady_clock::now() - start).count() < 60);
	 }
	
	
	;


	// Create and start a task
	if (0) {
		Eigen::VectorXd dq(5);
		dq << 0.1, 0.1, -0.1, 0.1, -0.1;
		MTask::Ptr task = std::make_shared<MTaskRawConstantJointSpeed>(dq, 10);
		modul.NewManipulatorTask(task, 5);
		auto start = std::chrono::steady_clock::now();
		do {
			//modul.GetStatus().LogStatus();
			SLEEP_MILLISEC(10);
		} while (std::chrono::duration_cast<std::chrono::seconds>(
			std::chrono::steady_clock::now() - start).count() < 5);
	}


	// Stop and go home
	modul.StopThread();

	return 0;




	//modul.NewManipulatorTask(task2, 50);

	// Lets see what's happening
	for (int i = 0; i < 700; i++) {
		SLEEP_MILLISEC(10);
		modul.GetStatus().LogStatus();
	}

	// Stop and go home
	modul.StopThread();

	return 0;
}