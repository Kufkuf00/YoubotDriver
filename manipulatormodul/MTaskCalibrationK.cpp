#include "MTaskCalibration.hpp"
#include <stdexcept>
#include <Config.hpp>
#include <Config.cpp>
#include "MotionLayer.hpp"
#include "MTask.hpp"
#include "Eigen/dense"
#include "Time.hpp"
#include "Logger.hpp"
#include "MTaskCommutation.hpp"
#include "JointPhysical.hpp"
#include "Manipulator.hpp"


using namespace youbot;

bool still_moving[5] = { true,true,true,true,true };
int cyclesInZero[5] = { 0,0,0,0,0 };
int reachedSince = 0;
double holdingCurrent = 200;
double calVelocity = 0.30;
double startCalVelocity = 0.55;
BLDCCommand mCommands[5];
bool firstCall = true;
std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();

//int jointSinceEndPoint[5];


ManipulatorCommand MTaskCalibration::GetCommand(const JointsState& new_state)
{
	//Manipulator::CheckI2tAndTimeoutErrorProcess();
		
	//log(Log::info, "elkezdte");
	log(Log::info, "firstCall: " + std::to_string(firstCall));
	if (firstCall) {
		 start = std::chrono::steady_clock::now();
		 firstCall = false;		
	}
	//double kk = (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count());

	if ((std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count()) < 300)
	{		
		return{ (BLDCCommand(BLDCCommand::JOINT_VELOCITY, -startCalVelocity)),(BLDCCommand(BLDCCommand::JOINT_VELOCITY, -startCalVelocity)),(BLDCCommand(BLDCCommand::JOINT_VELOCITY, startCalVelocity)),(BLDCCommand(BLDCCommand::JOINT_VELOCITY, -startCalVelocity)),(BLDCCommand(BLDCCommand::JOINT_VELOCITY, startCalVelocity)) };
	}
	/*
	for (size_t i = 0; i < 4; i++)
	{
		mCommands[i] = (BLDCCommand(BLDCCommand::JOINT_VELOCITY, 0));
	}
	mCommands[4] = (BLDCCommand(BLDCCommand::JOINT_VELOCITY, 1));
	*/
	log(Log::info, "200ms eltelt");
	for (size_t i = 0; i < 5; i++)
	{
		//log(Log::info, "i= " + std::to_string(i));
		if (still_moving[i]) {
			double v = new_state.joint[i].dq.value;
			//log(Log::info, "sebesseg= " + std::to_string(v));
			if ((v<0.01)&&(v>-0.01))
			{
				cyclesInZero[i]++;
			}
			else
			{
				cyclesInZero[i] = 0;
			}
			bool positive = false;
			if (v > 0)
			{
				positive = true;
			}
			log(Log::info, std::to_string(i) + " csuklo ideje nullaban: "+ std::to_string(cyclesInZero[i]));
			if (cyclesInZero[i] >4)
			{
				if (i == 0)
					log(Log::error, positive ? "positive": "negative");
				mCommands[i] = BLDCCommand(BLDCCommand::MOTOR_CURRENT_MA,positive? holdingCurrent: -holdingCurrent);
				still_moving[i] = false;
				log(Log::info, "holding current lett allitva");
				;
			}
			else
			{		 
				mCommands[i] = BLDCCommand(BLDCCommand::JOINT_VELOCITY,positive? calVelocity: -calVelocity);
				log(Log::info, "calVelocity lett allitva");
				;
			}
		}
		
		else
		{
			mCommands[i] = BLDCCommand(BLDCCommand::MOTOR_CURRENT_MA,holdingCurrent);
			log(Log::info, "nem mozog az alabbi csuklo: "+ std::to_string(i));
		}
	}
	//*/
	return mCommands;
}

MTask::TaskType MTaskCalibration::GetType() const {
	return MTask::TaskType::CALIBRATION;
}

bool MTaskCalibration::_taskFinished() const {
	

	log(Log::info, "taskFinished");

	int i = 0;
	while (i < 5 && !still_moving[i])
	{
		//log(Log::info, "STILLMOVING ACTUAL VALUE: "+ std::to_string(still_moving[i]));
		i++;
	} 
		if (i ==5) {
			
			reachedSince++;
		}
		else {
			reachedSince = 0;
		}
		log(Log::info, "REACHEDSINCE atctual value: "+ std::to_string(reachedSince));

	if (reachedSince < 6)
	{
		log(Log::info, "hamisan ter vissza " );
		
		return false;
	}
	else {
		log(Log::info, "IGAZKÉNT TÉr Vissza ");
		log(Log::info, "Vege van a Tasknak ");

		return true;
	}//*/

}



