#include "MTaskCalibration.hpp"
#include <stdexcept>
#include <Config.hpp>
#include <Config.cpp>
#include "MotionLayer.hpp"
#include "MTask.hpp"
#include "Eigen/dense"


using namespace youbot;

bool still_moving[5] = { true,true,true,true,true };
int cyclesInZero[5] = { 0,0,0,0,0 };
int reachedSince=0;
double holdingCurrent = 30;
double calVelocity = 0.10;
auto start = std::chrono::steady_clock::now();
BLDCCommand mCommands[5];

//int jointSinceEndPoint[5];


ManipulatorCommand MTaskCalibration::GetCommand(const JointsState& new_state) 
{
	if (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count() < 200)
	{	
		return ManipulatorCommand(BLDCCommand(BLDCCommand::JOINT_VELOCITY, calVelocity), BLDCCommand(BLDCCommand::JOINT_VELOCITY, calVelocity), BLDCCommand(BLDCCommand::JOINT_VELOCITY, calVelocity), BLDCCommand(BLDCCommand::JOINT_VELOCITY, calVelocity), BLDCCommand(BLDCCommand::JOINT_VELOCITY, calVelocity));
	}

	for (size_t i = 0; i < 5; i++)
	{
		if (still_moving[i]) {
			double v = new_state.joint[i].dq.value;
			if (v<0.01&&v>-0.01)
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
			if (cyclesInZero[i] >4)
			{
				mCommands[i] = BLDCCommand(BLDCCommand::MOTOR_CURRENT_MA,holdingCurrent);
				still_moving[i] = false;
			}
			else
			{
				mCommands[i] = BLDCCommand(BLDCCommand::JOINT_VELOCITY, calVelocity);
			}
		}
		else
		{
			mCommands[i] = BLDCCommand(BLDCCommand::MOTOR_CURRENT_MA,holdingCurrent);
		}
		

	}
	return ManipulatorCommand(mCommands);
}

MTask::TaskType MTaskCalibration::GetType() const {
	return CALIBRATION;
}

bool MTaskCalibration::_taskFinished() const {
	int i = 0;
	while (i < 5 && !still_moving[i]);
	{
		i++;
	} 

		if (i ==5) {
			
			reachedSince++;
		}
		else {
			reachedSince = 0;
		}


	if (reachedSince < 6)
	{
		return false;
	}
	else {
		return true;
	}

}
