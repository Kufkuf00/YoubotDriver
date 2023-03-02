#include "MTaskCalibration.hpp"
#include <stdexcept>
#include <Config.hpp>
#include <Config.cpp>
#include "MotionLayer.hpp"
#include "MTask.hpp"
#include "Eigen/dense"


using namespace youbot;

bool still_moving[5] = { true,true,true,true,true };
int cycles_in_zero_speed[5] = { 0,0,0,0,0 };
int sinceEndPoint;
bool forward[5];
int jointSinceEndPoint[5];
double calibrationSpeed = 0.5;

ManipulatorCommand MTaskCalibration::GetCommand(const JointsState& new_state) {
	Eigen::VectorXd dq(5);
	
	for (size_t i = 0; i < 5; i++)
	{
		
		double vel = (new_state.joint[i].dq).value;
		if (vel > 0)
		{
			forward[i] = true;
			cycles_in_zero_speed[i] = 0;
		}
		else if(vel<0)
		{
			forward[i] = false;
			cycles_in_zero_speed[i] = 0;
		}
		else
		{
			cycles_in_zero_speed[i]++;
		}

		if(cycles_in_zero_speed[i]>4)
		{
			still_moving[i] = false;
		}
	}

	if (still_moving[0] || still_moving[1] || still_moving[2] || still_moving[3] || still_moving[4]){
		sinceEndPoint = 0;
	}
	else {
		sinceEndPoint++;
	}
	
	for (size_t i = 0; i < 5; i++)
	{
		if (still_moving[i])
		{
			dq[i] = calibrationSpeed;
		}
		else {
			dq[i] = 0;
		}
	}
	if (sinceEndPoint > 4)
	{
		//finished = true;
	}
	return ManipulatorCommand(BLDCCommand::JOINT_VELOCITY, dq);

}

MTask::TaskType MTaskCalibration::GetType() const {
	return CALIBRATION;
}

bool MTaskCalibration::_taskFinished() const{
	return true;
}
