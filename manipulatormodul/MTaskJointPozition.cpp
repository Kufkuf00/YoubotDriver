#include "MTaskJointPozition.hpp"
#include "Time.hpp"
#include "Logger.hpp"

using namespace youbot;


youbot::MTaskJointPozition::MTaskJointPozition(const Eigen::VectorXd& dq, const double componentP):			//jelenlegi struktúra mellett 																									
	req_q(dq), componentP(componentP){																	   //lehet el is hagyható a konstruktor
	for (int i = 0; i < 5; i++)
	{
		move[i] = true;
		moveHelper[i] = 0;
	}
	timerStart = std::chrono::steady_clock::now();
	
}

youbot::MTaskJointPozition::MTaskJointPozition(const Eigen::VectorXd& dq) :MTaskJointPozition(dq,1)
{}

youbot::ManipulatorCommand youbot::MTaskJointPozition::GetCommand(const JointsState& new_state) {

	for (int i = 0; i < 5; i++)
	{
		//log(Log::info, "move: " + std::to_string(move[i]));

		double e = req_q[i] - new_state.joint[i].q.value;
		//log(Log::info, "## CSUKLO " + std::to_string(i) + " Aktualis hiba:  " + std::to_string(e));
		//log(Log::info, "## CSUKLO " + std::to_string(i) + " Elvart pozicio:  " + std::to_string(req_q[i]));

		/*if (abs(e) < pozTolerance)
		{
			moveHelper[i]++;
			if (moveHelper[i] > 15)
			{
				move[i] = false;
			}
		}
		else*/
		{
			double v = (e) * componentP;
			//log(Log::info, std::to_string(i) + " joint, sebesseg: " + std::to_string(v));
			cmd.commands[i] = BLDCCommand(BLDCCommand::JOINT_VELOCITY, v);
		}
	}
	return cmd;
	{
		/*
		double elvartP = 0;		//[rad]
		double csukloTartomany = 5.94;

		double e = elvartP- new_state.joint[0].q.value;
		double v = (e)*1.2;
		cmd.commands[0] = BLDCCommand(BLDCCommand::JOINT_VELOCITY, v);
		log(Log::info,  "## A Kiadott szebesseg:  " + std::to_string(v));
		log(Log::info,  "## Aktualis hiba:  " + std::to_string(e));
		log(Log::info, "## Elvart pozicio:  " + std::to_string(elvartP));
		if (abs(e) < 0.05)
		{
			reachedSince++;
		}
		for (int i = 1; i < 5; i++)
		{
			cmd.commands[i] = BLDCCommand(BLDCCommand::JOINT_VELOCITY, 0);
		}


		return cmd;*/
	}

}

MTask::TaskType youbot::MTaskJointPozition::GetType() const {
	return TaskType::JOINT_POZITION;
}


bool youbot::MTaskJointPozition::_taskFinished() const {
	return false;
}

//bool youbot::MTaskJointPozition::TaskFinishedHelper()
//{
//	if (timerMode)
//	{
//		if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - timerStart).count() < timeLimit)
//		{
//			return false;
//		}
//		log(Log::info, "#####TASK FINISHED####");
//		return true;
//	}
//	else {
//		int i = 0;
//		while (i < 5 && !move[i])
//		{
//			i++;
//		}
//		if (i < 5)
//		{
//			return false;
//		}
//		else
//		{
//			reachedSince++;
//		}
//		if (reachedSince > 5)
//		{
//			log(Log::info, "#####TASK FINISHED####");
//			return true;
//			timerStart = std::chrono::steady_clock::now();
//			timerMode = true;
//
//		}
//		return false;
//	}
//}

