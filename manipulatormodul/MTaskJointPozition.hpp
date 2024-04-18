#pragma once

#include "Config.hpp"
#include "MTask.hpp"

# define M_PI 3.14159265358979323846

namespace youbot {
	class MTaskJointPozition :public MTask
	{
	public:
		MTaskJointPozition(const Eigen::VectorXd& dq);
		MTaskJointPozition(const Eigen::VectorXd& dq, const double componentP);
		ManipulatorCommand GetCommand(const JointsState& new_state) override;  ///< returns the current command
		TaskType GetType() const override;

	protected:
		bool _taskFinished() const override; ///< returns if the task has finished

		double componentP;
		const Eigen::VectorXd req_q;
		bool finished = false;
		bool move[5];
		int moveHelper[5];
		double maxVelocity = 0.6;
		//int timeLimit;
		int reachedSince = 0;
		std::chrono::steady_clock::time_point timerStart;
		bool timerMode = false;
		double jointRange[5] = { 338 * M_PI / 180,155 * M_PI / 180,297 * M_PI / 180,205 * M_PI / 180,335 * M_PI / 180 };

		double pozTolerance = 0.003;

		ManipulatorCommand cmd;
	};
}
