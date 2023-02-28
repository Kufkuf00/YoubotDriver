#ifndef DUMMY_TASK_HPP
#define DUMMY_TASK_HPP

#include "YoubotManipulator.hpp"
#include "Eigen/dense"
#include "ManipulatorTask.hpp"

namespace youbot {
	class DummyTask:public ManipulatorTask
	{
	public:
		DummyTask();

		ManipulatorCommand GetCommand(const JointsState& new_state) override;

		TaskType GetType() const override;

	protected:
		bool _taskFinished() const override;	
		std::chrono::steady_clock::time_point lastDirChanged;
		std::chrono::steady_clock::time_point start;
		int counter;
	};

}
#endif