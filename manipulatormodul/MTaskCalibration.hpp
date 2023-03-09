#ifndef CALIBRATE_MANIPULATOR_TASK_HPP
#define CALIBRATE_MANIPULATOR_TASK_HPP

#include "MTask.hpp"
#include "Eigen/dense"

namespace youbot {
	/// <summary>
   /// Task that send out commutation initialization command and finishes as all of the joints are initialized
   /// 
   /// TODO!!
   /// </summary>
	class MTaskCalibration : public MTask 
	{
		

	public:
		ManipulatorCommand GetCommand(const JointsState& new_state) override ;
		

		TaskType GetType() const override;
		

	protected:
		bool _taskFinished() const override;

	};
}
#endif