#pragma once

#include "Config.hpp"
#include "MTask.hpp"

#include <iostream>
#include <fstream>
#include <cmath>
using namespace Eigen;

#define gravitational_acceleration 9.81
# define M_PI 3.14159265358979323846


#define lbx 0
#define lbz 0
#define l1x 0.024
#define l1z 0.115
#define l2x 0.033
#define l2z 0
#define l3x 0
#define l3z 0.155
#define l4x 0
#define l4z 0.135
#define l5x 0
#define l5z 0.1136
#define lfx 0
#define lfz 0.05716

#define l1m 2.351
#define l2m 1.318
#define l3m 0.821
#define l4m 0.769
#define l5m 0.687
#define mg 0.219

#define c1x 0.01516
#define c1y 0.00359
#define c1z -0.03105
#define c2x -0.01903
#define c2y 0.0150
#define c2z 0.11397
#define c3x 0.00013
#define c3y 0.02022
#define c3z 0.10441
#define c4x 0.00015
#define c4y -0.02464
#define c4z 0.05353
#define c5x 0
#define c5y 0.0012
#define c5z 0.01648
#define cgx 0
#define cgy 0
#define cgz 0.0289

#define I1x 0.0029525
#define I1y 0.0060091
#define I1z 0.0058821
#define I2x 0.0031145
#define I2y 0.0005843
#define I2z 0.0031631
#define I3x 0.00172767
#define I3y 0.00041967
#define I3z 0.0018468
#define I4x 0.0006764
#define I4y 0.0010573
#define I4z 0.0006610
#define I5x 0.0001934
#define I5y 0.0001602
#define I5z 0.0000689
#define Igx 0.0002324
#define Igy 0.0003629
#define Igz 0.0002067
namespace youbot {
	class MTaskFreeDrive : public MTask
	{
	public:
		virtual TaskType GetType() const override;
		virtual ManipulatorCommand GetCommand(const JointsState& new_state) override;

	protected:
		// Inherited via MTask
		virtual bool _taskFinished() const override;

		inline void getJacobi(MatrixXd& J, const youbot::JointsState& new_state)
		{
			double joint_pose_0 = new_state.joint[0].q.value;
			double joint_pose_1 = new_state.joint[1].q.value;
			double joint_pose_2 = new_state.joint[2].q.value;
			double joint_pose_3 = new_state.joint[3].q.value;
			double joint_pose_4 = new_state.joint[4].q.value;

			double cos00 = cos(joint_pose_0);
			double sin00 = sin(joint_pose_0);
			double cos01 = cos(joint_pose_1);
			double cos02 = cos(joint_pose_0 - joint_pose_1 + joint_pose_2 - joint_pose_3);
			double sin01 = sin(joint_pose_1);
			double cos03 = cos(joint_pose_0 + joint_pose_1 - joint_pose_2 + joint_pose_3);
			double sin02 = sin(joint_pose_0 + joint_pose_1 - joint_pose_2 + joint_pose_3);
			double sin03 = sin(joint_pose_0 - joint_pose_1 + joint_pose_2 - joint_pose_3);
			double cos04 = cos(joint_pose_1 - joint_pose_2 + joint_pose_3);
			double sin04 = sin(joint_pose_1 - joint_pose_2 + joint_pose_3);
			J(0, 0) = 0.0;
			J(0, 1) = cos00 * (l1z + l2z + lbz);
			J(0, 2) = -(cos00 * (l1z + l2z + lbz + (l3z * cos01)));
			J(0, 3) = cos00 * (l1z + l2z + lbz + (l3z * cos01) + (l4z * cos(joint_pose_1 - joint_pose_2)));
			J(0, 4) = (l1z * cos02 / 2.0) + (l2z * cos02 / 2.0) + (lbz * cos02 / 2.0) + (l2x * sin03 / 2.0) + (l3z * cos(joint_pose_0 + joint_pose_2 - joint_pose_3) / 2.0) - (l3z * cos(joint_pose_0 - joint_pose_2 + joint_pose_3) / 2.0) - (l4z * cos(joint_pose_0 + joint_pose_3) / 2.0) + (l5x * sin00) - (l1z * cos03 / 2.0) - (l2z * cos03 / 2.0) - (lbz * cos03 / 2.0) + (l2x * sin02 / 2.0) + (l4z * cos(joint_pose_0 - joint_pose_3) / 2.0);
			J(1, 0) = -l1x - lbx;
			J(1, 1) = sin00 * (l1z + l2z + lbz);
			J(1, 2) = -(sin00 * (l1z + l2z + lbz + (l3z * cos01)));
			J(1, 3) = sin00 * (l1z + l2z + lbz + (l3z * cos01) + (l4z * cos(joint_pose_1 - joint_pose_2)));
			J(1, 4) = (l1z * sin03 / 2.0) - (l2x * cos02 / 2.0) + (l2z * sin03 / 2.0) + (lbz * sin03 / 2.0) - (l1x * cos04) - (lbx * cos04) + (l3z * sin(joint_pose_0 + joint_pose_2 - joint_pose_3) / 2.0) - (l3z * sin(joint_pose_0 - joint_pose_2 + joint_pose_3) / 2.0) - (l4z * sin(joint_pose_0 + joint_pose_3) / 2.0) - (l5x * cos00) - (l2x * cos03 / 2.0) - (l1z * sin02 / 2.0) - (l2z * sin02 / 2.0) - (lbz * sin02 / 2.0) + (l4z * sin(joint_pose_0 - joint_pose_3) / 2.0);
			J(2, 0) = 0.0;
			J(2, 1) = -l2x - (l1x * cos00) - (lbx * cos00);
			J(2, 2) = (cos01 * (l1x + l2x + lbx)) - (sin01 * (l1z + l2z + l3z + lbz)) - (cos00 * ((cos00 * (((cos01 - 1.0) * (l1x + l2x + lbx)) - (sin01 * (l1z + l2z + lbz)))) + ((cos00 - 1.0) * (l1x + lbx)))) - (sin00 * ((sin00 * (l1x + lbx)) + (sin00 * (((cos01 - 1.0) * (l1x + l2x + lbx)) - (sin01 * (l1z + l2z + lbz))))));
			J(2, 3) = (l3z * sin01) - (l1x * cos00) - (lbx * cos00) - l2x + (l4z * sin(joint_pose_1 - joint_pose_2));
			J(2, 4) = -(sin04 * sin00 * (l1x + lbx));
			J(3, 0) = 0.0;
			J(3, 1) = sin(joint_pose_0);
			J(3, 2) = -sin00;
			J(3, 3) = sin(joint_pose_0);
			J(3, 4) = -(sin04 * cos00);
			J(4, 0) = 0.0;
			J(4, 1) = -cos00;
			J(4, 2) = cos(joint_pose_0);
			J(4, 3) = -cos00;
			J(4, 4) = -(sin04 * sin00);
			J(5, 0) = 1.0;
			J(5, 1) = 0.0;
			J(5, 2) = 0.0;
			J(5, 3) = 0.0;
			J(5, 4) = cos(joint_pose_1 - joint_pose_2 + joint_pose_3);
		}

		inline void getInverseJacobi(MatrixXd& Jinv, youbot::JointsState new_state)  
		{
			MatrixXd J(6, 5);
			MatrixXd Jt(5, 6);
			MatrixXd Jtemp(5, 5);
			getJacobi(J, new_state);
			Jt = J.transpose();
			Jtemp = Jt * J;
			Jtemp = Jtemp.inverse();
			Jinv = Jtemp * Jt;
		}
	};
}

