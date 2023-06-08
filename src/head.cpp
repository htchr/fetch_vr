#include "fetch_vr/head.h"

Head::Head()
: pan_client_("/head_controller/follow_joint_trajectory", true) {
	pan_client_.waitForServer();
}

void Head::panTilt(double pan, double tilt) {
	double MIN_PAN = -1.570796;
	double MAX_PAN = 1.570796;
	double MIN_TILT = -0.7853982;
	double MAX_TILT = 1.570796;

	if (pan < MIN_PAN || pan > MAX_PAN || tilt < MIN_TILT || tilt > MAX_TILT)
		return;

	trajectory_msgs::JointTrajectoryPoint point;
	point.positions.push_back(pan);
	point.positions.push_back(tilt);
	point.time_from_start = ros::Duration(0.5);

	control_msgs::FollowJointTrajectoryGoal goal;
	goal.trajectory.joint_names.push_back("head_pan_joint");
	goal.trajectory.joint_names.push_back("head_tilt_joint");
	goal.trajectory.points.push_back(point);

	pan_client_.cancelGoal(); // preempt last goal
	pan_client_.sendGoal(goal);
}
