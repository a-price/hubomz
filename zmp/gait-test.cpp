#include "gait.h"

using namespace std;
using namespace Eigen;

int main() {

	//////////////////////////////////////////////////////////////////////
	// test gait timer
	step_timer_t timer;
	timer.startup_time = 1.0; // hold ZMP at center for 1s at start
	timer.shutdown_time = 1.0; // hold ZMP at center for 1s at end
	timer.single_support_time = 0.70; // .70s of single support each step
	timer.double_support_time = 0.05; // .05s of double support each step

	//timer.set_duty_single(0.55, 0.70);

	std::cerr << timer << std::endl;

	int n_steps = 8;

	step_traj_t step_traj = step_times(steps(n_steps, true), timer);

	//////////////////////////////////////////////////////////////////////
	// print out, pipe to matlab/timing.txt and run matlab/zmp_plot.m
	size_t eric_ticks = step_traj.stance.size();
	EIGEN_V_VEC2D eric_zmpref = step_traj.zmpref;
	std::vector<stance_t> eric_stance = step_traj.stance;

	std::cout << "i x y stance" << std::endl;
	for(size_t i=0; i < eric_ticks; i++) {
		using namespace std;
		using namespace Eigen;

		Vector2d zmp = eric_zmpref[i];
		stance_t stance = eric_stance[i];

		cout << i << " "
				<< zmp(0) << " "
				<< zmp(1) << " "
				<< stance << endl;
	}
}
