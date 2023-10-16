/**
 * @file deliverable_1.cpp
 * @brief Hands-on introduction: Robot Motion + Robot Localization, as in
 * https://smartech.gatech.edu/handle/1853/45226
 */

/**
 * A simple 2D pose slam example with "GPS" measurements
 *  - The robot moves forward 2 meter each iteration
 *  - The robot initially faces along the X axis (horizontal, to the right in
 * 2D)
 *  - We have full odometry between pose
 *  - We have "GPS-like" measurements implemented with a custom factor
 */

// We will use Pose2 variables (x, y, theta) to represent the robot positions
#include <gtsam/geometry/Pose2.h>

// We will use simple integer Keys to refer to the robot poses.
#include <gtsam/inference/Key.h>

// As in OdometryExample.cpp, we use a BetweenFactor to model odometry
// measurements.
#include <gtsam/slam/BetweenFactor.h>

// We add all facors to a Nonlinear Factor Graph, as our factors are nonlinear.
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

// The nonlinear solvers within GTSAM are iterative solvers, meaning they
// linearize the nonlinear functions around an initial linearization point, then
// solve the linear system to update the linearization point. This happens
// repeatedly until the solver converges to a consistent set of variable values.
// This requires us to specify an initial guess for each variable, held in a
// Values container.
#include <gtsam/nonlinear/Values.h>

// Finally, once all of the factors have been added to our factor graph, we will
// want to solve/optimize to graph to find the best (Maximum A Posteriori) set
// of variable values. GTSAM includes several nonlinear optimizers to perform
// this step. Here we will use the standard Levenberg-Marquardt solver
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

// Once the optimized values have been calculated, we can also calculate the
// marginal covariance of desired variables
#include <gtsam/nonlinear/Marginals.h>

#include <gtsam/slam/PriorFactor.h>

using namespace std;
using namespace gtsam;

// Before we begin the example, we must create a custom unary factor to
// implement a "GPS-like" functionality. Because standard GPS measurements
// provide information only on the position, and not on the orientation, we
// cannot use a simple prior to properly model this measurement.
//
// The factor will be a unary factor, affect only a single system variable. It
// will also use a standard Gaussian noise model. Hence, we will derive our new
// factor from the NoiseModelFactor1.
#include <gtsam/nonlinear/NonlinearFactor.h>

class UnaryFactor : public NoiseModelFactor1<Pose2> {
  // The factor will hold a measurement consisting of an (X,Y) location
  // We could this with a Point2 but here we just use two doubles

  double mx_, my_;

 public:
  // The constructor requires the variable key, the (X, Y) measurement value,
  // and the noise model.
  UnaryFactor(Key j, double x, double y, const SharedNoiseModel& model)
      : NoiseModelFactor1<Pose2>(model, j), mx_(x), my_(y) {}

  virtual ~UnaryFactor() {}

  // Using the NoiseModelFactor1 base class there are two functions that must be
  // overridden. The first is the 'evaluateError' function. This function
  // implements the desired measurement function, returning a vector of errors
  // when evaluated at the provided variable value. It must also calculate the
  // Jacobians for this measurement function, if requested.
  Vector evaluateError(const Pose2& q,
                       boost::optional<Matrix&> H = boost::none) const {
    // The measurement function for a GPS-like measurement is simple:
    // error_x = pose.x - measurement.x
    // error_y = pose.y - measurement.y
    // The Jacobian arises from a first-order approximation of the error vector
    // See Sec 3.2 in https://gtsam.org/tutorials/intro.html
    // [ derror_x/dx  derror_x/dy  derror_x/dtheta ] = [cos(th) -sin(th) 0]
    // [ derror_y/dx  derror_y/dy  derror_y/dtheta ] = [sin(th) cos(th) 0]
    if (H) {
      // Calculate the jacobian (*H)
      const auto& R = q.rotation();
      *H = (Matrix(2, 3) << R.c(), -R.s(), 0.0, R.s(), R.c(), 0.0).finished();
    }
    // Fill here the actual error function.
    return (Vector(2) << q.x() - mx_, q.y() - my_).finished();
  }

};  // UnaryFactor, a factor that depends on a single variable.

int main(int argc, char** argv) {
  // Create a factor graph container and add factors to it
  NonlinearFactorGraph graph;
  Pose2 priorMean(0.0, 0.0, 0.0);
  noiseModel::Diagonal::shared_ptr priorNoise =
      noiseModel::Diagonal::Sigmas(Vector3(0.3, 0.3, 0.1));
  graph.add(PriorFactor<Pose2>(1, priorMean, priorNoise));

  // 1a. Add odometry factors
  // For simplicity, we will use the same noise model for each odometry
  // factor
  Pose2 odometry(2.0, 0.0, 0.0);
  noiseModel::Diagonal::shared_ptr odometryNoise =
      noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1));

  // TODO: Add the above odometry measurement and noise as between factors
  // between nodes 1, 2 and nodes 2, 3
  // Create odometry (Between) factors between consecutive poses
  //
  // Insert code below:

  // End of 1a.

  // 1b. Add "GPS-like" measurements
  // We will use our custom UnaryFactor (already defined for you) for this.
  noiseModel::Diagonal::shared_ptr unaryNoise =
      noiseModel::Diagonal::Sigmas(Vector2(0.1, 0.1));

  // TODO: Add "GPS" like measurement to nodes 1, 2, and 3 with UnaryFactor 
  // (0, 0) for node 1, (2, 0) for node 2, (4, 0) for node 3
  //
  // Insert code below:

  // End of 1b.

  // 1c. Create the data structure to hold the initialEstimate estimate to
  // the solution For illustrative purposes, these have been deliberately set to
  // incorrect values

  // TODO: Add initial estimates (Pick incorrect values to demonstrate estimate
  // before and after optimization)
  Values initial;
  //
  // Insert code below:

  // End of 1c.

  // print and report the initial results.
  graph.print();
  initial.print("Initial Estimate \n");

  // . Optimize using Levenberg-Marquardt optimization. The optimizer
  // accepts an optional set of configuration parameters, controlling
  // things like convergence criteria, the type of linear system solver
  // to use, and the amount of information displayed during optimization.
  // Here we will use the default set of parameters. See the
  // documentation for the full set of parameters.
  Values result = LevenbergMarquardtOptimizer(graph, initial).optimize();

  // Print and report the final results.
  result.print("Optimized Estimate \n");
  return 0;
}
