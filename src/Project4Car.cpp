///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: FILL ME OUT!!
//////////////////////////////////////

#include <iostream>

#include <ompl/base/ProjectionEvaluator.h>

#include <ompl/control/SimpleSetup.h>
#include <ompl/control/ODESolver.h>

// The collision checker routines
#include "CollisionChecking.h"

// Your implementation of RG-RRT
#include "RG-RRT.h"

// Your projection for the car
class CarProjection : public ompl::base::ProjectionEvaluator
{
public:
    CarProjection(const ompl::base::StateSpace *space) : ProjectionEvaluator(space)
    {
    }

    unsigned int getDimension() const override
    {
        // TODO: The dimension of your projection for the car
        return 2;
    }

    void project(const ompl::base::State *state, Eigen::Ref<Eigen::VectorXd> projection) const override
    {
        const double *values = state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
        projection(0) = values[0];
        projection(1) = values[1];
    }
};

void carODE(const ompl::control::ODESolver::StateType& q, const ompl::control::Control *control,
                 ompl::control::ODESolver::StateType& qdot)
{
    // TODO: Fill in the ODE for the pendulum's dynamics
    void ODE(const oc::ODESolver::StateType &q, const oc::Control* u, oc::ODESolver::StateType& qdot)
    //q is state vector, u is control, qdot is output config,
    {
     // Retrieve control values. pendulum theta is the first value
        const double *u = c->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
        const double omega = u[0];
        const double acceleration = u[1];
        // Retrieve the current orientation of the pendulum.  The memory for ompl::base::SE2StateSpace is mapped as:
        // 0: theta
        const double x = q[0];
        const double y = q[1]; // not sure if q has a [1]
        const double theta = q[2];
        const double velocity = q[3];
    
        // Ensure qdot is the same size as q.  Zero out all values.
        qdot.resize(q.size(), 0);
        // ode formula:
        qdot[0] = velocity * cos(theta);  // x-dot = velocity * cos(theta)
        qdot[1] = velocity * sin(theta);  // y-dot = velocoty * sin(theta)
        qdot[2] = omega;  // theta-dot = omega
        qdot[3] = acceleration;  // velocity-dot = acceleration
    }
}

void makeStreet(std::vector<Rectangle> & /* obstacles */)
{
    // TODO: Fill in the vector of rectangles with your street environment.
}

ompl::control::SimpleSetupPtr createCar(std::vector<Rectangle> & /* obstacles */)
{
    // TODO: Create and setup the car's state space, control space, validity checker, everything you need for planning.
    auto space(std::make_shared<ompl::base::CompoundStateSpace>());
    space->addSubspace(std::make_shared<ompl::base::SE2StateSpace>(), 1.0d); // x,y,heading
    space->addSubspace(std::make_shared<ompl::base::RealVectorSpace>(1), 1.0d); // velocity
    auto cspace(std::make_shared<ompl::control::RealVectorControlSpace>(space, 2)); // turn speed, velocity
    
    auto si(std::make_shared<ompl::base::SpaceInformation>(space));
    auto ss(std::make_shared<ompl::control::SimpleSetup>(si));

    si->setStateValidityChecker(
        [obstacles](const ompl::base::State *state) {
            return true;
        }
    );
    si->setup();
}

void planCar(ompl::control::SimpleSetupPtr &/* ss */, int /* choice */)
{
    // TODO: Do some motion planning for the car
    // choice is what planner to use.
}

void benchmarkCar(ompl::control::SimpleSetupPtr &/* ss */)
{
    // TODO: Do some benchmarking for the car
}

int main(int /* argc */, char ** /* argv */)
{
    std::vector<Rectangle> obstacles;
    makeStreet(obstacles);

    int choice;
    do
    {
        std::cout << "Plan or Benchmark? " << std::endl;
        std::cout << " (1) Plan" << std::endl;
        std::cout << " (2) Benchmark" << std::endl;

        std::cin >> choice;
    } while (choice < 1 || choice > 2);

    ompl::control::SimpleSetupPtr ss = createCar(obstacles);

    // Planning
    if (choice == 1)
    {
        int planner;
        do
        {
            std::cout << "What Planner? " << std::endl;
            std::cout << " (1) RRT" << std::endl;
            std::cout << " (2) KPIECE1" << std::endl;
            std::cout << " (3) RG-RRT" << std::endl;

            std::cin >> planner;
        } while (planner < 1 || planner > 3);

        planCar(ss, planner);
    }
    // Benchmarking
    else if (choice == 2)
        benchmarkCar(ss);

    else
        std::cerr << "How did you get here? Invalid choice." << std::endl;

    return 0;
}
