///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: Ian Rundle, Noah Spector, Sam Sarver
//////////////////////////////////////

#include <iostream>

#include <ompl/base/ProjectionEvaluator.h>

#include <ompl/control/SimpleSetup.h>
#include <ompl/control/ODESolver.h>

#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>

#include <ompl/tools/benchmark/Benchmark.h>

// Your implementation of RG-RRT
#include "RG-RRT.h"

// Your projection for the pendulum
class PendulumProjection : public ompl::base::ProjectionEvaluator
{
public:
    PendulumProjection(const ompl::base::StateSpace *space) : ProjectionEvaluator(space)
    {
    }

    unsigned int getDimension() const override
    {
        return 2;
    }

    void project(const ompl::base::State *state, Eigen::Ref<Eigen::VectorXd> projection) const override
    {
        const double *values = state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
        projection(0) = values[0];
        projection(1) = values[1];
    }
};

void pendulumODE(const ompl::control::ODESolver::StateType& q, const ompl::control::Control *control,
                 ompl::control::ODESolver::StateType& qdot)
{
    // TODO: Fill in the ODE for the pendulum's dynamics
        //q is state vector, u is control, qdot is output config,
    // void ODE(const ompl::control::ODESolver::StateType &q, const ompl::control::Control* u, ompl::control::ODESolver::StateType& qdot)
    // {
    // Retrieve control values. pendulum theta is the first value
    const double *u = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
    const double torque = u[0];
    // Retrieve the current orientation of the pendulum.  The memory for ompl::base::SE2StateSpace is mapped as:
    // 0: theta
    const double theta = q[0];
    const double omega = q[1]; // not sure if q has a [1]


    // Ensure qdot is the same size as q.  Zero out all values.
    qdot.resize(q.size(), 0);
    // ode formula:
    qdot[0] = omega;            // theta-dot = omega
    qdot[1] = -1 * 9.81 * cos(theta) + torque;  // omega-dot = -gcos(theta) + torque
    // }
}

ompl::control::SimpleSetupPtr createPendulum(double torque)
{
    // TODO: Create and setup the pendulum's state space, control space, validity checker, everything you need for
    // planning.
    auto space(std::make_shared<ompl::base::CompoundStateSpace>());
    space->addSubspace(std::make_shared<ompl::base::SO2StateSpace>(), 1.0d);
    auto speedSpace(std::make_shared<ompl::base::RealVectorStateSpace>(1));
    speedSpace->setBounds(-10,10);
    space->addSubspace(speedSpace, 1.0d);

    auto cspace(std::make_shared<ompl::control::RealVectorControlSpace>(space, 1));
    
    ompl::base::RealVectorBounds cbounds(1);
    cbounds.setLow(-torque);
    cbounds.setHigh(torque);

    cspace->setBounds(cbounds);

    auto ss(std::make_shared<ompl::control::SimpleSetup>(cspace));
    ompl::control::SpaceInformation *si = ss->getSpaceInformation().get();

    // Speed is accounted for in the bounds of the speed space
    ss->setStateValidityChecker(
        [si](const ompl::base::State *state) {
            return si->satisfiesBounds(state);
        }
    );

    ompl::base::ScopedState<ompl::base::CompoundStateSpace> start(space);
    start[0] = 0.0d;
    start[1] = 0.0d;

    ompl::base::ScopedState<ompl::base::CompoundStateSpace> goal(space);
    goal[0] = 3.14d;
    goal[1] = 0.0d;

    ss->setStartAndGoalStates(start, goal, 0.05);

    auto odeSolver(std::make_shared<ompl::control::ODEBasicSolver<>>(ss->getSpaceInformation(), &pendulumODE));
    ss->setStatePropagator(ompl::control::ODESolver::getStatePropagator(odeSolver));

    ss->setup();
    
    return ss;
}

void planPendulum(ompl::control::SimpleSetupPtr &ss, int choice)
{
    // TODO: Do some motion planning for the pendulum
    // choice is what planner to use.
    if (choice == 1) {
        auto planner(std::make_shared<ompl::control::RRT>(ss->getSpaceInformation()));
        ss->setPlanner(planner);
    } 
    else if (choice == 2) {
        auto projection(std::make_shared<PendulumProjection>(ss->getStateSpace().get()));
        auto planner(std::make_shared<ompl::control::KPIECE1>(ss->getSpaceInformation()));
        planner->setProjectionEvaluator(projection);
        ss->setPlanner(planner);
    } else if (choice == 3) {
        auto planner(std::make_shared<ompl::control::RGRRT>(ss->getSpaceInformation()));
        ss->setPlanner(planner);
    }
    ompl::base::PlannerStatus solved = ss->solve(5.0);
    if (solved)
    {
        std::cout << "Found solution:" << std::endl;

        ss->getSolutionPath().asGeometric().printAsMatrix(std::cout);
        std::cout << "Controls:" << std::endl;
        ss->getSolutionPath().printAsMatrix(std::cout);

    }
    else {
        std::cout << "No solution found" << std::endl;
    }
}

void benchmarkPendulum(ompl::control::SimpleSetupPtr &ss)
{
    // TODO: Do some benchmarking for the pendulum
    ss->setup();
    ss->print();
    ompl::tools::Benchmark b(*ss, "Benchmarking for Pendulum"); // create benchmark class
    b.addPlanner(ompl::base::PlannerPtr(new ompl::control::RGRRT(ss->getSpaceInformation())));
    b.addPlanner(ompl::base::PlannerPtr(new ompl::control::RRT(ss->getSpaceInformation()))); //add planners to evaluate
    auto projection(std::make_shared<PendulumProjection>(ss->getStateSpace().get()));
    auto planner(std::make_shared<ompl::control::KPIECE1>(ss->getSpaceInformation()));
    planner->setProjectionEvaluator(projection);
    b.addPlanner(planner);
    ompl::tools::Benchmark::Request req; // set benchmark parameters
    req.maxTime = 15;
    req.maxMem = 500;
    req.runCount = 20;
    req.displayProgress = true;
    b.benchmark(req); // saving results
    b.saveResultsToFile();
}

int main(int /* argc */, char ** /* argv */)
{
    int choice;
    do
    {
        std::cout << "Plan or Benchmark? " << std::endl;
        std::cout << " (1) Plan" << std::endl;
        std::cout << " (2) Benchmark" << std::endl;

        std::cin >> choice;
    } while (choice < 1 || choice > 2);

    int which;
    do
    {
        std::cout << "Torque? " << std::endl;
        std::cout << " (1)  3" << std::endl;
        std::cout << " (2)  5" << std::endl;
        std::cout << " (3) 10" << std::endl;

        std::cin >> which;
    } while (which < 1 || which > 3);

    double torques[] = {3., 5., 10.};
    double torque = torques[which - 1];

    ompl::control::SimpleSetupPtr ss = createPendulum(torque);

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

        planPendulum(ss, planner);
    }
    // Benchmarking
    else if (choice == 2)
        benchmarkPendulum(ss);

    else
        std::cerr << "How did you get here? Invalid choice." << std::endl;

    return 0;
}
