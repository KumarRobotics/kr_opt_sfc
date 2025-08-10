#ifndef PLAN_PATH_HPP
#define PLAN_PATH_HPP

#include <ompl/util/Console.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/DiscreteMotionValidator.h>
#include <ompl/control/PathControl.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/planners/sst/SST.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/PathSimplifier.h>

#include <deque>
#include <memory>
#include <Eigen/Eigen>

namespace plan_path
{
    namespace oc = ompl::control;
    namespace ob = ompl::base;
    namespace og = ompl::geometric;

    template <typename Map>
    void shortcutPath(std::vector<Eigen::Vector3d> &p, Map* mapPtr, 
        const Eigen::Vector3d &lb, const Eigen::Vector3d &hb, 
        double threshold = 0.8)
    {
        if (p.size() <= 2)
            return;  // Nothing to shortcut

        auto isCollisionFree = [&](const Eigen::Vector3d &start, const Eigen::Vector3d &end) {
            // Simple collision check along line segment using small steps
            const int steps = 20;
            for (int i = 0; i <= steps; ++i)
            {
                double t = double(i) / steps;
                Eigen::Vector3d pos = start + t * (end - start);

                // Check bounds if needed
                if ((pos.array() < lb.array()).any() || (pos.array() > hb.array()).any())
                    return false;

                if (mapPtr->query(pos) != 0)  // Assume non-zero means obstacle
                    return false;
            }
            return true;
        };

        size_t i = 0;
        while (i < p.size() - 2)
        {
            size_t k = i + 2;
            while (k < p.size())
            {
                double directDist = (p[k] - p[i]).norm();
                double oldDist = 0;
                for (size_t idx = i; idx < k; ++idx)
                    oldDist += (p[idx + 1] - p[idx]).norm();

                if (oldDist - directDist < threshold && isCollisionFree(p[i], p[k]))
                {
                    // Remove intermediate points between i and k
                    p.erase(p.begin() + i + 1, p.begin() + k);
                    // After erasing, check from the same i again
                    k = i + 2;
                }
                else
                {
                    ++k;
                }
            }
            ++i;
        }
    }


    template <typename Map>
    inline double planPath(const Eigen::Vector3d &s,
                        const Eigen::Vector3d &g,
                        const Eigen::Vector3d &lb,
                        const Eigen::Vector3d &hb,
                        Map *mapPtr,
                        const double &timeout,
                        const double &goal_tol,
                        std::vector<Eigen::Vector3d> &p)
    {
        using namespace ompl;
        namespace ob = ompl::base;
        namespace og = ompl::geometric;

        p.clear();

        auto space = std::make_shared<ob::RealVectorStateSpace>(3);
        ob::RealVectorBounds bounds(3);
        bounds.setLow(0, 0.0);
        bounds.setHigh(0, hb(0) - lb(0));
        bounds.setLow(1, 0.0);
        bounds.setHigh(1, hb(1) - lb(1));
        bounds.setLow(2, 0.0);
        bounds.setHigh(2, hb(2) - lb(2));
        space->setBounds(bounds);

        auto si = std::make_shared<ob::SpaceInformation>(space);
        si->setStateValidityChecker(
            [&](const ob::State *state)
            {
                const auto *pos = state->as<ob::RealVectorStateSpace::StateType>();
                Eigen::Vector3d position(lb(0) + (*pos)[0],
                                        lb(1) + (*pos)[1],
                                        lb(2) + (*pos)[2]);
                return mapPtr->query(position) == 0;
            });
        si->setup();

        ob::ScopedState<> start(space), goal(space);
        start[0] = s(0) - lb(0);
        start[1] = s(1) - lb(1);
        start[2] = s(2) - lb(2);
        goal[0]  = g(0) - lb(0);
        goal[1]  = g(1) - lb(1);
        goal[2]  = g(2) - lb(2);

        auto pdef = std::make_shared<ob::ProblemDefinition>(si);
        pdef->setStartAndGoalStates(start, goal, goal_tol);

        double dis = 1.8 * (s - g).norm();
        ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
        obj->setCostThreshold(ob::Cost(dis));
        pdef->setOptimizationObjective(obj);

        auto planner = std::make_shared<og::InformedRRTstar>(si);
        planner->setProblemDefinition(pdef);
        planner->setup();
        ompl::base::PlannerStatus solved;
        solved = planner->ompl::base::Planner::solve(timeout);

        double cost = INFINITY;

        if (solved == ob::PlannerStatus::EXACT_SOLUTION)
        {
            auto pathPtr = std::dynamic_pointer_cast<og::PathGeometric>(pdef->getSolutionPath());
            std::cout << "Waypoint count before smoothing: " << pathPtr->getStateCount() << "\n";

            p.clear();
            for (size_t i = 0; i < pathPtr->getStateCount(); ++i)
            {
                const auto state = pathPtr->getState(i)->as<ob::RealVectorStateSpace::StateType>()->values;
                p.emplace_back(lb(0) + state[0], lb(1) + state[1], lb(2) + state[2]);
            }

            shortcutPath(p, mapPtr, lb, hb, 0.5);

            cost = pathPtr->cost(pdef->getOptimizationObjective()).value();
        }

        return cost;
    }


    template <typename Map>
    inline void dyplanPath(const Eigen::Matrix3d &s,
                             const Eigen::Matrix3d &g,
                             const Eigen::Vector3d &lb,
                             const Eigen::Vector3d &hb,
                             Map *mapPtr,
                             const double &timeout,
                             const double &goal_tol,
                             std::vector<Eigen::VectorXd> &p)
    {
        auto space(std::make_shared<ob::RealVectorStateSpace>(6));
        p.clear();
        //std::cout << "goal is " << g << std::endl;
        //ompl::msg::setLogLevel(ompl::msg::LOG_NONE);

        ob::RealVectorBounds bounds(6);
        bounds.setLow(0, 0.0);
        bounds.setHigh(0, hb(0) - lb(0));
        bounds.setLow(1, 0.0);
        bounds.setHigh(1, hb(1) - lb(1));
        bounds.setLow(2, 0.0);
        bounds.setHigh(2, hb(2) - lb(2));

        bounds.setLow(3, -3.0);
        bounds.setHigh(3, 3.0);
        bounds.setLow(4, -3.0);
        bounds.setHigh(4, 3.0);
        bounds.setLow(5, -3.0);
        bounds.setHigh(5, 3.0);

        space->setBounds(bounds);

        // create a control space
        auto cspace(std::make_shared<oc::RealVectorControlSpace>(space, 3));
    
        // set the bounds for the control space
        ob::RealVectorBounds cbounds(3);
        cbounds.setLow(0, -3.0);
        cbounds.setHigh(0, 3.0);
        cbounds.setLow(1, -3.0);
        cbounds.setHigh(1, 3.0);
        cbounds.setLow(2, -2.0);
        cbounds.setHigh(2, 2.0);
        cspace->setBounds(cbounds);
  
        // define a simple setup class
        oc::SimpleSetup ss(cspace);

        ss.setStateValidityChecker(
            [&](const ob::State *state)
            {
                const auto *pos = state->as<ompl::base::RealVectorStateSpace::StateType>();
                const Eigen::Vector3d position(lb(0) + (*pos)[0],
                                               lb(1) + (*pos)[1],
                                               lb(2) + (*pos)[2]);
                return mapPtr->query(position) == 0;
            });
        // si->setMotionValidator(
        //     [&](const ompl::base::State *state)
        //     {
        //         const auto *pos = state->as<ompl::base::RealVectorStateSpace::StateType>();
        //         const Eigen::Vector3d position(lb(0) + (*pos)[0],
        //                                        lb(1) + (*pos)[1],
        //                                        lb(2) + (*pos)[2]);
        //         return mapPtr->query(position) == 0;
        //     });
        ss.setStatePropagator(
            [&](const ob::State *state, const oc::Control* control,
                const double duration, ob::State *result)
            {
                Eigen::MatrixXd phi_ = Eigen::MatrixXd::Identity(6, 6);
                for (int i = 0; i < 3; ++i) phi_(i, i + 3) = duration;

                const auto* spv0   = state->as<ob::RealVectorStateSpace::StateType>();
                const auto* ctrl = control->as<oc::RealVectorControlSpace::ControlType>();

                Eigen::Matrix<double, 6, 1> pv0, pv1;
                pv0 <<  (*spv0)[0],
                        (*spv0)[1],
                        (*spv0)[2],
                        (*spv0)[3],
                        (*spv0)[4],
                        (*spv0)[5];

                const Eigen::Vector3d um((*ctrl)[0], (*ctrl)[1], (*ctrl)[2]);

                Eigen::Matrix<double, 6, 1> integral;
                integral.head(3) = 0.5 * std::pow(duration, 2) * um;
                integral.tail(3) = duration * um;

                pv1 = phi_ * pv0 + integral;

                auto* spv1 = result->as<ob::RealVectorStateSpace::StateType>();
                (*spv1)[0] = pv1(0);
                (*spv1)[1] = pv1(1);
                (*spv1)[2] = pv1(2);
                (*spv1)[3] = pv1(3);
                (*spv1)[4] = pv1(4);
                (*spv1)[5] = pv1(5);

            });
        ss.getSpaceInformation()->setMinMaxControlDuration(1, 15);
        ss.getSpaceInformation()->setPropagationStepSize(0.1);

        ompl::base::ScopedState<> start(space), goal(space);
        start[0] = s(0, 0) - lb(0);
        start[1] = s(1, 0) - lb(1);
        start[2] = s(2, 0) - lb(2);
        start[3] = s(0, 1);
        start[4] = s(1, 1);
        start[5] = s(2, 1);

        goal[0] = g(0, 0) - lb(0);
        goal[1] = g(1, 0) - lb(1);
        goal[2] = g(2, 0) - lb(2);
        goal[3] = g(0, 1);
        goal[4] = g(1, 1);
        goal[5] = g(2, 1);

    
        ss.setStartAndGoalStates(start, goal, goal_tol);
        ss.setPlanner(std::make_shared<oc::SST>(ss.getSpaceInformation()));
        //ss.setOptimizationObjective(std::make_shared<ompl::base::StateCostIntegralObjective>(ss.getSpaceInformation(), true));
        //ss.print();
        double dis = (s - g).norm();
        ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(ss.getSpaceInformation()));
        obj->setCostThreshold(ob::Cost(dis));
        ss.setOptimizationObjective(obj);
        ss.setup();
        auto solved = ss.solve(timeout);
        std::cout << "solved" << solved << std::endl;

        if (solved == ompl::base::PlannerStatus::EXACT_SOLUTION)
        {
            p.clear();

            oc::PathControl path_ = ss.getSolutionPath();

            for (size_t i = 0; i < path_.getStateCount(); i++)
            {

                const auto   state = path_.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values;
                
                if(i == path_.getStateCount() -1)
                {
                    Eigen::VectorXd wpt(10);
                    wpt << lb(0) + state[0], lb(1) + state[1], lb(2) + state[2],
                        state[3], state[4], state[5], 
                        0.0, 0.0, 0.0, 0.0;
                    p.push_back(wpt);
                }else
                {
                    const auto   acc   = path_.getControl(i)->as<oc::RealVectorControlSpace::ControlType>()->values;
                    const double dur   = path_.getControlDuration(i);

                    Eigen::VectorXd wpt(10);
                    wpt << lb(0) + state[0], lb(1) + state[1], lb(2) + state[2],
                        state[3], state[4], state[5], 
                        acc[0], acc[1], acc[2],
                        dur;
                    p.push_back(wpt);

                }
        
            }
        }

        return;
    }

}

#endif
