///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: Ian Rundle, Noah Spector, Sam Sarver
//////////////////////////////////////

#ifndef RGRRT_H
#define RGRRT_H

#include "ompl/control/planners/PlannerIncludes.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/control/spaces/RealVectorControlSpace.h"

namespace ompl
{
    namespace control
    {

        class RGRRT : public ompl::base::Planner
        {
        public:
            /** \brief Constructor */
            RGRRT(const SpaceInformationPtr &si);

            ~RGRRT() override;

            /** \brief Continue solving for some amount of time. Return true if solution was found. */
            ompl::base::PlannerStatus solve(const ompl::base::PlannerTerminationCondition &ptc) override;

            /** \brief Clear datastructures. Call this function if the
                input data to the planner has changed and you do not
                want to continue planning */
            void clear() override;

            /** In the process of randomly selecting states in the state
                space to attempt to go towards, the algorithm may in fact
                choose the actual goal state, if it knows it, with some
                probability. This probability is a real number between 0.0
                and 1.0; its value should usually be around 0.05 and
                should not be too large. It is probably a good idea to use
                the default value. */
            void setGoalBias(double goalBias)
            {
                goalBias_ = goalBias;
            }

            /** \brief Get the goal bias the planner is using */
            double getGoalBias() const
            {
                return goalBias_;
            }

            /** \brief Return true if the intermediate states generated along motions are to be added to the tree itself
             */
            bool getIntermediateStates() const
            {
                return addIntermediateStates_;
            }

            /** \brief Specify whether the intermediate states generated along motions are to be added to the tree
             * itself */
            void setIntermediateStates(bool addIntermediateStates)
            {
                addIntermediateStates_ = addIntermediateStates;
            }

            void getPlannerData(ompl::base::PlannerData &data) const override;

            /** \brief Set a different nearest neighbors datastructure */
            template <template <typename T> class NN>
            void setNearestNeighbors()
            {
                if (nn_ && nn_->size() != 0)
                    OMPL_WARN("Calling setNearestNeighbors will clear all states.");
                clear();
                nn_ = std::make_shared<NN<Motion *>>();
                setup();
            }

            void setup() override;

        protected:
            /** \brief Representation of a motion

                This only contains pointers to parent motions as we
                only need to go backwards in the tree. */
            class Motion
            {
            public:
                Motion() = default;

                /** \brief Constructor that allocates memory for the state and the control */
                Motion(const SpaceInformation *si)
                  : state(si->allocState()), control(si->allocControl())
                {
                }

                ~Motion() = default;

                /** \brief The state contained by the motion */
                ompl::base::State *state{nullptr};

                /** \brief The control contained by the motion */
                Control *control{nullptr};

                /** \brief The number of steps the control is applied for */
                unsigned int steps{0};

                /** \brief The parent motion in the exploration tree */
                Motion *parent{nullptr};

                /** \brief The vector of reachable states */
                std::vector<ompl::base::State *> reachables;

                void constructReachables(const SpaceInformation *si, const SpaceInformation *siC)
                {
                    for (int i = 0; i <= 10; i++) {
                        ompl::base::State *s = si->allocState();
                        ompl::control::Control *c = siC->allocControl();
                        const std::vector<double>& low = siC->getControlSpace()->as<ompl::control::RealVectorControlSpace>()->getBounds().low;
                        const std::vector<double>& high = siC->getControlSpace()->as<ompl::control::RealVectorControlSpace>()->getBounds().high;
                        auto *rcontrol = static_cast<RealVectorControlSpace::ControlType *>(c);
                        rcontrol->values[0] = low[0] + (high[0] - low[0]) * (double)(i / 10.0);
                        si->propagate(state, c, 1, s);
                        reachables.push_back(s);
                        std::cout << *(rcontrol->values) << std::endl;
                    }
                }
            };

            /** \brief Free the memory allocated by this planner */
            void freeMemory();

            /** \brief Compute distance between motions (actually distance between contained states) */
            double distanceFunction(const Motion *a, const Motion *b) const
            {
                return si_->distance(a->state, b->state);
            }

            /** \brief State sampler */
            ompl::base::StateSamplerPtr sampler_;

            /** \brief Control sampler */
            DirectedControlSamplerPtr controlSampler_;

            /** \brief The base::SpaceInformation cast as control::SpaceInformation, for convenience */
            const SpaceInformation *siC_;

            /** \brief A nearest-neighbors datastructure containing the tree of motions */
            std::shared_ptr<NearestNeighbors<Motion *>> nn_;

            /** \brief The fraction of time the goal is picked as the state to expand towards (if such a state is
             * available) */
            double goalBias_{0.05};

            /** \brief Flag indicating whether intermediate states are added to the built tree of motions */
            bool addIntermediateStates_{false};

            /** \brief The random number generator */
            RNG rng_;

            /** \brief The most recent goal motion.  Used for PlannerData computation */
            Motion *lastGoalMotion_{nullptr};
        };

    }  // namespace control 
}  // namespace ompl

#endif


