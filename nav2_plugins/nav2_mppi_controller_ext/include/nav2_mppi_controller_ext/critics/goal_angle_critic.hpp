// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef aNAV2_MPPI_CONTROLLER_EXT__CRITICS__GOAL_ANGLE_CRITIC_HPP_
#define aNAV2_MPPI_CONTROLLER_EXT__CRITICS__GOAL_ANGLE_CRITIC_HPP_

#include "nav2_mppi_controller_ext/critic_function.hpp"
#include "nav2_mppi_controller_ext/models/state.hpp"
#include "nav2_mppi_controller_ext/tools/utils.hpp"

namespace mppi::critics
{

/**
 * @class mppi::critics::ConstraintCritic
 * @brief Critic objective function for driving towards goal orientation
 */
class GoalAngleCritic : public CriticFunction
{
public:
  /**
    * @brief Initialize critic
    */
  void initialize() override;

  /**
   * @brief Evaluate cost related to robot orientation at goal pose
   * (considered only if robot near last goal in current plan)
   *
   * @param costs [out] add goal angle cost values to this tensor
   */
  void score(CriticData & data) override;

protected:
  float threshold_to_consider_{0};
  unsigned int power_{0};
  float weight_{0};
};

}  // namespace mppi::critics

#endif  // aNAV2_MPPI_CONTROLLER_EXT__CRITICS__GOAL_ANGLE_CRITIC_HPP_