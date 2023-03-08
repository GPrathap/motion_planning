// Copyright (c) 2020, Samsung Research America
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
// limitations under the License. Reserved.

#ifndef kino_planner__TYPES_HPP_
#define kino_planner__TYPES_HPP_

#include <utility>

namespace kino_planner
{

typedef std::pair<float, unsigned int> NodeHeuristicPair;

/**
 * @struct kino_planner::SearchInfo
 * @brief Search properties and penalties
 */
struct SearchInfo
{
  float minimum_turning_radius;
  float non_straight_penalty;
  float change_penalty;
  float reverse_penalty;
  float cost_penalty;
  float analytic_expansion_ratio;
  double steering_angle = 15.0;
  int steering_angle_discrete_num = 1;
  double wheel_base = 2.0;
  double segment_length = 1.6;
  int segment_length_discrete_num = 8;
  double steering_penalty = 1.5;
  double steering_change_penalty = 2.0;
  double shot_distance = 5.0;
  double grid_size_phi = 72.0;
  double vehicle_width = 4.7;
  double vehicle_length = 2.0;
  double track;
  std::string vehicle_type;
};

}  // namespace kino_planner

#endif  // kino_planner__TYPES_HPP_
