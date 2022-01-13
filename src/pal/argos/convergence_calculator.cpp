/**
 * \file convergence_calculator.cpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * This file is part of COSM.
 *
 * COSM is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * COSM is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * COSM.  If not, see <http://www.gnu.org/licenses/
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/pal/argos/convergence_calculator.hpp"


#include "rcppsw/algorithm/closest_pair2D.hpp"
#include "rcppsw/math/vector2.hpp"

#include "cosm/pal/argos/controller/adaptor2D.hpp"
#include "cosm/pal/argos/controller/adaptorQ3D.hpp"
#include "cosm/pal/argos/swarm_iterator.hpp"
#include "cosm/hal/robot.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, pal, argos);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
template <class TController>
convergence_calculator<TController>::convergence_calculator(
    const cconvconfig::convergence_config* config,
    cpargos::swarm_manager_adaptor* sm)
    : ER_CLIENT_INIT("cosm.pal.argos.convergence_calculator"),
      decorator(config),
      m_sm(sm) {
  if (config->ang_order.enable) {
    decoratee().angular_order_init(
        std::bind(&convergence_calculator::calc_robot_headings2D,
                  this,
                  std::placeholders::_1));
  }
  if (config->interactivity.enable) {
    decoratee().interactivity_init(
        std::bind(&convergence_calculator::calc_robot_nn,
                  this,
                  std::placeholders::_1));
  }
  if (config->pos_entropy.enable) {
    decoratee().positional_entropy_init(
        std::bind(&convergence_calculator::calc_robot_positions,
                  this,
                  std::placeholders::_1));
  }
  if (config->velocity.enable) {
    decoratee().velocity_init(
        std::bind(&convergence_calculator::calc_robot_positions,
                  this,
                  std::placeholders::_1));
  }
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
template <class TController>
std::vector<double> convergence_calculator<TController>::calc_robot_nn(
    RCPPSW_UNUSED size_t n_threads) const {
  std::vector<rmath::vector2d> v;
  auto cb = [&](auto* robot) {
    v.push_back({ robot->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                  robot->GetEmbodiedEntity().GetOriginAnchor().Position.GetY() });
  };
  cpargos::swarm_iterator::robots<cpal::iteration_order::ekSTATIC>(
      m_sm, cb, cpal::kRobotType);

  /*
   * For each closest pair of robots we find, we add the corresponding distance
   * TWICE to our results vector, because 2 robots i and j are each other's
   * closest robots (if they were not, they would not have been returned by the
   * algorithm).
   */
  std::vector<double> res;
  size_t n_robots = m_sm->GetSpace().GetEntitiesByType(cpal::kRobotType).size();

#pragma omp parallel for num_threads(n_threads)
  for (size_t i = 0; i < n_robots / 2; ++i) {
    auto calculator = ralg::closest_pair2D<rmath::vector2d>();
    auto pts = calculator.operator()<decltype(&rmath::l2norm<rmath::vector2d>)>("recursive",
                                                                                v,
                                                                                rmath::l2norm);
    size_t old = v.size();
#pragma omp critical
    {
      v.erase(std::remove_if(v.begin(),
                             v.end(),
                             [&](const auto& pt) {
                               return pt == pts.p1 || pt == pts.p2;
                             }),
              v.end());

      ER_ASSERT(old == v.size() + 2,
                "Closest pair of points not removed from set");
      res.push_back(pts.dist);
      res.push_back(pts.dist);
    }
  } /* for(i..) */

  return res;
} /* calc_robot_nn() */

template <class TController>
std::vector<rmath::radians>
convergence_calculator<TController>::calc_robot_headings2D(size_t) const {
  std::vector<rmath::radians> v;

  auto cb = [&](const auto* controller) { v.push_back(controller->heading2D()); };
  cpargos::swarm_iterator::controllers<TController,
                                          cpal::iteration_order::ekSTATIC>(
      m_sm, cb, cpal::kRobotType);
  return v;
} /* calc_robot_headings2D() */

template <class TController>
std::vector<rmath::vector2d>
convergence_calculator<TController>::calc_robot_positions(size_t) const {
  std::vector<rmath::vector2d> v;

  auto cb = [&](const auto* controller) { v.push_back(controller->rpos2D()); };
  cpargos::swarm_iterator::controllers<TController,
                                          cpal::iteration_order::ekSTATIC>(
      m_sm, cb, cpal::kRobotType);
  return v;
} /* calc_robot_positions() */

/*******************************************************************************
 * Template Instantiations
 ******************************************************************************/
template class convergence_calculator<cpargos::controller::adaptor2D>;
template class convergence_calculator<cpargos::controller::adaptorQ3D>;

NS_END(argos, pal, cosm);
