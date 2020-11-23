/**
 * \file argos_convergence_calculator.hpp
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

#ifndef INCLUDE_COSM_PAL_ARGOS_CONVERGENCE_CALCULATOR_HPP_
#define INCLUDE_COSM_PAL_ARGOS_CONVERGENCE_CALCULATOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>

#include "rcppsw/patterns/decorator/decorator.hpp"

#include "cosm/convergence/convergence_calculator.hpp"
#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::pal {
class argos_sm_adaptor;
class argos_controller2D_adaptor;
class argos_controllerQ3D_adaptor;
} /* namespace cosm::pal */

NS_START(cosm, pal);

/*******************************************************************************
 * Classes
 ******************************************************************************/
/**
 * \class argos_convergence_calculator
 * \ingroup pal
 *
 * \brief Implements the necessary callbacks from \ref
 * cconvergence::convergence_calculator to calculate convergence in the ARGoS
 * environment.
 */
template <class TController>
class argos_convergence_calculator final
    : public rer::client<argos_convergence_calculator<TController>>,
      public rpdecorator::decorator<cconvergence::convergence_calculator> {
 public:
  argos_convergence_calculator(const cconvconfig::convergence_config* config,
                               cpal::argos_sm_adaptor* sm) RCPPSW_COLD;
  ~argos_convergence_calculator(void) override RCPPSW_COLD = default;

  /* Not copy constructible/assignable by default */
  argos_convergence_calculator(const argos_convergence_calculator&) = delete;
  argos_convergence_calculator& operator=(const argos_convergence_calculator&) =
      delete;

  RCPPSW_DECORATE_FUNC(update);
  RCPPSW_DECORATE_FUNC(converged);
  RCPPSW_DECORATE_FUNC(reset_metrics);
  RCPPSW_DECORATE_FUNC(task_dist_entropy_init);

 private:
  std::vector<double> calc_robot_nn(uint n_threads) const;
  std::vector<rmath::radians> calc_robot_headings2D(uint n_threads) const;
  std::vector<rmath::vector2d> calc_robot_positions(uint n_threads) const;

  /* clang-format off */
  cpal::argos_sm_adaptor* m_sm;
  /* clang-format on */
};

NS_END(pal, cosm);

#endif /* INCLUDE_COSM_PAL_ARGOS_CONVERGENCE_CALCULATOR_HPP_ */
