/**
 * \file base_perception_subsystem.hpp
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

#ifndef INCLUDE_COSM_SUBSYSTEM_PERCEPTION_BASE_PERCEPTION_SUBSYSTEM_HPP_
#define INCLUDE_COSM_SUBSYSTEM_PERCEPTION_BASE_PERCEPTION_SUBSYSTEM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>
#include <utility>

#include "cosm/subsystem/perception/config/perception_config.hpp"
#include "cosm/subsystem/perception/base_perception_model.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, subsystem, perception);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class base_perception_subsystem
 * \ingroup subsystem perception
 *
 * \brief Base class for robot perception common to all subsystems.
 */
template<typename TLOS>
class base_perception_subsystem {
 public:
  explicit base_perception_subsystem(
      const cspconfig::perception_config* const pconfig,
      std::unique_ptr<base_perception_model> model)
      : mc_los_dim(pconfig->los_dim),
        m_model(std::move(model)) {}

  virtual ~base_perception_subsystem(void) = default;

  /**
   * \brief Reset the robot's perception of the environment to an initial state
   */
  virtual void reset(void) {}

  /**
   * \brief Set the robots LOS for the next timestep.
   *
   * This is a hack to make it easy for me to run simulations, as I can computer
   * the line of sight for a robot within the loop functions, and just pass it
   * in here. In real robots this routine would be MUCH messier and harder to
   * work with.
   *
   * \param los The new los
   */
  void los(std::unique_ptr<TLOS> los) { m_los = std::move(los); }

  /**
   * \brief Get the robot's current line-of-sight (LOS)
   */
  const TLOS* los(void) const { return m_los.get(); }
  double los_dim(void) const { return mc_los_dim; }

  const base_perception_model* model(void) const { return m_model.get(); }
  base_perception_model* model(void) { return m_model.get(); }

 protected:
  void model(std::unique_ptr<base_perception_model> model) {
    m_model = std::move(model);
  }

 private:
  /* clang-format off */
  const double                           mc_los_dim;

  std::unique_ptr<TLOS>                  m_los{nullptr};
  std::unique_ptr<base_perception_model> m_model{nullptr};
  /* clang-format on */
};

NS_END(perception, subsystem, cosm);

#endif /* INCLUDE_COSM_SUBSYSTEM_PERCEPTION_BASE_PERCEPTION_SUBSYSTEM_HPP_ */
