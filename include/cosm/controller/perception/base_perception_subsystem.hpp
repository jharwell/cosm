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

#ifndef INCLUDE_COSM_CONTROLLER_PERCEPTION_BASE_PERCEPTION_SUBSYSTEM_HPP_
#define INCLUDE_COSM_CONTROLLER_PERCEPTION_BASE_PERCEPTION_SUBSYSTEM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>
#include <utility>

#include "cosm/controller/config/perception/perception_config.hpp"
#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, controller, perception);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class base_perception_subsystem
 * \ingroup controller perception
 *
 * \brief Base class for robot perception common to all controllers.
 */
template<typename TLOSType>
class base_perception_subsystem {
 public:
  explicit base_perception_subsystem(
      const ccontconfig::perception::perception_config* const pconfig)
      : mc_los_dim(pconfig->los_dim) {}

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
  void los(std::unique_ptr<TLOSType> los) { m_los = std::move(los); }

  /**
   * \brief Get the robot's current line-of-sight (LOS)
   */
  const TLOSType* los(void) const { return m_los.get(); }
  double los_dim(void) const { return mc_los_dim; }

 private:
  /* clang-format off */
  const double              mc_los_dim;

  std::unique_ptr<TLOSType> m_los{nullptr};
  /* clang-format on */
};

NS_END(perception, controller, cosm);

#endif /* INCLUDE_COSM_CONTROLLER_PERCEPTION_BASE_PERCEPTION_SUBSYSTEM_HPP_ */
