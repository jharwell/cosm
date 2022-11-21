/**
 * \file rlos_perception_subsystem.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>
#include <utility>

#include "cosm/subsystem/perception/rlos/config/rlos_config.hpp"
#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::subsystem::perception::rlos {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class rlos_perception_subsystem
 * \ingroup subsystem perception rlos
 *
 * \brief Base class for Line-Of-Sight (LOS) based robot perception without
 * memory (reactive behaviors only).
 */
template<typename TLOS>
class rlos_perception_subsystem {
 public:
  explicit rlos_perception_subsystem(const csprlos::config::rlos_config* const config)
      : mc_config(*config) {}

  virtual ~rlos_perception_subsystem(void) = default;

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

  const csprlos::config::rlos_config* config(void) const { return &mc_config; }

 private:
  /* clang-format off */
  const csprlos::config::rlos_config mc_config;

  std::unique_ptr<TLOS>              m_los{nullptr};
  /* clang-format on */
};

} /* namespace cosm::subsystem::perception::rlos */
