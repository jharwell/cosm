/**
 * \file block_carrying_controller.hpp
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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>
#include <utility>

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::repr {
class base_block3D;
} /* namespace cosm::repr */

NS_START(cosm, controller);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class block_carrying_controller
 * \ingroup controller
 *
 * \brief Defines the interface and common functionality for all controllers
 * which are capable of carrying blocks.
 */
class block_carrying_controller {
 public:
  block_carrying_controller(void) = default;
  virtual ~block_carrying_controller(void);

  /* Not copy constructable/assignable by default */
  block_carrying_controller(const block_carrying_controller&) = delete;
  const block_carrying_controller&
  operator=(const block_carrying_controller&) = delete;

  /**
   * \brief Return if the robot is currently carrying a block.
   */
  bool is_carrying_block(void) const { return nullptr != m_block; }

  /**
   * \brief Set the block that the robot is carrying. We use a unique_ptr to
   * convey that the robot owns the block it picks up from a C++ point of
   * view. In actuality it gets a clone of the block in the arena map.
   */
  void block(std::unique_ptr<crepr::base_block3D> block);

  /**
   * \brief Release the held block as part of a drop operation.
   */
  std::unique_ptr<crepr::base_block3D> block_release(void);

  void reset(void) RCPPSW_COLD;

  /**
   * \brief Return the block robot is carrying, or NULL if the robot is not
   * currently carrying a block.
   */
  const crepr::base_block3D* block(void) const { return m_block.get(); }
  crepr::base_block3D* block(void) { return m_block.get(); }

  /**
   * \brief If \c TRUE, then the robot thinks that it is on top of a block.
   *
   * On rare occasions this may be a false positive, which is why it is also
   * checked in the loop functions before passing any events to the
   * controller. One such occasion that is known to occur is the first timestep,
   * because the sensors have not yet finished initializing, and will return the
   * values that are incidentally the same as those that correspond to a block
   * being found.
   */
  virtual bool block_detected(void) const = 0;

 private:
  /* clang-format off */
  std::unique_ptr<crepr::base_block3D>  m_block{nullptr};
  /* clang-format on */
};

NS_END(controller, cosm);

