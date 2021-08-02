/**
 * \file base_perception_model.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_SUBSYSTEM_PERCEPTION_BASE_PERCEPTION_MODEL_HPP_
#define INCLUDE_COSM_SUBSYSTEM_PERCEPTION_BASE_PERCEPTION_MODEL_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, subsystem, perception);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class base_perception_model
 * \ingroup subsystem perception
 *
 * \brief The base class from which all robot perception models derived.
 */
class base_perception_model {
 public:
  base_perception_model(void) = default;
  virtual ~base_perception_model(void) = default;

  /* Not move/copy constructable/assignable by default */
  base_perception_model(const base_perception_model&) = delete;
  base_perception_model& operator=(const base_perception_model&) = delete;
  base_perception_model(base_perception_model&&) = delete;
  base_perception_model& operator=(base_perception_model&&) = delete;
};

NS_END(perception, subsystem, cosm);

#endif /* INCLUDE_COSM_SUBSYSTEM_PERCEPTION_BASE_PERCEPTION_MODEL_HPP_ */
