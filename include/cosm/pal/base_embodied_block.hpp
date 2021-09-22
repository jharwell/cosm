/**
 * \file base_embodied_block.hpp
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

#ifndef INCLUDE_COSM_PAL_BASE_EMBODIED_BLOCK_HPP_
#define INCLUDE_COSM_PAL_BASE_EMBODIED_BLOCK_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/cosm.hpp"
#include "cosm/repr/embodied_entity.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, pal);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/

class base_embodied_block : public crepr::embodied_entity {
 public:
  base_embodied_block(void);

  /* Not move/copy constructable/assignable by default */
  base_embodied_block(const base_embodied_block&) = delete;
  base_embodied_block& operator=(const base_embodied_block&) = delete;
  base_embodied_block(base_embodied_block&&) = delete;
  base_embodied_block& operator=(base_embodied_block&&) = delete;

 private:
  /* clang-format off */
  /* clang-format on */
};

NS_END(pal, cosm);

#endif /* INCLUDE_COSM_PAL_BASE_EMBODIED_BLOCK_HPP_ */
