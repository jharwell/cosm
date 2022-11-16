/**
 * \file base_embodied_block.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

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
