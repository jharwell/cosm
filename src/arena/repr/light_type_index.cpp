/**
 * \file light_type_index.cpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/arena/repr/light_type_index.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::arena::repr {

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
light_type_index::light_type_index(void)
    : m_index({ { kNest, rutils::color::kYELLOW },
                { kCache, rutils::color::kRED } }) {}

} /* namespace cosm::arena::repr */
