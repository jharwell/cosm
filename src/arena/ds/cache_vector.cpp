/**
 * \file cache_vector.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/arena/ds/cache_vector.hpp"

#include "cosm/arena/repr/arena_cache.hpp"
#include "cosm/ds/utils.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, arena, ds);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::string acache_vectoro::to_str(void) const {
  return cds::to_string(decoratee(), "c");
} /* to_str() */

std::string acache_vectorno::to_str(void) const {
  return cds::to_string(decoratee(), "c");
} /* to_str() */

std::string acache_vectorro::to_str(void) const {
  return cds::to_string(decoratee(), "c");
} /* to_str() */

std::string bcache_vectorno::to_str(void) const {
  return cds::to_string(decoratee(), "c");
} /* to_str() */

std::string bcache_vectorro::to_str(void) const {
  return cds::to_string(decoratee(), "c");
} /* to_str() */

NS_END(ds, arena, cosm);
