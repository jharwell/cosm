/**
 * \file base_subsystem.hpp
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

#ifndef INCLUDE_COSM_HAL_SUBSYSTEM_BASE_SUBSYSTEM_HPP_
#define INCLUDE_COSM_HAL_SUBSYSTEM_BASE_SUBSYSTEM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <boost/variant.hpp>

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, hal, subsystem);

/*******************************************************************************
 * Struct Definitions
 ******************************************************************************/
struct reset_visitor {
  template <typename TSAA>
  void operator()(TSAA& saa) const { saa.reset(); }
};

struct disable_visitor {
  template <typename TSAA>
  void operator()(TSAA& saa) const { saa.disable(); }
};

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
class base_subsystem {
 public:
  base_subsystem(void) = default;
  virtual ~base_subsystem(void) = default;


  /* Not move/copy constructable/assignable by default */
  base_subsystem(const base_subsystem&) = delete;
  base_subsystem& operator=(const base_subsystem&) = delete;
  base_subsystem(base_subsystem&&) = delete;
  base_subsystem& operator=(base_subsystem&&) = delete;

  template<typename TCollection>
  void reset(TCollection& collection) {
    for (auto& a : collection) {
      boost::apply_visitor(reset_visitor(), a.second);
    } /* for(&a..) */
  }

  template<typename TCollection>
  void disable(TCollection& collection) {
    for (auto& a : collection) {
      boost::apply_visitor(reset_visitor(), a.second);
      boost::apply_visitor(disable_visitor(), a.second);
    } /* for(&a..) */
  }
};


/*******************************************************************************
 * Macros
 ******************************************************************************/
#define COSM_HAL_SAA_ACCESSOR(category, Typelist, type, name, ...)      \
  __VA_ARGS__ type* name(void) __VA_ARGS__ {                            \
    return category<type>();                                            \
  }                                                                     \

#define COSM_HAL_SAA_ACCESSOR_VISITOR(type, ...)                        \
  template<typename U = T,                                              \
           COSM_SFINAE_DECLDEF(std::is_same<U, type>::value)>         \
  __VA_ARGS__ type* operator()(__VA_ARGS__ type& item)  __VA_ARGS__ {   \
    return &item;                                                       \
  }

NS_END(subsystem, hal, cosm);

#endif /* INCLUDE_COSM_HAL_SUBSYSTEM_BASE_SUBSYSTEM_HPP_ */
