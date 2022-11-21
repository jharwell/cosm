/**
 * \file base_subsystem.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <variant>

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::hal::subsystem {

/*******************************************************************************
 * Struct Definitions
 ******************************************************************************/
struct reset_visitor {
  template <typename TSAA>
  void operator()(TSAA& saa) const noexcept { saa.reset(); }
};

struct disable_visitor {
  template <typename TSAA>
  void operator()(TSAA& saa) const noexcept { saa.disable(); }
};

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
class base_subsystem {
 public:
  base_subsystem(void) = default;
  virtual ~base_subsystem(void) = default;

  /* Not copy constructable/assignable by default */
  base_subsystem(const base_subsystem&) = delete;
  base_subsystem& operator=(const base_subsystem&) = delete;
  base_subsystem(base_subsystem&&) = default;
  base_subsystem& operator=(base_subsystem&&) = default;

  template<typename TCollection>
  void reset(TCollection& collection) {
    for (auto& a : collection) {
      std::visit(reset_visitor(), a.second);
    } /* for(&a..) */
  }

  template<typename TCollection>
  void disable(TCollection& collection) {
    for (auto& a : collection) {
      std::visit(reset_visitor(), a.second);
      std::visit(disable_visitor(), a.second);
    } /* for(&a..) */
  }
};


/*******************************************************************************
 * Macros
 ******************************************************************************/
/**
 * \def COSM_HAL_SAA_ACCESSOR(category, Typelist, type, name, ...)
 *
 * \param SAACategoryAccessor A templated function available at class scope
 *                            which can be used to returned a handle to a
 *                            specific \p type.
 *
 * \param type The type of the sensor/actuator which will be returned by \p
 *             AccessorName.
 *
 * \param name The name that the accessor function should have.
 *
 * \c const can be passed as an additional argument to make the accessor const.
 */
#define COSM_HAL_SAA_ACCESSOR(SAACategoryAccessor, type, name, ...) \
  __VA_ARGS__ type* name(void) __VA_ARGS__ {                            \
    return SAACategoryAccessor<type>();                                 \
  }                                                                     \

} /* namespace cosm::hal::subsystem */
