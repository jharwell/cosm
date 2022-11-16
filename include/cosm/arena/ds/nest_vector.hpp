/**
 * \file nest_vector.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <vector>
#include <memory>

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::repr { class nest; }

NS_START(cosm, arena, ds);

using nest_vectorno_type = crepr::nest*;
using nest_vectorro_type = const crepr::nest*;

/*******************************************************************************
 * Type Definitions
 ******************************************************************************/
/**
 * \class nest_vectorno
 * \ingroup arena ds
 *
 * \brief Specialization of \ref std::vector indicating the nests are NOTO owned
 * by this class.
 *
 * Has a \ref to_str() method for more convenient debugging.
 */
class nest_vectorno : public std::vector<nest_vectorno_type> {
 public:
  using std::vector<nest_vectorno_type>::vector;
  using value_type = std::vector<nest_vectorno_type>::value_type;

  /**
   * \brief Get a string representation of the vector contents.
   */
  std::string to_str(void) const;
};


/**
 * \class nest_vectorro
 * \ingroup arena ds
 *
 * \brief Specialization of \ref std::vector indicating the nests are NOT owned
 * by this class and access is also read-only.
 *
 * Has a \ref to_str() method for more convenient debugging.
 */
class nest_vectorro : public std::vector<nest_vectorro_type> {
 public:
  using std::vector<nest_vectorro_type>::vector;
  using value_type = std::vector<nest_vectorro_type>::value_type;

  /**
   * \brief Get a string representation of the vector contents.
   */
  std::string to_str(void) const;
};

NS_END(ds, arena, cosm);

