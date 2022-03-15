/**
 * \file base_swarm_manager.cpp
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/pal/base_swarm_manager.hpp"

#include <sys/resource.h>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <chrono>
#include <csignal>

#include "rcppsw/math/rngm.hpp"
#include "rcppsw/er/stacktrace.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, pal);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
base_swarm_manager::base_swarm_manager(void) :
    ER_CLIENT_INIT("cosm.pal.base_swarm_manager") {
  /*
   * Enable stacktraces when we crash with a segfault.
   */
  signal(SIGSEGV, rer::sigsegv_sighandler);

  /*
   * Enable stacktraces when we crash with std::terminate().
   */
  std::set_terminate(rer::terminate_handler);

  /*
   * For some reason GNU parallel on MSI resets the core dump memory limit to 0
   * when running, even if you run "ulimit -s unlimited" right before. And on
   * development machines, it is easy to forget to run the ulimit cmd before
   * doing a test run to help catch a heisenbug in the act, necessitating
   * re-running and much gnashing of teeth.
   *
   * So, we fix the limit during initialization, so that IF we do crash, we
   * always get a core dump to help diagnose things, regardless of shell
   * configuration.
   */
  struct rlimit r {};
  getrlimit(RLIMIT_CORE, &r);
  r.rlim_cur = RLIM_INFINITY;
  setrlimit(RLIMIT_CORE, &r);

  /* verify environment variables set up for logging */
  ER_ENV_VERIFY();
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void base_swarm_manager::output_init(const cpconfig::output_config* config) {
  m_output_root = cpconfig::output_config::root_calc(config);

  if (!fs::exists(m_output_root)) {
    fs::create_directories(m_output_root);
  }
} /* output_init() */

void base_swarm_manager::rng_init(const rmath::config::rng_config* config) {
  rmath::rngm::instance().register_type<rmath::rng>("base_swarm_manager");
  if (nullptr == config || (nullptr != config && -1 == config->seed)) {
    ER_INFO("Using time seeded RNG");
    m_rng = rmath::rngm::instance().create(
        "base_swarm_manager",
        std::chrono::system_clock::now().time_since_epoch().count());
  } else {
    ER_INFO("Using user seeded RNG");
    m_rng = rmath::rngm::instance().create("base_swarm_manager", config->seed);
  }
} /* rng_init() */

NS_END(pal, cosm);
