.. SPDX-License-Identifier:  MIT

================================
Welcome to COSM's documentation!
================================

Overview
========

Core Swarm (COSM) is a middleware-esque library providing:

- A :ref:`ln-cosm-support-pal`: A common API to different platforms (ARgoS, ROS,
  etc) which client applications can write to.

- A see :ref:`ln-cosm-support-hal`: A common API to different agent/robot types
  which client applications to write to.

- A collection of reusable algorithms and scaffolding to maximize reuse across
  MAS projects and reduce development time (see :ref:`ln-cosm-modules` for a
  very brief overview).

.. toctree::
   :maxdepth: 2
   :caption: Getting Started

   src/modules.rst
   src/support.rst
   src/setup/index.rst
   src/usage/index.rst
   _api/api.rst

.. toctree::
   :maxdepth: 2
   :caption: Extending COSM

   src/extend/hal.rst
