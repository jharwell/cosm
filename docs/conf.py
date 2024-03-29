#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# COSM documentation build configuration file, created by
# sphinx-quickstart on Mon Oct 14 19:20:56 2019.
#
# This file is execfile()d with the current directory set to its
# containing dir.
#
# Note that not all possible configuration values are present in this
# autogenerated file.
#
# All configuration values have a default; values that are commented out
# serve to show the default.

# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.
#
import os
import sys
import textwrap
import pathlib
import subprocess

# -- Path setup --------------------------------------------------------------

# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.
#
sys.path.insert(0, os.path.abspath('..'))

# -- Project information -----------------------------------------------------

project = 'COSM'
copyright = '2022, John Harwell'
author = 'John Harwell'

version_major = subprocess.run(("grep "
                                "PROJECT_VERSION_MAJOR "
                                "../cmake/project-local.cmake |"
                                "grep -Eo [0-9]+"),
                               shell=True,
                               check=True,
                               stdout=subprocess.PIPE).stdout.decode().strip('\n')
version_minor = subprocess.run(("grep "
                                "PROJECT_VERSION_MINOR "
                                "../cmake/project-local.cmake |"
                                "grep -Eo [0-9]+"),
                               shell=True,
                               check=True,
                               stdout=subprocess.PIPE).stdout.decode().strip('\n')
version_patch = subprocess.run(("grep "
                                "PROJECT_VERSION_PATCH "
                                "../cmake/project-local.cmake |"
                                "grep -Eo [0-9]+"),
                               shell=True,
                               check=True,
                               stdout=subprocess.PIPE).stdout.decode().strip('\n')

# The version info for the project you're documenting, acts as replacement for
# |version| and |release|, also used in various other places throughout the
# built documents.
#
# The short X.Y version.
version = f'{version_major}.{version_minor}.{version_patch}'

# The full version, including alpha/beta/rc tags.
stdout = subprocess.run(['git', 'rev-parse', '--abbrev-ref', 'HEAD'],
                        stdout=subprocess.PIPE,
                        check=True).stdout
git_branch = stdout.decode("ascii").strip('\n')

if git_branch == 'devel':
    version = f'{version}.beta'

cosm_root = pathlib.Path(os.path.abspath(".."))
breathe_projects = {
    "COSM": str(cosm_root / "build/docs/cosm/xml")
}
breathe_default_project = "COSM"
paths = []
for root, subs, files in os.walk(str(cosm_root / "include/")):
    for f in files:
        paths.append(os.path.abspath(os.path.join(root, f)))

breathe_projects_source = {"COSM": (str(cosm_root / "include"), paths)}


exhale_args = {
    "containmentFolder": "./_api",
    "rootFileName": "api.rst",
    "rootFileTitle": "COSM API",
    "afterTitleDescription": textwrap.dedent('''
       .. note::

          Some functions declared+defined within classes using variadic macros
          might not be included, due to limits in the doxygen+breathe+exhale
          toolchain. Such functions are usually found in classes which wrap or
          decorate large portions of a base class functionality/member variable
          functionality.  '''),

    "doxygenStripFromPath": "../include",
    "verboseBuild": False,
    'exhaleExecutesDoxygen': False,
    "createTreeView": True
}

doxylink = {
    'cosm': (str(cosm_root / 'build/docs/cosm/COSM.tag'),
             str(cosm_root / 'build/docs/cosm/html/'))
}

# -- General configuration ------------------------------------------------

# If your documentation needs a minimal Sphinx version, state it here.
#
# needs_sphinx = '1.0'

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = ['sphinx.ext.intersphinx',
              'sphinx.ext.todo',
              'sphinx.ext.coverage',
              'sphinx.ext.mathjax',
              'sphinx.ext.ifconfig',
              'sphinxcontrib.doxylink',
              'breathe',
              'exhale',
              'sphinx_rtd_theme',
              'sphinx_last_updated_by_git']

# Add any paths that contain templates here, relative to this directory.
templates_path = ['_templates']

# The suffix(es) of source filenames.
# You can specify multiple suffix as a list of string:
#
# source_suffix = ['.rst', '.md']
source_suffix = '.rst'

# The master toctree document.
master_doc = 'index'

# The language for content autogenerated by Sphinx. Refer to documentation
# for a list of supported languages.
#
# This is also used if you do content translation via gettext catalogs.
# Usually you set "language" from the command line for these cases.
language = None

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This patterns also effect to html_static_path and html_extra_path
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']

# The name of the Pygments (syntax highlighting) style to use.
pygments_style = 'sphinx'

# If true, `todo` and `todoList` produce output, else they produce nothing.
todo_include_todos = True


xref_links = {
    "Auer2002": ("Auer2002", "https://link.springer.com/article/10.1023/A:1013689704352"),
    "Pini2011": ("Pini2011", "https://link.springer.com/article/10.1007/s11721-011-0060-1"),
    "Brutschy2014": ("Brutschy2014", "https://link.springer.com/article/10.1007/s10458-012-9212-y"),
    "Arkin1987": ("Arkin1987", "https://ieeexplore.ieee.org/document/1088037"),
    "SteeringTutorial": ("Steering Tutorial", "https://gamedevelopment.tutsplus.com/tutorials/search/Steering+Behaviors"),
    "Harwell2020a-demystify": ("Harwell2020a", "http://ifaamas.org/Proceedings/aamas2020/pdfs/p474.pdf")
}

# -- Options for HTML output ----------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
#
html_theme = 'sphinx_rtd_theme'

# Theme options are theme-specific and customize the look and feel of a theme
# further.  For a list of options available for each theme, see the
# documentation.
#
# html_theme_options = {}

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = ['_static']

# Custom sidebar templates, must be a dictionary that maps document names
# to template names.
#
# This is required for the alabaster theme
# refs: http://alabaster.readthedocs.io/en/latest/installation.html#sidebars
html_sidebars = {
    '**': [
        'relations.html',  # needs 'show_related': True theme option to display
        'searchbox.html',
    ]
}


# Example configuration for intersphinx: refer to the Python standard library.
intersphinx_mapping = {'libra': ('https://jharwell.github.io/libra/',
                                 None),
                       'rcsw': ('https://jharwell.github.io/rcsw/',
                                None),
                       'rcppsw': ('https://jharwell.github.io/rcppsw/',
                                  None)}
