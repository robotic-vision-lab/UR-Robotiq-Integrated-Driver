# Configuration file for the Sphinx documentation builder.
#
# This file only contains a selection of the most common options. For a full
# list see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Path setup --------------------------------------------------------------

# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.
#
# import os

import sys
sys.path.append('/root/catkin_ws/devel/lib/python3/dist-packages/rvl_robotiq_controller/msg/')
sys.path.append('/root/catkin_ws/src/rvl_ur_robotiq/rvl_robotiq_controller/src/rvl_robotiq_controller')
sys.path.append('/root/catkin_ws/src/rvl_ur_robotiq/rvl_ur_remote_dashboard/src/')

# -- Project information -----------------------------------------------------

project = 'RVL UR-Robotiq Integrated Driver Documentation'
copyright = '2022, Minh Tram'
author = 'Minh Tram'

# The full version, including alpha/beta/rc tags
version = '0.0.1-alpha'
release = '0.0.1-alpha'

# -- General configuration ---------------------------------------------------

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.doctest',
    'sphinx.ext.intersphinx',
    'sphinx.ext.todo',
    'sphinx.ext.coverage',
    'sphinx.ext.mathjax',
    'sphinx.ext.ifconfig',
    'sphinx.ext.viewcode',
    'sphinx.ext.githubpages',
    'myst_parser',
    'sphinx.ext.napoleon'
]

myst_enable_extensions = [
  "colon_fence",
]

# Add any paths that contain templates here, relative to this directory.
templates_path = ['_templates']

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = []

# hide annoying duplicated label warning
# suppress_warnings = ['autosectionlabel.*']

# TODOS
todo_include_todos = True
todo_emit_warnings = True

# include constructors
def skip(app, what, name, obj, would_skip, options):
    if name == "__init__":
        return False
    return would_skip

def setup(app):
    app.connect("autodoc-skip-member", skip)

# -- Options for HTML output -------------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
#
html_theme = 'sphinx_rtd_theme'

# HTML theme options
html_theme_options = {
    # 'analytics_id': 'G-XXXXXXXXXX',  #  Provided by Google in your dashboard
    # 'analytics_anonymize_ip': False,
    'logo_only': False,
    'display_version': True,
    'prev_next_buttons_location': 'bottom',
    # 'style_external_links': False,
    # 'vcs_pageview_mode': '',
    # 'style_nav_header_background': 'blue',
    # Toc options
    'collapse_navigation': True,
    'sticky_navigation': True,
    'navigation_depth': 4,
    'includehidden': True,
    'titles_only': False
}

# GIF support for HTML?
from sphinx.builders.html import StandaloneHTMLBuilder
StandaloneHTMLBuilder.supported_image_types = [
    'image/svg+xml',
    'image/gif',
    'image/png',
    'image/jpeg'
]

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = ['_static']

# These paths are either relative to html_static_path
# or fully qualified paths (eg. https://...)
html_css_files = [
    'css/custom.css',
]

# ----------------------------- LaTeX PDF Output ----------------------------- #

latex_engine = 'pdflatex'

latex_elements = {
     'preamble': r'''
        \usepackage{makeidx} 
        \usepackage[columns=1]{idxlayout} 
        \makeindex
        '''
}

latex_documents = [('index', 'rvl_driver_documentation.tex', project, author, 'manual')]