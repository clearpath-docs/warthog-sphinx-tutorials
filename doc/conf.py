# -*- coding: utf-8 -*-

import os
import sys
import xml.etree.ElementTree as etree

sys.path.insert(0, os.path.abspath(os.path.dirname(__file__)))

extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.extlinks',
    'sphinx.ext.doctest',
    'sphinx.ext.viewcode',
    'sphinx-prompt',
    'sphinx_substitution_extensions'
]

extlinks = {
    'roswiki': ('http://wiki.ros.org/%s', ''),
}

source_suffix = '.rst'
master_doc = 'index'

project = u'Warthog Tutorials'
copyright = u'2015, Clearpath Robotics'

# Get version number from package.xml.
tree = etree.parse('../package.xml')
version = tree.find("version").text
release = version
author = tree.find('author').text

html_theme = 'theme'
html_theme_path = ["."]

htmlhelp_basename = 'warthog_tutorialsdoc'
templates_path = ['./templates']
html_static_path = ['./theme/static']

html_sidebars = {
   '**': ['sidebartoc.html', 'sourcelink.html', 'searchbox.html']
}

# global substitutions -- THESE ONLY WORK OUTSIDE OTHER MARKUP (i.e. ``/opt/ros/|ros_distro|/`` won't work!)
rst_prolog = """
.. |ubuntu_distro| replace:: bionic
.. |ubuntu_version| replace:: 18.04
.. |ros_distro| replace:: melodic
.. |ros_path| replace:: ``/opt/ros/melodic``
.. |rosd_path| replace:: ``/etc/ros/melodic/ros.d``
.. |dpkg_s_firmware| replace:: ``dpkg -s ros-melodic-warthog-firmware``
"""

# substitutions for substitution-code-block -- THESE ONLY WORK IN CODE BLOCKS
substitutions = [
    ('|ros_distro|', 'melodic'),
    ('|ubuntu_distro|', 'bionic'),
    ('|fw_path|', '/opt/ros/melodic/share/warthog_firmware/mcu.bin'),
]

# If true, "Created using Sphinx" is shown in the HTML footer. Default is True.
html_show_sphinx = False

# The name of an image file (relative to this directory) to place at the top
# of the sidebar.
html_logo = 'graphics/clearpathlogo.png'

# The name of an image file (within the static path) to use as favicon of the
# docs.  This file should be a Windows icon file (.ico) being 16x16 or 32x32
# pixels large.
html_favicon = 'graphics/favicon.ico'

# -- Options for LaTeX output ------------------------------------------------
latex_engine = "xelatex"
latex_docclass = {
   'howto': 'clearpath-latex/clearpath-manual',
   'manual': 'clearpath-latex/clearpath-manual',
}
latex_elements = {
    # The paper size ('letterpaper' or 'a4paper').
    #
    # 'papersize': 'letterpaper',

    # The font size ('10pt', '11pt' or '12pt').
    #
    # 'pointsize': '10pt',

    # Additional stuff for the LaTeX preamble.
    #
    # 'preamble': '',

    # Latex figure (float) alignment
    #
    # 'figure_align': 'htbp',
}

# Grouping the document tree into LaTeX files. List of tuples
# (source start file, target name, title,
#  author, documentclass [howto, manual, or own class]).
latex_documents = [
    (master_doc, 'WarthogTutorials.tex', project,
     author, 'manual'),
]
