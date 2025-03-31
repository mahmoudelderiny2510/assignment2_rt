# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information


import os
import subprocess 
import sys
sys.path.insert(0, os.path.abspath('../'))

show_authors=True

project = 'assignment2_rt'
copyright = '2025, Mahmoud Elderiny'
author = 'Mahmoud Elderiny'
release = '1.0'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

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
    "sphinx.ext.napoleon",
    'sphinx.ext.inheritance_diagram',
    'breathe',
]

templates_path = ['_templates']
exclude_patterns = []



# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

highlight_language = 'c++' 
source_suffix = '.rst' 
master_doc = 'index'
html_theme ='sphinx_rtd_theme'
html_static_path = ['_static']

