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
import os
import sys
# Include top-level folder for autodoc to find the modules
sys.path.insert(0, os.path.abspath("../../"))


# -- Project information -----------------------------------------------------

project = 'PyLabware'
copyright = 'Cronin Group, 2021'
author = 'Sergey Zalesskiy'


# -- General configuration ---------------------------------------------------

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.viewcode',
    'sphinx.ext.napoleon',
    'sphinx_autodoc_typehints',
    'sphinx.ext.todo'
]

# Add any paths that contain templates here, relative to this directory.
templates_path = ['_templates']

# The language for content autogenerated by Sphinx. Refer to documentation
# for a list of supported languages.
#
# This is also used if you do content translation via gettext catalogs.
# Usually you set "language" from the command line for these cases.
language = 'en'

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']


# -- Options for HTML output -------------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
#
html_theme = 'sphinx_rtd_theme'

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
#html_static_path = ['_static']

# Logo options
html_logo = 'images/_static/logo_white_200px.png'

# -- Options for autodoc extension -------------------------------------------
# Include both class and __init__() docstring into class description
autoclass_content = "both"

# Show type hints in description of function/method
autodoc_typehints = "description"

# -- Options for todo extension -------------------------------------------

# If this is True, todo and todolist produce output, else they produce nothing. The default is False.
todo_include_todos = False

# If this is True, todo emits a warning for each TODO entries. The default is False.
todo_emit_warnings = True

# -- Options for autodoc-typehints extension --------------------------------

# If True, set typing.TYPE_CHECKING to True to enable "expensive" typing imports.
# The default is False.
set_type_checking_flag = False
# If True, class names are always fully qualified (e.g. module.for.Class).
# If False, just the class name displays (e.g. Class). The default is False.
typehints_fully_qualified = False
# If False, do not add type info for undocumented parameters. If True, add stub
# documentation for undocumented parameters to be able to add type info.
# The default is False.
always_document_param_types = False
# If False, never add an :rtype: directive. If True, add the :rtype: directive
# if no existing :rtype: is found. The default is True.
typehints_document_rtype = False
