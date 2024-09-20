project = 'Robotics Toolbox'
copyright = 'CTU'
author = 'Vladimir Petrik'

extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.autosummary',
    'sphinx.ext.napoleon',  # google and numpy docstrings
]

napoleon_google_docstring = True
napoleon_numpy_docstring = False
autodoc_typehints = "description"
autosummary_generate = True

templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']

html_theme = 'sphinx_rtd_theme'
# html_static_path = ['_static']
