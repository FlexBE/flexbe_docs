# Configuration file for the Sphinx documentation builder.

# -- Project information

project = 'FlexBE'
copyright = '2024, Philipp Schillinger and Christopher Newport University'
author = 'robotics@cnu.edu'

release = '0.0'
version = '0.0.1'

# -- General configuration

extensions = [
    'sphinx.ext.duration',
    'sphinx.ext.doctest',
    'sphinx.ext.autodoc',
    'sphinx.ext.autosectionlabel',
    'sphinx.ext.autosummary',
    'sphinx.ext.intersphinx',
    'sphinxcontrib.youtube'
]

intersphinx_mapping = {
    'python': ('https://docs.python.org/3/', None),
    'sphinx': ('https://www.sphinx-doc.org/en/master/', None),
}
intersphinx_disabled_domains = ['std']

templates_path = ['_templates']

# ------------ Options for HTML output -------------

html_theme = 'sphinx_rtd_theme'

html_theme_options = {
#     'logo_only': False,
#     'prev_next_buttons_location': 'bottom',
#     'style_external_links': False,
#     'vcs_pageview_mode': '',
#     'style_nav_header_background': 'white',
#     # Toc options
#     'collapse_navigation': True,
#     'sticky_navigation': True,
    'navigation_depth': 2,
    'includehidden': True,
#     'titles_only': False
}

html_logo = '../images/logo.png'

html_favicon = '../images/icon-16.png'

# If true, "Created using Sphinx" is shown in the HTML footer. Default is True.
html_show_sphinx = False

# If true, "(C) Copyright ..." is shown in the HTML footer. Default is True.
html_show_copyright = True

html_search_language = 'en'

# # A dictionary with options for the search language support, empty by default.
# # 'ja' uses this config value.
# # 'zh' user can custom change `jieba` dictionary path.
# #
# # html_search_options = {'type': 'default'}

# # The name of a javascript file (relative to the configuration directory) that
# # implements a search results scorer. If empty, the default will be used.
# #
# # html_search_scorer = 'scorer.js'

# Output file base name for HTML help builder.
htmlhelp_basename = 'FlexBE Documentation'