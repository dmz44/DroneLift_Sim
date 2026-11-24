# Sphinx configuration for the DroneLift reviewer simulator site.
project = "DroneLift Reviewer Simulator"
author = "Anonymous authors"
copyright = "2026, Anonymous authors"
release = "1.0"

extensions = [
    "sphinx.ext.mathjax",
    "sphinx.ext.githubpages",  # writes .nojekyll so _static is served on github.io
]

exclude_patterns = ["_build", "Thumbs.db", ".DS_Store"]

html_theme = "sphinx_rtd_theme"
html_title = "DroneLift Reviewer Simulator"
html_static_path = ["_static"]
html_css_files = ["dronelift/sim.css"]
html_js_files = ["dronelift/sim-core.js", "dronelift/sim-ui.js"]
html_show_sourcelink = False
html_theme_options = {
    "navigation_depth": 3,
    "collapse_navigation": False,
    "style_external_links": True,
    "prev_next_buttons_location": "bottom",
}
# Keep the review anonymous: no "Edit on GitHub" link, no repository context.
html_context = {"display_github": False}
