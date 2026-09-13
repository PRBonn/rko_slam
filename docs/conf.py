import os

project = "rko_slam"
copyright = "2026 Meher V.R. Malladi"
# "master" or the tag name; the switcher matches on it and the version banner names it
version = os.environ.get("DOCS_VERSION", "master").lstrip("v")
release = version

extensions = ["myst_parser"]
myst_enable_extensions = ["colon_fence"]
myst_heading_anchors = 3
source_suffix = {".rst": "restructuredtext", ".md": "markdown"}
exclude_patterns = ["_build", "root"]

html_title = "rko_slam: ROS2 LiDAR-inertial SLAM"
html_theme = "pydata_sphinx_theme"
rosdoc2_settings = {"override_theme": False}
html_static_path = ["_static"]
templates_path = ["_templates"]
html_css_files = ["custom.css"]
html_baseurl = "https://prbonn.github.io/rko_slam/master/"
html_context = {"default_mode": "dark"}
html_sidebars = {"**": []}
html_theme_options = {
    "logo": {"text": "rko_slam"},
    "github_url": "https://github.com/PRBonn/rko_slam",
    "announcement": 'These docs are still under construction. Anything you can improve, an issue or a PR on <a href="https://github.com/PRBonn/rko_slam">GitHub</a> is appreciated.',
    "navbar_start": ["navbar-logo"],
    "navbar_center": ["navbar-nav"],
    "navbar_end": ["version-switcher", "theme-switcher", "navbar-icon-links"],
    "switcher": {"json_url": "https://prbonn.github.io/rko_slam/switcher.json", "version_match": version},
    "check_switcher": False,
    "show_version_warning_banner": True,
    "secondary_sidebar_items": ["page-toc"],
    "footer_start": ["copyright"],
    "footer_center": ["credits"],
    "footer_end": ["sphinx-version", "theme-version"],
    "navigation_with_keys": False,
}
