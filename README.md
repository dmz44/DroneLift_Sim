# DroneLift reviewer simulator

A static Sphinx site (Read the Docs theme) with an in-browser simulator for the
*Fault-Tolerant Multi-Drone Cooperative Heavy Lift* submission. Reviewers can change the
payload configuration and the number of drones, inject crash failures, and compare the
Simplex runtime against the baselines. No server is needed: everything runs client-side
and is hosted on GitHub Pages.

## Layout

```
docs/
  conf.py, *.rst              site pages (Sphinx + sphinx_rtd_theme)
  _static/dronelift/
    sim-core.js               simulation core (no DOM; also runs in Node)
    sim-ui.js                 controls, 3D viewer, plots, comparison, batch
    sim.css
tests/
  test_core.js                regression tests (run in CI before each deploy)
  batch.js                    command-line Monte Carlo batch
.github/workflows/pages.yml   builds with -W and deploys to GitHub Pages
```

## Build locally

```bash
pip install -r docs/requirements.txt
sphinx-build -W -b html docs docs/_build/html
python -m http.server -d docs/_build/html 8000   # open http://localhost:8000/simulator.html
node tests/test_core.js
node tests/batch.js 100                           # 100-scenario batch in the terminal
```

## Deploy to github.io

1. Push this folder to a new GitHub repository (branch `main`).
2. In the repository, open **Settings → Pages** and set **Source** to **GitHub Actions**.
3. The workflow builds and deploys on every push. The site appears at
   `https://<account>.github.io/<repo>/`.

### Double-blind review

The site carries no author names, affiliations or repository links. A github.io address
contains the account name, though. For a double-blind venue, host the site under an
anonymous GitHub account or organization created for the submission, and give the
reviewers only the github.io link.
