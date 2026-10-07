#!/usr/bin/env bash
# One-time per clone: register the nbstripout git filter used by .gitattributes.
# Git filter definitions live in .git/config, which is never committed, so every
# collaborator runs this once after `uv sync`.
set -euo pipefail
cd "$(git rev-parse --show-toplevel)"

# Outputs, execution counts and Colab/Jupyter run metadata are stripped on
# `git add`, so notebook diffs show only code and prose.
EXTRA_KEYS="metadata.colab metadata.language_info metadata.widgets cell.metadata.colab cell.metadata.outputId cell.metadata.executionInfo"
STRIP="uv run --quiet nbstripout --drop-empty-cells --extra-keys '${EXTRA_KEYS}'"

git config filter.nbstripout.clean "${STRIP}"
git config filter.nbstripout.smudge cat
git config filter.nbstripout.required true
git config diff.ipynb.textconv "${STRIP} -t"

echo "nbstripout git filter configured for $(pwd)"
