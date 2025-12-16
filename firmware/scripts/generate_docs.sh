#!/usr/bin/env bash
set -euo pipefail

if ! command -v doxygen >/dev/null 2>&1; then
  echo "doxygen not found. Install doxygen and try again." >&2
  exit 1
fi

doxygen Doxyfile

echo "Doxygen finished. Open docs/html/index.html to view the docs."