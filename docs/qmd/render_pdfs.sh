#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$script_dir"

for qmd in *.qmd; do
  quarto render "$qmd" --to pdf --output-dir ..
done
