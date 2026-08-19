#!/bin/bash
# resolve_python.sh - shared Python interpreter resolution for KURSAD40 V32 launchers.
#
# PORTABILITY (Ubuntu -> macOS): the V32 launchers originally invoked
# "../../.venv/bin/python" directly, i.e. the Ubuntu repo-local venv at
# <repo>/.scripts/.venv. That path does not exist on macOS, where the
# canonical environment is ~/Projects/kursad40-venv, so every launcher died
# with "No such file or directory" before reaching any Python code.
#
# Source this file from a launcher; it sets PYTHON_BIN to the first candidate
# that exists and is executable, in this order:
#   1. $VIRTUAL_ENV        - an already-activated venv always wins (any OS)
#   2. $KURSAD40_VENV      - explicit override
#   3. ~/Projects/kursad40-venv - canonical macOS environment
#   4. <repo>/.scripts/.venv    - original Ubuntu repo-local environment
#   5. python3 on PATH     - last resort
#
# Candidates are anchored to this file's own location, not the caller's
# working directory, so launchers work when invoked from anywhere.
#
# Usage:
#   source "$(dirname "$0")/resolve_python.sh"
#   "$PYTHON_BIN" -u some_script.py "$@"

_RESOLVE_PYTHON_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"

# <repo>/.scripts/.venv -- two levels up from .scripts/olds/v32/
_KURSAD40_LEGACY_VENV="$_RESOLVE_PYTHON_DIR/../../.venv"

PYTHON_BIN=""

for _candidate in \
	"$VIRTUAL_ENV/bin/python" \
	"$KURSAD40_VENV/bin/python" \
	"$HOME/Projects/kursad40-venv/bin/python" \
	"$_KURSAD40_LEGACY_VENV/bin/python" \
	; do
	# Skip candidates whose env var was unset (leaving a bare "/bin/python").
	case "$_candidate" in
		/bin/python) continue ;;
	esac
	if [ -x "$_candidate" ]; then
		PYTHON_BIN="$_candidate"
		break
	fi
done

if [ -z "$PYTHON_BIN" ]; then
	if command -v python3 > /dev/null 2>&1; then
		PYTHON_BIN="$(command -v python3)"
	else
		echo "ERROR: no usable Python interpreter found." >&2
		echo "  Tried, in order:" >&2
		echo "    \$VIRTUAL_ENV/bin/python           (VIRTUAL_ENV=${VIRTUAL_ENV:-<unset>})" >&2
		echo "    \$KURSAD40_VENV/bin/python         (KURSAD40_VENV=${KURSAD40_VENV:-<unset>})" >&2
		echo "    $HOME/Projects/kursad40-venv/bin/python" >&2
		echo "    $_KURSAD40_LEGACY_VENV/bin/python" >&2
		echo "    python3 on PATH" >&2
		echo "  Fix: activate the KURSAD40 environment, or set KURSAD40_VENV" >&2
		echo "       to its root, e.g. export KURSAD40_VENV=\$HOME/Projects/kursad40-venv" >&2
		exit 1
	fi
fi

unset _candidate _RESOLVE_PYTHON_DIR _KURSAD40_LEGACY_VENV
