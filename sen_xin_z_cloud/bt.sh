#!/usr/bin/env bash
set -eo pipefail
WORKSPACE_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
export ENABLE_DECISION=true
exec "$WORKSPACE_DIR/nav.sh" "$@"
