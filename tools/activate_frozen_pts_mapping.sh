#!/usr/bin/env bash
set -euo pipefail

echo "This helper is disabled: Version5 now anchors b_session automatically" >&2
echo "after every STREAMON and forbids loading an old absolute b via SIGHUP." >&2
exit 2
