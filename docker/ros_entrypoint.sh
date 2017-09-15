#!/bin/bash
set -e

# setup ros environment
cd /ros
source "setup.sh"

# Run arguments
exec "$@"
