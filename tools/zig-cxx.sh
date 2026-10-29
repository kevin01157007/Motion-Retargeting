#!/bin/sh
# Fallback compiler on Unix hosts with the Python ziglang package installed.
exec python3 -m ziglang c++ "$@"
