#!/bin/bash
set -eux

cargo clippy -p avm-stats
cargo clippy -p avm-analyzer-app --target wasm32-unknown-unknown
cargo clippy -p avm-analyzer-common
cargo clippy -p avm-analyzer-server
cargo fmt --all

cargo test -p avm-stats
cargo test -p avm-analyzer-common
cargo test -p avm-analyzer-server