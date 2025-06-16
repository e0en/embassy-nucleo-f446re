#!/bin/bash
set -e
pushd "stm32-kit"
cargo fmt
cargo clippy -- -D warnings
popd

pushd "foc"
cargo fmt
cargo clippy -- -D warnings
popd
