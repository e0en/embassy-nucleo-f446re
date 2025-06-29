#!/bin/bash
set -e
pushd "stm32-kit"
cargo clean
cargo fmt
cargo clippy -- -D warnings
popd

pushd "foc"
cargo clean
cargo fmt
cargo clippy -- -D warnings
popd
