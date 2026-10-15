default:
    @just --list

test:
    cargo nextest run
    cargo test --doc

lint:
    cargo fmt --check
    cargo clippy --all-targets -- -D warnings
    cargo build --target thumbv7em-none-eabihf

# Library only: nothing to install on this PC
install: test

deploy:
    git pull --ff-only
