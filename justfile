default:
    @just --list

test *args:
    cargo nextest run --workspace {{args}}
    cargo test --workspace --doc

lint:
    cargo fmt --all --check
    cargo clippy --workspace --all-targets -- -D warnings
    cargo build -p motor_sim --target thumbv7em-none-eabihf

# The motor demo CLI (`motor_demo`) into ~/.cargo/bin
install: test
    cargo install --locked --path motor_demo

# motor_demo builds against the sibling wire_weaver checkout: bring it up to its main
deps-pull:
    git -C ../wire_weaver pull --ff-only

deploy: deps-pull
    git pull --ff-only
    just install
