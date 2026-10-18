default:
    @just --list

# the workspace's default members: everything but motor_twin (Bevy), which `just lint-twin` covers
test *args:
    cargo nextest run {{args}}
    cargo test --doc

lint:
    cargo fmt --all --check
    cargo clippy --all-targets -- -D warnings
    cargo build -p motor_sim --target thumbv7em-none-eabihf

# ns per Motor::step per configuration, and the thumbv7em code size of motor_sim, against motor_bench/baseline.tsv
bench:
    cargo run -p motor_bench --release

# motor_twin builds Bevy: several minutes and a few GB of RAM the first time
lint-twin:
    cargo clippy -p motor_twin --all-targets -- -D warnings

# the twin on a motor: a STEP file or a motor_cad .glb
twin model *args:
    cargo run --release -p motor_twin -- {{model}} {{args}}

# The motor demo CLI (`motor_demo`) into ~/.cargo/bin
install: test
    cargo install --locked --path motor_demo
    cargo install --locked --path motor_cad

# motor_demo builds against the sibling wire_weaver checkout: bring it up to its main
deps-pull:
    git -C ../wire_weaver pull --ff-only

deploy: deps-pull
    git pull --ff-only
    just install
