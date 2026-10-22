default:
    @just --list

# The tests. `just test`: all of them (test-all), the run of the `tpm land` gate. `just test --changed`: only the
# workspace crates changed against main and the crates that depend on them (tpm test changed), what a session runs
# before tpm work ready.
test *flags:
    if [ "{{flags}}" = "--changed" ]; then tpm test changed -- cargo nextest run --workspace; else {{just_executable()}} test-all {{flags}}; fi

# the workspace's default members: everything but motor_twin (Bevy), which `just lint-twin` covers
test-all *args:
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

# With tpm on PATH: tpm install (kept-releases stack, one copy on PATH); otherwise: cargo install
# Build the motor_demo and motor_cad command line tools (deploy profile) and install them on this PC
install:
    cargo build --profile deploy --locked -p motor_demo -p motor_cad
    if command -v tpm >/dev/null; then tpm install target/deploy/motor_demo target/deploy/motor_cad; else cargo install --locked --path motor_demo; cargo install --locked --path motor_cad; fi

# motor_demo builds against the sibling wire_weaver checkout: bring it up to its main
deps-pull:
    git -C ../wire_weaver pull --ff-only

# Refuses on local changes (names them, never discards them), before anything is pulled; runs the tests before installing.
# Bring this PC up to main: pull --ff-only, the sibling wire_weaver, test, install, check both --versions against main
deploy:
    #!/usr/bin/env bash
    set -eu
    dirty=$(git status --porcelain)
    if [ -n "$dirty" ]; then echo "local changes, not deploying:"; echo "$dirty"; exit 1; fi
    git pull --ff-only
    just deps-pull
    just test
    just install
    for bin in motor_demo motor_cad; do \
        $bin --version | grep -q "($(git rev-parse --short HEAD)" || { echo "$bin --version does not show $(git rev-parse --short HEAD):"; $bin --version; exit 1; }; \
        $bin --version; \
    done
