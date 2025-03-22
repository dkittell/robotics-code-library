# Install Rust
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
source $HOME/.cargo/env

brew install node cmake

make setup && make release


open /Users/dkittell/repo/FRC-DriverStation/Conductor/target/release/conductor