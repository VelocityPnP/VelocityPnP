.PHONY: check test run ui-install ui-test ui-build

check:
	cargo fmt --check
	cargo clippy --workspace --all-targets -- -D warnings

test:
	cargo test --workspace

run:
	cargo run -p velocitypnpd -- --simulate

ui-install:
	cd web && npm install

ui-test:
	cd web && npm test -- --run

ui-build:
	cd web && npm run build
