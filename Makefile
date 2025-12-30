# QUAR Engine - Makefile
# Build automation for Rust/WASM and TypeScript SDK

.PHONY: all build build-dev build-release test lint clean setup serve help

# Default target
all: build

# Setup development environment
setup:
	@echo "Setting up QUAR Engine development environment..."
	@which rustup > /dev/null || (echo "Please install Rust: https://rustup.rs" && exit 1)
	@rustup target add wasm32-unknown-unknown
	@which wasm-pack > /dev/null || cargo install wasm-pack
	@cd sdk && npm install
	@echo "Setup complete!"

# Build WASM module (development)
build-dev:
	@echo "Building WASM (development)..."
	wasm-pack build --target web --dev --out-dir pkg

# Build WASM module (release)
build-release:
	@echo "Building WASM (release)..."
	wasm-pack build --target web --release --out-dir pkg

# Default build (release)
build: build-release
	@echo "Build complete! Output in ./pkg/"

# Build with profiling enabled
build-profile:
	@echo "Building WASM with profiling..."
	wasm-pack build --target web --release --out-dir pkg -- --features profiling

# Build TypeScript SDK
build-sdk:
	@echo "Building TypeScript SDK..."
	cd sdk && npm run build

# Build everything
build-all: build build-sdk

# Run Rust tests
test:
	@echo "Running Rust tests..."
	cargo test

# Run WASM tests in headless browser
test-wasm:
	@echo "Running WASM tests..."
	wasm-pack test --headless --chrome

# Run SDK tests
test-sdk:
	@echo "Running SDK tests..."
	cd sdk && npm test

# Run all tests
test-all: test test-wasm test-sdk

# Format code
fmt:
	@echo "Formatting Rust code..."
	cargo fmt

# Check formatting
fmt-check:
	@echo "Checking Rust formatting..."
	cargo fmt --check

# Run linter
lint:
	@echo "Running Clippy..."
	cargo clippy -- -D warnings

# Run SDK linter
lint-sdk:
	@echo "Running ESLint..."
	cd sdk && npm run lint

# Type check SDK
typecheck-sdk:
	@echo "Type checking SDK..."
	cd sdk && npm run typecheck

# Clean build artifacts
clean:
	@echo "Cleaning build artifacts..."
	cargo clean
	rm -rf pkg/
	rm -rf sdk/dist/
	rm -rf sdk/node_modules/

# Serve the test page (requires a local HTTP server)
serve: build-dev
	@echo "Starting development server..."
	@echo "Open http://localhost:8080/sdk/index.html in your browser"
	@which python3 > /dev/null && python3 -m http.server 8080 || \
		(which npx > /dev/null && npx serve -l 8080 .)

# Watch for changes and rebuild (requires cargo-watch)
watch:
	@which cargo-watch > /dev/null || cargo install cargo-watch
	cargo watch -s "wasm-pack build --target web --dev --out-dir pkg"

# Generate documentation
docs:
	@echo "Generating documentation..."
	cargo doc --no-deps --open

# Check binary size
size: build-release
	@echo "WASM binary size:"
	@ls -lh pkg/quar_engine_bg.wasm 2>/dev/null || echo "Build first with 'make build'"
	@echo ""
	@echo "Gzipped size:"
	@gzip -c pkg/quar_engine_bg.wasm 2>/dev/null | wc -c | awk '{printf "%.2f KB\n", $$1/1024}' || echo "Build first"

# Run all checks (for CI)
ci: fmt-check lint test build-release size

# Help
help:
	@echo "QUAR Engine - Build Commands"
	@echo ""
	@echo "Setup:"
	@echo "  make setup        - Install dependencies and tools"
	@echo ""
	@echo "Build:"
	@echo "  make build        - Build release WASM (default)"
	@echo "  make build-dev    - Build development WASM"
	@echo "  make build-sdk    - Build TypeScript SDK"
	@echo "  make build-all    - Build WASM and SDK"
	@echo ""
	@echo "Test:"
	@echo "  make test         - Run Rust unit tests"
	@echo "  make test-wasm    - Run WASM tests in browser"
	@echo "  make test-sdk     - Run SDK tests"
	@echo "  make test-all     - Run all tests"
	@echo ""
	@echo "Quality:"
	@echo "  make fmt          - Format code"
	@echo "  make lint         - Run linter"
	@echo "  make ci           - Run all CI checks"
	@echo ""
	@echo "Development:"
	@echo "  make serve        - Start dev server"
	@echo "  make watch        - Watch and rebuild"
	@echo "  make docs         - Generate documentation"
	@echo ""
	@echo "Other:"
	@echo "  make clean        - Remove build artifacts"
	@echo "  make size         - Show binary size"
	@echo "  make help         - Show this help"
