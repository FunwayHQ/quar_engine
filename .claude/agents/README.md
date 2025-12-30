# Aether Engine - Claude Code Agents

This directory contains specialized agent configurations for Claude Code to assist with different aspects of the Aether WebAR engine development.

## Available Agents

| Agent | File | Purpose |
|-------|------|---------|
| **Rust/WASM Development** | `rust-wasm-development.md` | Rust core development, WASM compilation, memory management |
| **Computer Vision** | `computer-vision.md` | CV algorithms: FAST, optical flow, pose estimation, SLAM |
| **TypeScript SDK** | `typescript-sdk.md` | Browser SDK, API design, Web Worker architecture |
| **Performance Optimization** | `performance-optimization.md` | 60 FPS targeting, profiling, memory optimization |
| **WebGL/Three.js** | `webgl-threejs.md` | Three.js integration, coordinate systems, AR rendering |
| **Testing & QA** | `testing-qa.md` | Unit tests, benchmarks, E2E testing, browser compatibility |

## How to Use

When working on a specific area of the codebase, reference the appropriate agent file to get specialized guidance:

```
@agent rust-wasm-development.md Help me optimize the feature detection loop
```

Or simply read the agent file for context before starting work on that area.

## Agent Selection Guide

| Task | Recommended Agent |
|------|-------------------|
| Implementing FAST corners | Computer Vision |
| Fixing memory leak in WASM | Performance Optimization |
| Adding new SDK method | TypeScript SDK |
| Three.js camera not updating | WebGL/Three.js |
| Writing tests for tracker | Testing & QA |
| Reducing binary size | Rust/WASM Development + Performance |
| Debugging iOS Safari issues | TypeScript SDK |
| Implementing Kalman filter | Computer Vision |

## Cross-Cutting Concerns

Some tasks span multiple agents:

- **New Feature End-to-End**: Computer Vision → Rust/WASM → TypeScript SDK → Testing
- **Performance Issue**: Performance Optimization + the relevant implementation agent
- **Integration Bug**: WebGL/Three.js + TypeScript SDK
- **Algorithm Accuracy**: Computer Vision + Testing & QA

## Updating Agents

When project patterns or technologies change, update the relevant agent files to keep guidance current. Key things to update:

- Dependency versions
- API signatures
- Performance targets
- Test patterns
- Coordinate system conventions

## Related Documentation

- `/SPRINTS.md` - Sprint plan with detailed LLM prompts for each sprint
- `/PRD.md` - Product requirements and technical architecture
- `/CLAUDE.md` - General project guidance for Claude Code
