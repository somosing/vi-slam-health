# ADR 0001: Keep health and recovery backend-independent

- Status: accepted
- Date: 2026-08-06

## Decision

Define ROS interfaces between the SLAM backend and the health/recovery stack. Ship a deterministic mock implementation for integration testing; build a real SLAM adapter separately against a pinned backend.

## Consequences

The complete recovery behavior can be tested without heavyweight third-party libraries. Different estimators can be compared under one detector contract. Backend-specific instrumentation still requires a reviewed adapter and cannot be claimed complete merely because pose topics connect.
