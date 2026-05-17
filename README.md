# dronesimulator

Core physics modules implemented:

- `src/physics/MeshAnalyzer.js`
  - Mesh bounds, projected areas, center of mass, volume-based mass estimation, and inertia tensor from tetrahedral volume integration.
  - Inertia/mass explicitly assume **uniform density solid** using the selected material density.
- `src/physics/AerodynamicsEngine.js`
  - Directional aerodynamic drag (`Fd = 0.5 * rho * v^2 * Cd * A`) in body/world frames.
  - Angular aerodynamic damping torque.
  - Propeller H-force, blade flapping moment, and vortex-ring-state lift degradation factor.
  - First-order motor RPM response model with accel/decel time constants.

Application orchestration:

- `src/simulator/DroneSimulator.js`
  - Builds a runnable simulator from mesh geometry + motor/environment settings.
  - Integrates `MeshAnalyzer` + `AerodynamicsEngine` in a fixed-step update loop.
  - Updates motor RPM states, computes thrust/drag/weight net force, and advances body position/velocity.
- `src/index.js`
  - Top-level exports for mesh analysis, aerodynamics, materials, and simulator creation.
