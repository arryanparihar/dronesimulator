const test = require("node:test");
const assert = require("node:assert/strict");
const {
  computeAerodynamics,
  updateMotorRPM,
} = require("../src/physics/AerodynamicsEngine");

test("directional drag follows 0.5 * rho * v^2 * Cd * A per axis", () => {
  const result = computeAerodynamics({
    bodyVelocityWorld: { x: 10, y: 0, z: 0 },
    windVelocityWorld: { x: 0, y: 0, z: 0 },
    projectedAreasM2: { x: 1, y: 1, z: 1 },
    baseDragCoefficient: 1,
    materialDragScale: 1,
    airDensity: 1,
  });

  assert.ok(Math.abs(result.directionalHullDragForceBody.x + 50) < 1e-9);
  assert.ok(Math.abs(result.directionalHullDragForceBody.y) < 1e-9);
  assert.ok(Math.abs(result.directionalHullDragForceBody.z) < 1e-9);
});

test("motor RPM update uses first-order response and different time constants", () => {
  const up = updateMotorRPM({
    currentRPM: 1000,
    targetRPM: 2000,
    dtSeconds: 0.1,
    tauAccelSeconds: 0.2,
    tauDecelSeconds: 0.4,
  });
  const down = updateMotorRPM({
    currentRPM: 2000,
    targetRPM: 1000,
    dtSeconds: 0.1,
    tauAccelSeconds: 0.2,
    tauDecelSeconds: 0.4,
  });

  assert.ok(up > 1000 && up < 2000);
  assert.ok(down > 1000 && down < 2000);
  assert.ok(up - 1000 > 2000 - down);
});

test("vortex ring state factor drops during strong vertical descent", () => {
  const result = computeAerodynamics({
    bodyVelocityWorld: { x: 0, y: -8, z: 0 },
    windVelocityWorld: { x: 0, y: 0, z: 0 },
    projectedAreasM2: { x: 0.2, y: 0.4, z: 0.2 },
    baseDragCoefficient: 1,
    materialDragScale: 1,
    airDensity: 1.225,
  });

  assert.ok(result.vortexRingLiftFactor < 1);
  assert.ok(result.vortexRingLiftFactor >= 0.35);
});
