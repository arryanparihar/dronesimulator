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

  const expectedUp = 1000 + (2000 - 1000) * (1 - Math.exp(-0.1 / 0.2));
  const expectedDown = 2000 + (1000 - 2000) * (1 - Math.exp(-0.1 / 0.4));

  assert.ok(up > 1000 && up < 2000);
  assert.ok(down > 1000 && down < 2000);
  assert.ok(up - 1000 > 2000 - down);
  assert.ok(Math.abs(up - expectedUp) < 1e-9);
  assert.ok(Math.abs(down - expectedDown) < 1e-9);
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

test("angular drag torque uses signed quadratic drag per axis", () => {
  const result = computeAngularDragTorque({
    airDensity: 1,
    angularVelocityBody: { x: 2, y: -3, z: 0.5 },
    rotationalAreaM2: { x: 1, y: 2, z: 1 },
    torqueCoefficients: { x: 1, y: 1, z: 0.5 },
  });

  assert.ok(Math.abs(result.x + 2) < 1e-9);
  assert.ok(Math.abs(result.y - 9) < 1e-9);
  assert.ok(Math.abs(result.z + 0.0625) < 1e-9);
});

test("axis drag overrides apply per axis in aerodynamic drag", () => {
  const result = computeAerodynamics({
    bodyVelocityWorld: { x: 1, y: 0, z: 0 },
    projectedAreasM2: { x: 1, y: 1, z: 1 },
    baseDragCoefficient: 1,
    axisDragOverrides: { x: 2 },
    materialDragScale: 1,
    airDensity: 1,
  });

  assert.equal(result.effectiveDragCoefficients.x, 2);
  assert.equal(result.effectiveDragCoefficients.y, 1);
  assert.equal(result.effectiveDragCoefficients.z, 1);
  assert.ok(Math.abs(result.directionalHullDragForceBody.x + 1) < 1e-9);
});

test("propeller H-force and blade flapping respond to lateral flow", () => {
  const result = computeAerodynamics({
    bodyVelocityWorld: { x: 2, y: 0, z: 0 },
    projectedAreasM2: { x: 0, y: 0, z: 0 },
    baseDragCoefficient: 0,
    materialDragScale: 1,
    airDensity: 1,
    rotorDiskAreaM2: 0.5,
    propellerDragCoefficient: 0.2,
    averageRotorOmegaRadPerSec: 10,
    bladeFlapCoefficient: 1e-4,
  });

  assert.ok(Math.abs(result.propellerHForceWorld.x + 0.2) < 1e-9);
  assert.ok(Math.abs(result.propellerHForceWorld.y) < 1e-9);
  assert.ok(Math.abs(result.propellerHForceWorld.z) < 1e-9);
  assert.ok(Math.abs(result.bladeFlappingMomentWorld.z - 0.02) < 1e-9);
});

test("active braking uses brake time constant on deceleration", () => {
  const rpm = updateMotorRPM({
    currentRPM: 2000,
    targetRPM: 0,
    dtSeconds: 0.1,
    tauAccelSeconds: 0.2,
    tauDecelSeconds: 0.4,
    activeBraking: true,
    tauBrakeSeconds: 0.1,
  });
  const expected = 2000 + (0 - 2000) * (1 - Math.exp(-0.1 / 0.1));

  assert.ok(Math.abs(rpm - expected) < 1e-9);
});
