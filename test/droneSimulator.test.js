const test = require("node:test");
const assert = require("node:assert/strict");
const { createDroneSimulator } = require("../src/simulator/DroneSimulator");

function cubeMesh(sideLength = 1) {
  const s = sideLength / 2;
  const vertices = [
    { x: -s, y: -s, z: -s },
    { x: s, y: -s, z: -s },
    { x: s, y: s, z: -s },
    { x: -s, y: s, z: -s },
    { x: -s, y: -s, z: s },
    { x: s, y: -s, z: s },
    { x: s, y: s, z: s },
    { x: -s, y: s, z: s },
  ];

  const indices = [
    0, 2, 1, 0, 3, 2,
    4, 5, 6, 4, 6, 7,
    0, 1, 5, 0, 5, 4,
    3, 6, 2, 3, 7, 6,
    1, 2, 6, 1, 6, 5,
    0, 4, 7, 0, 7, 3,
  ];

  return { vertices, indices };
}

test("simulator rises with sustained motor command in zero gravity", () => {
  const sim = createDroneSimulator({
    mesh: cubeMesh(1),
    environment: {
      gravityWorld: { x: 0, y: 0, z: 0 },
      windVelocityWorld: { x: 0, y: 0, z: 0 },
    },
    motors: [
      { maxRPM: 8000, thrustCoefficient: 1e-5 },
      { maxRPM: 8000, thrustCoefficient: 1e-5 },
      { maxRPM: 8000, thrustCoefficient: 1e-5 },
      { maxRPM: 8000, thrustCoefficient: 1e-5 },
    ],
  });

  sim.setMotorCommands([0.6, 0.6, 0.6, 0.6]);
  const result = sim.step(0.1);

  assert.ok(result.velocityWorld.y > 0);
  assert.ok(result.positionWorld.y > 0);
});

test("simulator descends under gravity with zero motor command", () => {
  const sim = createDroneSimulator({
    mesh: cubeMesh(1),
    environment: {
      gravityWorld: { x: 0, y: -9.81, z: 0 },
      windVelocityWorld: { x: 0, y: 0, z: 0 },
    },
    motors: [
      { maxRPM: 7000, thrustCoefficient: 8e-6 },
      { maxRPM: 7000, thrustCoefficient: 8e-6 },
      { maxRPM: 7000, thrustCoefficient: 8e-6 },
      { maxRPM: 7000, thrustCoefficient: 8e-6 },
    ],
  });

  sim.setMotorCommands([0, 0, 0, 0]);
  const result = sim.step(0.1);

  assert.ok(result.velocityWorld.y < 0);
  assert.ok(result.positionWorld.y < 0);
});

test("motor commands are clamped to [0, 1]", () => {
  const sim = createDroneSimulator({
    mesh: cubeMesh(1),
    environment: {
      gravityWorld: { x: 0, y: 0, z: 0 },
    },
    motors: [{ maxRPM: 5000, thrustCoefficient: 1e-5 }],
  });

  sim.setMotorCommands([1.5]);
  const stateHigh = sim.getState();
  assert.equal(stateHigh.motorCommands[0], 1);

  sim.setMotorCommands([-1]);
  const stateLow = sim.getState();
  assert.equal(stateLow.motorCommands[0], 0);
});

test("createDroneSimulator validates motor configs", () => {
  assert.throws(
    () => createDroneSimulator({ mesh: cubeMesh(1) }),
    /requires at least one motor config/i
  );

  assert.throws(
    () => createDroneSimulator({ mesh: cubeMesh(1), motors: [null] }),
    /Invalid motor config/i
  );
});

test("createDroneSimulator rejects unknown materials in mesh analysis", () => {
  assert.throws(
    () =>
      createDroneSimulator({
        meshAnalysis: {
          material: "unknownAlloy",
          projectedAreasM2: { x: 1, y: 1, z: 1 },
          dragScale: 1,
          massKgSolid: 1,
          massKgShell: 1,
        },
        motors: [{ maxRPM: 5000, thrustCoefficient: 1e-5 }],
      }),
    /Unknown material/i
  );
});

test("step rejects non-positive time steps", () => {
  const sim = createDroneSimulator({
    mesh: cubeMesh(1),
    environment: { gravityWorld: { x: 0, y: 0, z: 0 } },
    motors: [{ maxRPM: 5000, thrustCoefficient: 1e-5 }],
  });

  assert.throws(() => sim.step(0), /dtSeconds/i);
});

test("mass override uses provided mass instead of mesh analysis", () => {
  const sim = createDroneSimulator({
    mesh: cubeMesh(1),
    massKg: 12.5,
    motors: [{ maxRPM: 5000, thrustCoefficient: 1e-5 }],
  });

  assert.equal(sim.getState().massKg, 12.5);
});

test("shell mass is used when solid mass is zero", () => {
  const sim = createDroneSimulator({
    meshAnalysis: {
      material: "carbonFiberPolished",
      projectedAreasM2: { x: 1, y: 1, z: 1 },
      dragScale: 1,
      massKgSolid: 0,
      massKgShell: 2,
    },
    motors: [{ maxRPM: 5000, thrustCoefficient: 1e-5 }],
  });

  assert.equal(sim.getState().massKg, 2);
});
