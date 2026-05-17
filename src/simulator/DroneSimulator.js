const { analyzeMesh } = require("../physics/MeshAnalyzer");
const {
  computeAerodynamics,
  updateMotorRPM,
} = require("../physics/AerodynamicsEngine");
const { MATERIAL_PROPERTIES } = require("../physics/materialProperties");

const EPSILON = 1e-9;

function vec3(x = 0, y = 0, z = 0) {
  return { x, y, z };
}

function add(a, b) {
  return vec3(a.x + b.x, a.y + b.y, a.z + b.z);
}

function sub(a, b) {
  return vec3(a.x - b.x, a.y - b.y, a.z - b.z);
}

function scale(v, s) {
  return vec3(v.x * s, v.y * s, v.z * s);
}

function identity3() {
  return [
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 1],
  ];
}

function normalizeMotorConfigs(motors = []) {
  if (!Array.isArray(motors) || motors.length === 0) {
    throw new Error("DroneSimulator requires at least one motor config.");
  }

  return motors.map((motor) => {
    if (!motor || typeof motor !== "object") {
      throw new Error("Invalid motor config.");
    }

    return {
      maxRPM: motor.maxRPM ?? 9000,
      tauAccelSeconds: motor.tauAccelSeconds ?? 0.12,
      tauDecelSeconds: motor.tauDecelSeconds ?? 0.25,
      tauBrakeSeconds: motor.tauBrakeSeconds,
      thrustCoefficient: motor.thrustCoefficient ?? 7e-6,
      activeBraking: Boolean(motor.activeBraking),
      rpm: motor.initialRPM ?? 0,
      command: motor.initialCommand ?? 0,
    };
  });
}

function deriveMass(meshAnalysis, fallbackMassKg) {
  if (typeof fallbackMassKg === "number" && fallbackMassKg > EPSILON) {
    return fallbackMassKg;
  }

  const mass = meshAnalysis.massKgSolid > EPSILON ? meshAnalysis.massKgSolid : meshAnalysis.massKgShell;
  if (mass <= EPSILON) {
    throw new Error("DroneSimulator could not derive a positive mass from mesh analysis.");
  }

  return mass;
}

function createDroneSimulator({
  mesh,
  meshAnalysis,
  material = "carbonFiberPolished",
  hullThicknessM = 0.002,
  massKg,
  orientationBodyToWorld = identity3(),
  initialPositionWorld = vec3(0, 0, 0),
  initialVelocityWorld = vec3(0, 0, 0),
  initialAngularVelocityBody = vec3(0, 0, 0),
  environment = {},
  aerodynamics = {},
  motors,
} = {}) {
  const resolvedMeshAnalysis = meshAnalysis || analyzeMesh({
    vertices: mesh?.vertices,
    indices: mesh?.indices,
    material,
    hullThicknessM,
  });

  if (!MATERIAL_PROPERTIES[resolvedMeshAnalysis.material]) {
    throw new Error(`Unknown material \"${resolvedMeshAnalysis.material}\".`);
  }

  const vehicleMassKg = deriveMass(resolvedMeshAnalysis, massKg);
  const motorState = normalizeMotorConfigs(motors);

  const env = {
    airDensity: environment.airDensity ?? 1.225,
    gravityWorld: environment.gravityWorld ?? vec3(0, -9.81, 0),
    windVelocityWorld: environment.windVelocityWorld ?? vec3(0, 0, 0),
  };

  const aeroConfig = {
    baseDragCoefficient: aerodynamics.baseDragCoefficient ?? 1,
    axisDragOverrides: aerodynamics.axisDragOverrides ?? {},
    rotationalAreaM2: aerodynamics.rotationalAreaM2,
    torqueCoefficients: aerodynamics.torqueCoefficients,
    rotorDiskAreaM2: aerodynamics.rotorDiskAreaM2 ?? 0.15,
    propellerDragCoefficient: aerodynamics.propellerDragCoefficient ?? 0.12,
    bladeFlapCoefficient: aerodynamics.bladeFlapCoefficient ?? 2e-4,
    inducedDownwashSpeed: aerodynamics.inducedDownwashSpeed ?? 4,
  };

  const state = {
    timeSeconds: 0,
    positionWorld: { ...initialPositionWorld },
    velocityWorld: { ...initialVelocityWorld },
    orientationBodyToWorld,
    angularVelocityBody: { ...initialAngularVelocityBody },
    motors: motorState,
  };

  function setMotorCommands(commands = []) {
    for (let i = 0; i < state.motors.length; i += 1) {
      const command = commands[i] ?? 0;
      state.motors[i].command = Math.max(0, Math.min(1, command));
    }
  }

  function step(dtSeconds) {
    if (!(dtSeconds > 0)) {
      throw new Error("DroneSimulator.step requires dtSeconds > 0.");
    }

    let totalThrustNewtons = 0;
    let totalOmega = 0;

    for (const motor of state.motors) {
      const targetRPM = motor.command * motor.maxRPM;
      motor.rpm = updateMotorRPM({
        currentRPM: motor.rpm,
        targetRPM,
        dtSeconds,
        tauAccelSeconds: motor.tauAccelSeconds,
        tauDecelSeconds: motor.tauDecelSeconds,
        activeBraking: motor.activeBraking,
        tauBrakeSeconds: motor.tauBrakeSeconds,
      });

      const omega = (motor.rpm * 2 * Math.PI) / 60;
      totalOmega += omega;
      totalThrustNewtons += motor.thrustCoefficient * omega * omega;
    }

    const averageOmega = totalOmega / state.motors.length;

    const aero = computeAerodynamics({
      bodyVelocityWorld: state.velocityWorld,
      windVelocityWorld: env.windVelocityWorld,
      angularVelocityBody: state.angularVelocityBody,
      orientationBodyToWorld: state.orientationBodyToWorld,
      projectedAreasM2: resolvedMeshAnalysis.projectedAreasM2,
      materialDragScale: resolvedMeshAnalysis.dragScale,
      airDensity: env.airDensity,
      baseDragCoefficient: aeroConfig.baseDragCoefficient,
      axisDragOverrides: aeroConfig.axisDragOverrides,
      rotationalAreaM2: aeroConfig.rotationalAreaM2,
      torqueCoefficients: aeroConfig.torqueCoefficients,
      rotorDiskAreaM2: aeroConfig.rotorDiskAreaM2,
      propellerDragCoefficient: aeroConfig.propellerDragCoefficient,
      averageRotorOmegaRadPerSec: averageOmega,
      bladeFlapCoefficient: aeroConfig.bladeFlapCoefficient,
      inducedDownwashSpeed: aeroConfig.inducedDownwashSpeed,
    });

    const thrustWorld = {
      x: state.orientationBodyToWorld[0][1] * totalThrustNewtons * aero.vortexRingLiftFactor,
      y: state.orientationBodyToWorld[1][1] * totalThrustNewtons * aero.vortexRingLiftFactor,
      z: state.orientationBodyToWorld[2][1] * totalThrustNewtons * aero.vortexRingLiftFactor,
    };

    const gravityForceWorld = scale(env.gravityWorld, vehicleMassKg);
    const netForceWorld = add(
      add(
        add(gravityForceWorld, thrustWorld),
        aero.directionalHullDragForceWorld
      ),
      aero.propellerHForceWorld
    );

    const accelerationWorld = scale(netForceWorld, 1 / vehicleMassKg);
    state.velocityWorld = add(state.velocityWorld, scale(accelerationWorld, dtSeconds));
    state.positionWorld = add(state.positionWorld, scale(state.velocityWorld, dtSeconds));
    state.timeSeconds += dtSeconds;

    return {
      timeSeconds: state.timeSeconds,
      positionWorld: { ...state.positionWorld },
      velocityWorld: { ...state.velocityWorld },
      accelerationWorld,
      netForceWorld,
      thrustWorld,
      motorRPMs: state.motors.map((motor) => motor.rpm),
      aerodynamics: aero,
    };
  }

  function getState() {
    return {
      timeSeconds: state.timeSeconds,
      positionWorld: { ...state.positionWorld },
      velocityWorld: { ...state.velocityWorld },
      angularVelocityBody: { ...state.angularVelocityBody },
      motorRPMs: state.motors.map((motor) => motor.rpm),
      motorCommands: state.motors.map((motor) => motor.command),
      massKg: vehicleMassKg,
      meshAnalysis: resolvedMeshAnalysis,
    };
  }

  return {
    setMotorCommands,
    step,
    getState,
  };
}

module.exports = {
  createDroneSimulator,
};
