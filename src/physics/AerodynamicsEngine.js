const EPSILON = 1e-9;
const MAX_DESCENT_RATIO = 2;
const LOW_FORWARD_TRANSITION_SPEED = 3;
const VRS_ONSET_RATIO = 0.7;
const VRS_TRANSITION_RANGE = 1.3;

function vec3(x = 0, y = 0, z = 0) {
  return { x, y, z };
}

function dot(a, b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

function magnitude(v) {
  return Math.sqrt(dot(v, v));
}

function sub(a, b) {
  return vec3(a.x - b.x, a.y - b.y, a.z - b.z);
}

function scale(v, s) {
  return vec3(v.x * s, v.y * s, v.z * s);
}

function add(a, b) {
  return vec3(a.x + b.x, a.y + b.y, a.z + b.z);
}

function cross(a, b) {
  return vec3(
    a.y * b.z - a.z * b.y,
    a.z * b.x - a.x * b.z,
    a.x * b.y - a.y * b.x
  );
}

function transposeMatrix3(m) {
  return [
    [m[0][0], m[1][0], m[2][0]],
    [m[0][1], m[1][1], m[2][1]],
    [m[0][2], m[1][2], m[2][2]],
  ];
}

function mulMat3Vec3(m, v) {
  return vec3(
    m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
    m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
    m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z
  );
}

function signedQuadraticDrag(velocity, area, cd, airDensity) {
  return -0.5 * airDensity * cd * area * Math.abs(velocity) * velocity;
}

function clamp(value, min, max) {
  return Math.min(max, Math.max(min, value));
}

function computeDirectionalDrag({
  airDensity = 1.225,
  dragCoefficients = { x: 1, y: 1, z: 1 },
  projectedAreasM2,
  relativeVelocityWorld,
  orientationBodyToWorld,
}) {
  const worldToBody = transposeMatrix3(orientationBodyToWorld);
  const vBody = mulMat3Vec3(worldToBody, relativeVelocityWorld);

  const dragBody = vec3(
    signedQuadraticDrag(vBody.x, projectedAreasM2.x, dragCoefficients.x, airDensity),
    signedQuadraticDrag(vBody.y, projectedAreasM2.y, dragCoefficients.y, airDensity),
    signedQuadraticDrag(vBody.z, projectedAreasM2.z, dragCoefficients.z, airDensity)
  );

  return {
    relativeVelocityBody: vBody,
    dragForceBody: dragBody,
    dragForceWorld: mulMat3Vec3(orientationBodyToWorld, dragBody),
  };
}

function computeAngularDragTorque({
  airDensity = 1.225,
  angularVelocityBody,
  rotationalAreaM2 = { x: 0.01, y: 0.01, z: 0.01 },
  torqueCoefficients = { x: 0.02, y: 0.02, z: 0.02 },
}) {
  return vec3(
    signedQuadraticDrag(
      angularVelocityBody.x,
      rotationalAreaM2.x,
      torqueCoefficients.x,
      airDensity
    ),
    signedQuadraticDrag(
      angularVelocityBody.y,
      rotationalAreaM2.y,
      torqueCoefficients.y,
      airDensity
    ),
    signedQuadraticDrag(
      angularVelocityBody.z,
      rotationalAreaM2.z,
      torqueCoefficients.z,
      airDensity
    )
  );
}

function computeVortexRingStateFactor({
  verticalVelocityBodyY,
  horizontalSpeedBody,
  inducedDownwashSpeed = 4,
  minFactor = 0.35,
}) {
  const descending = Math.max(0, -verticalVelocityBodyY);
  const descentRatio = clamp(
    descending / Math.max(inducedDownwashSpeed, EPSILON),
    0,
    MAX_DESCENT_RATIO
  );
  const lowForwardFlightFactor =
    1 - clamp(horizontalSpeedBody / LOW_FORWARD_TRANSITION_SPEED, 0, 1);
  const vrsStrength =
    clamp((descentRatio - VRS_ONSET_RATIO) / VRS_TRANSITION_RANGE, 0, 1) *
    lowForwardFlightFactor;
  return 1 - vrsStrength * (1 - minFactor);
}

function computePropellerHForce({
  airDensity = 1.225,
  relativeVelocityBody,
  rotorDiskAreaM2 = 0.15,
  propellerDragCoefficient = 0.12,
  orientationBodyToWorld,
}) {
  const horizontalVelocityBody = vec3(relativeVelocityBody.x, 0, relativeVelocityBody.z);
  const speed = magnitude(horizontalVelocityBody);
  if (speed < EPSILON) {
    return vec3(0, 0, 0);
  }
  const forceMag =
    0.5 *
    airDensity *
    propellerDragCoefficient *
    rotorDiskAreaM2 *
    speed *
    speed;
  const unit = scale(horizontalVelocityBody, 1 / speed);
  const bodyForce = scale(unit, -forceMag);
  return mulMat3Vec3(orientationBodyToWorld, bodyForce);
}

function computeBladeFlappingMoment({
  relativeVelocityBody,
  averageRotorOmegaRadPerSec,
  bladeFlapCoefficient = 2e-4,
  orientationBodyToWorld,
}) {
  // Assumes body-frame Y axis is the rotor thrust axis (Y-up frame convention).
  const rotorAxisBody = vec3(0, 1, 0);
  const lateralFlow = vec3(relativeVelocityBody.x, 0, relativeVelocityBody.z);
  const omega2 = averageRotorOmegaRadPerSec * averageRotorOmegaRadPerSec;
  const momentBody = scale(cross(rotorAxisBody, lateralFlow), -bladeFlapCoefficient * omega2);
  return mulMat3Vec3(orientationBodyToWorld, momentBody);
}

function updateMotorRPM({
  currentRPM,
  targetRPM,
  dtSeconds,
  tauAccelSeconds,
  tauDecelSeconds,
  activeBraking = false,
  tauBrakeSeconds,
}) {
  const isAccelerating = targetRPM > currentRPM;
  const tau = Math.max(
    EPSILON,
    isAccelerating
      ? tauAccelSeconds
      : activeBraking
      ? tauBrakeSeconds || tauDecelSeconds
      : tauDecelSeconds
  );
  const alpha = 1 - Math.exp(-dtSeconds / tau);
  return currentRPM + (targetRPM - currentRPM) * alpha;
}

function computeAerodynamics({
  bodyVelocityWorld,
  windVelocityWorld = vec3(0, 0, 0),
  angularVelocityBody = vec3(0, 0, 0),
  orientationBodyToWorld = [
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 1],
  ],
  projectedAreasM2 = { x: 0.1, y: 0.1, z: 0.1 },
  materialDragScale = 1,
  baseDragCoefficient = 1,
  axisDragOverrides = {},
  airDensity = 1.225,
  rotationalAreaM2,
  torqueCoefficients,
  rotorDiskAreaM2 = 0.15,
  propellerDragCoefficient = 0.12,
  averageRotorOmegaRadPerSec = 0,
  bladeFlapCoefficient = 2e-4,
  inducedDownwashSpeed = 4,
}) {
  const relativeVelocityWorld = sub(bodyVelocityWorld, windVelocityWorld);

  const cd = {
    x: (axisDragOverrides.x || baseDragCoefficient) * materialDragScale,
    y: (axisDragOverrides.y || baseDragCoefficient) * materialDragScale,
    z: (axisDragOverrides.z || baseDragCoefficient) * materialDragScale,
  };

  const drag = computeDirectionalDrag({
    airDensity,
    dragCoefficients: cd,
    projectedAreasM2,
    relativeVelocityWorld,
    orientationBodyToWorld,
  });

  const horizontalBodySpeed = Math.sqrt(
    drag.relativeVelocityBody.x * drag.relativeVelocityBody.x +
      drag.relativeVelocityBody.z * drag.relativeVelocityBody.z
  );
  const vrsFactor = computeVortexRingStateFactor({
    verticalVelocityBodyY: drag.relativeVelocityBody.y,
    horizontalSpeedBody: horizontalBodySpeed,
    inducedDownwashSpeed,
  });

  const hForceWorld = computePropellerHForce({
    airDensity,
    relativeVelocityBody: drag.relativeVelocityBody,
    rotorDiskAreaM2,
    propellerDragCoefficient,
    orientationBodyToWorld,
  });

  const bladeFlapMomentWorld = computeBladeFlappingMoment({
    relativeVelocityBody: drag.relativeVelocityBody,
    averageRotorOmegaRadPerSec,
    bladeFlapCoefficient,
    orientationBodyToWorld,
  });

  const angularDragBody = computeAngularDragTorque({
    airDensity,
    angularVelocityBody,
    rotationalAreaM2,
    torqueCoefficients,
  });

  return {
    relativeVelocityWorld,
    relativeVelocityBody: drag.relativeVelocityBody,
    directionalHullDragForceWorld: drag.dragForceWorld,
    directionalHullDragForceBody: drag.dragForceBody,
    propellerHForceWorld: hForceWorld,
    bladeFlappingMomentWorld: bladeFlapMomentWorld,
    angularDragTorqueBody: angularDragBody,
    vortexRingLiftFactor: vrsFactor,
    effectiveDragCoefficients: cd,
  };
}

module.exports = {
  computeAerodynamics,
  updateMotorRPM,
  computeDirectionalDrag,
  computeAngularDragTorque,
};
