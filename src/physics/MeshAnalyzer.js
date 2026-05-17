const { MATERIAL_PROPERTIES } = require("./materialProperties");

const EPSILON = 1e-9;

function vec3(x = 0, y = 0, z = 0) {
  return { x, y, z };
}

function add(a, b) {
  return vec3(a.x + b.x, a.y + b.y, a.z + b.z);
}

function scale(v, s) {
  return vec3(v.x * s, v.y * s, v.z * s);
}

function dot(a, b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

function cross(a, b) {
  return vec3(
    a.y * b.z - a.z * b.y,
    a.z * b.x - a.x * b.z,
    a.x * b.y - a.y * b.x
  );
}

function getTriangles(vertices, indices) {
  const triangles = [];
  if (indices && indices.length >= 3) {
    for (let i = 0; i + 2 < indices.length; i += 3) {
      triangles.push([
        vertices[indices[i]],
        vertices[indices[i + 1]],
        vertices[indices[i + 2]],
      ]);
    }
    return triangles;
  }
  for (let i = 0; i + 2 < vertices.length; i += 3) {
    triangles.push([vertices[i], vertices[i + 1], vertices[i + 2]]);
  }
  return triangles;
}

function computeBoundingBox(vertices) {
  const min = vec3(Infinity, Infinity, Infinity);
  const max = vec3(-Infinity, -Infinity, -Infinity);

  for (const v of vertices) {
    if (v.x < min.x) min.x = v.x;
    if (v.y < min.y) min.y = v.y;
    if (v.z < min.z) min.z = v.z;
    if (v.x > max.x) max.x = v.x;
    if (v.y > max.y) max.y = v.y;
    if (v.z > max.z) max.z = v.z;
  }

  return {
    min,
    max,
    size: vec3(max.x - min.x, max.y - min.y, max.z - min.z),
    center: vec3((min.x + max.x) / 2, (min.y + max.y) / 2, (min.z + max.z) / 2),
  };
}

function computeProjectedAreas(triangles) {
  let sumX = 0;
  let sumY = 0;
  let sumZ = 0;

  for (const [a, b, c] of triangles) {
    const ab = vec3(b.x - a.x, b.y - a.y, b.z - a.z);
    const ac = vec3(c.x - a.x, c.y - a.y, c.z - a.z);
    const areaNormal = cross(ab, ac);
    const triArea = 0.5 * Math.sqrt(dot(areaNormal, areaNormal));
    if (triArea < EPSILON) continue;

    const invLen = 1 / (2 * triArea);
    const n = scale(areaNormal, invLen);

    sumX += triArea * Math.abs(n.x);
    sumY += triArea * Math.abs(n.y);
    sumZ += triArea * Math.abs(n.z);
  }

  return {
    x: 0.5 * sumX,
    y: 0.5 * sumY,
    z: 0.5 * sumZ,
  };
}

function computeMassAndInertia(triangles, densityKgPerM3) {
  let volume6 = 0;
  let centroidNumerator = vec3(0, 0, 0);

  let intX2 = 0;
  let intY2 = 0;
  let intZ2 = 0;
  let intXY = 0;
  let intXZ = 0;
  let intYZ = 0;

  for (const [a, b, c] of triangles) {
    const det = dot(a, cross(b, c));
    const signedVolume = det / 6;
    volume6 += det;

    const tetraCentroid = scale(add(add(a, b), c), 0.25);
    centroidNumerator = add(centroidNumerator, scale(tetraCentroid, signedVolume));

    const f1x =
      a.x * a.x + b.x * b.x + c.x * c.x + a.x * b.x + a.x * c.x + b.x * c.x;
    const f1y =
      a.y * a.y + b.y * b.y + c.y * c.y + a.y * b.y + a.y * c.y + b.y * c.y;
    const f1z =
      a.z * a.z + b.z * b.z + c.z * c.z + a.z * b.z + a.z * c.z + b.z * c.z;

    const gxy =
      2 * a.x * a.y +
      2 * b.x * b.y +
      2 * c.x * c.y +
      a.x * b.y +
      a.y * b.x +
      a.x * c.y +
      a.y * c.x +
      b.x * c.y +
      b.y * c.x;
    const gxz =
      2 * a.x * a.z +
      2 * b.x * b.z +
      2 * c.x * c.z +
      a.x * b.z +
      a.z * b.x +
      a.x * c.z +
      a.z * c.x +
      b.x * c.z +
      b.z * c.x;
    const gyz =
      2 * a.y * a.z +
      2 * b.y * b.z +
      2 * c.y * c.z +
      a.y * b.z +
      a.z * b.y +
      a.y * c.z +
      a.z * c.y +
      b.y * c.z +
      b.z * c.y;

    intX2 += det * f1x / 60;
    intY2 += det * f1y / 60;
    intZ2 += det * f1z / 60;
    intXY += det * gxy / 120;
    intXZ += det * gxz / 120;
    intYZ += det * gyz / 120;
  }

  const signedVolume = volume6 / 6;
  const volume = Math.abs(signedVolume);
  const mass = volume * densityKgPerM3;

  if (volume < EPSILON || Math.abs(signedVolume) < EPSILON) {
    return {
      volumeM3: 0,
      centerOfMass: vec3(0, 0, 0),
      massKgSolid: 0,
      inertiaTensor: [
        [0, 0, 0],
        [0, 0, 0],
        [0, 0, 0],
      ],
      diagonalizedInertiaTensor: [
        [0, 0, 0],
        [0, 0, 0],
        [0, 0, 0],
      ],
    };
  }

  const centerOfMass = scale(centroidNumerator, 1 / signedVolume);
  const rho = densityKgPerM3;

  const ixxOrigin = rho * (intY2 + intZ2);
  const iyyOrigin = rho * (intX2 + intZ2);
  const izzOrigin = rho * (intX2 + intY2);
  const ixyOrigin = -rho * intXY;
  const ixzOrigin = -rho * intXZ;
  const iyzOrigin = -rho * intYZ;

  const r2 = dot(centerOfMass, centerOfMass);
  const shift = [
    [
      mass * (r2 - centerOfMass.x * centerOfMass.x),
      -mass * centerOfMass.x * centerOfMass.y,
      -mass * centerOfMass.x * centerOfMass.z,
    ],
    [
      -mass * centerOfMass.y * centerOfMass.x,
      mass * (r2 - centerOfMass.y * centerOfMass.y),
      -mass * centerOfMass.y * centerOfMass.z,
    ],
    [
      -mass * centerOfMass.z * centerOfMass.x,
      -mass * centerOfMass.z * centerOfMass.y,
      mass * (r2 - centerOfMass.z * centerOfMass.z),
    ],
  ];

  const inertiaTensor = [
    [
      ixxOrigin - shift[0][0],
      ixyOrigin - shift[0][1],
      ixzOrigin - shift[0][2],
    ],
    [
      ixyOrigin - shift[1][0],
      iyyOrigin - shift[1][1],
      iyzOrigin - shift[1][2],
    ],
    [
      ixzOrigin - shift[2][0],
      iyzOrigin - shift[2][1],
      izzOrigin - shift[2][2],
    ],
  ];

  return {
    volumeM3: volume,
    centerOfMass,
    massKgSolid: mass,
    inertiaTensor,
    diagonalizedInertiaTensor: [
      [inertiaTensor[0][0], 0, 0],
      [0, inertiaTensor[1][1], 0],
      [0, 0, inertiaTensor[2][2]],
    ],
  };
}

function analyzeMesh({
  vertices,
  indices,
  material = "carbonFiberPolished",
  hullThicknessM = 0.002,
}) {
  if (!Array.isArray(vertices) || vertices.length < 3) {
    throw new Error("MeshAnalyzer requires at least 3 vertices.");
  }

  const materialProps = MATERIAL_PROPERTIES[material];
  if (!materialProps) {
    throw new Error(`Unknown material "${material}".`);
  }

  const triangles = getTriangles(vertices, indices);
  const bounds = computeBoundingBox(vertices);
  const projectedAreas = computeProjectedAreas(triangles);
  const massInertia = computeMassAndInertia(triangles, materialProps.densityKgPerM3);

  // Heuristic shell area estimate from principal projections:
  // surface area ≈ 2 * (Ax + Ay + Az) for many near-convex hull-like meshes.
  // Concave meshes can deviate noticeably with no fixed error bound;
  // prefer explicit surface triangulation for high-fidelity mass.
  const shellSurfaceAreaEstimate =
    2 *
    (projectedAreas.x + projectedAreas.y + projectedAreas.z);
  const massKgShell =
    shellSurfaceAreaEstimate * Math.max(hullThicknessM, 0) * materialProps.densityKgPerM3;

  return {
    material,
    materialDensityKgPerM3: materialProps.densityKgPerM3,
    dragScale: materialProps.dragScale,
    boundingBox: bounds,
    projectedAreasM2: projectedAreas,
    volumeM3: massInertia.volumeM3,
    centerOfMass: massInertia.centerOfMass,
    massKgSolid: massInertia.massKgSolid,
    massKgShell,
    inertiaTensorKgM2: massInertia.inertiaTensor,
    diagonalizedInertiaTensorKgM2: massInertia.diagonalizedInertiaTensor,
    inertiaAssumption:
      "Uniform density solid derived from mesh tetrahedral volume integral using selected material density.",
  };
}

module.exports = {
  analyzeMesh,
};
