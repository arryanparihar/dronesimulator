const test = require("node:test");
const assert = require("node:assert/strict");
const { analyzeMesh } = require("../src/physics/MeshAnalyzer");

function cubeMesh(sideLength = 2) {
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
    0, 2, 1, 0, 3, 2, // back
    4, 5, 6, 4, 6, 7, // front
    0, 1, 5, 0, 5, 4, // bottom
    3, 6, 2, 3, 7, 6, // top
    1, 2, 6, 1, 6, 5, // right
    0, 4, 7, 0, 7, 3, // left
  ];

  return { vertices, indices };
}

test("MeshAnalyzer computes volume, COM, and projected areas for a cube", () => {
  const { vertices, indices } = cubeMesh(2);
  const result = analyzeMesh({
    vertices,
    indices,
    material: "carbonFiberPolished",
    hullThicknessM: 0.002,
  });

  assert.ok(Math.abs(result.volumeM3 - 8) < 1e-9);
  assert.ok(Math.abs(result.centerOfMass.x) < 1e-9);
  assert.ok(Math.abs(result.centerOfMass.y) < 1e-9);
  assert.ok(Math.abs(result.centerOfMass.z) < 1e-9);

  assert.ok(Math.abs(result.projectedAreasM2.x - 4) < 1e-9);
  assert.ok(Math.abs(result.projectedAreasM2.y - 4) < 1e-9);
  assert.ok(Math.abs(result.projectedAreasM2.z - 4) < 1e-9);

  assert.ok(result.massKgSolid > 0);
  assert.ok(result.massKgShell > 0);
  assert.equal(result.diagonalizedInertiaTensorKgM2[0][1], 0);
});
