const { analyzeMesh } = require("./physics/MeshAnalyzer");
const {
  computeAerodynamics,
  updateMotorRPM,
} = require("./physics/AerodynamicsEngine");
const { MATERIAL_PROPERTIES } = require("./physics/materialProperties");
const { createDroneSimulator } = require("./simulator/DroneSimulator");

module.exports = {
  analyzeMesh,
  computeAerodynamics,
  updateMotorRPM,
  MATERIAL_PROPERTIES,
  createDroneSimulator,
};
