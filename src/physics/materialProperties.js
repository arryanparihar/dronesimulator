const MATERIAL_PROPERTIES = {
  plaMatte3DPrinted: {
    label: "Matte 3D-Printed PLA",
    densityKgPerM3: 1240,
    dragScale: 1.2,
  },
  carbonFiberPolished: {
    label: "Polished Carbon Fiber",
    densityKgPerM3: 1600,
    dragScale: 0.78,
  },
  anodizedAluminum: {
    label: "Anodized Aluminum",
    densityKgPerM3: 2700,
    dragScale: 0.9,
  },
  fiberglassGlossy: {
    label: "Glossy Fiberglass",
    densityKgPerM3: 1850,
    dragScale: 0.85,
  },
};

module.exports = {
  MATERIAL_PROPERTIES,
};
