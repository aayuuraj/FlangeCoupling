export const ductileMaterials = [
  { id: "30c8", name: "30C8 (Plain Carbon Steel)", tensile: 500, yield: 400, g: 79000 },
  { id: "40c8", name: "40C8 (Plain Carbon Steel)", tensile: 580, yield: 380, g: 79000 },
  { id: "45c8", name: "45C8 (Plain Carbon Steel)", tensile: 630, yield: 380, g: 79000 },
  { id: "50c4", name: "50C4 (Plain Carbon Steel)", tensile: 660, yield: 460, g: 79000 },
  { id: "55c8", name: "55C8 (Plain Carbon Steel)", tensile: 720, yield: 460, g: 79000 },
  { id: "40cr4", name: "40Cr4 (Alloy Steel)", tensile: 690, yield: 490, g: 80000 },
  { id: "40cr4mo2", name: "40Cr4Mo2 (Alloy Steel)", tensile: 700, yield: 490, g: 80000 },
  { id: "35ni5cr2", name: "35Ni5Cr2 (Alloy Steel)", tensile: 690, yield: 490, g: 80000 }
];

export const brittleMaterials = [
  { id: "fg200", name: "FG 200 (Grey Cast Iron)", tensile: 200, yield: 200, g: 41000 },
  { id: "fg260", name: "FG 260 (Grey Cast Iron)", tensile: 260, yield: 260, g: 41000 }
];

export function getAllowableStresses(material, fos, isDuctile) {
  let tau_allow, sigma_c_allow;
  
  if (isDuctile) {
    tau_allow = material.yield / (2 * fos);
    sigma_c_allow = material.yield / fos;
  } else {
    tau_allow = material.tensile / (2 * fos); 
    sigma_c_allow = material.tensile / fos;
  }
  
  return {
    tau_allow,
    sigma_c_allow,
    g: material.g
  };
}
