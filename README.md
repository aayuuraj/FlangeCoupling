# Rigid Flanged Coupling CAD & Design Workstation

A standalone CAD telemetry and engineering design workstation for the static design of **Rigid Flanged Couplings (Protected Type)** complying with standard machine design criteria (**IS 2693 / DIN 116 / ISO Standards** and **Tresca Maximum Shear Stress Theory**).

---

## Features

- **Technical CAD Schematic Viewport**: Live SVG-rendered cross-section (Section A-A), elevation, and dual drafting views with dimensions, ISO hatching, centerlines, and bolt arrays.
- **Direct Torque & Power/Speed Calculator**: Primary torque input ($T$) with secondary speed ($N$), rated power ($P$), and service factor ($K_s$) sync.
- **Materials Database**: Ductile steel grades (e.g., 40C8, 30C8, 45C8, 40Cr4Mo2) and grey cast iron flange materials (FG 200, FG 260) with mechanical properties ($S_{yt}, S_{ut}, G, \tau_{allow}, \sigma_{c,allow}$).
- **Rigorous Constraint & Rigidity Verification**:
  - **Spatial Envelope ($D_{max}$)**: Verification against machine packaging clearance.
  - **Torsional Rigidity ($\theta_{max}$)**: Automatic stiffness recalibration if angular twist exceeds standard threshold ($\theta \le 1.0^\circ$ per 1000 mm span).
- **Comprehensive Output Telemetry**:
  - Shaft diameter ($D$)
  - Sunk key dimensions ($w, h, l$) with shear & crushing stress checks
  - Flange & hub dimensions ($D_1, L_{hub}, D_2, D_3, t_f, t_p$) with shear stress checks
  - Fastener array ($i \times M$) with empirical proportions and bolt shear checks
- **Desktop Engineering Themes**: Studio Light (default), Blueprint Drafting Paper, and Titanium Dark.
- **Export & Reporting**: Complete printable engineering calculation dossier and plaintext export.

---

## Quick Start

Open `index.html` in any modern web browser:

```bash
open index.html
```

No build steps or dependencies required.
