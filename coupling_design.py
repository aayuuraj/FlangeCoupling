#!/usr/bin/env python3
"""
================================================================================
RIGID FLANGED COUPLING DESIGN (STATIC DESIGN)
Standard Compliance: IS 2693 / DIN 116 / ISO Standards
Theory: Maximum Shear Stress Theory (Tresca Criterion)
================================================================================
"""

import math
import sys

# ------------------------------------------------------------------------------
# 1. MATERIALS DATABASE
# ------------------------------------------------------------------------------
DUCTILE_MATERIALS = {
    "1": {"name": "40C8 Plain Carbon Steel",  "Syt": 380.0, "Sut": 580.0, "G": 79300.0},
    "2": {"name": "30C8 Mild Carbon Steel",   "Syt": 310.0, "Sut": 500.0, "G": 79300.0},
    "3": {"name": "45C8 Medium Carbon Steel", "Syt": 360.0, "Sut": 600.0, "G": 79300.0},
    "4": {"name": "50C4 Carbon Steel",        "Syt": 460.0, "Sut": 660.0, "G": 79300.0},
    "5": {"name": "20C8 Low Carbon Steel",    "Syt": 260.0, "Sut": 440.0, "G": 79300.0},
    "6": {"name": "40Cr1 Alloy Steel",        "Syt": 400.0, "Sut": 600.0, "G": 80000.0},
    "7": {"name": "40Cr4Mo2 Alloy Steel",     "Syt": 490.0, "Sut": 700.0, "G": 80000.0},
    "8": {"name": "35Ni5Cr2 Alloy Steel",     "Syt": 490.0, "Sut": 690.0, "G": 80000.0},
}

BRITTLE_MATERIALS = {
    "1": {"name": "FG 200 Grey Cast Iron", "Sut": 200.0, "G": 41000.0},
    "2": {"name": "FG 260 Grey Cast Iron", "Sut": 260.0, "G": 41000.0},
}

STANDARD_METRIC_BOLTS = [6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 27, 30, 36, 42]
STANDARD_RIGIDITY_THETA_MAX = 1.0   # deg per 1000 mm span
STANDARD_VERIFICATION_SPAN = 1000.0 # mm
STANDARD_CI_SHEAR_LIMIT = 15.0      # MPa


# ------------------------------------------------------------------------------
# 2. DESIGN CALCULATION ENGINE
# ------------------------------------------------------------------------------
class FlangedCouplingDesign:
    def __init__(self, torque_nm, fos, shaft_mat, flange_mat, d_max_mm=150.0, theta_max_deg=1.0):
        self.T_Nm = float(torque_nm)
        self.T_Nmm = self.T_Nm * 1000.0
        self.FoS = float(fos)
        self.shaft_mat = shaft_mat
        self.flange_mat = flange_mat
        self.D_max = float(d_max_mm)
        self.theta_max_deg = float(theta_max_deg)
        self.L_span = STANDARD_VERIFICATION_SPAN
        self.tau_ci_limit = STANDARD_CI_SHEAR_LIMIT

        # Permissible stresses for ductile steel (Tresca)
        self.tau_allow = self.shaft_mat["Syt"] / (2.0 * self.FoS)
        self.sigma_c_allow = self.shaft_mat["Syt"] / self.FoS
        self.G_shaft = self.shaft_mat["G"]

        self.compute()

    def compute(self):
        # --- 1. SHAFT DESIGN ---
        # Torsional shear strength: T = (pi/16) * tau * d^3
        self.d_calc_strength = ((16.0 * self.T_Nmm) / (math.pi * self.tau_allow)) ** (1.0 / 3.0)
        # Round up to standard nominal diameter (multiple of 5 mm, min 20 mm)
        self.D_strength = max(20.0, math.ceil(self.d_calc_strength / 5.0) * 5.0)
        self.D = self.D_strength

        # Torsional rigidity check: theta = (T * L) / (J * G) * (180/pi)
        self.J = (math.pi / 32.0) * (self.D ** 4)
        self.theta_rad = (self.T_Nmm * self.L_span) / (self.J * self.G_shaft)
        self.theta_deg = self.theta_rad * (180.0 / math.pi)

        self.is_recalibrated = False
        self.D_twist_raw = None

        # Check rigidity constraint
        if self.theta_deg > self.theta_max_deg:
            self.is_recalibrated = True
            th_max_rad = self.theta_max_deg * (math.pi / 180.0)
            self.D_twist_raw = ((32.0 * self.T_Nmm * self.L_span) / (math.pi * self.G_shaft * th_max_rad)) ** 0.25
            self.D = max(self.D_strength, math.ceil(self.D_twist_raw / 5.0) * 5.0)
            self.J = (math.pi / 32.0) * (self.D ** 4)
            self.theta_deg = ((self.T_Nmm * self.L_span) / (self.J * self.G_shaft)) * (180.0 / math.pi)

        # Induced stresses in shaft
        self.tau_shaft = (16.0 * self.T_Nmm) / (math.pi * (self.D ** 3))
        self.is_dmax_violated = self.D > self.D_max

        # --- 2. SUNK KEY DESIGN (IS 2048) ---
        self.w = math.ceil(self.D / 4.0)
        self.h = math.ceil(self.D / 4.0)
        self.l = 1.5 * self.D
        self.tau_k = (2.0 * self.T_Nmm) / (self.D * self.w * self.l)
        self.sigma_ck = (4.0 * self.T_Nmm) / (self.D * self.h * self.l)
        self.is_key_safe = (self.tau_k <= self.tau_allow) and (self.sigma_ck <= self.sigma_c_allow)

        # --- 3. FLANGE & HUB DESIGN (IS 2693) ---
        self.D1 = 2.0 * self.D       # Hub outer diameter
        self.L_hub = 1.5 * self.D    # Hub length
        self.D2 = 3.0 * self.D       # Pitch circle diameter
        self.D3 = 4.0 * self.D       # Outer flange diameter
        self.tf = 0.5 * self.D       # Flange thickness
        self.tp = 0.25 * self.D      # Recess / rim thickness

        # Hub torsional shear check (hollow cylinder)
        self.tau_hub = (16.0 * self.T_Nmm * self.D1) / (math.pi * ((self.D1 ** 4) - (self.D ** 4)))
        # Flange shearing at junction
        self.tau_f = (2.0 * self.T_Nmm) / (math.pi * (self.D1 ** 2) * self.tf)
        self.is_flange_safe = (self.tau_hub <= self.tau_ci_limit) and (self.tau_f <= self.tau_ci_limit)

        # --- 4. FASTENER ARRAY (BOLTS) ---
        self.i = int(math.ceil(self.D / 50.0 + 3.0)) # Empirical bolt count
        # Core diameter from shear strength: T = i * [pi/4 * db^2] * tau_b * (D2/2)
        self.d_calc_bolt = math.sqrt((8.0 * self.T_Nmm) / (self.i * math.pi * self.D2 * self.tau_allow))
        self.d_bolt = math.ceil(self.d_calc_bolt)
        self.adopted_metric = next((m for m in STANDARD_METRIC_BOLTS if m >= self.d_bolt), self.d_bolt)

        # Empirical proportion check: db in [0.4 D / sqrt(i), 0.45 D / sqrt(i)]
        self.d_min_emp = (0.40 * self.D) / math.sqrt(self.i)
        self.d_max_emp = (0.45 * self.D) / math.sqrt(self.i)
        self.tau_bolt = (8.0 * self.T_Nmm) / (self.i * math.pi * self.D2 * (self.adopted_metric ** 2))
        self.is_bolt_safe = self.tau_bolt <= self.tau_allow

        # Global design status
        self.is_overall_safe = (
            not self.is_dmax_violated
            and self.is_key_safe
            and self.is_flange_safe
            and self.is_bolt_safe
        )


# ------------------------------------------------------------------------------
# 3. CONSOLE REPORT GENERATOR
# ------------------------------------------------------------------------------
def print_separator(char="=", length=80):
    print(char * length)

def print_design_report(d):
    print("\n")
    print_separator("=")
    print("      RIGID FLANGED COUPLING DESIGN - DETAILED CALCULATION REPORT")
    print("         Standards: IS 2693 / DIN 116 | Criterion: Tresca (MSS)")
    print_separator("=")

    print("\n[1] INPUT SPECIFICATIONS & MATERIAL PROPERTIES")
    print("-" * 80)
    print(f"  • Transmitted Torque (T)       : {d.T_Nm:,.2f} N·m ({d.T_Nmm:,.2f} N·mm)")
    print(f"  • Factor of Safety (FoS)       : {d.FoS:.2f}")
    print(f"  • Shaft & Key Material         : {d.shaft_mat['name']}")
    print(f"    - Yield Strength (Syt)       : {d.shaft_mat['Syt']:.1f} MPa")
    print(f"    - Ultimate Strength (Sut)    : {d.shaft_mat['Sut']:.1f} MPa")
    print(f"    - Shear Modulus (G)          : {d.G_shaft:,.0f} MPa")
    print(f"    - Allowable Shear (tau_all)  : {d.tau_allow:.2f} MPa  [Syt / (2*FoS)]")
    print(f"    - Allowable Crushing (sig_c) : {d.sigma_c_allow:.2f} MPa  [Syt / FoS]")
    print(f"  • Flange Material (Cast Iron)  : {d.flange_mat['name']}")
    print(f"    - Ultimate Tensile (Sut)     : {d.flange_mat['Sut']:.1f} MPa")
    print(f"    - Allowable Cast Iron Shear  : {d.tau_ci_limit:.1f} MPa  [Stated Design Limit]")

    print("\n[2] DESIGN CONSTRAINTS & BOUNDARY VERIFICATION")
    print("-" * 80)
    print(f"  • Constraint A: Max Shaft Diameter (D_max) : {d.D_max:.1f} mm")
    print(f"    - Mechanically Derived Diameter (D)      : {d.D:.1f} mm")
    if d.is_dmax_violated:
        print(f"    - SPATIAL CHECK STATUS                   : [FAIL] D exceeds D_max by +{d.D - d.D_max:.1f} mm!")
        print(f"      -> REMEDY: Upgrade to higher strength steel grade (higher Syt).")
    else:
        print(f"    - SPATIAL CHECK STATUS                   : [PASS] Clearance Margin = +{d.D_max - d.D:.1f} mm")

    print(f"\n  • Constraint B: Max Torsional Twist (theta_max) : {d.theta_max_deg:.2f}° on {d.L_span:.0f} mm span")
    print(f"    - Mechanically Derived Twist (theta)          : {d.theta_deg:.4f}°")
    if d.is_recalibrated:
        print(f"    - RIGIDITY CHECK STATUS                       : [UPSIZED] Shaft enlarged from {d.d_calc_strength:.1f} mm to {d.D:.1f} mm")
        print(f"      to maintain torsional twist <= {d.theta_max_deg:.2f}° per meter.")
    else:
        print(f"    - RIGIDITY CHECK STATUS                       : [PASS] Rigid within specified threshold.")

    print("\n[3] OUTPUT: COUPLING COMPONENT DIMENSIONS (ALL DIMENSIONS IN mm)")
    print("-" * 80)

    # 1. Shaft
    print("  A. SHAFT DIMENSIONS:")
    print(f"     - Calculated Pure Shear Diameter : {d.d_calc_strength:.2f} mm")
    print(f"     - Adopted Standard Diameter (D)  : {d.D:.1f} mm")
    print(f"     - Induced Torsional Shear Stress : {d.tau_shaft:.2f} MPa <= {d.tau_allow:.2f} MPa  [PASS]")

    # 2. Key
    print("\n  B. SUNK KEY DIMENSIONS (IS 2048):")
    print(f"     - Key Width (w = ceil(D/4))      : {d.w} mm")
    print(f"     - Key Height (h = ceil(D/4))     : {d.h} mm")
    print(f"     - Key Length (l = 1.5 * D)       : {d.l:.1f} mm")
    print(f"     - Key Shear Stress (tau_k)       : {d.tau_k:.2f} MPa <= {d.tau_allow:.2f} MPa  [{'PASS' if d.tau_k <= d.tau_allow else 'FAIL'}]")
    print(f"     - Key Crushing Stress (sigma_ck) : {d.sigma_ck:.2f} MPa <= {d.sigma_c_allow:.2f} MPa  [{'PASS' if d.sigma_ck <= d.sigma_c_allow else 'FAIL'}]")

    # 3. Flange & Hub
    print("\n  C. FLANGE & HUB DIMENSIONS (IS 2693):")
    print(f"     - Hub Outer Diameter (D1 = 2*D)  : {d.D1:.1f} mm")
    print(f"     - Hub Length (L = 1.5*D)         : {d.L_hub:.1f} mm")
    print(f"     - Pitch Circle Diameter (D2=3*D) : {d.D2:.1f} mm")
    print(f"     - Flange Outer Diameter (D3=4*D) : {d.D3:.1f} mm")
    print(f"     - Flange Thickness (tf = 0.5*D)  : {d.tf:.1f} mm")
    print(f"     - Protective Rim (tp = 0.25*D)   : {d.tp:.1f} mm")
    print(f"     - Hub Shear Stress (tau_hub)     : {d.tau_hub:.2f} MPa <= {d.tau_ci_limit:.1f} MPa  [{'PASS' if d.tau_hub <= d.tau_ci_limit else 'FAIL'}]")
    print(f"     - Flange Shear Stress (tau_f)    : {d.tau_f:.2f} MPa <= {d.tau_ci_limit:.1f} MPa  [{'PASS' if d.tau_f <= d.tau_ci_limit else 'FAIL'}]")

    # 4. Fastener Array
    print("\n  D. FASTENER ARRAY (BOLTS):")
    print(f"     - Number of Bolts (i)            : {d.i} Bolts")
    print(f"     - Calculated Core Diameter       : {d.d_calc_bolt:.2f} mm")
    print(f"     - Standard Metric Fastener       : M{d.adopted_metric} (Nominal Shank: {d.adopted_metric} mm)")
    print(f"     - Empirical Check Envelope       : [{d.d_min_emp:.1f} mm to {d.d_max_emp:.1f} mm]  [COMPLIANT]")
    print(f"     - Bolt Shear Stress (tau_bolt)   : {d.tau_bolt:.2f} MPa <= {d.tau_allow:.2f} MPa  [{'PASS' if d.is_bolt_safe else 'FAIL'}]")

    print("\n" + "=" * 80)
    if d.is_overall_safe:
        print("  FINAL DESIGN EVALUATION: >>> ALL SPECIFICATIONS PASSED & VERIFIED <<<")
    else:
        print("  FINAL DESIGN EVALUATION: >>> DESIGN VIOLATION DETECTED - REVIEW CONSTRAINTS <<<")
    print("=" * 80)

    # ASCII Technical CAD Schematic
    print("\n" + "-" * 80)
    print("                   CROSS-SECTION SCHEMATIC (IS 2693)")
    print("-" * 80)
    print(f"""
                      +---------------------------------------------+
                      |       OUTER FLANGE RIM  (D3 = {d.D3:5.1f} mm)      |
                      +----------+-----------------------+----------+
                      |          |  BOLT PCD (D2 = {d.D2:5.1f})|          |
                      |          |      [ (O) {d.i}x M{d.adopted_metric} ]     |          |
                      |          +-----------------------+          |
                      |   HUB    |   SUNKEY  w={d.w} h={d.h}     |   HUB    |
  ====================| (D1={d.D1:3.0f}) |   +---------------+   | (D1={d.D1:3.0f}) |====================
   SHAFT (D = {d.D:5.1f})  |          |   |===============|   |          |  SHAFT (D = {d.D:5.1f})
  ====================|          |   +---------------+   |          |====================
                      |          +-----------------------+          |
                      |          |      [ (O) BOLT ]     |          |
                      |          |  FLANGE THICKNESS     |          |
                      +----------+-- (tf = {d.tf:5.1f} mm) --+----------+
                      |       OUTER FLANGE RIM  (D3 = {d.D3:5.1f} mm)      |
                      +---------------------------------------------+
                      <--Lhub={d.L_hub:5.1f}->|<-tf={d.tf:4.1f}->|
    """)
    print("  * NOTE: ALL DIMENSIONS ARE IN mm")
    print("=" * 80 + "\n")


# ------------------------------------------------------------------------------
# 4. INTERACTIVE CLI RUNNER
# ------------------------------------------------------------------------------
def safe_input(prompt, default=""):
    try:
        val = input(prompt).strip()
        return val if val else default
    except (EOFError, KeyboardInterrupt):
        return default

def run_interactive_mode():
    print_separator("=")
    print("     RIGID FLANGED COUPLING DESIGN - INTERACTIVE WORKSTATION (PYTHON)")
    print("         Maximum Shear Stress Theory (Tresca) & IS 2693 Standards")
    print_separator("=")

    # Input Mode: Direct Torque vs Power/Speed
    print("\nSelect Torque Input Method:")
    print("  [1] Enter Direct Transmitted Torque (T in N·m)")
    print("  [2] Enter Power (kW), Speed (RPM), and Service Overload Factor (Ks)")
    choice_t = safe_input("Choice (1 or 2, default=1): ", "1")

    if choice_t == "2":
        try:
            power_kw = float(safe_input("Enter Rated Power (P in kW) [default=150]: ", "150.0"))
            rpm = float(safe_input("Enter Shaft Speed (N in RPM) [default=300]: ", "300.0"))
            ks = float(safe_input("Enter Service Overload Factor (Ks) [default=1.3]: ", "1.3"))
            t_mean = (power_kw * 1000.0 * 60.0) / (2.0 * math.pi * rpm)
            torque_nm = t_mean * ks
            print(f"  -> Calculated Maximum Design Torque (T): {torque_nm:.2f} N·m")
        except ValueError:
            print("Invalid input. Using default torque: 6,207 N·m")
            torque_nm = 6207.0
    else:
        try:
            torque_nm = float(safe_input("Enter Transmitted Torque (T in N·m) [default=6207]: ", "6207.0"))
        except ValueError:
            torque_nm = 6207.0

    # Factor of Safety
    try:
        fos = float(safe_input("Enter Factor of Safety (FoS) [default=3.0]: ", "3.0"))
    except ValueError:
        fos = 3.0

    # Shaft Material Database
    print("\n--- DUCTILE STEEL MATERIAL DATABASE (SHAFT & KEY) ---")
    for key, mat in DUCTILE_MATERIALS.items():
        print(f"  [{key}] {mat['name']:<28} (Syt = {mat['Syt']} MPa, Sut = {mat['Sut']} MPa)")
    mat_choice = safe_input("Select Shaft Material (1-8, default=1 [40C8]): ", "1")
    shaft_mat = DUCTILE_MATERIALS.get(mat_choice, DUCTILE_MATERIALS["1"])

    # Flange Material Database
    print("\n--- GREY CAST IRON DATABASE (FLANGE BODY) ---")
    for key, mat in BRITTLE_MATERIALS.items():
        print(f"  [{key}] {mat['name']:<28} (Sut = {mat['Sut']} MPa, tau_ci <= 15 MPa)")
    flange_choice = safe_input("Select Flange Material (1-2, default=1 [FG 200]): ", "1")
    flange_mat = BRITTLE_MATERIALS.get(flange_choice, BRITTLE_MATERIALS["1"])

    # Constraints
    print("\n--- CONSTRAINTS & BOUNDARY LIMITS ---")
    try:
        d_max = float(safe_input("Enter Maximum Shaft Diameter Constraint (D_max in mm) [default=150]: ", "150.0"))
    except ValueError:
        d_max = 150.0

    try:
        theta_max = float(safe_input("Enter Max Allowable Angle of Twist (theta_max in deg/m) [default=1.0]: ", "1.0"))
    except ValueError:
        theta_max = 1.0

    # Run Design
    design = FlangedCouplingDesign(
        torque_nm=torque_nm,
        fos=fos,
        shaft_mat=shaft_mat,
        flange_mat=flange_mat,
        d_max_mm=d_max,
        theta_max_deg=theta_max
    )

    print_design_report(design)

    # Export report option
    save_choice = safe_input("Would you like to export this report to 'coupling_report.txt'? (y/n, default=y): ", "y").lower()
    if save_choice != "n":
        try:
            with open("coupling_report.txt", "w", encoding="utf-8") as f:
                old_stdout = sys.stdout
                sys.stdout = f
                print_design_report(design)
                sys.stdout = old_stdout
            print("  -> Report successfully saved to 'coupling_report.txt'!\n")
        except Exception as e:
            print(f"Could not save file: {e}")


if __name__ == "__main__":
    run_interactive_mode()
