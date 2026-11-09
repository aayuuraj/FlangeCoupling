/**
- ============================================================================
- RIGID FLANGED COUPLING DESIGN (STATIC DESIGN) - C++ WORKSTATION
- Standard: IS 2693 / DIN 116 / ISO Standards
- Theory: Maximum Shear Stress Theory (Tresca Criterion)
- ============================================================================
 */

#include <iostream>
#include <iomanip>
#include <cmath>
#include <string>
#include <vector>

using namespace std;
typedef long long ll;

struct Material {
    string name;
    double Syt;  // Yield strength (MPa)
    double Sut;  // Ultimate strength (MPa)
    double G;    // Shear modulus (MPa)
};

// Standard metric bolt nominal diameters
const vector<int> METRIC_BOLTS = {6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 27, 30, 36, 42};

int main() {
    ios_base::sync_with_stdio(false); cin.tie(NULL);

    cout << "================================================================================\n";
    cout << "    RIGID FLANGED COUPLING DESIGN - C++ WORKSTATION (IS 2693 / TRESCA)\n";
    cout << "================================================================================\n\n";

    // 1. Materials Database
    vector<Material> ductileMaterials = {
        {"40C8 Plain Carbon Steel",  380.0, 580.0, 79300.0},
        {"30C8 Mild Carbon Steel",   310.0, 500.0, 79300.0},
        {"45C8 Medium Carbon Steel", 360.0, 600.0, 79300.0},
        {"50C4 Carbon Steel",        460.0, 660.0, 79300.0},
        {"20C8 Low Carbon Steel",    260.0, 440.0, 79300.0},
        {"40Cr1 Alloy Steel",        400.0, 600.0, 80000.0},
        {"40Cr4Mo2 Alloy Steel",     490.0, 700.0, 80000.0},
        {"35Ni5Cr2 Alloy Steel",     490.0, 690.0, 80000.0}
    };

    vector<Material> brittleMaterials = {
        {"FG 200 Grey Cast Iron", 0.0, 200.0, 41000.0},
        {"FG 260 Grey Cast Iron", 0.0, 260.0, 41000.0}
    };

    // User Inputs
    int torqueMode = 1;
    cout << "Select Torque Input Method:\n";
    cout << "  [1] Enter Direct Transmitted Torque (T in N*m)\n";
    cout << "  [2] Enter Power (kW), Speed (RPM), and Service Factor (Ks)\n";
    cout << "Choice (1 or 2): ";
    if (!(cin >> torqueMode)) torqueMode = 1;

    double T_Nm = 6207.0;
    if (torqueMode == 2) {
        double P_kW = 150.0, N_rpm = 300.0, Ks = 1.3;
        cout << "Enter Rated Power (P in kW): ";
        cin >> P_kW;
        cout << "Enter Shaft Speed (N in RPM): ";
        cin >> N_rpm;
        cout << "Enter Service Overload Factor (Ks): ";
        cin >> Ks;
        double T_mean = (P_kW * 1000.0 * 60.0) / (2.0 * M_PI * N_rpm);
        T_Nm = T_mean * Ks;
        cout << "-> Calculated Design Torque: " << fixed << setprecision(2) << T_Nm << " N*m\n";
    } else {
        cout << "Enter Transmitted Torque (T in N*m): ";
        cin >> T_Nm;
    }

    double FoS = 3.0;
    cout << "Enter Factor of Safety (FoS): ";
    cin >> FoS;

    cout << "\n--- DUCTILE STEEL MATERIAL DATABASE (SHAFT & KEY) ---\n";
    for (size_t idx = 0; idx < ductileMaterials.size(); ++idx) {
        cout << "  [" << (idx + 1) << "] " << ductileMaterials[idx].name
             << " (Syt=" << ductileMaterials[idx].Syt << " MPa)\n";
    }
    int shaftChoice = 1;
    cout << "Select Shaft Material (1-8): ";
    cin >> shaftChoice;
    if (shaftChoice < 1 || shaftChoice > (int)ductileMaterials.size()) shaftChoice = 1;
    Material shaftMat = ductileMaterials[shaftChoice - 1];

    cout << "\n--- CAST IRON DATABASE (FLANGE BODY) ---\n";
    for (size_t idx = 0; idx < brittleMaterials.size(); ++idx) {
        cout << "  [" << (idx + 1) << "] " << brittleMaterials[idx].name
             << " (Sut=" << brittleMaterials[idx].Sut << " MPa)\n";
    }
    int flangeChoice = 1;
    cout << "Select Flange Material (1-2): ";
    cin >> flangeChoice;
    if (flangeChoice < 1 || flangeChoice > (int)brittleMaterials.size()) flangeChoice = 1;
    Material flangeMat = brittleMaterials[flangeChoice - 1];

    double D_max = 150.0;
    cout << "Enter Maximum Shaft Diameter Constraint (D_max in mm): ";
    cin >> D_max;

    double theta_max_deg = 1.0;
    cout << "Enter Maximum Allowable Angle of Twist (theta_max in deg/m): ";
    cin >> theta_max_deg;

    // --- MECHANICAL CALCULATIONS ---
    double T_Nmm = T_Nm * 1000.0;
    double tau_allow = shaftMat.Syt / (2.0 * FoS);
    double sigma_c_allow = shaftMat.Syt / FoS;
    
    // 1. Shaft diameter from shear strength
    double d_calc_str = cbrt((16.0 * T_Nmm) / (M_PI * tau_allow));
    double D = max(20.0, ceil(d_calc_str / 5.0) * 5.0);

    // Torsional rigidity check on L = 1000 mm span
    double L_span = 1000.0;
    double J = (M_PI / 32.0) * pow(D, 4);
    double theta_rad = (T_Nmm * L_span) / (J * shaftMat.G);
    double theta_deg = theta_rad * (180.0 / M_PI);
    bool isRecalibrated = false;

    if (theta_deg > theta_max_deg) {
        isRecalibrated = true;
        double th_max_rad = theta_max_deg * (M_PI / 180.0);
        double D_twist_raw = pow((32.0 * T_Nmm * L_span) / (M_PI * shaftMat.G * th_max_rad), 0.25);
        D = max(D, ceil(D_twist_raw / 5.0) * 5.0);
        J = (M_PI / 32.0) * pow(D, 4);
        theta_deg = ((T_Nmm * L_span) / (J * shaftMat.G)) * (180.0 / M_PI);
    }

    double tau_shaft = (16.0 * T_Nmm) / (M_PI * pow(D, 3));
    bool isDmaxViolated = D > D_max;

    // 2. Sunk Key Dimensions (IS 2048)
    int w = ceil(D / 4.0);
    int h = ceil(D / 4.0);
    double l = 1.5 * D;
    double tau_k = (2.0 * T_Nmm) / (D * w * l);
    double sigma_ck = (4.0 * T_Nmm) / (D * h * l);

    // 3. Flange & Hub Dimensions (IS 2693)
    double D1 = 2.0 * D;
    double L_hub = 1.5 * D;
    double D2 = 3.0 * D;
    double D3 = 4.0 * D;
    double tf = 0.5 * D;
    double tp = 0.25 * D;
    double tau_hub = (16.0 * T_Nmm * D1) / (M_PI * (pow(D1, 4) - pow(D, 4)));
    double tau_f = (2.0 * T_Nmm) / (M_PI * pow(D1, 2) * tf);

    // 4. Fastener Array (Bolts)
    int i = ceil(D / 50.0 + 3.0);
    double d_calc_bolt = sqrt((8.0 * T_Nmm) / (i * M_PI * D2 * tau_allow));
    int d_bolt_ceil = ceil(d_calc_bolt);
    int adopted_metric = d_bolt_ceil;
    for (int mb : METRIC_BOLTS) {
        if (mb >= d_bolt_ceil) { adopted_metric = mb; break; }
    }
    double tau_bolt = (8.0 * T_Nmm) / (i * M_PI * D2 * pow(adopted_metric, 2));

    // Print Detailed Report
    cout << "\n================================================================================\n";
    cout << "                 COUPLING CALCULATION RESULTS (ALL DIMS IN mm)\n";
    cout << "================================================================================\n";
    cout << fixed << setprecision(2);
    
    // 1. SHAFT DIMENSIONS
    cout << "1. SHAFT DIMENSIONS:\n";
    cout << "   - Calculated Strength Dia: " << d_calc_str << " mm\n";
    cout << "   - Adopted Standard Dia D : " << D << " mm\n";
    cout << "   - Induced Shear Stress   : " << tau_shaft << " MPa <= " << tau_allow << " MPa " 
         << (tau_shaft <= tau_allow ? "[PASS]" : "[FAIL]") << "\n";
    cout << "   - Angular Twist on 1m    : " << setprecision(4) << theta_deg << " deg <= " << theta_max_deg << " deg\n";
    cout << "   - Constraint Check (Dmax): " << (isDmaxViolated ? "[FAIL: D > D_max]" : "[PASS: D <= D_max]") << "\n\n";

    cout << setprecision(2);
    
    // 2. KEY DIMENSIONS
    cout << "2. KEY DIMENSIONS (IS 2048):\n";
    cout << "   - Width (w): " << w << " mm | Height (h): " << h << " mm | Length (l): " << l << " mm\n";
    cout << "   - Induced Shear Stress   : " << tau_k << " MPa <= " << tau_allow << " MPa " 
         << (tau_k <= tau_allow ? "[PASS]" : "[FAIL]") << "\n";
    cout << "   - Induced Crushing Stress: " << sigma_ck << " MPa <= " << sigma_c_allow << " MPa " 
         << (sigma_ck <= sigma_c_allow ? "[PASS]" : "[FAIL]") << "\n\n";

    // 3. FLANGE & HUB DIMENSIONS
    cout << "3. FLANGE & HUB DIMENSIONS (IS 2693):\n";
    cout << "   - Hub Outer Dia (D1): " << D1 << " mm | Hub Length (L): " << L_hub << " mm\n";
    cout << "   - Pitch Circle  (D2): " << D2 << " mm | Flange Outer Dia (D3): " << D3 << " mm\n";
    cout << "   - Flange Thickness  : " << tf << " mm | Protective Rim (tp) : " << tp << " mm\n";
    cout << "   - Hub Shear Stress  : " << tau_hub << " MPa <= 15.0 MPa " 
         << (tau_hub <= 15.0 ? "[PASS]" : "[FAIL]") << "\n";
    cout << "   - Flange Shear Stress: " << tau_f << " MPa <= 15.0 MPa " 
         << (tau_f <= 15.0 ? "[PASS]" : "[FAIL]") << "\n\n";

    // 4. BOLT ARRAY
    cout << "4. BOLT ARRAY:\n";
    cout << "   - Number of Bolts (i): " << i << " Bolts\n";
    cout << "   - Core Diameter Calc : " << d_calc_bolt << " mm\n";
    cout << "   - Adopted Fastener   : M" << adopted_metric << " (Shear Stress = " << tau_bolt 
         << " MPa <= " << tau_allow << " MPa) " << (tau_bolt <= tau_allow ? "[PASS]" : "[FAIL]") << "\n";

    // 5. GLOBAL STATUS & REMEDIATION ENGINE
    bool allSafe = (tau_shaft <= tau_allow) && (tau_k <= tau_allow) && (sigma_ck <= sigma_c_allow) && 
                   (tau_hub <= 15.0) && (tau_f <= 15.0) && (tau_bolt <= tau_allow) && (!isDmaxViolated);

    cout << "================================================================================\n";
    if (allSafe) {
        cout << "STATUS: >>> ALL DESIGN SPECIFICATIONS VERIFIED & COMPLIANT <<<\n";
    } else {
        cout << "STATUS: >>> CRITICAL FAILURE DETECTED <<<\n";
        cout << "\n--- DIAGNOSIS ---\n";
        if (isDmaxViolated) {
            cout << "[!] SHAFT SPATIAL CONFLICT: Required D exceeds D_max.\n";
            cout << "    -> FIX: Select a higher-grade alloy steel (e.g., 40Cr4Mo2) to increase Syt, or reduce FoS.\n";
        }
        if (tau_k > tau_allow || sigma_ck > sigma_c_allow) {
            cout << "[!] KEY STRESS FAILURE: Key dimensions (derived from D) cannot handle the torque.\n";
            cout << "    -> FIX: Artificially upsize the shaft by tightening the max twist limit (theta_max), which will force a larger key.\n";
        }
        if (tau_hub > 15.0 || tau_f > 15.0) {
            cout << "[!] CAST IRON SHEAR FAILURE: Flange body is yielding.\n";
            cout << "    -> FIX: Upgrade to FG 260 Cast Iron, or increase geometric multipliers for Hub Dia/Flange Thickness.\n";
        }
        if (tau_bolt > tau_allow) {
            cout << "[!] FASTENER SHEAR FAILURE: Bolts are yielding at the pitch circle.\n";
            cout << "    -> FIX: Manually adopt a larger metric bolt size than M" << adopted_metric << ".\n";
        }
    }
    cout << "================================================================================\n\n";

    return 0;
}