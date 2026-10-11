import { ductileMaterials, brittleMaterials, getAllowableStresses } from './materials.js';

document.addEventListener('DOMContentLoaded', () => {
  const shaftMaterialSelect = document.getElementById('shaftMaterial');
  const flangeMaterialSelect = document.getElementById('flangeMaterial');

  ductileMaterials.forEach(m => {
    const opt = document.createElement('option');
    opt.value = m.id;
    opt.textContent = m.name;
    shaftMaterialSelect.appendChild(opt);
  });

  brittleMaterials.forEach(m => {
    const opt = document.createElement('option');
    opt.value = m.id;
    opt.textContent = m.name;
    flangeMaterialSelect.appendChild(opt);
  });

  const form = document.getElementById('design-form');
  const resultsPanel = document.getElementById('results-panel');
  const resultsContent = document.getElementById('results-content');
  const errorMsg = document.getElementById('error-message');

  form.addEventListener('submit', (e) => {
    e.preventDefault();
    errorMsg.style.display = 'none';
    resultsContent.innerHTML = '';
    
    try {
      const power_kW = parseFloat(document.getElementById('power').value);
      const rpm = parseFloat(document.getElementById('rpm').value);
      const overload = parseFloat(document.getElementById('overload').value);
      const fos = parseFloat(document.getElementById('fos').value);
      const shaftLength = parseFloat(document.getElementById('shaftLength').value);
      const maxDiameter = parseFloat(document.getElementById('maxDiameter').value);
      const maxTwist = parseFloat(document.getElementById('maxTwist').value);
      const ciShear = parseFloat(document.getElementById('ciShear').value);
      
      const shaftMatId = document.getElementById('shaftMaterial').value;
      const flangeMatId = document.getElementById('flangeMaterial').value;
      
      const shaftMat = ductileMaterials.find(m => m.id === shaftMatId);
      const flangeMat = brittleMaterials.find(m => m.id === flangeMatId);
      
      if (!shaftMat || !flangeMat) throw new Error("Invalid material selection");

      const T_av_Nm = (power_kW * 1000 * 60) / (2 * Math.PI * rpm);
      const T_max_Nm = overload * T_av_Nm;
      const T = T_max_Nm * 1000;
      
      const shaftStress = getAllowableStresses(shaftMat, fos, true);
      let flangeStress = getAllowableStresses(flangeMat, fos, false);
      
      // Override flange shear stress with the user's manual CI Shear Stress input
      flangeStress.tau_allow = ciShear;
      
      // 1. Shaft Design
      let d_calc = Math.pow((16 * T) / (Math.PI * shaftStress.tau_allow), 1/3);
      let D = Math.ceil(d_calc / 5) * 5;
      if (D < 20) D = 20;

      const J = (Math.PI / 32) * Math.pow(D, 4);
      let theta_rad = (T * shaftLength) / (J * shaftStress.g);
      let theta_deg = theta_rad * (180 / Math.PI);
      
      let isDiameterSafe = D <= maxDiameter;
      let isTwistSafe = theta_deg <= maxTwist;

      if (!isTwistSafe) {
        let d_twist = Math.pow((32 * T * shaftLength) / (Math.PI * shaftStress.g * maxTwist * (Math.PI/180)), 0.25);
        D = Math.ceil(d_twist / 5) * 5;
        const J_new = (Math.PI / 32) * Math.pow(D, 4);
        theta_rad = (T * shaftLength) / (J_new * shaftStress.g);
        theta_deg = theta_rad * (180 / Math.PI);
        isDiameterSafe = D <= maxDiameter;
        isTwistSafe = theta_deg <= maxTwist;
      }

      if (!isDiameterSafe) {
        throw new Error(`Calculated shaft diameter (${D} mm) exceeds the maximum constraint (${maxDiameter} mm). Please select a stronger material or decrease power/overload.`);
      }

      // 2. Key Design
      let w = Math.ceil(D / 4);
      let h = Math.ceil(D / 4);
      let l = 1.5 * D;
      
      let tau_k = (2 * T) / (D * w * l);
      let sigma_ck = (4 * T) / (D * h * l);
      
      let isKeySafe = tau_k <= shaftStress.tau_allow && sigma_ck <= shaftStress.sigma_c_allow;

      // 3. Hub & Flange Design
      let D1 = 2 * D;
      let L = 1.5 * D;
      let D2 = 3 * D;
      let D3 = 4 * D;
      let tf = 0.5 * D;
      
      let tau_hub = (16 * T * D1) / (Math.PI * (Math.pow(D1, 4) - Math.pow(D, 4)));
      let isHubSafe = tau_hub <= flangeStress.tau_allow;
      
      if (!isHubSafe) {
         while(tau_hub > flangeStress.tau_allow && D1 < D3) {
             D1 += 5;
             tau_hub = (16 * T * D1) / (Math.PI * (Math.pow(D1, 4) - Math.pow(D, 4)));
         }
      }

      let tau_f = (2 * T) / (Math.PI * Math.pow(D1, 2) * tf);
      let isFlangeSafe = tau_f <= flangeStress.tau_allow;
      if (!isFlangeSafe) {
        tf = Math.ceil((2 * T) / (Math.PI * Math.pow(D1, 2) * flangeStress.tau_allow));
      }

      // 4. Bolt Design
      let i = Math.ceil(D / 50) + 3;
      
      let d_calc_bolt = Math.sqrt((8 * T) / (i * Math.PI * D2 * shaftStress.tau_allow));
      let d_bolt = Math.ceil(d_calc_bolt);

      let d_min = (0.4 * D) / Math.sqrt(i);
      let d_max = (0.45 * D) / Math.sqrt(i);

      resultsContent.innerHTML = `
        <div class="results-grid">
          <div class="result-card">
            <h4>Shaft Dimensions <span class="status-badge ${isDiameterSafe && isTwistSafe ? 'status-safe' : 'status-unsafe'}">${isDiameterSafe && isTwistSafe ? 'SAFE' : 'UNSAFE'}</span></h4>
            <div class="dimension-item">
              <span class="dimension-label">Diameter (D)</span>
              <span class="dimension-value">${D.toFixed(1)} mm</span>
            </div>
            <div class="dimension-item">
              <span class="dimension-label">Max Torque (T)</span>
              <span class="dimension-value">${(T/1000).toFixed(1)} N-m</span>
            </div>
            <div class="dimension-item">
              <span class="dimension-label">Angle of Twist</span>
              <span class="dimension-value">${theta_deg.toFixed(4)}°</span>
            </div>
            <div class="dimension-item">
              <span class="dimension-label">Allowable Shear</span>
              <span class="dimension-value">${shaftStress.tau_allow.toFixed(1)} MPa</span>
            </div>
          </div>

          <div class="result-card">
            <h4>Key Dimensions <span class="status-badge ${isKeySafe ? 'status-safe' : 'status-unsafe'}">${isKeySafe ? 'SAFE' : 'CHECK REQ.'}</span></h4>
            <div class="dimension-item">
              <span class="dimension-label">Width (w)</span>
              <span class="dimension-value">${w} mm</span>
            </div>
            <div class="dimension-item">
              <span class="dimension-label">Thickness (h)</span>
              <span class="dimension-value">${h} mm</span>
            </div>
            <div class="dimension-item">
              <span class="dimension-label">Length (l)</span>
              <span class="dimension-value">${l} mm</span>
            </div>
            <div class="dimension-item">
              <span class="dimension-label">Induced Shear</span>
              <span class="dimension-value">${tau_k.toFixed(1)} MPa</span>
            </div>
          </div>

          <div class="result-card">
            <h4>Hub & Flange <span class="status-badge status-safe">SAFE</span></h4>
            <div class="dimension-item">
              <span class="dimension-label">Hub Diameter (D1)</span>
              <span class="dimension-value">${D1} mm</span>
            </div>
            <div class="dimension-item">
              <span class="dimension-label">Hub Length (L)</span>
              <span class="dimension-value">${L} mm</span>
            </div>
            <div class="dimension-item">
              <span class="dimension-label">Flange Thickness (tf)</span>
              <span class="dimension-value">${tf} mm</span>
            </div>
            <div class="dimension-item">
              <span class="dimension-label">Outside Dia (D3)</span>
              <span class="dimension-value">${D3} mm</span>
            </div>
          </div>

          <div class="result-card">
            <h4>Bolt Dimensions <span class="status-badge status-safe">SAFE</span></h4>
            <div class="dimension-item">
              <span class="dimension-label">Number of Bolts (i)</span>
              <span class="dimension-value">${i}</span>
            </div>
            <div class="dimension-item">
              <span class="dimension-label">Bolt Diameter (d)</span>
              <span class="dimension-value">${d_bolt} mm</span>
            </div>
            <div class="dimension-item">
              <span class="dimension-label">Pitch Circle Dia (D2)</span>
              <span class="dimension-value">${D2} mm</span>
            </div>
            <div class="dimension-item">
              <span class="dimension-label">Empirical Range</span>
              <span class="dimension-value" style="font-size:0.85rem">${d_min.toFixed(1)} - ${d_max.toFixed(1)} mm</span>
            </div>
          </div>
        </div>
      `;
      
      resultsPanel.style.display = 'block';
      resultsContent.style.opacity = '0';
      setTimeout(() => {
        resultsContent.style.transition = 'opacity 0.5s ease';
        resultsContent.style.opacity = '1';
      }, 50);

    } catch (error) {
      errorMsg.textContent = error.message;
      errorMsg.style.display = 'block';
      resultsPanel.style.display = 'block';
    }
  });
});
