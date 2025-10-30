const statusEls = {
  inside: document.getElementById('insideTemp'),
  exhaust: document.getElementById('exhaustTemp'),
  current: document.getElementById('current'),
  phase: document.getElementById('phase'),
  level: document.getElementById('level'),
  target: document.getElementById('targetTemp'),
  wifi: document.getElementById('wifi'),
  slider: document.getElementById('targetSlider'),
  stroke: document.getElementById('strokeState'),
};

async function fetchJSON(url, options = {}) {
  const response = await fetch(url, {
    headers: { 'Content-Type': 'application/json' },
    ...options,
  });
  if (!response.ok) {
    throw new Error(await response.text());
  }
  return response.status === 204 ? null : response.json();
}

function formatNumber(value) {
  return typeof value === 'number' && !Number.isNaN(value) ? value.toFixed(1) : '--';
}

function updateStatusUI(data) {
  statusEls.inside.textContent = formatNumber(data.inside_temp);
  statusEls.exhaust.textContent = formatNumber(data.exhaust_temp);
  statusEls.current.textContent = formatNumber(data.current);
  statusEls.phase.textContent = data.phase || '--';
  statusEls.level.textContent = typeof data.level === 'number' ? data.level : '--';
  statusEls.target.textContent = formatNumber(data.target_temp);
  statusEls.slider.value = typeof data.target_temp === 'number' ? data.target_temp : 21;
  statusEls.wifi.textContent = typeof data.wifi_rssi === 'number' ? `${data.wifi_rssi} dBm` : '--';
  if (statusEls.stroke) {
    let text = '--';
    if (typeof data.heater_on === 'boolean') {
      text = data.heater_on ? 'AN' : 'AUS';
      if (typeof data.heater_request === 'boolean' && data.heater_request !== data.heater_on) {
        text += data.heater_request ? ' (AN gefordert)' : ' (AUS gefordert)';
      }
    }
    if (data.stroke_feedback === false) {
      text += text === '--' ? ' Feedback fehlt' : ' (ohne Feedback)';
    }
    statusEls.stroke.textContent = text;
  }
}

async function loadStatus() {
  try {
    const data = await fetchJSON('/api/status');
    updateStatusUI(data);
  } catch (error) {
    console.error('status error', error);
  }
}

async function sendControl(payload) {
  try {
    await fetchJSON('/api/control', {
      method: 'POST',
      body: JSON.stringify(payload),
    });
    await loadStatus();
  } catch (error) {
    console.error('control error', error);
  }
}

function setupControls() {
  document.querySelectorAll('.buttons button').forEach((btn) => {
    btn.addEventListener('click', () => {
      const command = btn.dataset.command;
      if (command === 'heater_on') {
        sendControl({ heater_on: true });
      } else if (command === 'heater_off') {
        sendControl({ heater_on: false });
      } else {
        sendControl({ command });
      }
    });
  });

  statusEls.slider.addEventListener('change', (event) => {
    const value = parseFloat(event.target.value);
    statusEls.target.textContent = value.toFixed(1);
    sendControl({ target_temp: value });
  });

  document.getElementById('reloadConfig').addEventListener('click', async () => {
    await loadStatus();
  });

  document.getElementById('wifiReset').addEventListener('click', async () => {
    if (!confirm('WLAN-Einstellungen wirklich zurücksetzen?')) return;
    try {
      await fetch('/api/wifi/reset', { method: 'POST' });
    } catch (error) {
      console.error('wifi reset', error);
    }
  });
}

setupControls();
loadStatus();
setInterval(loadStatus, 3000);
