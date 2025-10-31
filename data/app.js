const statusEls = {
  inside: document.getElementById('insideTemp'),
  exhaust: document.getElementById('exhaustTemp'),
  current: document.getElementById('current'),
  phase: document.getElementById('phase'),
  level: document.getElementById('level'),
  target: document.getElementById('targetTemp'),
  wifi: document.getElementById('wifi'),
  ip: document.getElementById('ip'),
  slider: document.getElementById('targetSlider'),
  targetUp: document.getElementById('targetUp'),
  targetDown: document.getElementById('targetDown'),
  stroke: document.getElementById('strokeState'),
  mode: document.getElementById('mode'),
  modeAuto: document.getElementById('modeAuto'),
  modeManual: document.getElementById('modeManual'),
  stageUp: document.getElementById('stageUp'),
  stageDown: document.getElementById('stageDown'),
  heaterState: document.getElementById('heaterState'),
  heaterOn: document.getElementById('heaterOn'),
  heaterOff: document.getElementById('heaterOff'),
  alert: document.getElementById('alert'),
  reload: document.getElementById('reloadConfig'),
};

const displayContainer = document.getElementById('displayMirror');
const displayLines = displayContainer ? Array.from(displayContainer.querySelectorAll('[data-line]')) : [];

const sliderState = {
  isUserInteracting: false,
  debounceId: null,
};

const pending = new Set();

function showAlert(message) {
  if (!statusEls.alert) return;
  if (!message) {
    statusEls.alert.hidden = true;
    statusEls.alert.textContent = '';
    return;
  }
  statusEls.alert.hidden = false;
  statusEls.alert.textContent = message;
}

function setPending(element, isPending) {
  if (!element) return;
  if (isPending) {
    pending.add(element);
    element.classList.add('loading');
    if (element.tagName === 'BUTTON') {
      element.dataset.disabledBefore = element.disabled ? '1' : '0';
      element.disabled = true;
    }
  } else {
    pending.delete(element);
    element.classList.remove('loading');
    if (element.tagName === 'BUTTON') {
      const disabledBefore = element.dataset.disabledBefore === '1';
      if (!disabledBefore) {
        element.disabled = false;
      }
      delete element.dataset.disabledBefore;
    }
  }
}

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
  const manualMode = data.mode === 'manual';
  if (statusEls.slider && !sliderState.isUserInteracting) {
    const target = typeof data.target_temp === 'number' ? data.target_temp : 21;
    statusEls.slider.value = target;
    statusEls.target.textContent = target.toFixed(1);
  }
  statusEls.wifi.textContent = typeof data.wifi_rssi === 'number' ? `${data.wifi_rssi} dBm` : 'offline';
  if (statusEls.ip) {
    statusEls.ip.textContent = data.ip || 'keine IP';
  }
  if (statusEls.mode) {
    statusEls.mode.textContent = manualMode ? 'Manuell' : 'Automatik';
  }
  if (statusEls.modeAuto && statusEls.modeManual) {
    statusEls.modeAuto.classList.toggle('active', !manualMode);
    statusEls.modeManual.classList.toggle('active', manualMode);
  }
  if (statusEls.heaterOn && statusEls.heaterOff) {
    const heaterIsOn = data.heater_on === true;
    statusEls.heaterOn.classList.toggle('active', heaterIsOn);
    statusEls.heaterOff.classList.toggle('active', data.heater_on === false);
    if (statusEls.heaterState) {
      statusEls.heaterState.classList.remove('off', 'unknown');
      if (data.heater_on === true) {
        statusEls.heaterState.textContent = 'Heizung läuft';
      } else if (data.heater_on === false) {
        statusEls.heaterState.textContent = 'Heizung ausgeschaltet';
        statusEls.heaterState.classList.add('off');
      } else {
        statusEls.heaterState.textContent = 'Heizung unbekannt';
        statusEls.heaterState.classList.add('unknown');
      }
    }
  }
  [statusEls.stageUp, statusEls.stageDown].forEach((btn) => {
    if (btn) {
      btn.disabled = !manualMode;
    }
  });
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

  if (displayContainer && displayLines.length > 0) {
    const lines = Array.isArray(data.display) ? data.display : [];
    let hasContent = false;
    displayLines.forEach((el, index) => {
      const value = typeof lines[index] === 'string' ? lines[index] : '';
      const trimmed = value.trim();
      if (trimmed.length > 0) {
        hasContent = true;
      }
      el.textContent = trimmed.length > 0 ? value : ' ';
      el.classList.toggle('empty', trimmed.length === 0);
    });
    displayContainer.classList.toggle('inactive', !hasContent);
  }
}

async function loadStatus() {
  try {
    const data = await fetchJSON('/api/status');
    updateStatusUI(data);
    showAlert('');
  } catch (error) {
    console.error('status error', error);
    showAlert('Status konnte nicht aktualisiert werden. Bitte Verbindung prüfen.');
  }
}

async function sendControl(payload, sourceElement) {
  try {
    if (sourceElement) {
      setPending(sourceElement, true);
    }
    await fetchJSON('/api/control', {
      method: 'POST',
      body: JSON.stringify(payload),
    });
    await loadStatus();
    showAlert('');
  } catch (error) {
    console.error('control error', error);
    showAlert('Befehl konnte nicht gesendet werden. Bitte erneut versuchen.');
  } finally {
    if (sourceElement) {
      setPending(sourceElement, false);
    }
  }
}

function setupControls() {
  document.querySelectorAll('.control-buttons button').forEach((btn) => {
    btn.addEventListener('click', () => {
      const command = btn.dataset.command;
      if (command === 'heater_on') {
        sendControl({ heater_on: true }, btn);
      } else if (command === 'heater_off') {
        if (confirm('Heizung wirklich ausschalten?')) {
          sendControl({ heater_on: false }, btn);
        }
      } else if (command === 'mode_auto') {
        sendControl({ mode: 'auto' }, btn);
      } else if (command === 'mode_manual') {
        sendControl({ mode: 'manual' }, btn);
      } else {
        sendControl({ command }, btn);
      }
    });
  });

  if (statusEls.slider) {
    ['pointerdown', 'touchstart', 'mousedown'].forEach((evt) => {
      statusEls.slider.addEventListener(evt, () => {
        sliderState.isUserInteracting = true;
      });
    });
    ['pointerup', 'touchend', 'mouseup', 'mouseleave'].forEach((evt) => {
      statusEls.slider.addEventListener(evt, () => {
        sliderState.isUserInteracting = false;
      });
    });

    const scheduleUpdate = (value) => {
      if (sliderState.debounceId) {
        clearTimeout(sliderState.debounceId);
      }
      sliderState.debounceId = setTimeout(() => {
        sendControl({ target_temp: value });
      }, 400);
    };

    statusEls.slider.addEventListener('input', (event) => {
      const value = parseFloat(event.target.value);
      statusEls.target.textContent = value.toFixed(1);
      scheduleUpdate(value);
    });

    const adjustTarget = (delta) => {
      const current = parseFloat(statusEls.slider.value);
      const next = Math.min(28, Math.max(10, current + delta));
      statusEls.slider.value = next;
      statusEls.target.textContent = next.toFixed(1);
      scheduleUpdate(next);
    };

    if (statusEls.targetUp) {
      statusEls.targetUp.addEventListener('click', () => adjustTarget(0.5));
    }
    if (statusEls.targetDown) {
      statusEls.targetDown.addEventListener('click', () => adjustTarget(-0.5));
    }
  }

  if (statusEls.reload) {
    statusEls.reload.addEventListener('click', async () => {
      setPending(statusEls.reload, true);
      await loadStatus();
      setPending(statusEls.reload, false);
    });
  }

  document.getElementById('wifiReset').addEventListener('click', async () => {
    if (!confirm('WLAN-Einstellungen wirklich zurücksetzen?')) return;
    try {
      await fetch('/api/wifi/reset', { method: 'POST' });
      showAlert('Gerät startet neu, um WLAN zurückzusetzen.');
    } catch (error) {
      console.error('wifi reset', error);
      showAlert('WLAN-Reset fehlgeschlagen.');
    }
  });
}

setupControls();
loadStatus();
setInterval(loadStatus, 3000);
