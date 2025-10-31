const form = document.getElementById('configForm');
const alertEl = document.getElementById('settingsAlert');
const saveBtn = document.getElementById('configSave');
const reloadBtn = document.getElementById('configReload');

const fields = {
  targetTemp: document.getElementById('targetTempInput'),
  deltaDown: document.getElementById('deltaDownInput'),
  deltaUp: document.getElementById('deltaUpInput'),
  deltaUpCritical: document.getElementById('deltaUpCriticalInput'),
  minStepInterval: document.getElementById('minStepIntervalInput'),
  stabilizeDuration: document.getElementById('stabilizeDurationInput'),
  maxSteps: document.getElementById('maxStepsInput'),
  exhaustWarn: document.getElementById('exhaustWarnInput'),
  exhaustOff: document.getElementById('exhaustOffInput'),
  telegramEnabled: document.getElementById('telegramEnabledInput'),
  telegramToken: document.getElementById('telegramTokenInput'),
  telegramChatId: document.getElementById('telegramChatIdInput'),
  sensorInside: document.getElementById('sensorInsideInput'),
  sensorExhaust: document.getElementById('sensorExhaustInput'),
  acsZero: document.getElementById('acsZeroInput'),
  acsSensitivity: document.getElementById('acsSensitivityInput'),
};

function showAlert(message, type = 'error') {
  if (!alertEl) return;
  if (!message) {
    alertEl.hidden = true;
    alertEl.textContent = '';
    alertEl.classList.remove('success');
    return;
  }
  alertEl.hidden = false;
  alertEl.textContent = message;
  alertEl.classList.toggle('success', type === 'success');
}

function setPending(element, isPending) {
  if (!element) return;
  if (isPending) {
    element.classList.add('loading');
    element.dataset.disabledBefore = element.disabled ? '1' : '0';
    element.disabled = true;
  } else {
    const disabledBefore = element.dataset.disabledBefore === '1';
    if (!disabledBefore) {
      element.disabled = false;
    }
    element.classList.remove('loading');
    delete element.dataset.disabledBefore;
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

function setInputValue(input, value, digits = null) {
  if (!input) return;
  if (typeof value !== 'number' || Number.isNaN(value)) {
    input.value = '';
    return;
  }
  if (typeof digits === 'number') {
    input.value = value.toFixed(digits);
  } else {
    input.value = value.toString();
  }
}

function populateForm(data) {
  const regulation = data?.regulation ?? {};
  const telegram = data?.telegram ?? {};
  const calibration = data?.calibration ?? {};
  const sensors = data?.sensors ?? {};

  setInputValue(fields.targetTemp, data?.target_temp, 1);
  setInputValue(fields.deltaDown, regulation.delta_down, 1);
  setInputValue(fields.deltaUp, regulation.delta_up, 1);
  setInputValue(fields.deltaUpCritical, regulation.delta_up_critical, 1);
  setInputValue(fields.minStepInterval, regulation.min_step_interval);
  setInputValue(fields.stabilizeDuration, regulation.stabilize_duration);
  setInputValue(fields.maxSteps, regulation.max_steps_per_quarter);
  setInputValue(fields.exhaustWarn, regulation.exhaust_warn, 1);
  setInputValue(fields.exhaustOff, regulation.exhaust_off, 1);

  if (fields.telegramEnabled) {
    fields.telegramEnabled.checked = Boolean(telegram.enabled);
  }
  if (fields.telegramToken) {
    fields.telegramToken.value = telegram.token || '';
  }
  if (fields.telegramChatId) {
    fields.telegramChatId.value = telegram.chat_id || '';
  }

  setInputValue(fields.acsZero, calibration.acs_zero);
  setInputValue(fields.acsSensitivity, calibration.acs_sensitivity);

  if (fields.sensorInside) {
    fields.sensorInside.value = sensors.inside ? sensors.inside.toUpperCase() : '';
  }
  if (fields.sensorExhaust) {
    fields.sensorExhaust.value = sensors.exhaust ? sensors.exhaust.toUpperCase() : '';
  }
}

function numberFromInput(input, round = false) {
  if (!input) throw new Error('Feld fehlt');
  const value = input.value.trim();
  if (value === '') {
    throw new Error('Bitte alle Felder ausfüllen.');
  }
  const num = Number(value);
  if (!Number.isFinite(num)) {
    throw new Error(`Ungültiger Wert in Feld "${input.previousElementSibling?.textContent || input.name}".`);
  }
  return round ? Math.round(num) : num;
}

function sanitizeSensor(value) {
  const trimmed = value.trim();
  return trimmed === '' ? null : trimmed.toUpperCase();
}

function collectPayload() {
  return {
    target_temp: numberFromInput(fields.targetTemp),
    regulation: {
      delta_down: numberFromInput(fields.deltaDown),
      delta_up: numberFromInput(fields.deltaUp),
      delta_up_critical: numberFromInput(fields.deltaUpCritical),
      min_step_interval: numberFromInput(fields.minStepInterval, true),
      stabilize_duration: numberFromInput(fields.stabilizeDuration, true),
      max_steps_per_quarter: numberFromInput(fields.maxSteps, true),
      exhaust_warn: numberFromInput(fields.exhaustWarn),
      exhaust_off: numberFromInput(fields.exhaustOff),
    },
    telegram: {
      enabled: fields.telegramEnabled?.checked ?? false,
      token: fields.telegramToken?.value.trim() ?? '',
      chat_id: fields.telegramChatId?.value.trim() ?? '',
    },
    calibration: {
      acs_zero: numberFromInput(fields.acsZero),
      acs_sensitivity: numberFromInput(fields.acsSensitivity),
    },
    sensors: {
      inside: sanitizeSensor(fields.sensorInside ? fields.sensorInside.value : ''),
      exhaust: sanitizeSensor(fields.sensorExhaust ? fields.sensorExhaust.value : ''),
    },
  };
}

async function loadConfig(showReloadPending = false) {
  const pendingButton = showReloadPending ? reloadBtn : null;
  if (pendingButton) {
    setPending(pendingButton, true);
  }
  try {
    const data = await fetchJSON('/api/config');
    populateForm(data);
    showAlert('');
  } catch (error) {
    console.error('config load', error);
    showAlert('Konfiguration konnte nicht geladen werden. Bitte Verbindung prüfen.');
  } finally {
    if (pendingButton) {
      setPending(pendingButton, false);
    }
  }
}

async function saveConfig(event) {
  event.preventDefault();
  if (form && !form.reportValidity()) {
    return;
  }

  try {
    setPending(saveBtn, true);
    setPending(reloadBtn, true);
    const payload = collectPayload();
    await fetchJSON('/api/config', {
      method: 'POST',
      body: JSON.stringify(payload),
    });
    showAlert('Konfiguration gespeichert.', 'success');
    await loadConfig();
  } catch (error) {
    console.error('config save', error);
    const message = error && typeof error.message === 'string' && error.message.trim().length > 0
      ? error.message
      : 'Konfiguration konnte nicht gespeichert werden. Bitte Eingaben prüfen.';
    showAlert(message);
  } finally {
    setPending(saveBtn, false);
    setPending(reloadBtn, false);
  }
}

if (form) {
  form.addEventListener('submit', saveConfig);
}

if (reloadBtn) {
  reloadBtn.addEventListener('click', (event) => {
    event.preventDefault();
    showAlert('');
    loadConfig(true);
  });
}

loadConfig();
