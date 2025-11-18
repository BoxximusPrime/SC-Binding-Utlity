const { invoke } = window.__TAURI__.core;
const { listen } = window.__TAURI__.event;
const { open, save } = window.__TAURI__.dialog;
import { toStarCitizenFormat } from './input-utils.js';
import { initializeUpdateChecker } from './update-checker.js';

// Global error handler for uncaught errors
window.addEventListener('error', async (event) =>
{
  console.error('Uncaught error:', event.error);
  try
  {
    await invoke('log_error', {
      message: event.error?.message || event.message || 'Unknown error',
      stack: event.error?.stack || null
    });
  } catch (e)
  {
    console.error('Failed to log error to backend:', e);
  }
});

// Global handler for unhandled promise rejections
window.addEventListener('unhandledrejection', async (event) =>
{
  console.error('Unhandled promise rejection:', event.reason);
  try
  {
    await invoke('log_error', {
      message: event.reason?.message || String(event.reason) || 'Unknown promise rejection',
      stack: event.reason?.stack || null
    });
  } catch (e)
  {
    console.error('Failed to log promise rejection to backend:', e);
  }
});

// Helper function to log info messages
window.logInfo = async (message) =>
{
  console.log(message);
  try
  {
    await invoke('log_info', { message });
  } catch (e)
  {
    console.error('Failed to log info to backend:', e);
  }
};

// Keyboard detection state
let keyboardDetectionActive = false;
let keyboardDetectionHandler = null;
let isDetectionActive = false; // Global flag to track if input detection is active
let ignoreModalMouseInputs = false; // Set while hovering cancel/save to avoid accidental detections
let currentBindingId = null; // Unique ID for the current binding attempt - helps ignore stale events
let bindingModalSaveBtn = null;

// State
let currentKeybindings = null;
let currentFilter = 'all';
let currentCategory = null;
let searchTerm = '';
let bindingMode = false;
let currentBindingAction = null;
let countdownInterval = null;
let secondaryDetectionTimeout = null;
let hasUnsavedChanges = false;
let customizedOnly = false;
let showDefaultBindings = true;
let currentTab = 'main';
let categoryFriendlyNames = {};
let currentFilename = null; // Track the current file name for the copy command
const SECONDARY_WINDOW_MS = 1000; // One-second window for multi-input capture
let deviceAxisNames = {}; // Cache of device_name -> { axis_id -> axis_name } from HID descriptors

function setBindingSaveEnabled(enabled)
{
  if (!bindingModalSaveBtn) return;
  bindingModalSaveBtn.disabled = !enabled;
}

/**
 * Load axis names for all connected devices from HID descriptors
 * This populates the deviceAxisNames cache used for display
 */
async function loadDeviceAxisNames()
{
  try
  {
    if (!currentKeybindings || !currentKeybindings.devices) return;

    // Collect all unique device names from joysticks
    const deviceNames = new Set();

    if (currentKeybindings.devices.joysticks)
    {
      currentKeybindings.devices.joysticks.forEach(js => deviceNames.add(js.device_name));
    }

    if (currentKeybindings.devices.gamepads)
    {
      currentKeybindings.devices.gamepads.forEach(gp => deviceNames.add(gp.device_name));
    }

    // Load axis names for each device
    for (const deviceName of deviceNames)
    {
      if (!deviceAxisNames[deviceName])
      {
        try
        {
          const axisNames = await invoke('get_axis_names_for_device', { deviceName });
          deviceAxisNames[deviceName] = axisNames || {};
          console.log(`[Axis Names] Loaded ${Object.keys(axisNames || {}).length} axes for device: ${deviceName}`);
        } catch (error)
        {
          console.warn(`[Axis Names] Failed to load axis names for ${deviceName}:`, error);
          deviceAxisNames[deviceName] = {}; // Cache empty result to avoid repeated attempts
        }
      }
    }
  } catch (error)
  {
    console.error('[Axis Names] Failed to load device axis names:', error);
  }
}

/**
 * Get the HID axis name for a joystick axis binding
 * @param {string} binding - The binding string (e.g., "js1_x", "js2_ry")
 * @returns {string|null} - The HID axis name (e.g., "X", "Ry") or null if not found
 */
function getHidAxisNameForBinding(binding)
{
  if (!binding || !currentKeybindings || !currentKeybindings.devices) return null;

  // Parse binding format: jsX_axis or gpX_axis
  const match = binding.match(/^(js|gp)(\d+)_([a-z0-9_]+)$/i);
  if (!match) return null;

  const [, deviceType, deviceNum, axisName] = match;

  // Map deviceType and number to actual device name
  let device = null;

  if (deviceType.toLowerCase() === 'js')
  {
    const jsNum = parseInt(deviceNum);
    if (currentKeybindings.devices.joysticks && currentKeybindings.devices.joysticks[jsNum - 1])
    {
      device = currentKeybindings.devices.joysticks[jsNum - 1];
    }
  } else if (deviceType.toLowerCase() === 'gp')
  {
    const gpNum = parseInt(deviceNum);
    if (currentKeybindings.devices.gamepads && currentKeybindings.devices.gamepads[gpNum - 1])
    {
      device = currentKeybindings.devices.gamepads[gpNum - 1];
    }
  }

  if (!device || !device.device_name) return null;

  // Get axis names for this device
  const axisMap = deviceAxisNames[device.device_name];
  if (!axisMap) return null;

  // Map the axis letter to axis index
  // Common mappings: x=1, y=2, z=3, rx=4, ry=5, rz=6, slider=7, slider2=8, hat=9
  const axisIndexMap = {
    'x': 1, 'y': 2, 'z': 3,
    'rx': 4, 'ry': 5, 'rz': 6,
    'rotx': 4, 'roty': 5, 'rotz': 6,
    'slider': 7, 'slider1': 7, 'slider2': 8,
    'hat': 9, 'hat_switch': 9
  };

  const axisIndex = axisIndexMap[axisName.toLowerCase()];
  if (axisIndex === undefined) return null;

  return axisMap[axisIndex] || null;
}

// Convert JavaScript KeyboardEvent.code to Star Citizen keyboard format
function convertKeyCodeToSC(code, key)
{
  // Handle special keys
  const specialKeys = {
    'Space': 'space',
    'Enter': 'enter',
    'Escape': 'escape',
    'Tab': 'tab',
    'Backspace': 'backspace',
    'Delete': 'delete',
    'Insert': 'insert',
    'Home': 'home',
    'End': 'end',
    'PageUp': 'pgup',
    'PageDown': 'pgdown',
    'ArrowUp': 'up',
    'ArrowDown': 'down',
    'ArrowLeft': 'left',
    'ArrowRight': 'right',
    'CapsLock': 'capslock',
    'NumLock': 'numlock',
    'ScrollLock': 'scrolllock',
    'Pause': 'pause',
    'PrintScreen': 'print',
    'ContextMenu': 'apps',
    'Backquote': 'grave',
    'Minus': 'minus',
    'Equal': 'equals',
    'BracketLeft': 'lbracket',
    'BracketRight': 'rbracket',
    'Backslash': 'backslash',
    'Semicolon': 'semicolon',
    'Quote': 'apostrophe',
    'Comma': 'comma',
    'Period': 'period',
    'Slash': 'slash',
  };

  if (specialKeys[code])
  {
    return specialKeys[code];
  }

  // Handle letter keys (KeyA -> a)
  if (code.startsWith('Key'))
  {
    return code.substring(3).toLowerCase();
  }

  // Handle number keys (Digit1 -> 1)
  if (code.startsWith('Digit'))
  {
    return code.substring(5);
  }

  // Handle numpad keys (Numpad1 -> np_1)
  if (code.startsWith('Numpad'))
  {
    const numpadKey = code.substring(6).toLowerCase();
    const numpadMap = {
      'divide': 'np_divide',
      'multiply': 'np_multiply',
      'subtract': 'np_subtract',
      'add': 'np_add',
      'enter': 'np_enter',
      'decimal': 'np_period',
    };
    return numpadMap[numpadKey] || `np_${numpadKey}`;
  }

  // Handle function keys (F1 -> f1)
  if (code.match(/^F\d+$/))
  {
    return code.toLowerCase();
  }

  // Handle modifiers (these are typically detected as part of combinations)
  if (code === 'ShiftLeft') return 'lshift';
  if (code === 'ShiftRight') return 'rshift';
  if (code === 'ControlLeft') return 'lctrl';
  if (code === 'ControlRight') return 'rctrl';
  if (code === 'AltLeft') return 'lalt';
  if (code === 'AltRight') return 'ralt';
  if (code === 'MetaLeft' || code === 'MetaRight') return 'lwin'; // Windows key

  // Fallback to lowercase key
  return key.toLowerCase();
}

function renderDetectedInputMessage(container, message)
{
  container.innerHTML = '';
  const span = document.createElement('span');
  span.className = 'action-binding-button-found';
  span.textContent = message;
  container.appendChild(span);
}

function clearPrimaryCountdown()
{
  if (!countdownInterval) return;
  console.log('[TIMER] Clearing primary countdown timer, ID:', countdownInterval);
  clearInterval(countdownInterval);
  countdownInterval = null;
}

function clearSecondaryDetectionTimer()
{
  if (!secondaryDetectionTimeout) return;
  console.log('[TIMER] Clearing secondary detection timer');
  clearTimeout(secondaryDetectionTimeout);
  secondaryDetectionTimeout = null;
}

function cleanupInputDetectionListeners()
{
  if (window.currentInputDetectionUnlisten)
  {
    window.currentInputDetectionUnlisten();
    window.currentInputDetectionUnlisten = null;
  }
  if (window.currentCompletionUnlisten)
  {
    window.currentCompletionUnlisten();
    window.currentCompletionUnlisten = null;
  }

  if (keyboardDetectionHandler)
  {
    document.removeEventListener('keydown', keyboardDetectionHandler, true);
    keyboardDetectionHandler = null;
  }
  keyboardDetectionActive = false;

  if (window.mouseDetectionHandler)
  {
    document.removeEventListener('mousedown', window.mouseDetectionHandler, true);
    window.mouseDetectionHandler = null;
  }
  if (window.mouseUpHandler)
  {
    document.removeEventListener('mouseup', window.mouseUpHandler, true);
    window.mouseUpHandler = null;
  }
  if (window.contextMenuHandler)
  {
    document.removeEventListener('contextmenu', window.contextMenuHandler, true);
    window.contextMenuHandler = null;
  }
  if (window.beforeUnloadHandler)
  {
    window.removeEventListener('beforeunload', window.beforeUnloadHandler, true);
    window.beforeUnloadHandler = null;
  }
  if (window.mouseDetectionActive !== undefined)
  {
    window.mouseDetectionActive = false;
  }
}

function stopDetection(reason = 'unspecified')
{
  const wasActive = isDetectionActive || countdownInterval || secondaryDetectionTimeout;
  if (!wasActive)
  {
    cleanupInputDetectionListeners();
    return;
  }

  ignoreModalMouseInputs = false;

  console.log(`[TIMER] stopDetection called (${reason})`);
  isDetectionActive = false;
  clearPrimaryCountdown();
  clearSecondaryDetectionTimer();
  cleanupInputDetectionListeners();
}

function startSecondaryDetectionWindow()
{
  clearSecondaryDetectionTimer();
  secondaryDetectionTimeout = setTimeout(() =>
  {
    console.log('[TIMER] Secondary detection window expired');
    secondaryDetectionTimeout = null;
    stopDetection('secondary-window-expired');
  }, SECONDARY_WINDOW_MS);
}

// ============================================================================
// CUSTOM CONFIRMATION DIALOG
// ============================================================================

/**
 * Show a custom confirmation dialog
 * @param {string} message - The confirmation message to display
 * @param {string} title - Optional title for the dialog (default: "Confirm Action")
 * @param {string} confirmText - Optional text for confirm button (default: "Confirm")
 * @param {string} cancelText - Optional text for cancel button (default: "Cancel")
 * @returns {Promise<boolean>} - Resolves to true if confirmed, false if cancelled
 */
async function showConfirmation(message, title = "Confirm Action", confirmText = "Confirm", cancelText = "Cancel", confirmBtnClass = "btn-primary")
{
  return new Promise((resolve) =>
  {
    const modal = document.getElementById('confirmation-modal');
    const titleEl = document.getElementById('confirmation-title');
    const messageEl = document.getElementById('confirmation-message');
    const confirmBtn = document.getElementById('confirmation-confirm-btn');
    const cancelBtn = document.getElementById('confirmation-cancel-btn');

    // Set content
    titleEl.textContent = title;
    messageEl.textContent = message;
    confirmBtn.textContent = confirmText;
    cancelBtn.textContent = cancelText;

    // Reset and apply button classes
    confirmBtn.className = 'btn ' + confirmBtnClass;
    cancelBtn.className = 'btn btn-secondary';

    // Show modal
    modal.style.display = 'flex';

    // Handle confirm
    const handleConfirm = () =>
    {
      cleanup();
      resolve(true);
    };

    // Handle cancel
    const handleCancel = () =>
    {
      cleanup();
      resolve(false);
    };

    // Handle escape key
    const handleEscape = (e) =>
    {
      if (e.key === 'Escape')
      {
        handleCancel();
      }
    };

    // Cleanup function
    const cleanup = () =>
    {
      modal.style.display = 'none';
      confirmBtn.removeEventListener('click', handleConfirm);
      cancelBtn.removeEventListener('click', handleCancel);
      document.removeEventListener('keydown', handleEscape);
    };

    // Add event listeners
    confirmBtn.addEventListener('click', handleConfirm);
    cancelBtn.addEventListener('click', handleCancel);
    document.addEventListener('keydown', handleEscape);

    // Focus confirm button
    setTimeout(() => confirmBtn.focus(), 100);
  });
}

async function showAlert(message, title = "Information", buttonText = "OK")
{
  return new Promise((resolve) =>
  {
    const modal = document.getElementById('confirmation-modal');
    const titleEl = document.getElementById('confirmation-title');
    const messageEl = document.getElementById('confirmation-message');
    const confirmBtn = document.getElementById('confirmation-confirm-btn');
    const cancelBtn = document.getElementById('confirmation-cancel-btn');

    // Set content
    titleEl.textContent = title;
    messageEl.textContent = message;
    confirmBtn.textContent = buttonText;

    // Hide cancel button for alert
    cancelBtn.style.display = 'none';

    // Show modal
    modal.style.display = 'flex';

    // Handle confirm
    const handleConfirm = () =>
    {
      cleanup();
      resolve();
    };

    // Handle escape key
    const handleEscape = (e) =>
    {
      if (e.key === 'Escape')
      {
        handleConfirm();
      }
    };

    // Cleanup function
    const cleanup = () =>
    {
      modal.style.display = 'none';
      cancelBtn.style.display = '';
      confirmBtn.removeEventListener('click', handleConfirm);
      document.removeEventListener('keydown', handleEscape);
    };

    // Add event listeners
    confirmBtn.addEventListener('click', handleConfirm);
    document.addEventListener('keydown', handleEscape);

    // Focus confirm button
    setTimeout(() => confirmBtn.focus(), 100);
  });
}

// Make showConfirmation and showAlert globally available for other modules
window.showConfirmation = showConfirmation;
window.showAlert = showAlert;

// ============================================================================
// WHAT'S NEW MODAL
// ============================================================================

function initializeWhatsNewModal()
{
  const CURRENT_VERSION = '0.7.1';
  const WHATS_NEW_KEY = 'whatsNew';

  // Check if the stored version matches the current version
  const storedVersion = localStorage.getItem(WHATS_NEW_KEY);

  if (storedVersion !== CURRENT_VERSION)
  {
    // Show the modal if version has changed or never been set
    showWhatsNewModal();
  }
}

function showWhatsNewModal()
{
  const CURRENT_VERSION = '0.7.1';
  const WHATS_NEW_KEY = 'whatsNew';

  const modal = document.getElementById('whats-new-modal');
  const closeBtn = document.getElementById('whats-new-close-btn');

  if (!modal || !closeBtn) return;

  // Show modal
  modal.style.display = 'flex';

  // Handle close
  const handleClose = () =>
  {
    modal.style.display = 'none';
    localStorage.setItem(WHATS_NEW_KEY, CURRENT_VERSION);
    closeBtn.removeEventListener('click', handleClose);
    escapeHandler && document.removeEventListener('keydown', escapeHandler);
  };

  // Handle escape key
  const escapeHandler = (e) =>
  {
    if (e.key === 'Escape')
    {
      handleClose();
    }
  };

  closeBtn.addEventListener('click', handleClose);
  document.addEventListener('keydown', escapeHandler);

  // Focus the close button
  setTimeout(() => closeBtn.focus(), 100);
}

// Make showWhatsNewModal globally available for testing
window.showWhatsNewModal = showWhatsNewModal;

// Main app initialization
window.addEventListener("DOMContentLoaded", async () =>
{
  initializeEventListeners();
  initializeTabSystem();
  initializeWhatsNewModal();
  initializeFontSizeScaling();

  // Initialize update checker
  try
  {
    await initializeUpdateChecker();
  } catch (error)
  {
    console.error('Failed to initialize update checker:', error);
    // Don't block app startup if update checker fails
  }

  // Show default file indicator
  document.getElementById('loaded-file-indicator').style.display = 'flex';

  // Load persisted template name
  const savedTemplateName = localStorage.getItem('currentTemplateName');
  if (savedTemplateName)
  {
    const savedFileName = localStorage.getItem('templateFileName');
    updateTemplateIndicator(savedTemplateName, savedFileName);
  }

  // Load categories
  try
  {
    await loadCategoryMappings();
  } catch (error)
  {
    console.error('Failed to load categories:', error);
  }

  // Load AllBinds.xml on startup
  try
  {
    await invoke('load_all_binds');
    console.log('AllBinds.xml loaded successfully');
  } catch (error)
  {
    console.error('Failed to load AllBinds.xml:', error);
    await showAlert(`Warning: Failed to load AllBinds.xml: ${error}\n\nSome features may not work correctly.`, 'Warning');
  }

  await loadPersistedKeybindings();
});

function initializeTabSystem()
{
  // Add tab click handlers
  document.querySelectorAll('.tab-btn[data-tab]').forEach(btn =>
  {
    btn.addEventListener('click', (e) =>
    {
      const tabName = e.target.dataset.tab;
      if (!tabName) return;
      switchTab(tabName);
    });
  });

  // Save current tab to localStorage
  const savedTab = localStorage.getItem('currentTab') || 'welcome';
  switchTab(savedTab);

  // Initialize settings page elements
  initializeSettingsPage();
}

function initializeSettingsPage()
{
  const resetCacheBtn = document.getElementById('reset-cache-btn');
  const manualUpdateCheckBtn = document.getElementById('manual-update-check-btn');

  // Reset cache button
  if (resetCacheBtn)
  {
    resetCacheBtn.addEventListener('click', async () =>
    {
      const confirmed = await showConfirmation(
        'Are you sure you want to reset the application cache?',
        'Reset Application Cache',
        'Reset Cache',
        'Cancel',
        'btn-danger'
      );

      if (confirmed)
      {
        try
        {
          // Clear all localStorage
          localStorage.clear();
          await showAlert('Application cache has been reset. The app will now refresh.', 'Cache Reset');
          // Reload the page to apply the reset
          window.location.reload();
        } catch (error)
        {
          console.error('Error resetting cache:', error);
          await showAlert(`Error resetting cache: ${error}`, 'Error');
        }
      }
    });
  }

  // Manual update check button
  if (manualUpdateCheckBtn)
  {
    manualUpdateCheckBtn.addEventListener('click', async () =>
    {
      manualUpdateCheckBtn.disabled = true;
      try
      {
        if (window.manualUpdateCheck)
        {
          await window.manualUpdateCheck();
        }
      } catch (error)
      {
        console.error('Error during manual update check:', error);
      } finally
      {
        manualUpdateCheckBtn.disabled = false;
      }
    });
  }
}

async function loadCategoryMappings()
{
  try
  {
    // Use Tauri's resource resolver to load from the app directory
    const response = await fetch(new URL('../Categories.json', import.meta.url).href);
    if (!response.ok)
    {
      throw new Error(`HTTP error! status: ${response.status}`);
    }
    const data = await response.json();

    // Convert array of objects to a flat mapping object
    // e.g., { "@ui_CCSpaceFlight": "Space Flight Controls", ... }
    categoryFriendlyNames = {};
    data.categories.forEach(categoryObj =>
    {
      // Each category object has one key-value pair
      const entries = Object.entries(categoryObj);
      if (entries.length > 0)
      {
        const [key, value] = entries[0];
        categoryFriendlyNames[key] = value;
      }
    });

    console.log('Category mappings loaded:', categoryFriendlyNames);
  } catch (error)
  {
    console.error('Error loading Categories.json:', error);
    // Set default fallback mapping to ensure the app still works
    categoryFriendlyNames = {
      '@ui_CCSeatGeneral': 'General Seat Controls',
      '@ui_CCSpaceFlight': 'Space Flight Controls',
      '@ui_CCOrientationControl': 'Orientation Control',
      '@ui_CCFlightModes': 'Flight Modes',
      '@ui_CCTurrets': 'Turrets',
      '@ui_CGLightControllerDesc': 'Light Controller',
      '@ui_CCFPS': 'First Person Shooter Controls',
      '@ui_CCEVA': 'EVA Controls',
      '@ui_CCEVAZGT': 'EVA Zero Gravity Traversal',
      '@ui_CCVehicle': 'Vehicle Controls',
      '@ui_CC_DriveModes': 'Drive Modes',
      '@ui_CGEASpectator': 'Spectator Mode',
      '@ui_CGUIGeneral': 'General UI Controls',
      '@ui_CGOpticalTracking': 'Optical Tracking',
      '@ui_CGInteraction': 'Interaction',
      '@ui_CCCamera': 'Camera Controls',
      '': 'Uncategorized'
    };
  }
}

function switchTab(tabName)
{
  if (!tabName)
  {
    console.warn('switchTab called without a tab name');
    return;
  }

  currentTab = tabName;

  // Update active tab button
  document.querySelectorAll('.tab-btn').forEach(btn =>
  {
    btn.classList.toggle('active', btn.dataset.tab === tabName);
  });

  // Update active tab content
  document.querySelectorAll('.tab-content').forEach(content =>
  {
    content.classList.toggle('active', content.id === `tab-content-${tabName}`);
  });

  // Update body class for CSS selectors to show/hide template info
  document.body.classList.remove('tab-welcome', 'tab-main', 'tab-visual', 'tab-template', 'tab-debugger', 'tab-character', 'tab-help', 'tab-settings');
  document.body.classList.add(`tab-${tabName}`);

  // Save to localStorage
  localStorage.setItem('currentTab', tabName);

  // Handle tab-specific initialization
  if (tabName === 'visual')
  {
    // Initialize visual view if needed
    if (window.initializeVisualView)
    {
      window.initializeVisualView();
    }
    // Refresh visual view when switching to it
    if (window.refreshVisualView)
    {
      window.refreshVisualView();
    }
  }
  else if (tabName === 'template')
  {
    // Initialize template editor if needed
    if (window.initializeTemplateEditor)
    {
      window.initializeTemplateEditor();
    }
  }
  else if (tabName === 'character')
  {
    // Initialize character manager if needed
    if (window.initCharacterManager)
    {
      window.initCharacterManager();
    }
  }
  else if (tabName === 'debugger')
  {
    // Initialize debugger if needed
    if (window.initializeDebugger)
    {
      window.initializeDebugger();
    }
  }
}

function initializeEventListeners()
{
  // Version number click to show What's New
  const versionEl = document.getElementById('app-version');
  if (versionEl)
  {
    versionEl.style.cursor = 'pointer';
    versionEl.title = 'Click to see what\'s new';
    versionEl.addEventListener('click', showWhatsNewModal);
  }

  // Load button
  const loadBtn = document.getElementById('load-btn');
  const welcomeLoadBtn = document.getElementById('welcome-load-btn');
  if (loadBtn) loadBtn.addEventListener('click', loadKeybindingsFile);
  if (welcomeLoadBtn) welcomeLoadBtn.addEventListener('click', loadKeybindingsFile);

  // Save buttons
  const saveBtn = document.getElementById('save-btn');
  const saveAsBtn = document.getElementById('save-as-btn');
  if (saveBtn) saveBtn.addEventListener('click', saveKeybindings);
  if (saveAsBtn) saveAsBtn.addEventListener('click', saveKeybindingsAs);

  // SC Directory button
  const scDirectoryBtn = document.getElementById('sc-directory-btn');
  if (scDirectoryBtn) scDirectoryBtn.addEventListener('click', () =>
  {
    window.location.href = 'sc-directory.html';
  });

  // Ko-fi header button
  const kofiBtn = document.getElementById('kofi-header-btn');
  if (kofiBtn) kofiBtn.addEventListener('click', () =>
  {
    // Switch to welcome tab
    switchTab('welcome');
    // Scroll to the support section
    setTimeout(() =>
    {
      const supportSection = document.querySelector('.support-section');
      if (supportSection)
      {
        supportSection.scrollIntoView({ behavior: 'smooth', block: 'center' });
      }
    }, 50);
  });

  // Debugger view switchers
  const switchToBasicDebug = document.getElementById('switch-to-basic-debug');
  const switchToHIDDebug = document.getElementById('switch-to-hid-debug');
  if (switchToBasicDebug && switchToHIDDebug)
  {
    switchToBasicDebug.addEventListener('click', () =>
    {
      document.getElementById('basic-debugger-view').style.display = 'block';
      document.getElementById('hid-debugger-view').style.display = 'none';
      switchToBasicDebug.classList.add('active');
      switchToHIDDebug.classList.remove('active');
    });
    switchToHIDDebug.addEventListener('click', () =>
    {
      document.getElementById('basic-debugger-view').style.display = 'none';
      document.getElementById('hid-debugger-view').style.display = 'block';
      switchToBasicDebug.classList.remove('active');
      switchToHIDDebug.classList.add('active');
    });
  }

  // Filter buttons
  const filterBtns = document.querySelectorAll('.filter-section .category-item');
  if (filterBtns.length > 0)
  {
    filterBtns.forEach(btn =>
    {
      btn.addEventListener('click', (e) =>
      {
        filterBtns.forEach(b => b.classList.remove('active'));
        e.target.classList.add('active');
        currentFilter = e.target.dataset.filter;
        renderKeybindings();
      });
    });
  }

  // Search input
  const searchInput = document.getElementById('search-input');
  const searchClearBtn = document.getElementById('search-clear-btn');

  if (searchInput)
  {
    searchInput.addEventListener('input', (e) =>
    {
      searchTerm = e.target.value.toLowerCase();
      // Show/hide clear button based on input value
      if (searchClearBtn)
      {
        searchClearBtn.style.display = e.target.value ? 'flex' : 'none';
      }
      renderKeybindings();
    });
  }

  if (searchClearBtn)
  {
    searchClearBtn.addEventListener('click', () =>
    {
      searchInput.value = '';
      searchTerm = '';
      searchClearBtn.style.display = 'none';
      renderKeybindings();
    });
  }

  // Copy command button
  const copyCommandBtn = document.getElementById('copy-command-btn');
  if (copyCommandBtn)
  {
    copyCommandBtn.addEventListener('click', async () =>
    {
      if (!currentFilename) return;

      const command = `pp_RebindKeys ${currentFilename}`;

      try
      {
        // Copy to clipboard
        await navigator.clipboard.writeText(command);

        // Show temporary success message
        const originalText = copyCommandBtn.textContent;
        copyCommandBtn.textContent = '✓ Copied!';
        copyCommandBtn.style.opacity = '0.8';

        setTimeout(() =>
        {
          copyCommandBtn.textContent = originalText;
          copyCommandBtn.style.opacity = '1';
        }, 2000);

        // Log the command for user convenience
        console.log('Command copied to clipboard:', command);
      } catch (error)
      {
        console.error('Failed to copy to clipboard:', error);
      }
    });
  }
  const customizedCheckbox = document.getElementById('customized-only-checkbox');
  if (customizedCheckbox)
  {
    customizedCheckbox.addEventListener('change', (e) =>
    {
      customizedOnly = e.target.checked;
      renderKeybindings();
    });
  }

  const showDefaultsCheckbox = document.getElementById('show-defaults-checkbox');
  if (showDefaultsCheckbox)
  {
    showDefaultsCheckbox.checked = showDefaultBindings;
    showDefaultsCheckbox.addEventListener('change', (e) =>
    {
      showDefaultBindings = e.target.checked;
      renderKeybindings();
    });
  }

  // Binding modal buttons
  const bindingCancelBtn = document.getElementById('binding-cancel-btn');
  bindingModalSaveBtn = document.getElementById('binding-modal-save-btn');
  if (bindingCancelBtn) bindingCancelBtn.addEventListener('click', cancelBinding);
  if (bindingModalSaveBtn)
  {
    bindingModalSaveBtn.addEventListener('click', async () =>
    {
      if (!window.pendingBinding) return;

      const { actionMapName, actionName, mappedInput, multiTap } = window.pendingBinding;

      // Get the selected activation mode
      const activationModeSelect = document.getElementById('activation-mode-select');
      const activationMode = activationModeSelect ? activationModeSelect.value : null;

      stopDetection('user-save-modal');
      await applyBinding(actionMapName, actionName, mappedInput, multiTap, activationMode);
      window.pendingBinding = null;
      setBindingSaveEnabled(false);
    });
    setBindingSaveEnabled(false);
  }

  const setIgnoreModalMouse = (value) =>
  {
    ignoreModalMouseInputs = value;
  };

  const attachHoverGuard = (element) =>
  {
    if (!element) return;
    element.addEventListener('pointerenter', () => setIgnoreModalMouse(true));
    element.addEventListener('pointerleave', () => setIgnoreModalMouse(false));
    element.addEventListener('pointerdown', () => setIgnoreModalMouse(true));
    element.addEventListener('pointerup', () => setIgnoreModalMouse(false));
  };

  attachHoverGuard(bindingCancelBtn);
  attachHoverGuard(bindingModalSaveBtn);

  // Conflict modal buttons
  const conflictCancelBtn = document.getElementById('conflict-cancel-btn');
  const conflictConfirmBtn = document.getElementById('conflict-confirm-btn');
  if (conflictCancelBtn) conflictCancelBtn.addEventListener('click', closeConflictModal);
  if (conflictConfirmBtn) conflictConfirmBtn.addEventListener('click', confirmConflictBinding);

  // Joystick mapping modal buttons
  const configureBtn = document.getElementById('configure-joystick-mapping-btn');
  const joyMappingClose = document.getElementById('joystick-mapping-close');
  const joyMappingCancel = document.getElementById('joystick-mapping-cancel');
  const detectJs1Btn = document.getElementById('detect-js1-btn');
  const detectJs2Btn = document.getElementById('detect-js2-btn');
  const detectGp1Btn = document.getElementById('detect-gp1-btn');
  const resetJs1Btn = document.getElementById('reset-js1-btn');
  const resetJs2Btn = document.getElementById('reset-js2-btn');
  const resetGp1Btn = document.getElementById('reset-gp1-btn');
  const joyMappingSave = document.getElementById('joystick-mapping-save');
  if (configureBtn) configureBtn.addEventListener('click', openJoystickMappingModal);
  if (joyMappingClose) joyMappingClose.addEventListener('click', closeJoystickMappingModal);
  if (joyMappingCancel) joyMappingCancel.addEventListener('click', closeJoystickMappingModal);
  if (detectJs1Btn) detectJs1Btn.addEventListener('click', () => detectDevice('js1'));
  if (detectJs2Btn) detectJs2Btn.addEventListener('click', () => detectDevice('js2'));
  if (detectGp1Btn) detectGp1Btn.addEventListener('click', () => detectDevice('gp1'));
  if (resetJs1Btn) resetJs1Btn.addEventListener('click', () => resetDeviceMapping('js1'));
  if (resetJs2Btn) resetJs2Btn.addEventListener('click', () => resetDeviceMapping('js2'));
  if (resetGp1Btn) resetGp1Btn.addEventListener('click', () => resetDeviceMapping('gp1'));
  if (joyMappingSave) joyMappingSave.addEventListener('click', saveJoystickMapping);

  // New Keybinding button
  const newKeybindingBtn = document.getElementById('new-keybinding-btn');
  if (newKeybindingBtn) newKeybindingBtn.addEventListener('click', newKeybinding);

  // Clear SC Binds button
  const clearSCBindsBtn = document.getElementById('clear-sc-binds-btn');
  if (clearSCBindsBtn) clearSCBindsBtn.addEventListener('click', openClearSCBindsModal);

  // Clear SC Binds modal buttons
  const clearBindsGenerateBtn = document.getElementById('clear-binds-generate-btn');
  const copyUnbindCommandBtn = document.getElementById('copy-unbind-command-btn');
  const removeUnbindFilesBtn = document.getElementById('remove-unbind-files-btn');
  if (clearBindsGenerateBtn) clearBindsGenerateBtn.addEventListener('click', generateUnbindProfile);
  if (copyUnbindCommandBtn) copyUnbindCommandBtn.addEventListener('click', copyUnbindCommand);
  if (removeUnbindFilesBtn) removeUnbindFilesBtn.addEventListener('click', removeUnbindFiles);
}

async function loadKeybindingsFile()
{
  try
  {
    const filePath = await open({
      filters: [{
        name: 'Star Citizen Keybindings',
        extensions: ['xml']
      }],
      multiple: false
    });

    if (!filePath) return; // User cancelled

    // Extract filename from path
    const filename = filePath.split('\\').pop() || filePath.split('/').pop();
    currentFilename = filename;

    // Load the keybindings (this loads into state on backend)
    await invoke('load_keybindings', { filePath });

    // Now get the merged bindings (AllBinds + user customizations)
    currentKeybindings = await invoke('get_merged_bindings');

    // Persist file path so we know where to save
    localStorage.setItem('keybindingsFilePath', filePath);

    // Cache only the user customizations (delta), not the full merged view
    // This keeps the cache small and prevents stale data issues
    await cacheUserCustomizations();

    // Reset unsaved changes flag
    hasUnsavedChanges = false;
    localStorage.setItem('hasUnsavedChanges', 'false');
    updateUnsavedIndicator();

    // Update UI
    displayKeybindings();
    updateFileIndicator(filePath);

  } catch (error)
  {
    console.error('Error loading keybindings:', error);
    await showAlert(`Failed to load keybindings: ${error}`, 'Error');
  }
}

async function loadPersistedKeybindings()
{
  try
  {
    const savedPath = localStorage.getItem('keybindingsFilePath');
    const cachedUnsavedState = localStorage.getItem('hasUnsavedChanges');
    const cachedDelta = localStorage.getItem('userCustomizationsDelta');

    console.log('loadPersistedKeybindings - checking state:', {
      hasSavedPath: !!savedPath,
      cachedUnsavedState,
      hasCachedDelta: !!cachedDelta,
      cachedDeltaLength: cachedDelta?.length || 0
    });

    // Set filename if we have a saved path
    if (savedPath)
    {
      const filename = savedPath.split('\\').pop() || savedPath.split('/').pop();
      currentFilename = filename;
    }

    if (savedPath)
    {
      // Check if we have unsaved changes cached
      // Note: cachedDelta might be the string "null", so check for that too
      if (cachedUnsavedState === 'true' && cachedDelta && cachedDelta !== 'null')
      {
        try
        {
          console.log('Restoring unsaved changes from cache...');

          // Load the cached delta into backend state (this is the unsaved work)
          const userCustomizations = JSON.parse(cachedDelta);
          console.log('Parsed cached delta:', {
            hasData: !!userCustomizations,
            actionMapsCount: userCustomizations?.action_maps?.length || 0
          });
          await invoke('restore_user_customizations', { customizations: userCustomizations });

          // Get fresh merged bindings (AllBinds + cached unsaved delta)
          currentKeybindings = await invoke('get_merged_bindings');

          // Restore unsaved changes state and update UI
          hasUnsavedChanges = true;
          localStorage.setItem('hasUnsavedChanges', 'true');

          displayKeybindings();
          updateFileIndicator(savedPath);
          updateUnsavedIndicator();

          console.log('Unsaved changes restored successfully');
          return;
        } catch (error)
        {
          console.error('Error restoring cached changes:', error);
          // Fall through to load from file
        }
      }

      // No unsaved changes - load from file
      try
      {
        console.log('Loading keybindings from file:', savedPath);
        await invoke('load_keybindings', { filePath: savedPath });

        // Get fresh merged bindings (AllBinds + user delta)
        currentKeybindings = await invoke('get_merged_bindings');

        // No unsaved changes
        hasUnsavedChanges = false;
        updateUnsavedIndicator();

        displayKeybindings();
        updateFileIndicator(savedPath);
        return;
      } catch (error)
      {
        console.error('Error loading persisted file:', error);
        await showAlert(
          `Could not load keybindings file:\n${savedPath}\n\nThe file may have been moved or deleted. Starting with default bindings.`,
          'File Load Error'
        );
        // Clear the saved path and show defaults
        localStorage.removeItem('keybindingsFilePath');
        localStorage.removeItem('hasUnsavedChanges');
        localStorage.removeItem('userCustomizationsDelta');
      }
    }

    // No user file loaded - check if we have unsaved changes for a new keybinding set
    if (cachedUnsavedState === 'true' && cachedDelta && cachedDelta !== 'null')
    {
      try
      {
        console.log('Restoring unsaved new keybinding set from cache...');

        // Load the cached delta into backend state (this is the unsaved work)
        const userCustomizations = JSON.parse(cachedDelta);
        await invoke('restore_user_customizations', { customizations: userCustomizations });

        // Get fresh merged bindings (AllBinds + cached unsaved delta)
        currentKeybindings = await invoke('get_merged_bindings');

        // Restore unsaved changes state
        hasUnsavedChanges = true;
        updateUnsavedIndicator();

        displayKeybindings();
        showUnsavedFileIndicator();

        console.log('Unsaved new keybinding set restored successfully');
        return;
      } catch (error)
      {
        console.error('Error restoring cached new keybinding set:', error);
        // Fall through to show AllBinds only
      }
    }

    // No user file loaded, just show all available bindings from AllBinds
    await loadAllBindsOnly();

  } catch (error)
  {
    console.error('Error loading persisted keybindings:', error);
  }
}

async function loadAllBindsOnly()
{
  try
  {
    // Get merged bindings with no user customizations
    currentKeybindings = await invoke('get_merged_bindings');

    // No need to cache - AllBinds is always available and this is the default state
    hasUnsavedChanges = false;
    localStorage.setItem('hasUnsavedChanges', 'false');

    displayKeybindings();
  } catch (error)
  {
    console.error('Error loading AllBinds:', error);
    // Show welcome screen if AllBinds failed to load
  }
}

/**
 * Cache only the user's customizations (delta) to localStorage.
 * This is much smaller than caching the full merged view and prevents stale data issues.
 */
async function cacheUserCustomizations()
{
  try
  {
    // Get the user's customizations from backend (just the delta, not merged with AllBinds)
    const userCustomizations = await invoke('get_user_customizations');

    // Cache the delta - this is typically < 100 KB vs 25+ MB for full merged view
    localStorage.setItem('userCustomizationsDelta', JSON.stringify(userCustomizations));

    console.log('Cached user customizations delta:', {
      hasData: !!userCustomizations,
      actionMapsCount: userCustomizations?.action_maps?.length || 0,
      profileName: userCustomizations?.profile_name
    });
  } catch (error)
  {
    console.error('Failed to cache user customizations:', error);
    // Non-critical error - we can always reload from file
  }
}

async function newKeybinding()
{
  // Check if there are unsaved changes
  if (hasUnsavedChanges)
  {
    const confirmed = await showConfirmation(
      'You have unsaved keybinding changes. Do you want to discard them and start fresh?',
      'Unsaved Changes',
      'Discard & Start New',
      'Cancel',
      'btn-danger'
    );

    if (!confirmed) return;
  }

  try
  {
    // Clear backend customizations and reload AllBinds
    await invoke('clear_custom_bindings');
    await invoke('load_all_binds');

    // Get fresh merged bindings (AllBinds only, no customizations)
    currentKeybindings = await invoke('get_merged_bindings');

    // Clear persisted state - we're starting fresh
    localStorage.removeItem('keybindingsFilePath');
    localStorage.setItem('hasUnsavedChanges', 'false');

    // Clear the current filename since we're creating new bindings
    currentFilename = null;

    // Reset unsaved changes flag and update UI
    hasUnsavedChanges = false;
    updateUnsavedIndicator();

    // Display the fresh keybindings
    displayKeybindings();
    showUnsavedFileIndicator();

    // Reset filters and search
    currentFilter = 'all';
    searchTerm = '';
    customizedOnly = false;

    // Update filter buttons
    const filterBtns = document.querySelectorAll('.filter-section .category-item');
    filterBtns.forEach(btn =>
    {
      btn.classList.remove('active');
      if (btn.dataset.filter === 'all') btn.classList.add('active');
    });

    // Clear search input
    const searchInput = document.getElementById('search-input');
    if (searchInput) searchInput.value = '';

    // Uncheck customized only checkbox
    const customizedCheckbox = document.getElementById('customized-only-checkbox');
    if (customizedCheckbox) customizedCheckbox.checked = false;

    // Re-render with fresh state
    renderKeybindings();

  } catch (error)
  {
    console.error('Error creating new keybinding:', error);
    await showAlert(`Failed to create new keybinding: ${error}`, 'Error');
  }
}

function updateFileIndicator(filePath)
{
  const indicator = document.getElementById('loaded-file-indicator');
  const fileNameEl = document.getElementById('loaded-file-name');

  if (indicator && fileNameEl)
  {
    // Extract just the filename from the path
    const fileName = filePath.split(/[\\/]/).pop();
    fileNameEl.textContent = fileName;
    indicator.style.display = 'flex';
  }
}

function showUnsavedFileIndicator()
{
  const indicator = document.getElementById('loaded-file-indicator');
  const fileNameEl = document.getElementById('loaded-file-name');

  if (indicator && fileNameEl)
  {
    fileNameEl.textContent = 'Unsaved Keybinding Set';
    indicator.style.display = 'flex';
  }
}

function updateTemplateIndicator(templateName, fileName = null)
{
  const templateNameEl = document.getElementById('header-template-name');
  console.log('updateTemplateIndicator called with:', templateName, fileName);
  console.log('templateNameEl:', templateNameEl);
  if (templateNameEl)
  {
    let displayText = templateName || 'Untitled Template';
    if (fileName)
    {
      displayText += ` (${fileName})`;
    }
    templateNameEl.textContent = displayText;
    console.log('Updated header to:', templateNameEl.textContent);
  }
  // Always save to localStorage for persistence
  if (templateName)
  {
    localStorage.setItem('currentTemplateName', templateName);
  }
}

// Make it globally accessible
window.updateTemplateIndicator = updateTemplateIndicator;

// Helper to call updateTemplateIndicator safely (waits if not yet defined)
window.safeUpdateTemplateIndicator = function (name)
{
  if (window.updateTemplateIndicator)
  {
    window.updateTemplateIndicator(name);
  } else
  {
    // If the function isn't ready yet, store in localStorage and it will be called on DOMContentLoaded
    localStorage.setItem('pendingTemplateName', name);
  }
}

// Search for a button ID in the main keybindings view
window.searchMainTabForButtonId = function (buttonId)
{
  // Switch to the main tab
  switchTab('main');

  // Get the search input element
  const searchInput = document.getElementById('search-input');
  if (searchInput)
  {
    // Set the search input value
    searchInput.value = buttonId;

    // Trigger the search by firing an input event
    searchInput.dispatchEvent(new Event('input', { bubbles: true }));

    // Scroll the search input into view
    searchInput.scrollIntoView({ behavior: 'smooth', block: 'nearest' });

    // Focus the search input
    searchInput.focus();
  }
};

async function saveKeybindings()
{
  if (!currentKeybindings)
  {
    await showAlert('No keybindings loaded to save!', 'Save Keybindings');
    return;
  }

  try
  {
    // Get the saved file path
    const savedPath = localStorage.getItem('keybindingsFilePath');

    if (!savedPath)
    {
      // No file path - redirect to Save As
      await saveKeybindingsAs();
      return;
    }

    // Save to the current file path
    await invoke('export_keybindings', { filePath: savedPath });

    // Clear unsaved changes flag
    hasUnsavedChanges = false;
    localStorage.setItem('hasUnsavedChanges', 'false');
    updateUnsavedIndicator();

    console.log('Keybindings saved successfully to:', savedPath);

    // Check if auto-save to all installations is enabled
    const autoSaveEnabled = localStorage.getItem('autoSaveToAllInstallations') === 'true';
    const scInstallDirectory = localStorage.getItem('scInstallDirectory');

    if (autoSaveEnabled && scInstallDirectory)
    {
      try
      {
        // Get all detected installations
        const installations = await invoke('scan_sc_installations', { basePath: scInstallDirectory });

        if (installations.length > 0)
        {
          console.log(`Auto-saving to ${installations.length} installation(s)...`);

          // Filter out installations that contain the currently-opened file
          let skippedInstallation = null;
          const installationsToUpdate = installations.filter(installation =>
          {
            // Check if the current file path is within this installation
            if (savedPath && savedPath.toLowerCase().includes(installation.path.toLowerCase()))
            {
              skippedInstallation = installation.name;
              return false; // Skip this installation
            }
            return true;
          });

          // Save to each installation (except the one with the currently-open file)
          for (const installation of installationsToUpdate)
          {
            await invoke('save_bindings_to_install', {
              installationPath: installation.path
            });
            console.log(`Saved to ${installation.name}`);
          }

          // Build success message
          let successMsg = `Saved & deployed to ${installationsToUpdate.length} installation(s)`;
          if (skippedInstallation)
          {
            successMsg += ` (${skippedInstallation} was skipped as it's the currently open file location)`;
          }
          successMsg += '!';
          showSuccessMessage(successMsg);
        } else
        {
          showSuccessMessage('Saved!');
        }
      } catch (error)
      {
        console.error('Error auto-saving to installations:', error);
        showSuccessMessage('Saved (failed to deploy to installations)');
      }
    } else
    {
      // Show brief success message
      showSuccessMessage('Saved!');
    }
  } catch (error)
  {
    console.error('Error saving keybindings:', error);
    await showAlert(`Failed to save keybindings: ${error}`, 'Error');
  }
}

async function saveKeybindingsAs()
{
  if (!currentKeybindings)
  {
    await showAlert('No keybindings loaded to save!', 'Save Keybindings As');
    return;
  }

  try
  {
    // Prompt for a new file path
    const filePath = await save({
      filters: [{
        name: 'Star Citizen Keybindings',
        extensions: ['xml']
      }],
      defaultPath: 'layout_exported.xml'
    });

    if (!filePath)
    {
      // User cancelled
      return;
    }

    // Save to the new file path
    await invoke('export_keybindings', { filePath });

    // Update the stored file path
    localStorage.setItem('keybindingsFilePath', filePath);

    // Extract and set the filename
    const filename = filePath.split('\\').pop() || filePath.split('/').pop();
    currentFilename = filename;

    updateFileIndicator(filePath);
    updateCopyCommandButtonVisibility();

    // Clear unsaved changes flag
    hasUnsavedChanges = false;
    localStorage.setItem('hasUnsavedChanges', 'false');
    updateUnsavedIndicator();

    console.log('Keybindings saved successfully to:', filePath);

    // Show brief success message
    showSuccessMessage('Saved!');
  } catch (error)
  {
    console.error('Error saving keybindings:', error);
    await showAlert(`Failed to save keybindings: ${error}`, 'Error');
  }
}

function showSuccessMessage(message)
{
  // Create a temporary success indicator
  const indicator = document.createElement('div');
  indicator.textContent = message;
  indicator.style.cssText = `
    position: fixed;
    top: 80px;
    right: 20px;
    background-color: #28a745;
    color: white;
    padding: 12px 24px;
    border-radius: 4px;
    font-weight: 500;
    box-shadow: 0 2px 8px rgba(0,0,0,0.3);
    z-index: 10000;
    animation: slideIn 0.3s ease-out;
  `;

  document.body.appendChild(indicator);

  // Remove after 2 seconds
  setTimeout(() =>
  {
    indicator.style.animation = 'slideOut 0.3s ease-out';
    setTimeout(() => indicator.remove(), 300);
  }, 2000);
}

// Helper function to update copy command button visibility
function updateCopyCommandButtonVisibility()
{
  const copyCommandBtn = document.getElementById('copy-command-btn');
  if (copyCommandBtn)
  {
    copyCommandBtn.style.display = currentFilename ? 'inline-flex' : 'none';
  }
}

function displayKeybindings()
{
  if (!currentKeybindings) return;

  // Hide welcome screen, show bindings content
  document.getElementById('welcome-screen').style.display = 'none';
  document.getElementById('bindings-content').style.display = 'flex';

  // Enable save buttons
  document.getElementById('save-btn').disabled = false;
  document.getElementById('save-as-btn').disabled = false;

  // Update profile name - use a default if not available
  const profileName = currentKeybindings.profile_name || 'Star Citizen Keybindings';
  document.getElementById('profile-name').textContent = profileName;

  // Update copy command button visibility
  updateCopyCommandButtonVisibility();

  // Load axis names from HID descriptors for display
  loadDeviceAxisNames();

  // Render categories
  renderCategories();

  // Don't render device info for merged bindings (AllBinds doesn't have device info)
  if (currentKeybindings.devices)
  {
    renderDeviceInfo();
  }

  // Render keybindings
  renderKeybindings();
}

function renderCategories()
{
  const categoryList = document.getElementById('category-list');

  // Group action maps by their UICategory
  const categoryGroups = new Map();

  currentKeybindings.action_maps.forEach(actionMap =>
  {
    // Use ui_category if available, otherwise use a default group
    const category = actionMap.ui_category || '';

    if (!categoryGroups.has(category))
    {
      categoryGroups.set(category, []);
    }
    categoryGroups.get(category).push(actionMap);
  });

  // Sort categories alphabetically, but keep empty string at the end
  const sortedCategories = Array.from(categoryGroups.keys()).sort((a, b) =>
  {
    if (a === '') return 1;
    if (b === '') return -1;
    return a.localeCompare(b);
  });

  let html = `
    <div class="category-item ${currentCategory === null ? 'active' : ''}" 
         data-category="all">
      All Categories
    </div>
  `;

  // Render grouped categories
  sortedCategories.forEach(categoryName =>
  {
    const actionMaps = categoryGroups.get(categoryName);

    // Get friendly name from mapping, fallback to original category name
    const friendlyName = categoryFriendlyNames[categoryName] || categoryName || 'Uncategorized';

    // Add category header if we have multiple action maps in this category
    if (actionMaps.length > 1)
    {
      html += `<div class="category-header">${friendlyName}</div>`;
    }

    // Add action maps under this category
    actionMaps.forEach(actionMap =>
    {
      const displayName = actionMap.ui_label || actionMap.display_name || actionMap.name;
      const isActive = currentCategory === actionMap.name;
      const indent = actionMaps.length > 1 ? 'category-item-indented' : '';

      html += `
        <div class="category-item ${isActive ? 'active' : ''} ${indent}" 
             data-category="${actionMap.name}">
          ${displayName}
        </div>
      `;
    });
  });

  categoryList.innerHTML = html;

  // Add click listeners
  categoryList.querySelectorAll('.category-item').forEach(item =>
  {
    item.addEventListener('click', (e) =>
    {
      // Only remove active from items within this category list, not filter buttons
      categoryList.querySelectorAll('.category-item').forEach(i => i.classList.remove('active'));
      e.target.classList.add('active');
      currentCategory = e.target.dataset.category === 'all' ? null : e.target.dataset.category;
      renderKeybindings();
    });
  });
}

function renderDeviceInfo()
{
  const deviceList = document.getElementById('device-list');

  let html = '';

  if (currentKeybindings.devices.keyboards.length > 0)
  {
    html += '<div class="device-item"><div class="device-label">Keyboard</div>';
    currentKeybindings.devices.keyboards.forEach(kb =>
    {
      html += `<div>${kb}</div>`;
    });
    html += '</div>';
  }

  if (currentKeybindings.devices.mice.length > 0)
  {
    html += '<div class="device-item"><div class="device-label">Mouse</div>';
    currentKeybindings.devices.mice.forEach(mouse =>
    {
      html += `<div>${mouse}</div>`;
    });
    html += '</div>';
  }

  if (currentKeybindings.devices.joysticks.length > 0)
  {
    html += '<div class="device-item"><div class="device-label">Joysticks</div>';
    currentKeybindings.devices.joysticks.forEach(js =>
    {
      html += `<div>${js}</div>`;
    });
    html += '</div>';
  }

  deviceList.innerHTML = html;
}

// Helper function to check if an action has any bindings that will be displayed
function actionHasVisibleBindings(action)
{
  if (!action.bindings || action.bindings.length === 0) return true; // Show actions with no bindings (they're unbound)

  // Check if ALL bindings are empty/space-only defaults
  const allBindingsAreEmptyDefaults = action.bindings.every(binding =>
  {
    // Check the pattern BEFORE trimming to catch 'kb1_ ', 'js1_ ', etc.
    const isEmptyBinding = !!binding.input.match(/^(js\d+|kb\d+|mouse\d+|gp\d+)_\s*$/);
    return binding.is_default && isEmptyBinding;
  });

  // Always show actions that have only empty defaults (unbound by default actions)
  // These are actions users might want to bind themselves
  // BUT respect the input type filter
  if (allBindingsAreEmptyDefaults)
  {
    // If filtering by input type, check if there's at least one empty binding of that type
    if (currentFilter !== 'all')
    {
      return action.bindings.some(binding =>
      {
        if (currentFilter === 'keyboard') return binding.input_type === 'Keyboard';
        if (currentFilter === 'mouse') return binding.input_type === 'Mouse';
        if (currentFilter === 'joystick') return binding.input_type === 'Joystick';
        if (currentFilter === 'gamepad') return binding.input_type === 'Gamepad';
        return false;
      });
    }
    return true;
  }

  return action.bindings.some(binding =>
  {
    const trimmedInput = binding.input.trim();

    // Check if this is a cleared binding (check BEFORE trimming for the pattern)
    const isClearedBinding = !!binding.input.match(/^(js\d+|kb\d+|mouse\d+|gp\d+)_\s*$/);

    // Skip truly unbound bindings, but keep cleared bindings that override defaults
    if (!trimmedInput || trimmedInput === '') return false;

    // Filter out default bindings if showDefaultBindings is false
    if (!showDefaultBindings && binding.is_default && !isClearedBinding) return false;

    // Filter display based on current filter
    if (currentFilter !== 'all')
    {
      if (currentFilter === 'keyboard' && binding.input_type !== 'Keyboard') return false;
      if (currentFilter === 'mouse' && binding.input_type !== 'Mouse') return false;
      if (currentFilter === 'joystick' && binding.input_type !== 'Joystick') return false;
      if (currentFilter === 'gamepad' && binding.input_type !== 'Gamepad') return false;
    }

    return true;
  });
}

function renderKeybindings()
{
  if (!currentKeybindings) return;

  // Debug: log a sample binding to see its structure
  if (currentKeybindings.action_maps && currentKeybindings.action_maps.length > 0)
  {
    const firstMap = currentKeybindings.action_maps[0];
    if (firstMap.actions && firstMap.actions.length > 0)
    {
      const firstAction = firstMap.actions[0];
      if (firstAction.bindings && firstAction.bindings.length > 0)
      {
        console.log('Sample binding structure:', firstAction.bindings[0]);
      }
    }
  }

  const container = document.getElementById('action-maps-container');

  // Filter action maps
  let actionMaps = currentKeybindings.action_maps;

  if (currentCategory)
  {
    actionMaps = actionMaps.filter(am => am.name === currentCategory);
  }

  let html = '';

  actionMaps.forEach(actionMap =>
  {
    // Use ui_label if available (from merged bindings), otherwise use display_name
    const actionMapLabel = actionMap.ui_label || actionMap.display_name || actionMap.name;

    // Filter actions based on search term and input type filter
    let actions = actionMap.actions.filter(action =>
    {
      // Use ui_label if available, otherwise display_name
      const displayName = action.ui_label || action.display_name || action.name;

      // Input type filter
      if (currentFilter !== 'all')
      {
        const hasMatchingBinding = action.bindings && action.bindings.some(binding =>
        {
          if (currentFilter === 'keyboard') return binding.input_type === 'Keyboard';
          if (currentFilter === 'mouse') return binding.input_type === 'Mouse';
          if (currentFilter === 'joystick') return binding.input_type === 'Joystick';
          if (currentFilter === 'gamepad') return binding.input_type === 'Gamepad';
          return true;
        });

        if (!hasMatchingBinding) return false;
      }

      // Customized only filter - if checked, skip actions without customized bindings for the current device type
      if (customizedOnly)
      {
        const hasCustomizedBinding = action.bindings && action.bindings.some(binding =>
        {
          // Check if this binding is customized (not default)
          if (binding.is_default) return false;

          // If a specific device type is selected, only count customizations for that type
          if (currentFilter !== 'all')
          {
            if (currentFilter === 'keyboard' && binding.input_type !== 'Keyboard') return false;
            if (currentFilter === 'mouse' && binding.input_type !== 'Mouse') return false;
            if (currentFilter === 'joystick' && binding.input_type !== 'Joystick') return false;
            if (currentFilter === 'gamepad' && binding.input_type !== 'Gamepad') return false;
          }

          return true;
        });

        if (!hasCustomizedBinding) return false;
      }

      // Search filter - search in action name AND binding names
      if (searchTerm)
      {
        const searchInAction = displayName.toLowerCase().includes(searchTerm) ||
          action.name.toLowerCase().includes(searchTerm);

        const searchInBindings = action.bindings && action.bindings.some(binding =>
          binding.display_name.toLowerCase().includes(searchTerm) ||
          binding.input.toLowerCase().includes(searchTerm)
        );

        if (!searchInAction && !searchInBindings)
        {
          return false;
        }
      }

      return true;
    });

    if (actions.length === 0) return; // Skip empty action maps

    // Filter actions to only those with visible bindings, and collect them
    const visibleActions = actions.filter(action => actionHasVisibleBindings(action));

    // Skip this action map if there are no visible actions
    if (visibleActions.length === 0) return;

    html += `
      <div class="action-map">
        <div class="action-map-header" onclick="toggleActionMap(this)">
          <h3>${actionMapLabel}</h3>
          <span class="action-map-toggle">▼</span>
        </div>
        <div class="actions-list">
    `;

    visibleActions.forEach(action =>
    {
      const displayName = action.ui_label || action.display_name || action.name;
      const isCustomized = action.is_customized || false;
      const onHold = action.on_hold || false;

      html += `
        <div class="action-item ${isCustomized ? 'customized' : ''}">
          <div class="action-name">
            ${isCustomized ? '<span class="customized-indicator" title="Customized binding">★</span>' : ''}
            ${displayName}${onHold ? ' <span class="hold-indicator" title="Requires holding">(Hold)</span>' : ''}
          </div>
          <div class="action-buttons">
          <button class="action-btn btn-manage btn btn-secondary" 
                  data-action-map="${actionMap.name}"
                  data-action-name="${action.name}"
                  data-action-display="${displayName}"
                  title="Manage all bindings for this action"
                  onclick="openActionBindingsModal(this.dataset.actionMap, this.dataset.actionName, this.dataset.actionDisplay)">
            ⚙️ Manage
          </button>
          <button class="action-btn btn-reset btn btn-secondary" 
                  data-action-map="${actionMap.name}"
                  data-action-name="${action.name}"
                  title="Reset to default bindings"
                  onclick="resetActionBinding(this.dataset.actionMap, this.dataset.actionName)">
            ↻ Reset
          </button>
          <button class="action-btn btn-clear btn btn-secondary" 
          data-action-map="${actionMap.name}"
          data-action-name="${action.name}"
          title="Clear all bindings for this action"
          onclick="clearActionBinding(this.dataset.actionMap, this.dataset.actionName)">
          ✕ Clear
          </button>
          <button class="action-btn btn-bind btn btn-success" 
                  data-action-map="${actionMap.name}"
                  data-action-name="${action.name}"
                  data-action-display="${displayName}"
                  onclick="startBinding(this.dataset.actionMap, this.dataset.actionName, this.dataset.actionDisplay)">
            Bind
          </button>
          </div>
          <div class="bindings-container">
      `;

      // Check if this action only has the special "unbound" placeholder binding
      const hasOnlyUnboundPlaceholder = action.bindings && action.bindings.length === 1 &&
        action.bindings[0].input.match(/^(js\d+|kb\d+|mouse\d+|gp\d+)_\s*$/) &&
        action.bindings[0].is_default &&
        action.bindings[0].display_name === 'Unbound';

      if (!action.bindings || action.bindings.length === 0 || hasOnlyUnboundPlaceholder)
      {
        // Show nothing - the action will just appear without any binding tags
        html += `<span class="binding-tag unbound" style="visibility: hidden;">Unbound</span>`;
      } else
      {
        action.bindings.forEach(binding =>
        {
          const trimmedInput = binding.input.trim();

          // Skip the unbound placeholder binding if it exists alongside real bindings
          const isUnboundPlaceholder = binding.input.match(/^(js\d+|kb\d+|mouse\d+|gp\d+)_\s*$/) &&
            binding.is_default &&
            binding.display_name === 'Unbound';
          if (isUnboundPlaceholder) return;

          // Check if this is a cleared binding (overriding a default with blank)
          const isClearedBinding = !!trimmedInput.match(/^(js\d+|kb\d+|mouse\d+|gp\d+)_\s*$/) && binding.is_default;

          // Skip truly unbound bindings
          if (!trimmedInput || trimmedInput === '') return;

          // Filter out default bindings if showDefaultBindings is false
          if (!showDefaultBindings && binding.is_default && !isClearedBinding) return;

          // Filter display based on current filter
          if (currentFilter !== 'all')
          {
            if (currentFilter === 'keyboard' && binding.input_type !== 'Keyboard') return;
            if (currentFilter === 'mouse' && binding.input_type !== 'Mouse') return;
            if (currentFilter === 'joystick' && binding.input_type !== 'Joystick') return;
            if (currentFilter === 'gamepad' && binding.input_type !== 'Gamepad') return;
          }

          let typeClass = 'unbound';
          let icon = '○';

          // Only assign a type class if there's an actual input type
          // Don't categorize truly unbound bindings
          if (binding.input_type && binding.input_type !== 'Unbound')
          {
            if (binding.input_type === 'Keyboard')
            {
              typeClass = 'keyboard';
              icon = '⌨';
            } else if (binding.input_type === 'Mouse')
            {
              typeClass = 'mouse';
              icon = '🖱';
            } else if (binding.input_type === 'Joystick')
            {
              typeClass = 'joystick';
              icon = '🕹';
            } else if (binding.input_type === 'Gamepad')
            {
              typeClass = 'gamepad';
              icon = '🎮';
            }
          }

          // Show if it's a default binding or a cleared override
          const defaultIndicator = !isClearedBinding && binding.is_default ? ' (default)' : '';

          // Show multi-tap indicator if present
          const multiTapIndicator = binding.multi_tap ? ` <span class="multi-tap-indicator" title="Double-tap binding">(${binding.multi_tap}x tap)</span>` : '';

          // Format display name with activation mode appended
          let displayText = binding.display_name;

          if (binding.activation_mode && !isClearedBinding)
          {
            const formattedMode = binding.activation_mode
              .split('_')
              .map(word => word.charAt(0).toUpperCase() + word.slice(1))
              .join(' ');
            displayText += ` - ${formattedMode}`;
          }

          // For cleared bindings, show the original default binding with strikethrough
          // display_name already contains the formatted original binding text for cleared bindings
          const clearedDisplayText = isClearedBinding ? binding.display_name : displayText;

          // Try to get HID axis name for joystick/gamepad axis bindings
          const hidAxisName = getHidAxisNameForBinding(binding.input);
          const axisHint = hidAxisName ? ` <span class="hid-axis-hint" title="Hardware axis: ${hidAxisName}">[${hidAxisName}]</span>` : '';

          // Only show remove button for non-unbound bindings
          const removeButton = typeClass !== 'unbound' ? `
              <button class="binding-remove-btn" 
                      title="Clear this binding"
                      data-action-map="${actionMap.name}"
                      data-action-name="${action.name}"
                      data-input="${binding.input.replace(/"/g, '&quot;')}">×</button>` : '';

          html += `
            <span class="binding-tag ${typeClass} ${binding.is_default ? 'default-binding' : ''} ${isClearedBinding ? 'cleared-binding' : ''}">
              <span class="binding-icon">${icon}</span>
              <span class="binding-label ${isClearedBinding ? 'cleared-text' : ''}">
                ${isClearedBinding ? `<span class="cleared-default-text">${clearedDisplayText}</span>` : displayText}${axisHint}
              </span>
              ${defaultIndicator}${multiTapIndicator}${removeButton}
            </span>
          `;
        });
      }

      html += `
          </div>
        </div>
      `;
    });

    html += `
        </div>
      </div>
    `;
  });

  if (html === '')
  {
    html = '<div class="empty-state"><p>No actions match your current filters</p></div>';
  }

  container.innerHTML = html;

  // Add event delegation for binding remove buttons
  container.querySelectorAll('.binding-remove-btn').forEach(btn =>
  {
    btn.addEventListener('click', async (e) =>
    {
      e.stopPropagation();
      e.preventDefault();

      const actionMap = btn.dataset.actionMap;
      const actionName = btn.dataset.actionName;
      const input = btn.dataset.input;

      await removeBinding(actionMap, actionName, input);
    });
  });
}

// Toggle action map visibility
window.toggleActionMap = function (headerEl)
{
  const actionsList = headerEl.nextElementSibling;
  const toggle = headerEl.querySelector('.action-map-toggle');

  if (actionsList.style.display === 'none')
  {
    actionsList.style.display = 'grid';
    toggle.classList.remove('collapsed');
  } else
  {
    actionsList.style.display = 'none';
    toggle.classList.add('collapsed');
  }
};

// Binding mode functions
async function startBinding(actionMapName, actionName, actionDisplayName)
{
  // Ensure any previous detection session is fully cleaned up
  stopDetection('start-new-binding');

  // Generate unique ID for this binding attempt to ignore stale events
  currentBindingId = Date.now() + Math.random();
  console.log('[TIMER] startBinding called for:', actionDisplayName, 'new currentBindingId:', currentBindingId);

  currentBindingAction = { actionMapName, actionName, actionDisplayName };
  bindingMode = true;
  setBindingSaveEnabled(false);
  ignoreModalMouseInputs = false;

  // Capture the binding ID for this attempt (for closure)
  const thisBindingId = currentBindingId;

  // Show modal
  const modal = document.getElementById('binding-modal');
  modal.style.display = 'flex';

  // Reset activation mode dropdown to default
  const activationModeSelect = document.getElementById('activation-mode-select');
  if (activationModeSelect)
  {
    activationModeSelect.value = '';
  }

  // Clear any previous conflict display
  const conflictDisplay = document.getElementById('binding-conflict-display');
  if (conflictDisplay)
  {
    conflictDisplay.style.display = 'none';
    conflictDisplay.innerHTML = '';
  }

  document.getElementById('binding-modal-action').textContent = 'Binding Action: ' + actionDisplayName;
  document.getElementById('binding-modal-status').textContent = 'Press any button, key, mouse button, or move any axis...';

  // Start countdown
  const countdown = 10;
  let remaining = countdown;
  const countdownEl = document.getElementById('binding-modal-countdown');
  countdownEl.textContent = countdown;

  console.log('[TIMER] Starting new countdownInterval for binding:', actionDisplayName);
  const intervalId = setInterval(() =>
  {
    remaining--;
    console.log('[TIMER] Countdown tick:', remaining, 'intervalId:', intervalId);
    countdownEl.textContent = remaining;
    if (remaining <= 0)
    {
      console.log('[TIMER] Countdown reached 0, clearing interval:', intervalId);
      clearInterval(intervalId);
      if (countdownInterval === intervalId)
      {
        countdownInterval = null;
      }
    }
  }, 1000);
  countdownInterval = intervalId;
  console.log('[TIMER] countdownInterval ID assigned:', countdownInterval);

  try
  {
    // Set global detection flag to active
    isDetectionActive = true;

    // Track all detected inputs
    const allDetectedInputs = new Map(); // key: input_string, value: processed input object
    let selectionContainer = null;
    let statusEl = document.getElementById('binding-modal-status');

    // Multi-input selection state
    let selectedInputKey = null;
    const selectionButtons = new Map();
    let selectionMessageEl = null;

    const setSelectionMessage = (text) =>
    {
      if (selectionMessageEl)
      {
        selectionMessageEl.textContent = text;
      }
    };

    const setPendingBindingSelection = async (input) =>
    {
      const conflicts = await invoke('find_conflicting_bindings', {
        input: input.scFormattedInput,
        excludeActionMap: actionMapName,
        excludeAction: actionName
      });

      window.pendingBinding = {
        actionMapName,
        actionName,
        mappedInput: input.scFormattedInput,
        displayName: input.displayName,
        conflicts
      };

      setBindingSaveEnabled(true);

      return conflicts;
    };

    const updateConflictDisplay = (conflicts = []) =>
    {
      const existingWarning = statusEl.querySelector('.binding-conflict-warning');
      if (existingWarning)
      {
        existingWarning.remove();
      }

      if (!conflicts || conflicts.length === 0)
      {
        displayConflictsInModal([]);
        return;
      }

      displayConflictsInModal(conflicts);
    };

    const updateSelectionButtonStates = () =>
    {
      selectionButtons.forEach((btn, key) =>
      {
        btn.classList.toggle('selected', key === selectedInputKey);
      });
    };

    // Activate keyboard detection
    keyboardDetectionActive = true;

    // Activate mouse button detection
    let mouseDetectionHandler = null;

    // Create mouse event handler
    mouseDetectionHandler = async (event) =>
    {
      // Ignore if detection window has ended or we're hovering modal buttons
      if (!isDetectionActive || !window.mouseDetectionActive || ignoreModalMouseInputs) return;

      // Only capture mouse events within the modal itself
      const modal = document.getElementById('binding-modal');
      if (!modal.contains(event.target)) return;

      // Prevent default browser behavior
      event.preventDefault();
      event.stopPropagation();

      // Map mouse button numbers to Star Citizen format
      const buttonMap = {
        0: 'mouse1',  // Left button
        1: 'mouse3',  // Middle button
        2: 'mouse2',  // Right button
        3: 'mouse4',  // Side button (back)
        4: 'mouse5'   // Side button (forward)
      };

      const scButton = buttonMap[event.button] || `mouse${event.button + 1}`;

      // Build the input string (mouse format)
      const inputString = scButton;

      // Build display name
      const buttonNames = {
        'mouse1': 'Left Mouse Button',
        'mouse2': 'Right Mouse Button',
        'mouse3': 'Middle Mouse Button',
        'mouse4': 'Mouse Button 4',
        'mouse5': 'Mouse Button 5'
      };
      const displayName = buttonNames[scButton] || `Mouse Button ${event.button}`;

      // Create a synthetic event that matches the structure from Rust backend
      const syntheticResult = {
        input_string: inputString,
        display_name: displayName,
        device_type: 'Mouse',
        axis_value: null,
        modifiers: [],
        is_modifier: false
      };

      // Process this mouse input through the same pipeline
      const processed = processInput(syntheticResult);

      if (!processed) return;

      // Only add to map if not already there
      if (!allDetectedInputs.has(processed.scFormattedInput))
      {
        allDetectedInputs.set(processed.scFormattedInput, processed);

        if (allDetectedInputs.size === 1)
        {
          statusEl.innerHTML = '';
          renderDetectedInputMessage(statusEl, `✅ Detected: ${processed.displayName}`);

          clearPrimaryCountdown();
          document.getElementById('binding-modal-countdown').textContent = '';
          startSecondaryDetectionWindow();

          const helperNote = document.createElement('div');
          helperNote.className = 'input-confirm-note';
          helperNote.textContent = 'Press another input within 1 second to pick a different option, or click Save Binding to confirm.';
          statusEl.appendChild(helperNote);

          selectedInputKey = processed.scFormattedInput;
          const conflicts = await setPendingBindingSelection(processed);
          updateConflictDisplay(conflicts);
          updateSelectionButtonStates();
        }
        else if (allDetectedInputs.size === 2)
        {
          // Second input detected - remove confirm UI and switch to selection UI
          clearPrimaryCountdown();

          // Clear any existing UI and show selection
          statusEl.innerHTML = '';
          document.getElementById('binding-modal-countdown').textContent = '';

          selectionMessageEl = document.createElement('div');
          selectionMessageEl.className = 'input-selection-message';
          const initiallySelected = allDetectedInputs.get(selectedInputKey) || processed;
          selectionMessageEl.textContent = `Multiple inputs detected. Selected: ${initiallySelected.displayName}`;
          statusEl.appendChild(selectionMessageEl);

          const helperNote = document.createElement('div');
          helperNote.className = 'input-confirm-note';
          helperNote.textContent = 'Click the input you want to keep, then press Save Binding.';
          statusEl.appendChild(helperNote);

          selectionContainer = document.createElement('div');
          selectionContainer.className = 'input-selection-container';
          statusEl.appendChild(selectionContainer);

          selectionButtons.clear();

          // Add both inputs
          Array.from(allDetectedInputs.values()).forEach((input) =>
          {
            addDetectedInputButton(input);
          });

          updateSelectionButtonStates();
          updateConflictDisplay(window.pendingBinding?.conflicts || []);
        }
        else
        {
          // More inputs - just add the new button
          addDetectedInputButton(processed);
        }
      }
    };

    // Prevent right-click context menu during recording
    const contextMenuHandler = (event) =>
    {
      if (!window.mouseDetectionActive) return;
      const modal = document.getElementById('binding-modal');
      if (!modal.contains(event.target)) return;
      event.preventDefault();
    };

    // Prevent browser navigation for back/forward buttons
    const mouseUpHandler = (event) =>
    {
      if (!window.mouseDetectionActive) return;
      const modal = document.getElementById('binding-modal');
      if (!modal.contains(event.target)) return;

      // Prevent default for buttons 3 and 4 (back/forward)
      if (event.button === 3 || event.button === 4)
      {
        event.preventDefault();
        event.stopPropagation();
      }
    };

    // Prevent beforeunload navigation during recording
    const beforeUnloadHandler = (event) =>
    {
      if (!window.mouseDetectionActive) return;
      event.preventDefault();
      event.returnValue = '';
    };

    // Store handlers on window for cleanup
    window.mouseDetectionHandler = mouseDetectionHandler;
    window.contextMenuHandler = contextMenuHandler;
    window.mouseUpHandler = mouseUpHandler;
    window.beforeUnloadHandler = beforeUnloadHandler;
    window.mouseDetectionActive = true;

    // Add mouse listeners (capture phase)
    document.addEventListener('mousedown', mouseDetectionHandler, true);
    document.addEventListener('mouseup', mouseUpHandler, true);
    document.addEventListener('contextmenu', contextMenuHandler, true);
    window.addEventListener('beforeunload', beforeUnloadHandler, true);

    // Create keyboard event handler
    keyboardDetectionHandler = async (event) =>
    {
      // Ignore if detection window has ended
      if (!isDetectionActive || !keyboardDetectionActive) return;

      // Prevent default browser behavior
      event.preventDefault();
      event.stopPropagation();

      const code = event.code;
      const key = event.key;

      // Detect modifiers being held
      const modifiers = [];
      if (event.shiftKey) modifiers.push(event.location === 1 ? 'LSHIFT' : 'RSHIFT');
      if (event.ctrlKey) modifiers.push(event.location === 1 ? 'LCTRL' : 'RCTRL');
      if (event.altKey) modifiers.push(event.location === 1 ? 'LALT' : 'RALT');

      // Convert to Star Citizen format
      const scKey = convertKeyCodeToSC(code, key);

      // Skip if this is just a modifier key by itself - don't trigger detection for modifiers
      const isModifierKey = ['lshift', 'rshift', 'lctrl', 'rctrl', 'lalt', 'ralt', 'lwin'].includes(scKey);

      if (isModifierKey)
      {
        // Don't process modifier keys by themselves
        return;
      }

      // Build the input string (kb1_key format)
      const inputString = `kb1_${scKey}`;

      // Build display name
      const displayName = `Keyboard - ${code}`;

      // Create a synthetic event that matches the structure from Rust backend
      const syntheticResult = {
        input_string: inputString,
        display_name: displayName,
        device_type: 'Keyboard',
        axis_value: null,
        modifiers: modifiers,
        is_modifier: false
      };

      // Process this keyboard input through the same pipeline
      const processed = processInput(syntheticResult);

      if (!processed) return;

      // Only add to map if not already there
      if (!allDetectedInputs.has(processed.scFormattedInput))
      {
        allDetectedInputs.set(processed.scFormattedInput, processed);

        if (allDetectedInputs.size === 1)
        {
          statusEl.innerHTML = '';
          renderDetectedInputMessage(statusEl, `✅ Detected: ${processed.displayName}`);

          clearPrimaryCountdown();
          document.getElementById('binding-modal-countdown').textContent = '';
          startSecondaryDetectionWindow();

          const helperNote = document.createElement('div');
          helperNote.className = 'input-confirm-note';
          helperNote.textContent = 'Press another input within 1 second to pick a different option, or click Save Binding to confirm.';
          statusEl.appendChild(helperNote);

          selectedInputKey = processed.scFormattedInput;
          const conflicts = await setPendingBindingSelection(processed);
          updateConflictDisplay(conflicts);
          updateSelectionButtonStates();
        }
        else if (allDetectedInputs.size === 2)
        {
          // Second input detected - remove confirm UI and switch to selection UI
          clearPrimaryCountdown();

          // Clear any existing UI and show selection
          statusEl.innerHTML = '';
          document.getElementById('binding-modal-countdown').textContent = '';

          selectionMessageEl = document.createElement('div');
          selectionMessageEl.className = 'input-selection-message';
          const initiallySelected = allDetectedInputs.get(selectedInputKey) || processed;
          selectionMessageEl.textContent = `Multiple inputs detected. Selected: ${initiallySelected.displayName}`;
          statusEl.appendChild(selectionMessageEl);

          const helperNote = document.createElement('div');
          helperNote.className = 'input-confirm-note';
          helperNote.textContent = 'Click the input you want to keep, then press Save Binding.';
          statusEl.appendChild(helperNote);

          selectionContainer = document.createElement('div');
          selectionContainer.className = 'input-selection-container';
          statusEl.appendChild(selectionContainer);

          selectionButtons.clear();

          // Add both inputs
          Array.from(allDetectedInputs.values()).forEach((input) =>
          {
            addDetectedInputButton(input);
          });

          updateSelectionButtonStates();
          updateConflictDisplay(window.pendingBinding?.conflicts || []);
        }
        else
        {
          // More inputs - just add the new button
          addDetectedInputButton(processed);
        }
      }
    };

    // Add keyboard listener (capture phase)
    document.addEventListener('keydown', keyboardDetectionHandler, true);

    // Helper function to process a raw input result
    const processInput = (result) =>
    {
      console.log('INPUT DETECTED (raw):', result.display_name, result.input_string);

      // Apply joystick mapping if applicable
      const mappedInput = applyJoystickMapping(result.input_string);

      if (mappedInput === null)
      {
        return null; // Skip disabled joysticks
      }

      // Convert to Star Citizen format (handles axis naming)
      let scFormattedInput = toStarCitizenFormat(mappedInput);

      // Add modifier prefixes if any (lowercase to match AllBinds.xml format)
      if (result.modifiers && result.modifiers.length > 0)
      {
        const modifierOrder = ['LALT', 'RALT', 'LCTRL', 'RCTRL', 'LSHIFT', 'RSHIFT'];
        const sortedModifiers = result.modifiers
          .filter(mod => modifierOrder.includes(mod))
          .sort((a, b) => modifierOrder.indexOf(a) - modifierOrder.indexOf(b))
          .map(mod => mod.toLowerCase());

        if (sortedModifiers.length > 0)
        {
          scFormattedInput = sortedModifiers.join('+') + '+' + scFormattedInput;
        }
      }

      // Update display name if mapping was applied
      let displayName = result.display_name;
      if (mappedInput !== result.input_string)
      {
        displayName = displayName.replace(/Joystick \d+/, (match) =>
        {
          const newJsNum = mappedInput.match(/^js(\d+)_/)[1];
          return `Joystick ${newJsNum}`;
        });
      }

      // Add modifiers to display name
      if (result.modifiers && result.modifiers.length > 0)
      {
        const modifierOrder = ['LALT', 'RALT', 'LCTRL', 'RCTRL', 'LSHIFT', 'RSHIFT'];
        const sortedModifiers = result.modifiers
          .filter(mod => modifierOrder.includes(mod))
          .sort((a, b) => modifierOrder.indexOf(a) - modifierOrder.indexOf(b));

        if (sortedModifiers.length > 0)
        {
          displayName = sortedModifiers.join(' + ') + ' + ' + displayName;
        }
      }

      return {
        scFormattedInput,
        displayName,
        originalResult: result
      };
    };

    // Helper function to add a button to the selection UI
    const addDetectedInputButton = (processedInput) =>
    {
      if (!selectionContainer) return;

      const btn = document.createElement('button');
      btn.className = 'input-selection-btn';
      btn.innerHTML = `
        <span class="input-selection-icon">🎮</span>
        <span class="input-selection-name">${processedInput.displayName}</span>
      `;

      const inputKey = processedInput.scFormattedInput;

      btn.addEventListener('click', async () =>
      {
        const selectedInput = allDetectedInputs.get(inputKey);

        if (!selectedInput) return;

        selectedInputKey = inputKey;
        updateSelectionButtonStates();
        setSelectionMessage(`Selected: ${selectedInput.displayName}`);

        stopDetection('user-selection-option');

        const conflicts = await setPendingBindingSelection(selectedInput);
        updateConflictDisplay(conflicts);
      });

      selectionButtons.set(inputKey, btn);
      selectionContainer.appendChild(btn);
      updateSelectionButtonStates();
    };

    // Listen for input-detected events (from joystick/backend)
    const unlistenInputs = await listen('input-detected', async (event) =>
    {
      console.log('[TIMER] [EVENT] input-detected received, session_id:', event.payload.session_id, 'thisBindingId:', thisBindingId.toString(), 'isDetectionActive:', isDetectionActive);

      // Ignore if detection window has ended
      if (!isDetectionActive)
      {
        console.log('[TIMER] [EVENT] Ignoring input-detected because detection is no longer active');
        return;
      }

      // Ignore if this event is from a previous binding attempt (check session ID)
      if (event.payload.session_id !== thisBindingId.toString())
      {
        console.log('[TIMER] [EVENT] Ignoring stale input-detected event (session ID mismatch)');
        return;
      }

      // Ignore if this event is from a previous binding attempt
      if (currentBindingId !== thisBindingId)
      {
        console.log('[TIMER] [EVENT] Ignoring stale input-detected event (binding ID mismatch)');
        return;
      }

      const result = event.payload;
      const processed = processInput(result);

      if (!processed) return;

      // Only add to map if not already there
      if (!allDetectedInputs.has(processed.scFormattedInput))
      {
        allDetectedInputs.set(processed.scFormattedInput, processed);

        if (allDetectedInputs.size === 1)
        {
          statusEl.innerHTML = '';
          renderDetectedInputMessage(statusEl, `✅ Detected: ${processed.displayName}`);

          clearPrimaryCountdown();
          document.getElementById('binding-modal-countdown').textContent = '';
          startSecondaryDetectionWindow();

          const helperNote = document.createElement('div');
          helperNote.className = 'input-confirm-note';
          helperNote.textContent = 'Press another input within 1 second to pick a different option, or click Save Binding to confirm.';
          statusEl.appendChild(helperNote);

          selectedInputKey = processed.scFormattedInput;
          const conflicts = await setPendingBindingSelection(processed);
          updateConflictDisplay(conflicts);
          updateSelectionButtonStates();
        }
        else if (allDetectedInputs.size === 2)
        {
          // Second input detected - remove confirm UI and switch to selection UI
          clearPrimaryCountdown();

          // Clear any existing UI and show selection
          statusEl.innerHTML = '';
          document.getElementById('binding-modal-countdown').textContent = '';

          selectionMessageEl = document.createElement('div');
          selectionMessageEl.className = 'input-selection-message';
          const initiallySelected = allDetectedInputs.get(selectedInputKey) || processed;
          selectionMessageEl.textContent = `Multiple inputs detected. Selected: ${initiallySelected.displayName}`;
          statusEl.appendChild(selectionMessageEl);

          const helperNote = document.createElement('div');
          helperNote.className = 'input-confirm-note';
          helperNote.textContent = 'Click the input you want to keep, then press Save Binding.';
          statusEl.appendChild(helperNote);

          selectionContainer = document.createElement('div');
          selectionContainer.className = 'input-selection-container';
          statusEl.appendChild(selectionContainer);

          selectionButtons.clear();

          // Add both inputs
          Array.from(allDetectedInputs.values()).forEach((input) =>
          {
            addDetectedInputButton(input);
          });

          updateSelectionButtonStates();
          updateConflictDisplay(window.pendingBinding?.conflicts || []);
        }
        else
        {
          // More inputs - just add the new button
          addDetectedInputButton(processed);
        }
      }
    });

    // Store unlisten function for cleanup
    window.currentInputDetectionUnlisten = unlistenInputs;

    // Listen for completion event
    const unlistenCompletion = await listen('input-detection-complete', async (event) =>
    {
      console.log('[TIMER] [EVENT] input-detection-complete received, session_id:', event.payload?.session_id, 'thisBindingId:', thisBindingId.toString(), 'currentBindingId:', currentBindingId, 'isDetectionActive:', isDetectionActive, 'detectedInputs:', allDetectedInputs.size);

      // Ignore if this event is from a previous binding attempt (check session ID)
      if (event.payload?.session_id !== thisBindingId.toString())
      {
        console.log('[TIMER] [EVENT] Ignoring stale input-detection-complete event (session ID mismatch)');
        return;
      }

      // Ignore if this event is from a previous binding attempt
      if (currentBindingId !== thisBindingId)
      {
        console.log('[TIMER] [EVENT] Ignoring stale input-detection-complete event (ID mismatch)');
        return;
      }

      // Ignore if detection was already completed/cancelled
      if (!isDetectionActive)
      {
        console.log('[TIMER] [EVENT] Ignoring input-detection-complete, detection not active');
        return;
      }

      // Double-check the modal is still visible and we're still in binding mode
      const modal = document.getElementById('binding-modal');
      if (!modal || modal.style.display === 'none' || !bindingMode)
      {
        console.log('[TIMER] [EVENT] Ignoring input-detection-complete, modal not visible or not in binding mode');
        return;
      }

      // If we have at least one input detected, IGNORE completion event
      // Keep listening for potential double-tap within the 1-second window
      if (allDetectedInputs.size > 0)
      {
        console.log('[TIMER] [EVENT] Ignoring input-detection-complete - waiting for potential double-tap (inputs detected:', allDetectedInputs.size, ')');
        return;
      }

      console.log('[TIMER] [EVENT] Processing input-detection-complete event');
      stopDetection('backend-timeout');

      // Only reach here if no inputs were detected at all
      console.log('[TIMER] [EVENT] No inputs detected, showing timeout message');
      statusEl.textContent = 'No input detected - timed out';
      document.getElementById('binding-modal-countdown').textContent = '';

      // Store the binding ID to check it hasn't changed
      const timeoutBindingId = currentBindingId;
      setTimeout(() =>
      {
        // Only close if we're still on the same binding session
        if (currentBindingId === timeoutBindingId && bindingMode)
        {
          console.log('[TIMER] [EVENT] Closing modal after 2s timeout, binding ID match');
          closeBindingModal();
        }
        else
        {
          console.log('[TIMER] [EVENT] NOT closing modal - binding ID changed or modal already closed');
        }
      }, 2000);
    });

    // Store unlisten function for cleanup
    window.currentCompletionUnlisten = unlistenCompletion;

    // Start event-based detection (doesn't return a value, just emits events)
    console.log('[TIMER] [RUST] Calling wait_for_inputs_with_events with bindingId:', thisBindingId);
    invoke('wait_for_inputs_with_events', {
      sessionId: thisBindingId.toString(),
      initialTimeoutSecs: countdown,
      collectDurationSecs: 2
    }).catch((error) =>
    {
      console.error('[TIMER] [RUST] Error during input detection:', error);

      // Cleanup listeners
      if (window.currentInputDetectionUnlisten)
      {
        window.currentInputDetectionUnlisten();
        window.currentInputDetectionUnlisten = null;
      }
      if (window.currentCompletionUnlisten)
      {
        window.currentCompletionUnlisten();
        window.currentCompletionUnlisten = null;
      }

      // Cleanup keyboard detection
      if (keyboardDetectionHandler)
      {
        document.removeEventListener('keydown', keyboardDetectionHandler, true);
        keyboardDetectionHandler = null;
      }
      keyboardDetectionActive = false;

      // Cleanup mouse detection
      if (window.mouseDetectionHandler)
      {
        document.removeEventListener('mousedown', window.mouseDetectionHandler, true);
        document.removeEventListener('mouseup', window.mouseUpHandler, true);
        document.removeEventListener('contextmenu', window.contextMenuHandler, true);
        window.removeEventListener('beforeunload', window.beforeUnloadHandler, true);
        window.mouseDetectionHandler = null;
        window.contextMenuHandler = null;
        window.mouseUpHandler = null;
        window.beforeUnloadHandler = null;
      }
      window.mouseDetectionActive = false;
    });
  } catch (error)
  {
    // Clear the timer in case of error
    if (countdownInterval)
    {
      clearInterval(countdownInterval);
      countdownInterval = null;
    }
    console.error('Error waiting for input:', error);
    await showAlert(`Error waiting for input: ${error}`, 'Error');
    closeBindingModal();
  }
}

function cancelBinding()
{
  closeBindingModal();
}

async function clearBinding()
{
  if (!currentBindingAction) return;

  try
  {
    await invoke('update_binding', {
      actionMapName: currentBindingAction.actionMapName,
      actionName: currentBindingAction.actionName,
      newInput: ''
    });

    // Mark as unsaved
    hasUnsavedChanges = true;
    updateUnsavedIndicator();

    await refreshBindings();
    closeBindingModal();
  } catch (error)
  {
    console.error('Error clearing binding:', error);
    await showAlert(`Error clearing binding: ${error}`, 'Error');
  }
}

function closeBindingModal()
{
  console.log('[TIMER] closeBindingModal called');
  stopDetection('modal-close');

  bindingMode = false;
  currentBindingAction = null;
  document.getElementById('binding-modal').style.display = 'none';
  setBindingSaveEnabled(false);
}

async function refreshBindings()
{
  try
  {
    console.log('Refreshing bindings from backend...');
    currentKeybindings = await invoke('get_merged_bindings');
    console.log('Got merged bindings with', currentKeybindings.action_maps?.length, 'action maps');

    // Update working copy with latest changes
    if (currentKeybindings)
    {
      // Cache only the user customizations (delta), not the full merged view
      await cacheUserCustomizations();
      localStorage.setItem('hasUnsavedChanges', hasUnsavedChanges.toString());
      console.log('Updated user customizations delta in localStorage');
    }

    renderKeybindings();
  } catch (error)
  {
    console.error('Error refreshing bindings:', error);
  }
}

// Make startBinding available globally
window.startBinding = startBinding;

// Global function to clear all bindings for an action (called from action-level Clear button)
window.clearActionBinding = async function (actionMapName, actionName)
{
  console.log('clearActionBinding called with:', { actionMapName, actionName });

  // Show custom confirmation dialog
  const confirmed = await showConfirmation(
    'Clear all bindings for this action?',
    'Clear Action Bindings',
    'Clear',
    'Cancel'
  );

  if (!confirmed)
  {
    console.log('User cancelled action clearing');
    return;
  }

  try
  {
    // Find the action to see what default bindings it has
    const action = currentKeybindings.action_maps
      .find(am => am.name === actionMapName)
      ?.actions.find(a => a.name === actionName);

    if (!action)
    {
      console.error('Action not found');
      return;
    }

    // Get all the current bindings (including defaults) to determine which input types to clear
    const inputTypesToClear = new Set();
    if (action.bindings)
    {
      action.bindings.forEach(binding =>
      {
        if (binding.input && binding.input.trim())
        {
          // Determine input type from the binding
          if (binding.input.startsWith('js'))
          {
            inputTypesToClear.add('joystick');
          }
          else if (binding.input.startsWith('kb'))
          {
            inputTypesToClear.add('keyboard');
          }
          else if (binding.input.startsWith('mouse'))
          {
            inputTypesToClear.add('mouse');
          }
          else if (binding.input.startsWith('gp'))
          {
            inputTypesToClear.add('gamepad');
          }
        }
      });
    }

    console.log('Input types to clear:', Array.from(inputTypesToClear));

    // Clear each input type by providing the appropriate cleared binding format
    for (const inputType of inputTypesToClear)
    {
      let clearedInput = '';
      switch (inputType)
      {
        case 'joystick':
          clearedInput = 'js1_ '; // Cleared joystick binding
          break;
        case 'keyboard':
          clearedInput = 'kb1_ '; // Cleared keyboard binding
          break;
        case 'mouse':
          clearedInput = 'mouse1_ '; // Cleared mouse binding
          break;
        case 'gamepad':
          clearedInput = 'gp1_ '; // Cleared gamepad binding
          break;
      }

      if (clearedInput)
      {
        console.log(`Clearing ${inputType} with: "${clearedInput}"`);
        await invoke('update_binding', {
          actionMapName: actionMapName,
          actionName: actionName,
          newInput: clearedInput
        });
      }
    }

    // Mark as unsaved
    hasUnsavedChanges = true;
    updateUnsavedIndicator();

    await refreshBindings();
  } catch (error)
  {
    console.error('Error clearing action binding:', error);
    await showAlert(`Error clearing action binding: ${error}`, 'Error');
  }
};

// Global function to reset an action to default bindings (called from action-level Reset button)
window.resetActionBinding = async function (actionMapName, actionName)
{
  console.log('resetActionBinding called with:', { actionMapName, actionName });

  // Show custom confirmation dialog
  const confirmed = await showConfirmation(
    'Reset this action to default bindings?',
    'Reset to Default',
    'Reset',
    'Cancel'
  );

  if (!confirmed)
  {
    console.log('User cancelled action reset');
    return;
  }

  try
  {
    // Call backend to reset binding (remove customization)
    await invoke('reset_binding', {
      actionMapName: actionMapName,
      actionName: actionName
    });

    // Mark as unsaved
    hasUnsavedChanges = true;
    updateUnsavedIndicator();

    // Refresh to show default bindings
    await refreshBindings();
  } catch (error)
  {
    console.error('Error resetting action binding:', error);
    // Fallback to old method if reset_binding doesn't exist
    if (error.toString().includes('not found'))
    {
      console.log('Using fallback reset method');
      try
      {
        await invoke('update_binding', {
          actionMapName: actionMapName,
          actionName: actionName,
          newInput: ''
        });
        hasUnsavedChanges = true;
        updateUnsavedIndicator();
        await refreshBindings();
      } catch (fallbackError)
      {
        console.error('Error in fallback reset:', fallbackError);
        await showAlert(`Error resetting binding: ${fallbackError}`, 'Error');
      }
    } else
    {
      await showAlert(`Error resetting binding: ${error}`, 'Error');
    }
  }
};

// Function to remove a specific binding
window.removeBinding = async function (actionMapName, actionName, inputToClear)
{
  console.log('removeBinding called with:', { actionMapName, actionName, inputToClear });

  // Show custom confirmation dialog BEFORE doing anything
  const confirmed = await showConfirmation(
    'Clear this binding?',
    'Clear Binding',
    'Clear',
    'Cancel'
  );

  // If user cancelled, stop immediately
  if (!confirmed)
  {
    console.log('User cancelled binding removal');
    return false;
  }

  console.log('User confirmed, proceeding with removal');

  try
  {
    // Call backend to remove the specific input binding
    // This sets the binding to a cleared state (e.g., "js1_ " with trailing space)
    await invoke('clear_specific_binding', {
      actionMapName: actionMapName,
      actionName: actionName,
      inputToClear: inputToClear
    });

    // Mark as unsaved
    hasUnsavedChanges = true;
    updateUnsavedIndicator();

    // Refresh bindings
    await refreshBindings();

    return true;
  } catch (error)
  {
    console.error('Error removing binding:', error);
    await showAlert(`Error removing binding: ${error}`, 'Error');
    return false;
  }
};

function updateUnsavedIndicator()
{
  const indicator = document.getElementById('loaded-file-indicator');
  const fileNameEl = document.getElementById('loaded-file-name');

  if (indicator && fileNameEl)
  {
    if (hasUnsavedChanges)
    {
      indicator.style.borderColor = 'var(--accent-warning)';
      indicator.style.backgroundColor = 'rgba(206, 145, 120, 0.1)';
      if (!fileNameEl.textContent.includes('*'))
      {
        fileNameEl.textContent += ' *';
      }
    }
    else
    {
      indicator.style.removeProperty('border-color');
      indicator.style.removeProperty('background-color');
      fileNameEl.textContent = fileNameEl.textContent.replace(' *', '');
    }
  }
}

// ============================================================================
// JOYSTICK MAPPING FUNCTIONS
// ============================================================================

let detectedJoysticks = [];

// JOYSTICK MAPPING
// ============================================================================

let currentDetectingDevice = null; // 'js1', 'js2', or 'gp1'
let deviceDetectionSessionId = null;
let deviceMappings = {}; // Stores { js1: { detectedNum: 3, detectedPrefix: 'js', deviceName: 'VKB', deviceUuid: 'uuid-string' }, ... }
let deviceUuidMapping = {}; // Stores UUID-based mappings: { 'uuid-string': 'js1', ... }

async function openJoystickMappingModal()
{
  const modal = document.getElementById('joystick-mapping-modal');
  modal.style.display = 'flex';

  // Load existing mappings and update display
  loadDeviceMappings();
  updateDeviceInfoDisplays();
}

function closeJoystickMappingModal()
{
  const modal = document.getElementById('joystick-mapping-modal');
  modal.style.display = 'none';

  // Stop any active detection
  if (currentDetectingDevice !== null)
  {
    stopDeviceDetection();
  }
}

function loadDeviceMappings()
{
  const saved = localStorage.getItem('joystickMapping');
  if (saved)
  {
    try
    {
      deviceMappings = JSON.parse(saved);
    }
    catch (e)
    {
      console.error('Failed to parse saved joystick mapping:', e);
      deviceMappings = {};
    }
  }

  // Load UUID-based mappings
  const savedUuidMapping = localStorage.getItem('joystickUuidMapping');
  if (savedUuidMapping)
  {
    try
    {
      deviceUuidMapping = JSON.parse(savedUuidMapping);
      console.log('Loaded UUID-based device mappings:', deviceUuidMapping);

      // Try to auto-restore mappings based on UUID
      tryAutoRestoreMappings();
    }
    catch (e)
    {
      console.error('Failed to parse saved UUID mapping:', e);
      deviceUuidMapping = {};
    }
  }
}

// Attempt to automatically restore device mappings based on connected devices' UUIDs
async function tryAutoRestoreMappings()
{
  // This will be called when opening the mapping modal
  // We'll check if any currently connected devices match saved UUIDs
  // For now, just log that this feature is ready
  console.log('[UUID-MAPPING] Auto-restore ready, will attempt when devices detected');
}

async function detectDevice(targetDevice)
{
  // If already detecting this device, stop it
  if (currentDetectingDevice === targetDevice)
  {
    stopDeviceDetection();
    return;
  }

  // Stop any other detection first
  if (currentDetectingDevice !== null)
  {
    stopDeviceDetection();
  }

  currentDetectingDevice = targetDevice;
  const buttonId = `detect-${targetDevice}-btn`;
  const infoId = `${targetDevice}-info`;

  const button = document.getElementById(buttonId);
  const infoDiv = document.getElementById(infoId);

  // Update UI to detecting state
  if (button)
  {
    button.textContent = '⏹️ Stop Detecting';
    button.classList.add('detecting');
  }

  if (infoDiv)
  {
    infoDiv.classList.add('detecting');
    infoDiv.innerHTML = '<div style="color: #ffc107; font-weight: 500;">👂 Listening... Press any button on your device!</div>';
  }

  // Generate session ID
  deviceDetectionSessionId = 'device-detect-' + Date.now();
  const sessionId = deviceDetectionSessionId;

  try
  {
    console.log(`[DEVICE-DETECTION] Detecting ${targetDevice}, session:`, sessionId);

    const result = await invoke('wait_for_input_binding', {
      sessionId: sessionId,
      timeoutSecs: 15
    });

    // Check if this session is still active
    if (deviceDetectionSessionId !== sessionId)
    {
      console.log(`[DEVICE-DETECTION] Session ${sessionId} cancelled, ignoring result`);
      return;
    }

    if (result)
    {
      console.log(`[DEVICE-DETECTION] Detected input:`, result);

      // Extract js/gp number and device info
      const match = result.input_string.match(/^(js|gp)(\d+)_/);
      if (match)
      {
        const prefix = match[1];
        const detectedNum = parseInt(match[2]);

        // Get device name and UUID from result
        const deviceName = result.display_name || `Device ${detectedNum}`;
        const deviceUuid = result.device_uuid || null;

        console.log(`[UUID-MAPPING] Device detected - UUID: ${deviceUuid}, Name: ${deviceName}`);

        // Store the mapping
        deviceMappings[targetDevice] = {
          detectedNum: detectedNum,
          detectedPrefix: prefix,
          deviceName: deviceName,
          deviceUuid: deviceUuid
        };

        // If we have a UUID, also store the reverse mapping (UUID -> target device)
        if (deviceUuid)
        {
          deviceUuidMapping[deviceUuid] = targetDevice;
          console.log(`[UUID-MAPPING] Saved UUID mapping: ${deviceUuid} -> ${targetDevice}`);
        }

        console.log(`[DEVICE-DETECTION] Mapped ${targetDevice}: ${prefix}${detectedNum}`);

        // Update display
        if (infoDiv)
        {
          infoDiv.classList.remove('detecting');
          infoDiv.classList.add('configured');
          infoDiv.innerHTML = `
            <div class="device-name">${deviceName}</div>
            <div class="device-details">Detected as: ${prefix}${detectedNum}</div>
            <div class="device-mapping">Maps to: ${targetDevice}</div>
          `;
        }
      }
    }
    else
    {
      // Timeout
      if (infoDiv)
      {
        infoDiv.classList.remove('detecting');
        infoDiv.innerHTML = '<div style="color: #d9534f;">⏱️ Timeout - no input detected. Try again.</div>';

        setTimeout(() =>
        {
          updateDeviceInfoDisplays();
        }, 3000);
      }
    }
  }
  catch (error)
  {
    console.error('Error loading keybindings:', error);
    await showAlert(`Error loading keybindings: ${error}`, 'Error');
    {
      infoDiv.classList.remove('detecting');
      infoDiv.innerHTML = `<div style="color: #d9534f;">❌ Error: ${error.message || error}</div>`;

      setTimeout(() =>
      {
        updateDeviceInfoDisplays();
      }, 3000);
    }
  }
  finally
  {
    // Reset button
    if (button)
    {
      const btn = document.getElementById(buttonId);
      if (btn)
      {
        btn.textContent = `🎮 Detect ${targetDevice}`;
        btn.classList.remove('detecting');
      }
    }

    if (currentDetectingDevice === targetDevice)
    {
      currentDetectingDevice = null;
    }
    deviceDetectionSessionId = null;
  }
}

function stopDeviceDetection()
{
  if (currentDetectingDevice === null) return;

  console.log(`[DEVICE-DETECTION] Stopping detection for ${currentDetectingDevice}`);

  const buttonId = `detect-${currentDetectingDevice}-btn`;
  const button = document.getElementById(buttonId);

  if (button)
  {
    button.textContent = `🎮 Detect ${currentDetectingDevice}`;
    button.classList.remove('detecting');
  }

  updateDeviceInfoDisplays();

  currentDetectingDevice = null;
  deviceDetectionSessionId = null;
}

function resetDeviceMapping(device)
{
  console.log(`[DEVICE-MAPPING] Resetting mapping for ${device}`);

  // Remove from deviceMappings
  if (deviceMappings[device])
  {
    delete deviceMappings[device];
  }

  // Remove from deviceUuidMapping if it exists
  const uuidToRemove = Object.keys(deviceUuidMapping).find(uuid => deviceUuidMapping[uuid] === device);
  if (uuidToRemove)
  {
    delete deviceUuidMapping[uuidToRemove];
  }

  // Update display
  updateDeviceInfoDisplays();
}

function updateDeviceInfoDisplays()
{
  // Update all device info displays
  ['js1', 'js2', 'gp1'].forEach(device =>
  {
    const infoDiv = document.getElementById(`${device}-info`);
    if (!infoDiv) return;

    const mapping = deviceMappings[device];
    if (mapping && mapping.detectedNum)
    {
      infoDiv.classList.add('configured');
      infoDiv.classList.remove('detecting');
      infoDiv.innerHTML = `
        <div class="device-name">${mapping.deviceName}</div>
        <div class="device-details">Detected as: ${mapping.detectedPrefix}${mapping.detectedNum}</div>
        <div class="device-mapping">Maps to: ${device}</div>
      `;
    }
    else
    {
      infoDiv.classList.remove('configured', 'detecting');
      infoDiv.innerHTML = '<div class="not-configured">Not configured</div>';
    }
  });
}

function saveJoystickMapping()
{
  // Save the device mappings
  localStorage.setItem('joystickMapping', JSON.stringify(deviceMappings));

  // Save UUID-based mappings for persistent device recognition
  localStorage.setItem('joystickUuidMapping', JSON.stringify(deviceUuidMapping));

  console.log('Saved joystick mapping:', deviceMappings);
  console.log('Saved UUID mapping:', deviceUuidMapping);

  closeJoystickMappingModal();
}

// Function to apply joystick mapping to detected input
function applyJoystickMapping(detectedInput)
{
  const mappings = JSON.parse(localStorage.getItem('joystickMapping') || '{}');

  // Extract the prefix and number from the detected input (e.g., "js3_button1" -> js, 3)
  const match = detectedInput.match(/^(js|gp)(\d+)_/);
  if (!match)
  {
    return detectedInput; // No device number found, return as-is
  }

  const detectedPrefix = match[1];
  const detectedNum = parseInt(match[2]);

  // Find which target device (js1, js2, gp1) maps to this detected device
  for (const [targetDevice, mapping] of Object.entries(mappings))
  {
    if (mapping.detectedPrefix === detectedPrefix && mapping.detectedNum === detectedNum)
    {
      // Found a mapping - replace the device prefix and number
      const mappedInput = detectedInput.replace(/^(js|gp)\d+_/, `${targetDevice}_`);
      console.log(`Applied joystick mapping: ${detectedInput} -> ${mappedInput}`);
      return mappedInput;
    }
  }

  // No mapping found, return as-is
  console.log(`No mapping found for ${detectedInput}, using as-is`);
  return detectedInput;
}

// ============================================================================
// JOYSTICK TEST DETECTION
// ============================================================================

let testingJoystickNum = null;
let testTimeout = null;

async function startJoystickTest(detectedScNum, devicePrefix)
{
  if (testingJoystickNum !== null)
  {
    // Stop current test
    stopJoystickTest();
    return;
  }

  console.log(`Starting test for ${devicePrefix}${detectedScNum}`);
  testingJoystickNum = detectedScNum;

  const btn = document.querySelector(`.joystick-test-btn[data-detected-sc-num="${detectedScNum}"][data-device-prefix="${devicePrefix}"]`);
  const indicator = document.getElementById(`test-indicator-${devicePrefix}${detectedScNum}`);

  btn.textContent = 'Stop';
  btn.classList.add('active');
  indicator.textContent = '👂 Listening for input... Press any button!';

  // Start listening for input
  try
  {
    const sessionId = 'keybinding-test-' + Date.now();
    const result = await invoke('wait_for_input_binding', {
      sessionId: sessionId,
      timeoutSecs: 10
    });

    if (result && result.input_string.startsWith(`${devicePrefix}${detectedScNum}_`))
    {
      // Detected input from this device!
      renderDetectedInputMessage(indicator, `✅ Detected: ${result.display_name}`);
      indicator.classList.add('detected');

      // Reset after 2 seconds
      setTimeout(() =>
      {
        stopJoystickTest(devicePrefix, detectedScNum);
      }, 2000);
    } else if (result)
    {
      // Input detected but from wrong device
      const match = result.input_string.match(/^(gp|js)(\d+)_/);
      if (match)
      {
        const detectedPrefix = match[1];
        const detectedNum = match[2];
        indicator.textContent = `❌ Detected input from ${detectedPrefix}${detectedNum}, not ${devicePrefix}${detectedScNum}`;
      } else
      {
        indicator.textContent = `❌ Detected input from different device`;
      }
      setTimeout(() =>
      {
        stopJoystickTest(devicePrefix, detectedScNum);
      }, 2000);
    } else
    {
      // Timeout
      indicator.textContent = `⏱️ No input detected`;
      setTimeout(() =>
      {
        stopJoystickTest(devicePrefix, detectedScNum);
      }, 1500);
    }
  } catch (error)
  {
    console.error('Error during joystick test:', error);
    indicator.textContent = `❌ Error: ${error}`;
    setTimeout(() =>
    {
      stopJoystickTest(devicePrefix, detectedScNum);
    }, 2000);
  }
}

function stopJoystickTest(devicePrefix = null, detectedScNum = null)
{
  if (testingJoystickNum === null) return;

  if (!devicePrefix || !detectedScNum)
  {
    // If not provided, find from testingJoystickNum (for backwards compatibility)
    detectedScNum = testingJoystickNum;
    devicePrefix = 'js'; // Default
  }

  const btn = document.querySelector(`.joystick-test-btn[data-detected-sc-num="${detectedScNum}"][data-device-prefix="${devicePrefix}"]`);
  const indicator = document.getElementById(`test-indicator-${devicePrefix}${detectedScNum}`);

  if (btn)
  {
    btn.textContent = 'Test';
    btn.classList.remove('active');
  }
  if (indicator)
  {
    indicator.classList.remove('detected');
    indicator.textContent = 'Press a button on this device to identify it...';
  }

  testingJoystickNum = null;

  if (testTimeout)
  {
    clearTimeout(testTimeout);
    testTimeout = null;
  }
}

// ============================================================================
// CONFLICT MODAL FUNCTIONS
// ============================================================================

function showConflictModal(conflicts)
{
  const modal = document.getElementById('conflict-modal');
  const conflictList = document.getElementById('conflict-list');

  // Populate conflict list
  conflictList.innerHTML = conflicts.map(c =>
  {
    // Handle localization keys that start with @ - use the action name as fallback
    const actionLabel = (c.action_label && !c.action_label.startsWith('@'))
      ? c.action_label
      : formatDisplayName(c.action_name);

    const mapLabel = (c.action_map_label && !c.action_map_label.startsWith('@'))
      ? c.action_map_label
      : formatDisplayName(c.action_map_name);

    return `
      <div class="conflict-item">
        <div class="conflict-action-label">${actionLabel}</div>
        <div class="conflict-map-label">${mapLabel}</div>
      </div>
    `;
  }).join('');

  modal.style.display = 'flex';
}

// Helper function to format display names (converts snake_case to Title Case)
function formatDisplayName(name)
{
  if (!name) return '';

  // Remove common prefixes
  let cleaned = name
    .replace(/^v_/, '')
    .replace(/^pc_/, '')
    .replace(/^ui_/, '')
    .replace(/^spectate_/, '');

  // Split on underscores and capitalize each word
  return cleaned
    .split('_')
    .map(word => word.charAt(0).toUpperCase() + word.slice(1).toLowerCase())
    .join(' ');
}

// Helper function to show conflicts in the binding modal
function displayConflictsInModal(conflicts)
{
  const conflictDisplay = document.getElementById('binding-conflict-display');

  if (!conflicts || conflicts.length === 0)
  {
    conflictDisplay.style.display = 'none';
    conflictDisplay.innerHTML = '';
    return;
  }

  const conflictItems = conflicts.map(c =>
  {
    const actionLabel = (c.action_label && !c.action_label.startsWith('@'))
      ? c.action_label
      : formatDisplayName(c.action_name);

    const mapLabel = (c.action_map_label && !c.action_map_label.startsWith('@'))
      ? c.action_map_label
      : formatDisplayName(c.action_map_name);

    return `
      <div class="conflict-item-inline">
        <div class="conflict-action-label">${actionLabel}</div>
        <div class="conflict-map-label">${mapLabel}</div>
      </div>
    `;
  }).join('');

  conflictDisplay.innerHTML = `
    <div class="conflict-warning-header">
      <span class="conflict-icon">⚠️</span>
      <span>This input is already used by ${conflicts.length} action${conflicts.length > 1 ? 's' : ''}:</span>
    </div>
    <div class="conflict-list-inline">
      ${conflictItems}
    </div>
  `;
  conflictDisplay.style.display = 'block';
}

function closeConflictModal()
{
  const modal = document.getElementById('conflict-modal');
  modal.style.display = 'none';

  // Clear pending binding
  window.pendingBinding = null;
  setBindingSaveEnabled(false);

  // Update binding modal status
  document.getElementById('binding-modal-status').textContent = 'Binding cancelled';

  setTimeout(() =>
  {
    closeBindingModal();
  }, 1000);
}

async function confirmConflictBinding()
{
  const modal = document.getElementById('conflict-modal');
  modal.style.display = 'none';

  if (window.pendingBinding)
  {
    const { actionMapName, actionName, mappedInput, multiTap } = window.pendingBinding;

    // Get the selected activation mode
    const activationModeSelect = document.getElementById('activation-mode-select');
    const activationMode = activationModeSelect ? activationModeSelect.value : null;

    await applyBinding(actionMapName, actionName, mappedInput, multiTap, activationMode);
    window.pendingBinding = null;
    setBindingSaveEnabled(false);
  }
}

async function applyBinding(actionMapName, actionName, mappedInput, multiTap = null, activationMode = null)
{
  console.log('Calling update_binding...');
  // Update the binding in backend
  await invoke('update_binding', {
    actionMapName: actionMapName,
    actionName: actionName,
    newInput: mappedInput,
    multiTap: multiTap,
    activationMode: activationMode
  });
  console.log('update_binding completed');

  // Mark as unsaved
  hasUnsavedChanges = true;
  updateUnsavedIndicator();

  // Immediately refresh and save to localStorage
  console.log('Binding updated, refreshing data...');
  await refreshBindings();
  console.log('Bindings refreshed and saved to localStorage');

  // Close modal after a short delay
  setTimeout(() =>
  {
    closeBindingModal();
  }, 1000);
}

async function resetBinding()
{
  if (!currentBindingAction) return;

  try
  {
    const actionMapName = currentBindingAction.actionMapName;
    const actionName = currentBindingAction.actionName;

    // Call backend to reset binding (remove customization)
    // This will cause the action to use defaults from AllBinds again
    await invoke('reset_binding', {
      actionMapName: actionMapName,
      actionName: actionName
    });

    // Mark as unsaved
    hasUnsavedChanges = true;
    updateUnsavedIndicator();

    // Refresh to show default bindings
    await refreshBindings();
    closeBindingModal();
  } catch (error)
  {
    console.error('Error resetting binding:', error);
    // Fallback to old method if reset_binding doesn't exist
    if (error.toString().includes('not found'))
    {
      console.log('Using fallback reset method');
      await fallbackResetBinding();
    } else
    {
      await showAlert(`Error resetting binding: ${error}`, 'Error');
    }
  }
}

// Fallback method for reset if backend doesn't have reset_binding command yet
async function fallbackResetBinding()
{
  if (!currentBindingAction) return;

  try
  {
    const actionMapName = currentBindingAction.actionMapName;
    const actionName = currentBindingAction.actionName;

    // Clear the binding by setting empty input
    await invoke('update_binding', {
      actionMapName: actionMapName,
      actionName: actionName,
      newInput: ''
    });

    hasUnsavedChanges = true;
    updateUnsavedIndicator();
    await refreshBindings();
    closeBindingModal();
  } catch (error)
  {
    console.error('Error in fallback reset:', error);
    await showAlert(`Error resetting binding: ${error}`, 'Error');
  }
}

// =====================
// INITIALIZE VERSION ON STARTUP
// =====================
(async () =>
{
  try
  {
    const version = await invoke('get_app_version');
    const versionElement = document.getElementById('app-version');
    if (versionElement)
    {

      // If the version starts with "0.", it's a beta build - append " (Beta)"
      if (version.startsWith('0.'))
      {
        versionElement.textContent = `v${version} (Beta)`;
      } else

        versionElement.textContent = `v${version}`;
    }
  } catch (error)
  {
    console.error('Failed to load app version:', error);
  }
})();

// =====================
// INITIALIZE LOG FILE PATH
// =====================
(async () =>
{
  try
  {
    const logPath = await invoke('get_log_file_path');
    const logPathElement = document.getElementById('debug-log-path');

    if (logPathElement)
    {
      logPathElement.title = `Log file: ${logPath}\nClick to copy path`;
      logPathElement.addEventListener('click', async () =>
      {
        try
        {
          await navigator.clipboard.writeText(logPath);
          const originalText = logPathElement.textContent;
          logPathElement.textContent = '✓ Copied!';
          setTimeout(() =>
          {
            logPathElement.textContent = originalText;
          }, 2000);
        } catch (e)
        {
          console.error('Failed to copy to clipboard:', e);
          await showAlert(`Log file path:\n${logPath}`, 'Log File Path');
        }
      });

      await logInfo(`Application started - version ${await invoke('get_app_version')}`);
    }
  } catch (error)
  {
    console.error('Failed to get log file path:', error);
  }
})();

// =====================
// FONT SIZE SCALING (Ctrl +/- / Cmd +/-)
// =====================

const FONT_SIZE_MIN = 10; // pixels
const FONT_SIZE_MAX = 24; // pixels
const FONT_SIZE_DEFAULT = 14; // pixels
const FONT_SIZE_STEP = 1; // pixels per increment

function initializeFontSizeScaling()
{
  // Load saved font size or use default
  const savedFontSize = localStorage.getItem('appFontSize');
  if (savedFontSize)
  {
    setFontSize(parseInt(savedFontSize));
  } else
  {
    setFontSize(FONT_SIZE_DEFAULT);
  }

  // Add keyboard listener for font size controls
  document.addEventListener('keydown', (e) =>
  {
    // Check for Ctrl (Windows/Linux) or Cmd (Mac)
    const isModifierPressed = e.ctrlKey || e.metaKey;

    if (isModifierPressed && !e.altKey && !e.shiftKey)
    {
      if (e.key === '+' || e.key === '=')
      {
        e.preventDefault();
        increaseFontSize();
      } else if (e.key === '-' || e.key === '_')
      {
        e.preventDefault();
        decreaseFontSize();
      } else if (e.key === '0')
      {
        e.preventDefault();
        resetFontSize();
      }
    }
  });
}

function setFontSize(size)
{
  // Clamp size between min and max
  size = Math.max(FONT_SIZE_MIN, Math.min(FONT_SIZE_MAX, size));

  // Apply to root element
  document.documentElement.style.fontSize = `${size}px`;

  // Save to localStorage
  localStorage.setItem('appFontSize', size);

  console.log(`Font size set to ${size}px`);
}

function increaseFontSize()
{
  const current = parseInt(localStorage.getItem('appFontSize') || FONT_SIZE_DEFAULT);
  setFontSize(current + FONT_SIZE_STEP);
}

function decreaseFontSize()
{
  const current = parseInt(localStorage.getItem('appFontSize') || FONT_SIZE_DEFAULT);
  setFontSize(current - FONT_SIZE_STEP);
}

function resetFontSize()
{
  setFontSize(FONT_SIZE_DEFAULT);
}

// Make font size controls globally available
window.increaseFontSize = increaseFontSize;
window.decreaseFontSize = decreaseFontSize;
window.resetFontSize = resetFontSize;

// =====================
// ACTION BINDINGS MANAGER MODAL
// =====================

let currentActionBindingsData = null;

async function openActionBindingsModal(actionMapName, actionName, actionDisplayName)
{
  currentActionBindingsData = {
    actionMapName,
    actionName,
    actionDisplayName
  };

  // Get the action data
  const actionMap = currentKeybindings.action_maps.find(am => am.name === actionMapName);
  if (!actionMap) return;

  const action = actionMap.actions.find(a => a.name === actionName);
  if (!action) return;

  // Show modal
  const modal = document.getElementById('action-bindings-modal');
  const title = document.getElementById('action-bindings-title');
  const listContainer = document.getElementById('action-bindings-list');

  title.textContent = `Manage Bindings: ${actionDisplayName}`;

  // Render bindings list
  let html = '';

  // Check if this action only has the special "unbound" placeholder
  const hasOnlyUnboundPlaceholder = action.bindings && action.bindings.length === 1 &&
    action.bindings[0].input.match(/^(js\d+|kb\d+|mouse\d+|gp\d+)_\s*$/) &&
    action.bindings[0].is_default &&
    action.bindings[0].display_name === 'Unbound';

  if (!action.bindings || action.bindings.length === 0 || hasOnlyUnboundPlaceholder)
  {
    html = '<div class="empty-state" style="padding: 2rem; text-align: center; color: var(--text-secondary);">No bindings for this action. Click "Add New Binding" to create one.</div>';
  }
  else
  {
    action.bindings.forEach((binding, index) =>
    {
      const trimmedInput = binding.input.trim();

      // Skip truly empty bindings
      if (!trimmedInput || trimmedInput === '') return;

      // Check if this is a cleared binding (e.g., "js1_ ", "kb1_ ", etc.)
      const isClearedBinding = trimmedInput.match(/^(js\d+|kb\d+|mouse\d+|gp\d+)_\s*$/);

      let icon = '○';
      if (binding.input_type === 'Keyboard') icon = '⌨️';
      else if (binding.input_type === 'Mouse') icon = '🖱️';
      else if (binding.input_type === 'Joystick') icon = '🕹️';
      else if (binding.input_type === 'Gamepad') icon = '🎮';

      const defaultBadge = binding.is_default ? '<span class="action-binding-default-badge">Default</span>' : '';
      const customBadge = !binding.is_default ? '<span class="action-binding-custom-badge">Custom</span>' : '';
      const clearedBadge = isClearedBinding ? '<span class="action-binding-cleared-badge">Cleared</span>' : '';
      const activationValue = binding.activation_mode || '';

      // Disable remove button for unbound bindings
      const isUnbound = binding.input_type === 'Unknown';
      const removeButtonDisabled = isUnbound ? 'disabled' : '';

      html += `
        <div class="action-binding-item ${binding.is_default ? 'is-default' : ''} ${isClearedBinding ? 'is-cleared' : ''}" data-binding-index="${index}">
          <div class="action-binding-icon">${icon}</div>
          <div class="action-binding-device">
            ${binding.input_type}${defaultBadge}${customBadge}${clearedBadge}
          </div>
          <div class="action-binding-input ${isClearedBinding ? 'cleared-text' : ''}">${isClearedBinding && binding.original_default ? `<span style="text-decoration: line-through;">${binding.original_default}</span>` : binding.display_name}</div>
          <div class="action-binding-activation">
            <select class="binding-activation-select" data-binding-index="${index}" ${isClearedBinding ? 'disabled' : ''}>
              <option value="">Default (Press)</option>
              <option value="press" ${activationValue === 'press' ? 'selected' : ''}>Press</option>
              <option value="press_quicker" ${activationValue === 'press_quicker' ? 'selected' : ''}>Press (Quicker)</option>
              <option value="delayed_press" ${activationValue === 'delayed_press' ? 'selected' : ''}>Delayed Press</option>
              <option value="delayed_press_medium" ${activationValue === 'delayed_press_medium' ? 'selected' : ''}>Delayed Press (Medium)</option>
              <option value="delayed_press_long" ${activationValue === 'delayed_press_long' ? 'selected' : ''}>Delayed Press (Long)</option>
              <option value="tap" ${activationValue === 'tap' ? 'selected' : ''}>Tap</option>
              <option value="tap_quicker" ${activationValue === 'tap_quicker' ? 'selected' : ''}>Tap (Quicker)</option>
              <option value="double_tap" ${activationValue === 'double_tap' ? 'selected' : ''}>Double Tap</option>
              <option value="double_tap_nonblocking" ${activationValue === 'double_tap_nonblocking' ? 'selected' : ''}>Double Tap (Non-blocking)</option>
              <option value="hold" ${activationValue === 'hold' ? 'selected' : ''}>Hold</option>
              <option value="delayed_hold" ${activationValue === 'delayed_hold' ? 'selected' : ''}>Delayed Hold</option>
              <option value="delayed_hold_long" ${activationValue === 'delayed_hold_long' ? 'selected' : ''}>Delayed Hold (Long)</option>
              <option value="hold_no_retrigger" ${activationValue === 'hold_no_retrigger' ? 'selected' : ''}>Hold (No Retrigger)</option>
              <option value="hold_toggle" ${activationValue === 'hold_toggle' ? 'selected' : ''}>Hold Toggle</option>
              <option value="smart_toggle" ${activationValue === 'smart_toggle' ? 'selected' : ''}>Smart Toggle</option>
              <option value="all" ${activationValue === 'all' ? 'selected' : ''}>All</option>
            </select>
          </div>
          <div class="action-binding-remove">
            <button onclick="removeBindingFromModal(${index})" ${removeButtonDisabled}>×</button>
          </div>
        </div>
      `;
    });
  }

  listContainer.innerHTML = html;
  modal.style.display = 'flex';

  // Setup event listeners for modal buttons
  document.getElementById('action-bindings-cancel-btn').onclick = closeActionBindingsModal;
  document.getElementById('action-bindings-save-btn').onclick = saveActionBindingsChanges;
  document.getElementById('action-bindings-add-btn').onclick = addNewBindingFromModal;
}

function closeActionBindingsModal()
{
  document.getElementById('action-bindings-modal').style.display = 'none';
  currentActionBindingsData = null;
}

async function saveActionBindingsChanges()
{
  if (!currentActionBindingsData) return;

  const { actionMapName, actionName } = currentActionBindingsData;

  // Get all activation mode selects
  const selects = document.querySelectorAll('.binding-activation-select');

  // Get the action data
  const actionMap = currentKeybindings.action_maps.find(am => am.name === actionMapName);
  if (!actionMap) return;

  const action = actionMap.actions.find(a => a.name === actionName);
  if (!action) return;

  // Update each binding's activation mode
  const updatePromises = [];

  selects.forEach(select =>
  {
    const index = parseInt(select.dataset.bindingIndex);
    const newActivationMode = select.value || null;
    const binding = action.bindings[index];
    const currentActivationMode = binding.activation_mode || null;

    if (binding && currentActivationMode !== newActivationMode)
    {
      console.log(`Updating activation mode for binding ${index}: ${currentActivationMode} -> ${newActivationMode}`);

      // If this is a default binding, we're creating a custom binding with the same input
      if (binding.is_default)
      {
        console.log(`Creating custom binding from default: ${binding.input} with activation mode: ${newActivationMode}`);
      }

      // Update via backend
      const promise = invoke('update_binding', {
        actionMapName,
        actionName,
        newInput: binding.input,
        multiTap: binding.multi_tap,
        activationMode: newActivationMode
      }).catch(err =>
      {
        console.error('Failed to update binding:', err);
      });

      updatePromises.push(promise);
    }
  });

  if (updatePromises.length > 0)
  {
    // Wait for all updates to complete
    await Promise.all(updatePromises);

    // Mark as unsaved
    hasUnsavedChanges = true;
    updateUnsavedIndicator();

    // Refresh bindings
    await refreshBindings();
  }

  closeActionBindingsModal();
}

async function removeBindingFromModal(index)
{
  if (!currentActionBindingsData) return;

  const { actionMapName, actionName, actionDisplayName } = currentActionBindingsData;

  // Get the action data
  const actionMap = currentKeybindings.action_maps.find(am => am.name === actionMapName);
  if (!actionMap) return;

  const action = actionMap.actions.find(a => a.name === actionName);
  if (!action || !action.bindings[index]) return;

  const binding = action.bindings[index];
  if (!binding || !binding.input) return;

  const removalSucceeded = await window.removeBinding(actionMapName, actionName, binding.input);
  if (!removalSucceeded) return;

  const modal = document.getElementById('action-bindings-modal');
  if (modal && modal.style.display !== 'none')
  {
    openActionBindingsModal(actionMapName, actionName, actionDisplayName || action.display_name || action.name);
  }
}

function addNewBindingFromModal()
{
  if (!currentActionBindingsData) return;

  const { actionMapName, actionName, actionDisplayName } = currentActionBindingsData;

  // Close this modal and open the binding detection modal
  closeActionBindingsModal();
  startBinding(actionMapName, actionName, actionDisplayName);
}

// Make it globally available
window.openActionBindingsModal = openActionBindingsModal;
window.removeBindingFromModal = removeBindingFromModal;

// ============================================================================
// CLEAR SC BINDS FUNCTIONS
// ============================================================================

function openClearSCBindsModal()
{
  const modal = document.getElementById('clear-sc-binds-modal');
  modal.style.display = 'flex';
}

function closeClearSCBindsModal()
{
  const modal = document.getElementById('clear-sc-binds-modal');
  modal.style.display = 'none';
}

function closeClearSCBindsSuccessModal()
{
  const modal = document.getElementById('clear-sc-binds-success-modal');
  modal.style.display = 'none';
}

async function generateUnbindProfile()
{
  const statusDiv = document.getElementById('clear-binds-status');

  try
  {
    // Get selected devices
    const devices = {
      keyboard: document.getElementById('unbind-keyboard').checked,
      mouse: document.getElementById('unbind-mouse').checked,
      gamepad: document.getElementById('unbind-gamepad').checked,
      joystick1: document.getElementById('unbind-joystick1').checked,
      joystick2: document.getElementById('unbind-joystick2').checked,
    };

    // Check if at least one device is selected
    if (!Object.values(devices).some(v => v))
    {
      statusDiv.style.display = 'block';
      statusDiv.style.color = 'var(--accent-primary)';
      statusDiv.textContent = '⚠️ Please select at least one device to unbind.';
      return;
    }

    // Get the SC installation path from localStorage
    const scInstallPath = localStorage.getItem('scInstallDirectory');
    if (!scInstallPath)
    {
      statusDiv.style.display = 'block';
      statusDiv.style.color = 'var(--accent-warning)';
      statusDiv.textContent = '⚠️ No SC installation directory configured. Configure it in Auto Save Settings first.';
      return;
    }

    statusDiv.style.display = 'block';
    statusDiv.style.color = 'var(--text-secondary)';
    statusDiv.textContent = '⏳ Generating unbind profile...';

    console.log('Generating unbind profile with base path:', scInstallPath);

    // Call backend to generate the unbind profile
    const result = await invoke('generate_unbind_profile', {
      devices,
      basePath: scInstallPath
    });

    console.log('Unbind profile generation result:', result);

    // Close the main modal
    closeClearSCBindsModal();

    // Show success modal with results
    const successModal = document.getElementById('clear-sc-binds-success-modal');
    const locationsDiv = document.getElementById('unbind-save-locations');

    if (result.saved_locations && result.saved_locations.length > 0)
    {
      let html = '<p><strong>📁 Saved to:</strong></p><ul style="margin: 0.5rem 0 0 1.5rem; padding: 0;">';
      result.saved_locations.forEach(loc =>
      {
        html += `<li><code>${loc}</code></li>`;
      });
      html += '</ul>';
      locationsDiv.innerHTML = html;
    }
    else
    {
      locationsDiv.innerHTML = '<p class="info-text">⚠️ No SC installation directories found. File created in current directory.</p>';
    }

    successModal.style.display = 'flex';

  } catch (error)
  {
    console.error('Error generating unbind profile:', error);
    statusDiv.style.display = 'block';
    statusDiv.style.color = 'var(--accent-primary)';
    statusDiv.textContent = `❌ Error: ${error}`;
  }
}

async function copyUnbindCommand()
{
  const command = 'pp_RebindKeys UNBIND_ALL';

  try
  {
    await navigator.clipboard.writeText(command);

    // Visual feedback
    const btn = document.getElementById('copy-unbind-command-btn');
    const originalText = btn.textContent;
    btn.textContent = '✅ Copied!';
    btn.disabled = true;

    setTimeout(() =>
    {
      btn.textContent = originalText;
      btn.disabled = false;
    }, 2000);
  } catch (error)
  {
    console.error('Failed to copy command:', error);
    await showAlert('Failed to copy command to clipboard', 'Error');
  }
}

async function removeUnbindFiles()
{
  const confirmed = await showConfirmation(
    'Are you sure you want to remove the UNBIND_ALL.xml files from all SC installation directories?',
    'Remove Unbind Files'
  );

  if (!confirmed) return;

  try
  {
    const result = await invoke('remove_unbind_profile');

    if (result.removed_count > 0)
    {
      await showAlert(
        `Successfully removed ${result.removed_count} unbind profile file(s).`,
        'Files Removed'
      );
      closeClearSCBindsSuccessModal();
    }
    else
    {
      await showAlert('No unbind profile files found to remove.', 'Info');
    }
  } catch (error)
  {
    console.error('Error removing unbind files:', error);
    await showAlert(`Error removing files: ${error}`, 'Error');
  }
}

// Make functions globally available
window.closeClearSCBindsModal = closeClearSCBindsModal;
window.closeClearSCBindsSuccessModal = closeClearSCBindsSuccessModal;
