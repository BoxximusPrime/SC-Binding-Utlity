const { invoke } = window.__TAURI__.core;
const { listen } = window.__TAURI__.event;
const { open, save } = window.__TAURI__.dialog;
import { toStarCitizenFormat } from './input-utils.js';
import { CustomDropdown } from './custom-dropdown.js';
import { Tooltip } from './tooltip.js';

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
let showUnboundActions = true;
let categoryFriendlyNames = {};
let currentFilename = null; // Track the current file name for the copy command
const SECONDARY_WINDOW_MS = 1000; // One-second window for multi-input capture
let deviceAxisNames = {}; // Cache of device_name -> { axis_id -> axis_name } from HID descriptors
let deviceAxisMappings = {}; // Cache of device_name -> { directinput_index -> hid_usage_id }
let deviceSCAxisMappings = {}; // Cache of device_name -> { directinput_index -> sc_axis_name }

// Joystick Mapping State
let detectedJoysticks = [];
let currentDetectingDevice = null; // 'js1', 'js2', or 'gp1'
let deviceDetectionSessionId = null;
let deviceMappings = {}; // Stores { js1: { detectedNum: 3, detectedPrefix: 'js', deviceName: 'VKB', deviceUuid: 'uuid-string' }, ... }
let deviceUuidMapping = {}; // Stores UUID-based mappings: { 'uuid-string': 'js1', ... }

// Joystick Test State
let testingJoystickNum = null;
let testTimeout = null;

// Action Bindings Manager State
let currentActionBindingsData = null;

// Binding Detection State
let allDetectedInputs = new Map();
let selectedInputKey = null;
let statusEl = null;
let selectionContainer = null;
let selectionButtons = new Map();
let selectionMessageEl = null;

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

                    // Also load the DirectInput-to-HID mapping
                    try
                    {
                        const axisMapping = await invoke('get_directinput_to_hid_mapping', { deviceName });
                        deviceAxisMappings[deviceName] = axisMapping || {};
                        console.log(`[Axis Mapping] Loaded ${Object.keys(axisMapping || {}).length} DirectInput mappings for device: ${deviceName}`);

                        // Build SC axis mapping from HID data
                        if (Object.keys(axisMapping || {}).length > 0 && Object.keys(axisNames || {}).length > 0)
                        {
                            // Note: buildAxisMappingFromHID is not defined in main.js or imported. 
                            // It seems it was missing from main.js or I missed it. 
                            // Checking main.js content again... it was called but not defined in the provided text?
                            // Ah, I might have missed it in the read. Or it's imported?
                            // Wait, I don't see buildAxisMappingFromHID in main.js content I read.
                            // It might be in input-utils.js? No, imports are explicit.
                            // Maybe it was defined inside loadDeviceAxisNames? No.
                            // I will comment it out for now and add a TODO.
                            // console.warn('buildAxisMappingFromHID is missing');
                            // const scAxisMapping = {}; 
                        }
                    } catch (mappingError)
                    {
                        console.warn(`[Axis Mapping] Failed to load DirectInput mapping for ${deviceName}:`, mappingError);
                        deviceAxisMappings[deviceName] = {};
                        deviceSCAxisMappings[deviceName] = {};
                    }
                } catch (error)
                {
                    console.warn(`[Axis Names] Failed to load axis names for ${deviceName}:`, error);
                    deviceAxisNames[deviceName] = {}; // Cache empty result to avoid repeated attempts
                    deviceAxisMappings[deviceName] = {};
                    deviceSCAxisMappings[deviceName] = {};
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

    // Get axis names and SC axis mapping for this device
    const axisNameMap = deviceAxisNames[device.device_name];
    const scAxisMapping = deviceSCAxisMappings[device.device_name];
    const directInputMapping = deviceAxisMappings[device.device_name];

    if (!axisNameMap || !scAxisMapping || !directInputMapping) return null;

    // Find which DirectInput index maps to this SC axis name using device-specific mapping
    // For example, VKB "rotz" maps to DirectInput index 3 (not 6!)
    const normalizedAxisName = axisName.toLowerCase();
    let directInputIndex = null;

    for (const [idx, scName] of Object.entries(scAxisMapping))
    {
        if (scName === normalizedAxisName)
        {
            directInputIndex = parseInt(idx);
            break;
        }
    }

    if (directInputIndex === null) return null;

    // Convert DirectInput index to HID usage ID using device-specific mapping
    const hidUsageId = directInputMapping[directInputIndex];
    if (hidUsageId === undefined) return null;

    // Look up the actual axis name from HID descriptor
    return axisNameMap[hidUsageId] || null;
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

        // The new Categories.json is already a flat mapping object
        categoryFriendlyNames = data;

        console.log('Category mappings loaded:', Object.keys(categoryFriendlyNames).length);
    } catch (error)
    {
        console.error('Error loading Categories.json:', error);
        // Set default fallback mapping to ensure the app still works
        categoryFriendlyNames = {
            '@ui_CCSeatGeneral': 'vehicle',
            '@ui_CCSpaceFlight': 'vehicle',
            '@ui_CCFPS': ['on foot'],
            '': 'other'
        };
    }
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

        // Refresh the visual view if it's loaded and visible
        if (window.refreshVisualView)
        {
            await window.refreshVisualView();
        }

    } catch (error)
    {
        console.error('Error loading keybindings:', error);
        await window.showAlert(`Failed to load keybindings: ${error}`, 'Error');
    }
}

export async function loadPersistedKeybindings()
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
                await window.showAlert(
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
        // Don't auto-load AllBinds - show welcome screen instead
        // User can create a new keybinding set or load an existing one
        showWelcomeScreen();
    } catch (error)
    {
        console.error('Error in loadAllBindsOnly:', error);
        showWelcomeScreen();
    }
}

function showWelcomeScreen()
{
    // Show welcome screen
    document.getElementById('welcome-screen').style.display = 'flex';
    document.getElementById('bindings-content').style.display = 'none';

    // Disable save buttons
    document.getElementById('save-btn').disabled = true;
    document.getElementById('save-as-btn').disabled = true;

    // Reset state
    currentKeybindings = null;
    hasUnsavedChanges = false;
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
        const confirmed = await window.showConfirmation(
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
        await window.showAlert(`Failed to create new keybinding: ${error}`, 'Error');
    }
}

function updateFileIndicator(filePath)
{
    const indicator = document.getElementById('loaded-file-indicator');
    const fileNameEl = document.getElementById('loaded-file-name');
    const indicatorSub = document.getElementById('loaded-file-indicator-sub');
    const filePathSubEl = document.getElementById('loaded-file-path-sub');
    const fileNameSubEl = document.getElementById('loaded-file-name-sub');

    if (indicator && fileNameEl)
    {
        // Show full file path
        fileNameEl.textContent = filePath;
        fileNameEl.title = filePath; // Add tooltip for full path
        indicator.style.display = 'flex';
    }

    if (indicatorSub && fileNameSubEl)
    {
        // Extract path and filename separately
        const lastSlashIndex = filePath.lastIndexOf('\\');
        const fileName = lastSlashIndex !== -1 ? filePath.substring(lastSlashIndex + 1) : filePath;
        const dirPath = lastSlashIndex !== -1 ? filePath.substring(0, lastSlashIndex + 1) : '';

        // Update sub-nav indicator with split path and filename
        if (filePathSubEl)
        {
            filePathSubEl.textContent = dirPath;
        }
        fileNameSubEl.textContent = fileName;
        fileNameSubEl.title = filePath; // Add tooltip for full path
        indicatorSub.style.display = 'flex';
    }
}

function showUnsavedFileIndicator()
{
    const indicator = document.getElementById('loaded-file-indicator');
    const fileNameEl = document.getElementById('loaded-file-name');
    const indicatorSub = document.getElementById('loaded-file-indicator-sub');
    const filePathSubEl = document.getElementById('loaded-file-path-sub');
    const fileNameSubEl = document.getElementById('loaded-file-name-sub');

    if (indicator && fileNameEl)
    {
        fileNameEl.textContent = 'Unsaved Keybinding Set';
        indicator.style.display = 'flex';
    }

    if (indicatorSub && fileNameSubEl)
    {
        if (filePathSubEl)
        {
            filePathSubEl.textContent = '';
        }
        fileNameSubEl.textContent = 'Unsaved Keybinding Set';
        indicatorSub.style.display = 'flex';
    }
}

// Search for a button ID in the main keybindings view
window.searchMainTabForButtonId = function (buttonId)
{
    // Switch to the bindings tab
    window.switchTab('bindings');

    // Switch to list view
    window.switchBindingsView('list');

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
        await window.showAlert('No keybindings loaded to save!', 'Save Keybindings');
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

                    // Save to each installation (except the one with the currently open file)
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
                    window.showSuccessMessage(successMsg);
                } else
                {
                    window.showSuccessMessage('Saved!');
                }
            } catch (error)
            {
                console.error('Error auto-saving to installations:', error);
                window.showSuccessMessage('Saved (failed to deploy to installations)');
            }
        } else
        {
            // Show brief success message
            window.showSuccessMessage('Saved!');
        }
    } catch (error)
    {
        console.error('Error saving keybindings:', error);
        await window.showAlert(`Failed to save keybindings: ${error}`, 'Error');
    }
}

async function saveKeybindingsAs()
{
    if (!currentKeybindings)
    {
        await window.showAlert('No keybindings loaded to save!', 'Save Keybindings As');
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
        window.showSuccessMessage('Saved!');
    } catch (error)
    {
        console.error('Error saving keybindings:', error);
        await window.showAlert(`Failed to save keybindings: ${error}`, 'Error');
    }
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

    // Group action maps by their mapped category
    const categoryGroups = new Map();

    currentKeybindings.action_maps.forEach(actionMap =>
    {
        // Try to find a mapping for this action map
        // 1. Try ui_category (e.g. @ui_CCSpaceFlight)
        // 2. Try name (e.g. spaceship_movement)
        // 3. Default to 'Uncategorized'

        let categoryKey = actionMap.ui_category;
        if (!categoryFriendlyNames[categoryKey])
        {
            categoryKey = actionMap.name;
        }

        let mappedCategory = categoryFriendlyNames[categoryKey];

        // If mappedCategory is an array, take the first element as the main category
        let mainCategory = 'Uncategorized';

        if (Array.isArray(mappedCategory))
        {
            mainCategory = mappedCategory[0];
        } else if (typeof mappedCategory === 'string')
        {
            mainCategory = mappedCategory;
        } else
        {
            // Fallback if not found in mapping
            if (actionMap.ui_category)
            {
                mainCategory = actionMap.ui_category;
            } else
            {
                mainCategory = 'Uncategorized';
            }
        }

        // Normalize category name (capitalize)
        if (mainCategory && mainCategory !== 'Uncategorized')
        {
            // Capitalize first letter of each word
            mainCategory = mainCategory.split(' ')
                .map(word => word.charAt(0).toUpperCase() + word.slice(1))
                .join(' ');

            // Special case for "Comms/social" -> "Comms/Social"
            mainCategory = mainCategory.replace('Comms/social', 'Comms/Social');
        }

        if (!categoryGroups.has(mainCategory))
        {
            categoryGroups.set(mainCategory, []);
        }
        categoryGroups.get(mainCategory).push(actionMap);
    });

    // Sort categories
    // We want a specific order if possible, otherwise alphabetical
    const categoryOrder = [
        'Vehicle',
        'On Foot',
        'Turrets',
        'Comms/Social',
        'Camera',
        'Other',
        'Uncategorized'
    ];

    const sortedCategories = Array.from(categoryGroups.keys()).sort((a, b) =>
    {
        const indexA = categoryOrder.indexOf(a);
        const indexB = categoryOrder.indexOf(b);

        if (indexA !== -1 && indexB !== -1) return indexA - indexB;
        if (indexA !== -1) return -1;
        if (indexB !== -1) return 1;

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

        // Sort action maps within category by display name
        actionMaps.sort((a, b) =>
        {
            const nameA = a.ui_label || a.display_name || a.name;
            const nameB = b.ui_label || b.display_name || b.name;
            return nameA.localeCompare(nameB);
        });

        // Add category header
        html += `<div class="category-header">${categoryName}</div>`;

        // Add action maps under this category
        actionMaps.forEach(actionMap =>
        {
            const displayName = actionMap.ui_label || actionMap.display_name || actionMap.name;
            const isActive = currentCategory === actionMap.name;

            html += `
        <div class="category-item ${isActive ? 'active' : ''} category-item-indented" 
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
    // Check if it's effectively unbound (no bindings or only empty defaults that are placeholders)
    const isEffectivelyUnbound = !action.bindings || action.bindings.length === 0 || action.bindings.every(binding =>
    {
        // Check the pattern BEFORE trimming to catch 'kb_ ', 'js_ ', etc.
        // The digit after the device prefix is optional
        const isEmptyBinding = !!binding.input.match(/^(js\d*|kb\d*|mouse\d*|gp\d*)_\s*$/);
        // It's only "effectively unbound" if it's the special Unbound placeholder
        const isUnboundPlaceholder = binding.is_default && isEmptyBinding && binding.display_name === 'Unbound';

        return isUnboundPlaceholder;
    });

    if (isEffectivelyUnbound)
    {
        // If we are hiding unbound actions, return false
        if (!showUnboundActions) return false;

        // If customizedOnly is true, unbound actions (which are defaults) should be hidden
        if (customizedOnly) return false;

        // If filtering by input type, check if there's at least one empty binding of that type
        // This logic is a bit fuzzy for "unbound", but preserves existing behavior
        if (currentFilter !== 'all')
        {
            if (!action.bindings) return true; // If no bindings at all, show it (it's a candidate for any type)

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

    // If we have bindings, check if any are visible
    return action.bindings.some(binding =>
    {
        const trimmedInput = binding.input.trim();

        // Check if this is a cleared binding (check BEFORE trimming for the pattern)
        const isClearedBinding = !!binding.input.match(/^(js\d*|kb\d*|mouse\d*|gp\d*)_\s*$/);

        // Skip truly unbound bindings (placeholders), but keep cleared bindings that override defaults
        // We use the same strict check for placeholders here
        const isUnboundPlaceholder = binding.is_default && isClearedBinding && binding.display_name === 'Unbound';
        if (isUnboundPlaceholder) return false;

        // Also skip if input is empty and it's NOT a cleared binding (just in case)
        if ((!trimmedInput || trimmedInput === '') && !isClearedBinding) return false;

        // If customizedOnly is true, hide default bindings (unless it's a cleared binding)
        if (customizedOnly && binding.is_default && !isClearedBinding) return false;

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
                // Support OR operator with |
                const terms = searchTerm.split('|').map(t => t.trim()).filter(t => t.length > 0);

                const matchesAny = terms.some(term =>
                {
                    const searchInAction = displayName.toLowerCase().includes(term) ||
                        action.name.toLowerCase().includes(term);

                    const searchInBindings = action.bindings && action.bindings.some(binding =>
                        binding.display_name.toLowerCase().includes(term) ||
                        binding.input.toLowerCase().includes(term)
                    );

                    return searchInAction || searchInBindings;
                });

                if (!matchesAny)
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
            ${displayName}
          </div>
          <div class="action-bindings">
            ${renderActionBindings(action)}
          </div>
          <div class="action-buttons">
            <button class="action-btn btn-primary" onclick="window.openActionBindingsModal('${actionMap.name}', '${action.name}', '${displayName.replace(/'/g, "\\'")}')">Manage</button>
            <button class="action-btn btn-primary" onclick="window.clearActionBinding('${actionMap.name}', '${action.name}')" ${action.bindings && action.bindings.length > 0 ? '' : 'disabled'}>Clear</button>
            <button class="action-btn btn-primary" onclick="window.resetActionBinding('${actionMap.name}', '${action.name}')">Reset</button>
            <button class="action-btn btn-success" onclick="window.startBinding('${actionMap.name}', '${action.name}')">Bind</button>
          </div>
        </div>
      `;
        });

        html += `
        </div>
      </div>
    `;
    });

    container.innerHTML = html;

    // Setup scroll listener for sticky header category tracking
    setupScrollCategoryTracker();
}

function renderActionBindings(action)
{
    if (!action.bindings || action.bindings.length === 0)
    {
        return '<span class="no-bindings">No bindings</span>';
    }

    return action.bindings.map(binding =>
    {
        const inputType = binding.input_type || 'Unknown';
        const trimmedInput = binding.input.trim();

        // Skip cleared placeholder bindings (format: "js1_ ", "kb1_ ", etc. with trailing space)
        const isClearedBinding = binding.input.match(/^(js\d*|kb\d*|mouse\d*|gp\d*)_\s*$/);
        if (isClearedBinding) return '';

        // Skip truly unbound bindings
        if (!trimmedInput || trimmedInput === '') return '';

        // Format display name with activation mode appended
        let displayText = binding.display_name || binding.input;

        if (binding.activation_mode)
        {
            const formattedMode = binding.activation_mode
                .split('_')
                .map(word => word.charAt(0).toUpperCase() + word.slice(1))
                .join(' ');
            displayText += ` [${formattedMode}]`;
        }

        // Show multi-tap indicator if present
        if (binding.multi_tap)
        {
            displayText += ` [${binding.multi_tap}x Tap]`;
        }

        return `<span class="binding-tag ${inputType.toLowerCase()}">${displayText}</span>`;
    }).filter(Boolean).join('');
}

// Global success message helper
window.showSuccessMessage = function (message)
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
};

// Refresh bindings from backend and update UI
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

// Helper function to process a raw input result
const processInput = (result) =>
{
    console.log('INPUT DETECTED (raw):', result.display_name, result.input_string);

    // Apply joystick mapping if applicable (pass the full result object for device UUID)
    const mappedInput = applyJoystickMapping(result.input_string, result.device_uuid);

    if (mappedInput === null)
    {
        return null; // Skip disabled joysticks
    }

    // Convert to Star Citizen format using HID axis name from backend if available
    let scFormattedInput;
    if (result.hid_axis_name && mappedInput.includes('_axis'))
    {
        // Check if this is a hat switch (backend should already have converted it, but check name just in case)
        const hidNameLower = result.hid_axis_name.toLowerCase().replace(/\s+/g, '');

        if (hidNameLower === 'hatswitch')
        {
            // This is a hat switch - backend should have already converted it to hat format
            // If it didn't, use the mappedInput as-is
            scFormattedInput = toStarCitizenFormat(mappedInput);
            console.log('Hat switch detected in HID name, using standard format:', scFormattedInput);
        }
        else
        {
            // Use the actual HID axis name from the device descriptor
            // Convert HID axis name to lowercase SC format (Rz -> rotz, X -> x, etc.)
            const hidName = result.hid_axis_name.toLowerCase();
            const scAxisName = hidName === 'rx' ? 'rotx' :
                hidName === 'ry' ? 'roty' :
                    hidName === 'rz' ? 'rotz' : hidName;

            // Replace axis number format with axis name format
            scFormattedInput = mappedInput.replace(/axis\d+(?:_(positive|negative))?/, scAxisName);
            console.log(`Converted to SC format using HID axis name "${result.hid_axis_name}":`, scFormattedInput);
        }
    }
    else
    {
        // Fallback: use hardcoded mapping for XInput gamepads or if no HID axis name available
        scFormattedInput = toStarCitizenFormat(mappedInput);
    }

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

async function startBinding(actionMapName, actionName, actionDisplayName = null)
{
    console.log('[TIMER] startBinding called for:', actionMapName, actionName);

    // If we're already binding, don't start another session
    if (bindingMode)
    {
        console.log('[TIMER] Already in binding mode, ignoring request');
        return;
    }

    bindingMode = true;
    isDetectionActive = true;
    currentBindingAction = { actionMapName, actionName };

    // Generate a unique ID for this binding session
    const thisBindingId = Date.now();
    currentBindingId = thisBindingId;

    // Reset detection state
    allDetectedInputs.clear();
    selectedInputKey = null;
    window.pendingBinding = null;

    // Update UI
    const modal = document.getElementById('binding-modal');
    const actionNameEl = document.getElementById('binding-modal-action');
    statusEl = document.getElementById('binding-modal-status');
    const countdownEl = document.getElementById('binding-modal-countdown');
    bindingModalSaveBtn = document.getElementById('binding-modal-save-btn');

    // Reset UI state
    statusEl.innerHTML = 'Press any button, key, mouse button, or move any axis...';
    statusEl.className = 'binding-status';
    countdownEl.textContent = '';
    setBindingSaveEnabled(false);

    // Clear any previous selection UI
    const existingSelection = statusEl.querySelector('.input-selection-container');
    if (existingSelection) existingSelection.remove();

    const existingMessage = statusEl.querySelector('.input-selection-message');
    if (existingMessage) existingMessage.remove();

    // Clear any previous conflict display
    const conflictDisplay = document.getElementById('binding-conflict-display');
    if (conflictDisplay)
    {
        conflictDisplay.style.display = 'none';
        conflictDisplay.innerHTML = '';
    }

    // Reset activation mode dropdown to default
    const activationModeSelect = document.getElementById('activation-mode-select');
    if (activationModeSelect)
    {
        activationModeSelect.value = '';
    }

    actionNameEl.textContent = 'Binding Action: ' + (actionDisplayName || actionName);
    modal.style.display = 'flex';

    // Start countdown
    let countdown = 10; // 10 seconds timeout
    let remaining = countdown;
    countdownEl.textContent = countdown;

    // Clear any existing interval
    if (countdownInterval)
    {
        clearInterval(countdownInterval);
    }

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

    // Start listening for inputs
    try
    {
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

        // Activate keyboard detection
        keyboardDetectionActive = true;

        // Activate mouse button detection
        let mouseDetectionHandler = null;

        // Create mouse event handler
        mouseDetectionHandler = async (event) =>
        {
            // Ignore if detection window has ended or we're hovering modal buttons
            if (!isDetectionActive || !window.mouseDetectionActive || window.getIgnoreModalMouseInputs()) return;

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
        if (window.showAlert) await window.showAlert(`Error waiting for input: ${error}`, 'Error');
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
        if (window.updateUnsavedIndicator) window.updateUnsavedIndicator();

        await refreshBindings();
        closeBindingModal();
    } catch (error)
    {
        console.error('Error clearing binding:', error);
        if (window.showAlert) await window.showAlert(`Error clearing binding: ${error}`, 'Error');
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
        if (window.updateUnsavedIndicator) window.updateUnsavedIndicator();

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
            if (window.showAlert) await window.showAlert(`Error resetting binding: ${error}`, 'Error');
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
        if (window.updateUnsavedIndicator) window.updateUnsavedIndicator();
        await refreshBindings();
        closeBindingModal();
    } catch (error)
    {
        console.error('Error in fallback reset:', error);
        if (window.showAlert) await window.showAlert(`Error resetting binding: ${error}`, 'Error');
    }
}

// Make startBinding available globally
window.startBinding = startBinding;
window.cancelBinding = cancelBinding;
window.clearBinding = clearBinding;
window.closeBindingModal = closeBindingModal;
window.resetBinding = resetBinding;

// Binding detection and conflict handling helpers
function applyJoystickMapping(inputString, deviceUuid)
{
    // Use the device manager's prefix override system
    if (window.applyDevicePrefixOverride && deviceUuid)
    {
        const mappedInput = window.applyDevicePrefixOverride(inputString, deviceUuid);
        if (mappedInput !== inputString)
        {
            console.log(`[KEYBINDINGS] Applied device prefix override: ${inputString} -> ${mappedInput}`);
            return mappedInput;
        }
    }

    // Fallback: check old deviceUuidMapping (legacy support)
    if (deviceUuid && deviceUuidMapping[deviceUuid])
    {
        const logicalDevice = deviceUuidMapping[deviceUuid];
        const match = inputString.match(/^([a-z]+\d+)_/);
        if (match)
        {
            const currentPrefix = match[1];
            if (currentPrefix !== logicalDevice)
            {
                return inputString.replace(currentPrefix, logicalDevice);
            }
        }
    }

    // Fallback: check old deviceMappings (legacy support)
    for (const [logicalDevice, mapping] of Object.entries(deviceMappings))
    {
        if (mapping && mapping.detectedPrefix && mapping.detectedNum !== undefined)
        {
            const detectedId = mapping.detectedPrefix + mapping.detectedNum;
            if (inputString.startsWith(detectedId + '_'))
            {
                return inputString.replace(detectedId, logicalDevice);
            }
        }
    }

    return inputString;
}

async function setPendingBindingSelection(processedInput)
{
    if (!currentBindingAction) return [];

    const { actionMapName, actionName } = currentBindingAction;
    const mappedInput = processedInput.scFormattedInput;

    // Check for conflicts by searching through all actions
    const conflicts = [];
    if (window.getCurrentKeybindings())
    {
        const keybindings = window.getCurrentKeybindings();
        keybindings.action_maps.forEach(actionMap =>
        {
            actionMap.actions.forEach(action =>
            {
                // Skip the current action we're binding
                if (actionMap.name === actionMapName && action.name === actionName) return;

                // Check if this action has a binding that matches our new input
                if (action.bindings)
                {
                    action.bindings.forEach(binding =>
                    {
                        if (binding.input === mappedInput)
                        {
                            conflicts.push({
                                action_map: actionMap.name,
                                action_name: action.name,
                                action_display_name: action.ui_label || action.display_name || action.name,
                                input: binding.input
                            });
                        }
                    });
                }
            });
        });
    }

    window.pendingBinding = {
        actionMapName,
        actionName,
        mappedInput,
        conflicts
    };

    // Enable save button
    setBindingSaveEnabled(true);

    return conflicts;
}

function updateConflictDisplay(conflicts)
{
    const conflictDisplay = document.getElementById('binding-conflict-display');
    if (!conflictDisplay) return;

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
            : c.action_name;

        const mapLabel = (c.action_map_label && !c.action_map_label.startsWith('@'))
            ? c.action_map_label
            : c.action_map_name;

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

function updateSelectionButtonStates()
{
    selectionButtons.forEach((btn, key) =>
    {
        if (key === selectedInputKey)
        {
            btn.classList.add('selected');
        } else
        {
            btn.classList.remove('selected');
        }
    });
}

function setSelectionMessage(message)
{
    if (selectionMessageEl)
    {
        selectionMessageEl.textContent = message;
    }
}

// Conflict Modal Functions
function showConflictModal(conflicts)
{
    const modal = document.getElementById('conflict-modal');
    const conflictList = document.getElementById('conflict-list');

    // Populate conflict list
    conflictList.innerHTML = conflicts.map(c =>
    {
        const actionLabel = (c.action_label && !c.action_label.startsWith('@'))
            ? c.action_label
            : c.action_name;

        const mapLabel = (c.action_map_label && !c.action_map_label.startsWith('@'))
            ? c.action_map_label
            : c.action_map_name;

        return `
      <div class="conflict-item">
        <div class="conflict-action-label">${actionLabel}</div>
        <div class="conflict-map-label">${mapLabel}</div>
      </div>
    `;
    }).join('');

    modal.style.display = 'flex';
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
    if (window.updateUnsavedIndicator) window.updateUnsavedIndicator();

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

// Make conflict functions globally available
window.showConflictModal = showConflictModal;
window.closeConflictModal = closeConflictModal;
window.confirmConflictBinding = confirmConflictBinding;

// Make keybinding management functions globally available
window.loadKeybindingsFile = loadKeybindingsFile;
window.newKeybinding = newKeybinding;
window.saveKeybindings = saveKeybindings;
window.saveKeybindingsAs = saveKeybindingsAs;
window.displayKeybindings = displayKeybindings;
window.cacheUserCustomizations = cacheUserCustomizations;
window.renderKeybindings = renderKeybindings;
window.applyBinding = applyBinding;
window.cancelBinding = cancelBinding;
window.setBindingSaveEnabled = setBindingSaveEnabled;
window.loadCategoryMappings = loadCategoryMappings;
window.refreshBindings = refreshBindings;
window.stopDetection = stopDetection;

// Make state variables globally available
window.getShowUnboundActions = () => showUnboundActions;
window.setShowUnboundActions = (value) => { showUnboundActions = value; };
window.getCustomizedOnly = () => customizedOnly;
window.setCustomizedOnly = (value) => { customizedOnly = value; };
window.getCurrentFilter = () => currentFilter;
window.setCurrentFilter = (value) => { currentFilter = value; };
window.getSearchTerm = () => searchTerm;
window.setSearchTerm = (value) => { searchTerm = value; };
window.getCurrentFilename = () => currentFilename;
window.getCurrentKeybindings = () => currentKeybindings;
window.getHasUnsavedChanges = () => hasUnsavedChanges;
window.setHasUnsavedChanges = (value) => { hasUnsavedChanges = value; };
window.getIgnoreModalMouseInputs = () => ignoreModalMouseInputs;
window.setIgnoreModalMouseInputs = (value) => { ignoreModalMouseInputs = value; };

/**
 * Toggle the visibility of an action map's actions list
 * @param {HTMLElement} headerEl - The action-map-header element that was clicked
 */
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

/**
 * Setup scroll listener to track which category header is visible
 * Updates the profile-name header with the current visible category
 */
function setupScrollCategoryTracker()
{
    const scrollContainer = document.getElementById('action-maps-container');
    if (!scrollContainer) return;

    // Remove any existing listener to avoid duplicates
    if (window.categoryTrackerScrollListener)
    {
        scrollContainer.removeEventListener('scroll', window.categoryTrackerScrollListener);
    }

    window.categoryTrackerScrollListener = () =>
    {
        const profileNameEl = document.getElementById('profile-name');
        if (!profileNameEl) return;

        // Get all action map headers (use the correct class)
        const headers = document.querySelectorAll('.action-map-header');
        if (headers.length === 0) return;

        let lastVisibleCategory = null;
        const scrollTop = scrollContainer.scrollTop;

        // Iterate through headers to find the last one that's above the scroll position
        headers.forEach((header) =>
        {
            const headerTop = header.offsetTop - scrollContainer.offsetTop;
            const h3 = header.querySelector('h3');
            const headerText = h3 ? h3.textContent.trim() : null;

            if (!headerText) return;

            // Check if header is above current scroll position + small threshold
            if (headerTop <= scrollTop + 10)
            {
                lastVisibleCategory = headerText;
            }
        });

        // Update the profile name if we found a category
        if (lastVisibleCategory && profileNameEl.textContent !== lastVisibleCategory)
        {
            profileNameEl.textContent = lastVisibleCategory;
        } else if (!lastVisibleCategory && profileNameEl.textContent !== 'Keybindings')
        {
            // Reset to default if no category is visible
            profileNameEl.textContent = 'Keybindings';
        }
    };

    scrollContainer.addEventListener('scroll', window.categoryTrackerScrollListener);
}
