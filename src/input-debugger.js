const { invoke } = window.__TAURI__.core;
import { getInputType } from './input-utils.js';

let isDetecting = false;
let detectionLoop = null;
let eventCount = 0;
let uniqueButtons = new Set();
let uniqueAxes = new Set();
let uniqueHats = new Set();
let uniqueKeys = new Set();

// DOM element references (will be set during initialization)
let startBtn, stopBtn, clearBtn, showDevicesBtn, statusIndicator, timeline, eventCountSpan, autoScrollCheckbox;
let statTotal, statButtons, statAxes, statHats, statKeys;
let deviceModal, closeModalBtn, deviceList;

// Define all functions first, before initializeDebugger

async function startDetecting()
{
    if (isDetecting) return;

    isDetecting = true;
    startBtn.disabled = true;
    stopBtn.disabled = false;
    statusIndicator.textContent = '🔴 Detecting...';
    statusIndicator.classList.add('detecting');

    // Clear the empty message
    const emptyMessage = timeline.querySelector('.timeline-empty');
    if (emptyMessage)
    {
        emptyMessage.remove();
    }

    // Start detection loop
    detectInputs();
}

function stopDetecting()
{
    isDetecting = false;
    startBtn.disabled = false;
    stopBtn.disabled = true;
    statusIndicator.textContent = 'Stopped';
    statusIndicator.classList.remove('detecting');

    if (detectionLoop)
    {
        clearTimeout(detectionLoop);
        detectionLoop = null;
    }
}

function clearLog()
{
    timeline.innerHTML = '<div class="timeline-empty">Press "Start Detecting" and then press any button, key, or move any axis...</div>';
    eventCount = 0;
    uniqueButtons.clear();
    uniqueAxes.clear();
    uniqueHats.clear();
    uniqueKeys.clear();
    updateStats();
}

async function detectInputs()
{
    if (!isDetecting) return;

    try
    {
        // Wait for input with a 1-second timeout
        const sessionId = 'debug-session-' + Date.now();
        const result = await invoke('wait_for_input_binding', {
            sessionId: sessionId,
            timeoutSecs: 1
        });

        if (result)
        {
            addEvent(result);
        }
    } catch (error)
    {
        console.error('Error detecting input:', error);
    }

    // Continue the loop
    if (isDetecting)
    {
        detectionLoop = setTimeout(detectInputs, 10);
    }
}

function addEvent(inputData)
{
    eventCount++;

    // Use shared utility to determine event type
    const eventType = getInputType(inputData.input_string);

    // Track unique inputs (without direction for axes)
    if (eventType === 'hat')
    {
        uniqueHats.add(inputData.input_string);
    } else if (eventType === 'axis')
    {
        // Track base axis without direction
        const baseAxis = inputData.input_string.replace(/_(positive|negative)$/, '');
        uniqueAxes.add(baseAxis);
    } else if (eventType === 'button')
    {
        uniqueButtons.add(inputData.input_string);
    } else if (eventType === 'keyboard')
    {
        uniqueKeys.add(inputData.input_string);
    }

    // Determine axis direction for styling
    let cssClass = eventType;
    let displayType = eventType;
    if (eventType === 'axis')
    {
        if (inputData.input_string.includes('_positive'))
        {
            cssClass = 'axis-positive';
            displayType = 'axis +';
        } else if (inputData.input_string.includes('_negative'))
        {
            cssClass = 'axis-negative';
            displayType = 'axis -';
        }
    }

    // Create event element
    const eventEl = document.createElement('div');
    eventEl.className = `timeline-event ${cssClass}`;

    const now = new Date();
    const timeString = now.toLocaleTimeString('en-US', { hour12: false, fractionalSecondDigits: 3 });

    // Add axis value if available
    let valueDisplay = '';
    if (inputData.axis_value !== undefined && inputData.axis_value !== null)
    {
        const valueClass = inputData.axis_value > 0 ? 'positive' : 'negative';
        const valuePercent = (inputData.axis_value * 100).toFixed(1);
        valueDisplay = `<div class="event-value ${valueClass}">Value: ${valuePercent}%</div>`;
    }

    // Add modifiers display if available
    let modifiersDisplay = '';
    if (inputData.modifiers && inputData.modifiers.length > 0)
    {
        const modifiersText = inputData.modifiers.join(' + ');
        modifiersDisplay = `<div class="event-modifiers">🎮 Modifiers: ${modifiersText}</div>`;
    }

    eventEl.innerHTML = `
        <div class="event-time">${timeString}</div>
        <div class="event-details">
            <div class="event-input">${inputData.input_string}</div>
            <div class="event-display">${inputData.display_name}</div>
            ${valueDisplay}
            ${modifiersDisplay}
        </div>
        <div class="event-type ${cssClass}">${displayType}</div>
    `;

    // Add to timeline
    timeline.insertBefore(eventEl, timeline.firstChild);

    // Auto-scroll to top if enabled
    if (autoScrollCheckbox.checked)
    {
        timeline.scrollTop = 0;
    }

    // Limit timeline to 100 events
    while (timeline.children.length > 100)
    {
        timeline.removeChild(timeline.lastChild);
    }

    // Update stats
    updateStats();
}

function updateStats()
{
    if (eventCountSpan) eventCountSpan.textContent = `${eventCount} event${eventCount !== 1 ? 's' : ''}`;
    if (statTotal) statTotal.textContent = eventCount;
    if (statButtons) statButtons.textContent = uniqueButtons.size;
    if (statAxes) statAxes.textContent = uniqueAxes.size;
    if (statHats) statHats.textContent = uniqueHats.size;
    if (statKeys) statKeys.textContent = uniqueKeys.size;
}

function updateFileIndicator()
{
    const indicator = document.getElementById('loaded-file-indicator');
    const fileNameEl = document.getElementById('loaded-file-name');
    const savedPath = localStorage.getItem('keybindingsFilePath');

    if (indicator && fileNameEl && savedPath)
    {
        const fileName = savedPath.split(/[\\\\/]/).pop();
        fileNameEl.textContent = fileName;
        indicator.style.display = 'flex';
    }
}

// Convert JavaScript KeyboardEvent.code to Star Citizen keyboard format
function convertKeyCodeToSC(code, key, location)
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

// Keyboard event listener for detecting keyboard input
function handleKeyboardInput(event)
{
    if (!isDetecting) return;

    // Prevent default browser behavior
    event.preventDefault();
    event.stopPropagation();

    const code = event.code;
    const key = event.key;
    const location = event.location;

    // Detect modifiers being held
    const modifiers = [];
    if (event.shiftKey) modifiers.push(event.location === 1 ? 'LSHIFT' : 'RSHIFT');
    if (event.ctrlKey) modifiers.push(event.location === 1 ? 'LCTRL' : 'RCTRL');
    if (event.altKey) modifiers.push(event.location === 1 ? 'LALT' : 'RALT');

    // Convert to Star Citizen format
    const scKey = convertKeyCodeToSC(code, key, location);

    // Build the input string (kb1_key format)
    const inputString = `kb1_${scKey}`;

    // Build display name
    const displayName = `Keyboard - ${code} (${scKey})`;

    // Add the event
    addEvent({
        input_string: inputString,
        display_name: displayName,
        device_type: 'Keyboard',
        axis_value: null,
        modifiers: modifiers,
        is_modifier: ['lshift', 'rshift', 'lctrl', 'rctrl', 'lalt', 'ralt', 'lwin'].includes(scKey)
    });
}

// Helper function to determine if a device is a gamepad
function isGamepad(deviceName)
{
    const gamepadPatterns = ['xbox', 'controller', 'gamepad'];
    return gamepadPatterns.some(pattern => deviceName.toLowerCase().includes(pattern));
}

// Show device modal with list of connected devices
async function showDevices()
{
    try
    {
        const devices = await invoke('detect_joysticks');

        console.log('Detected devices:', devices); // Debug log

        deviceList.innerHTML = '';

        if (!devices || devices.length === 0)
        {
            deviceList.innerHTML = '<div style="padding: 1rem; text-align: center; color: #888;">No devices detected</div>';
        } else
        {
            console.log('Building device cards for', devices.length, 'devices');

            devices.forEach((device, index) =>
            {
                const isGp = device.device_type === 'Gamepad';
                const typeLabel = device.device_type;
                const typeClass = isGp ? 'gamepad' : 'joystick';
                // Use index + 1 for the SC instance ID (SC uses 1-based indexing)
                const deviceId = isGp ? `gp${index + 1}` : `js${index + 1}`;

                console.log(`Device ${index}: ${device.name} -> ${deviceId} (${typeLabel})`);

                const deviceCard = document.createElement('div');
                deviceCard.className = `device-card device-${typeClass}`;
                deviceCard.innerHTML = `
                    <div class="device-header">
                        <div class="device-name">${device.name}</div>
                        <span class="device-badge ${typeClass}">${typeLabel}</span>
                    </div>
                    <div class="device-details">
                        <div class="device-detail-row">
                            <span class="label">SC Instance:</span>
                            <code>${deviceId}</code>
                        </div>
                    </div>
                    <div class="device-status">${device.is_connected ? 'Connected' : 'Disconnected'}</div>
                `;

                deviceList.appendChild(deviceCard);
                console.log('Appended device card to list');
            });

            console.log('Final deviceList innerHTML length:', deviceList.innerHTML.length);
        }

        console.log('Setting modal display to flex');
        deviceModal.style.display = 'flex';
        console.log('Modal display style:', window.getComputedStyle(deviceModal).display);
    } catch (error)
    {
        console.error('Error fetching devices:', error);
        deviceList.innerHTML = '<div style="padding: 1rem; color: #f44;">Error loading devices</div>';
        deviceModal.style.display = 'flex';
    }
}

// Close device modal
function closeDeviceModal()
{
    deviceModal.style.display = 'none';
}

// Initialize debugger when tab is opened
window.initializeDebugger = function ()
{
    // Only initialize once
    if (startBtn) return;

    // DOM elements
    startBtn = document.getElementById('start-debug-btn');
    stopBtn = document.getElementById('stop-debug-btn');
    clearBtn = document.getElementById('clear-debug-btn');
    showDevicesBtn = document.getElementById('show-devices-btn');
    statusIndicator = document.getElementById('debug-status');
    timeline = document.getElementById('timeline');
    eventCountSpan = document.getElementById('event-count');
    autoScrollCheckbox = document.getElementById('auto-scroll-checkbox');

    // Modal elements
    deviceModal = document.getElementById('device-modal');
    closeModalBtn = document.getElementById('close-modal-btn');
    const closeModalFooterBtn = document.getElementById('close-device-modal-footer');
    deviceList = document.getElementById('device-modal-list');

    console.log('Device modal elements:', { deviceModal, closeModalBtn, closeModalFooterBtn, deviceList });

    // Stats elements
    statTotal = document.getElementById('stat-total');
    statButtons = document.getElementById('stat-buttons');
    statAxes = document.getElementById('stat-axes');
    statHats = document.getElementById('stat-hats');
    statKeys = document.getElementById('stat-keys');

    // Event listeners
    startBtn.addEventListener('click', startDetecting);
    stopBtn.addEventListener('click', stopDetecting);
    clearBtn.addEventListener('click', clearLog);
    showDevicesBtn.addEventListener('click', showDevices);
    if (closeModalBtn) closeModalBtn.addEventListener('click', closeDeviceModal);
    if (closeModalFooterBtn) closeModalFooterBtn.addEventListener('click', closeDeviceModal);

    // Close modal when clicking outside
    deviceModal.addEventListener('click', (e) =>
    {
        if (e.target === deviceModal)
        {
            closeDeviceModal();
        }
    });

    // Keyboard event listener (capture phase to catch before browser defaults)
    document.addEventListener('keydown', handleKeyboardInput, true);

    // Initialize file indicator on load
    updateFileIndicator();
    updateStats();
};
