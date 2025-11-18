const { invoke } = window.__TAURI__.core;

let isPolling = false;
let pollingInterval = null;
let selectedDevice = null;
let eventCount = 0;
let lastAxisValues = new Map(); // Track last values for change detection
let axisBitDepths = new Map(); // Track maximum observed bit depth per axis
let gilrsAxes = new Set();
let hidAxes = new Set();
let is16BitDevice = false; // Track if device uses 16-bit axes
let maxAxisValue = 255; // Max value detected (255 for 8-bit, 65535 for 16-bit)
let deviceAxisNames = {}; // Cached axis names from HID descriptor

// DOM elements
let startBtn, stopBtn, clearBtn, selectDeviceBtn, statusIndicator;
let deviceInfoDisplay, liveAxisGrid, rawEventStream, eventCounter;
let gilrsAxisList, hidAxisList, missingAxesAlert, missingAxesList;
let deviceModal, closeModalBtn, closeModalFooterBtn, deviceSelectionList;
let showUnchangedCheckbox;

// Initialize when page loads
document.addEventListener('DOMContentLoaded', () =>
{
    initializeHIDDebugger();
});

function initializeHIDDebugger()
{
    // Get DOM elements
    startBtn = document.getElementById('start-hid-debug-btn');
    stopBtn = document.getElementById('stop-hid-debug-btn');
    clearBtn = document.getElementById('clear-hid-debug-btn');
    selectDeviceBtn = document.getElementById('select-device-btn');
    statusIndicator = document.getElementById('hid-debug-status');

    deviceInfoDisplay = document.getElementById('device-info-display');
    liveAxisGrid = document.getElementById('live-axis-grid');
    rawEventStream = document.getElementById('raw-event-stream');
    eventCounter = document.getElementById('event-counter');

    gilrsAxisList = document.getElementById('gilrs-axes');
    hidAxisList = document.getElementById('hid-axes');
    missingAxesAlert = document.getElementById('missing-axes-alert');
    missingAxesList = document.getElementById('missing-axes-list');

    deviceModal = document.getElementById('device-selection-modal');
    closeModalBtn = document.getElementById('close-device-selection-modal');
    closeModalFooterBtn = document.getElementById('close-device-selection-footer');
    deviceSelectionList = document.getElementById('device-selection-list');

    showUnchangedCheckbox = document.getElementById('show-unchanged-values');

    // Event listeners
    startBtn.addEventListener('click', startPolling);
    stopBtn.addEventListener('click', stopPolling);
    clearBtn.addEventListener('click', clearData);
    selectDeviceBtn.addEventListener('click', showDeviceSelection);

    if (closeModalBtn) closeModalBtn.addEventListener('click', closeDeviceModal);
    if (closeModalFooterBtn) closeModalFooterBtn.addEventListener('click', closeDeviceModal);

    deviceModal.addEventListener('click', (e) =>
    {
        if (e.target === deviceModal)
        {
            closeDeviceModal();
        }
    });

    console.log('HID Debugger initialized');
}

async function showDeviceSelection()
{
    try
    {
        deviceSelectionList.innerHTML = '<div class="loading-spinner">Loading devices...</div>';
        deviceModal.classList.add('show');

        // Get list of HID devices using the new HID API
        const hidDevices = await invoke('list_hid_devices');

        deviceSelectionList.innerHTML = '';

        if (!hidDevices || hidDevices.length === 0)
        {
            deviceSelectionList.innerHTML = '<div style="padding: 2rem; text-align: center; color: #888;">No HID game controllers detected</div>';
            return;
        }

        hidDevices.forEach((device, index) =>
        {
            const card = document.createElement('div');
            card.className = 'device-select-card';

            const deviceName = device.product || `${device.manufacturer} Device` || 'Unknown Device';
            const deviceType = 'HID Controller'; // We know these are game controllers from the filter

            card.innerHTML = `
                <div class="device-select-header">
                    <div class="device-select-name">${deviceName}</div>
                    <span class="device-select-badge">${deviceType}</span>
                </div>
                <div class="device-select-details">
                    VID: 0x${device.vendor_id.toString(16).padStart(4, '0').toUpperCase()} | 
                    PID: 0x${device.product_id.toString(16).padStart(4, '0').toUpperCase()}
                    ${device.serial_number ? ` | SN: ${device.serial_number}` : ''}
                </div>
                <div class="device-select-uuid">Path: ${device.path}</div>
            `;

            // Store the full device info for selection
            card.addEventListener('click', () => selectDevice({
                name: deviceName,
                device_type: deviceType,
                path: device.path,
                vendor_id: device.vendor_id,
                product_id: device.product_id,
                serial_number: device.serial_number,
                manufacturer: device.manufacturer
            }));
            deviceSelectionList.appendChild(card);
        });

    } catch (error)
    {
        console.error('Error loading HID devices:', error);
        deviceSelectionList.innerHTML = '<div style="padding: 2rem; color: #f44;">Error loading HID devices: ' + error + '</div>';
    }
}

async function selectDevice(device)
{
    selectedDevice = device;
    console.log('Selected HID device:', device);

    // Update device info display
    deviceInfoDisplay.innerHTML = `
        <div class="device-info-content">
            <div class="device-info-grid">
                <div class="device-info-item">
                    <div class="device-info-label">Device Name</div>
                    <div class="device-info-value">${device.name}</div>
                </div>
                <div class="device-info-item">
                    <div class="device-info-label">Type</div>
                    <div class="device-info-value">${device.device_type}</div>
                </div>
                <div class="device-info-item">
                    <div class="device-info-label">Vendor ID</div>
                    <div class="device-info-value"><code>0x${device.vendor_id.toString(16).padStart(4, '0').toUpperCase()}</code></div>
                </div>
                <div class="device-info-item">
                    <div class="device-info-label">Product ID</div>
                    <div class="device-info-value"><code>0x${device.product_id.toString(16).padStart(4, '0').toUpperCase()}</code></div>
                </div>
                ${device.serial_number ? `
                <div class="device-info-item">
                    <div class="device-info-label">Serial Number</div>
                    <div class="device-info-value"><code>${device.serial_number}</code></div>
                </div>
                ` : ''}
                <div class="device-info-item">
                    <div class="device-info-label">HID Path</div>
                    <div class="device-info-value"><code style="font-size: 0.8em;">${device.path}</code></div>
                </div>
            </div>
        </div>
    `;

    closeDeviceModal();

    // Load axis names from the HID descriptor using the new library-based parser
    // This gives us proper names like "X", "Y", "Rz", "Slider", etc.
    await loadAxisNames(device.path);

    // Show how many axis names were successfully loaded
    const axisCount = Object.keys(deviceAxisNames).length;
    if (axisCount > 0)
    {
        console.log(`[HID] Successfully loaded ${axisCount} axis names from descriptor`);
    }
}

function closeDeviceModal()
{
    deviceModal.classList.remove('show');
}

async function loadAxisNames(devicePath)
{
    try
    {
        // Load axis names using the hidreport + hut libraries for proper HID parsing
        // This provides accurate names like "X", "Y", "Rz", "Slider", etc.
        deviceAxisNames = await invoke('get_hid_axis_names', { devicePath });
        console.log('[HID] Loaded axis names from descriptor:', deviceAxisNames);
    } catch (error)
    {
        console.warn('[HID] Could not load axis names from descriptor:', error);
        deviceAxisNames = {};
    }
}

async function startPolling()
{
    if (!selectedDevice)
    {
        alert('Please select a device first!');
        return;
    }

    if (isPolling) return;

    isPolling = true;
    startBtn.disabled = true;
    stopBtn.disabled = false;
    statusIndicator.textContent = '🔴 Polling HID...';
    statusIndicator.classList.add('polling');

    // Clear placeholders
    if (liveAxisGrid.querySelector('.axis-placeholder'))
    {
        liveAxisGrid.innerHTML = '';
    }
    if (rawEventStream.querySelector('.event-placeholder'))
    {
        rawEventStream.innerHTML = '';
    }

    // Start polling loop
    pollDevice();
}

function stopPolling()
{
    isPolling = false;
    startBtn.disabled = false;
    stopBtn.disabled = true;
    statusIndicator.textContent = 'Stopped';
    statusIndicator.classList.remove('polling');

    if (pollingInterval)
    {
        clearTimeout(pollingInterval);
        pollingInterval = null;
    }
}

function clearData()
{
    rawEventStream.innerHTML = '<div class="event-placeholder">Events will appear here...</div>';
    eventCount = 0;
    eventCounter.textContent = '0 events';
    lastAxisValues.clear();
    axisBitDepths.clear(); // Clear tracked bit depths
    gilrsAxes.clear();
    hidAxes.clear();
    is16BitDevice = false;
    maxAxisValue = 255;

    // Clear live axis grid
    liveAxisGrid.innerHTML = '<div class="axis-placeholder">Start polling to see live axis data...</div>';

    // Reset comparison lists
    gilrsAxisList.innerHTML = '<div class="axis-placeholder">No Gilrs data yet</div>';
    hidAxisList.innerHTML = '<div class="axis-placeholder">No HID data yet</div>';
    missingAxesAlert.style.display = 'none';
}

async function pollDevice()
{
    if (!isPolling) return;

    try
    {
        // Read raw HID report from the device
        const report = await invoke('read_hid_device_report', {
            devicePath: selectedDevice.path,
            timeoutMs: 50
        });

        if (report && report.length > 0)
        {
            // Parse the report to extract axis data
            const axisReport = await invoke('parse_hid_report', {
                report: report
            });

            if (axisReport && axisReport.axis_values)
            {
                // Update bit depth tracking
                const previousBitMode = is16BitDevice;
                if (axisReport.is_16bit !== undefined)
                {
                    is16BitDevice = axisReport.is_16bit;
                    maxAxisValue = is16BitDevice ? 65535 : 255;
                }

                // If bit mode changed, update all existing axis cards
                if (previousBitMode !== is16BitDevice)
                {
                    updateAllAxisCardRanges();
                }

                // Process each axis
                for (const [axisId, value] of Object.entries(axisReport.axis_values))
                {
                    const axis_id = parseInt(axisId);
                    const bitDepth = axisReport.axis_bit_depths ? axisReport.axis_bit_depths[axisId] : null;
                    // Use cached axis names from descriptor (loaded once at device selection)
                    const axisName = deviceAxisNames[axisId] || null;

                    handleAxisMovement({
                        axis_id: axis_id,
                        value: value,
                        bit_depth: bitDepth,
                        axis_name: axisName
                    });
                }
            }
        }

        // Also get current Gilrs state for comparison
        await updateGilrsComparison();

    } catch (error)
    {
        // Silently ignore timeout errors (no data available)
        if (!error.toString().includes('timeout') && !error.toString().includes('timed out'))
        {
            console.error('Error polling HID device:', error);
        }
    }

    // Continue polling
    if (isPolling)
    {
        pollingInterval = setTimeout(pollDevice, 10);
    }
}

function handleAxisMovement(axisData)
{
    const { axis_id, value, bit_depth, axis_name } = axisData;

    // Track this axis for HID comparison
    hidAxes.add(`Axis ${axis_id}`);

    // Track maximum observed bit depth for this axis
    if (bit_depth)
    {
        const currentMaxBitDepth = axisBitDepths.get(axis_id) || 0;
        if (bit_depth > currentMaxBitDepth)
        {
            axisBitDepths.set(axis_id, bit_depth);
        }
    }

    // Use the maximum observed bit depth
    const maxObservedBitDepth = axisBitDepths.get(axis_id) || bit_depth;

    // Check if value changed significantly
    const lastValue = lastAxisValues.get(axis_id) || 0;
    const changed = Math.abs(value - lastValue) > 0.01; // 1% threshold

    const showUnchanged = showUnchangedCheckbox.checked;

    if (changed || showUnchanged)
    {
        lastAxisValues.set(axis_id, value);

        // Update live axis display with max observed bit depth and axis name
        updateAxisCard(axis_id, value, maxObservedBitDepth, axis_name, changed);

        // Add to event stream
        if (changed)
        {
            addEventToStream('AXIS_CHANGE', axis_id, value);
            eventCount++;
            eventCounter.textContent = `${eventCount} events`;
        }

        // Update comparison
        updateComparison();
    }
}

function updateAxisCard(axisId, value, bitDepth, axisName, changed)
{
    let card = document.getElementById(`axis-card-${axisId}`);

    if (!card)
    {
        card = document.createElement('div');
        card.id = `axis-card-${axisId}`;
        card.className = 'axis-card';

        // Use axis name from HID descriptor if available, otherwise use friendly name
        const displayName = axisName || getAxisName(axisId);

        const rangeText = bitDepth ? `0 - ${(1 << bitDepth) - 1} (${bitDepth}-bit)` :
            (is16BitDevice ? '0 - 65535 (16-bit)' : '0 - 255 (8-bit)');

        card.innerHTML = `
            <div class="axis-card-header">
                <div class="axis-name">${displayName}</div>
                <div class="axis-index">Index ${axisId}</div>
            </div>
            <div class="axis-value-display">0</div>
            <div class="axis-bar-container">
                <div class="axis-bar" style="width: 50%"></div>
            </div>
            <div class="axis-range">${rangeText}</div>
        `;

        liveAxisGrid.appendChild(card);
    }

    // Update values
    const valueDisplay = card.querySelector('.axis-value-display');
    const bar = card.querySelector('.axis-bar');
    const rangeDisplay = card.querySelector('.axis-range');

    // Calculate max value based on bit depth
    const maxValue = bitDepth ? (1 << bitDepth) - 1 : maxAxisValue;

    // Normalize to percentage based on detected max value
    const normalized = (value / maxValue) * 100;

    valueDisplay.textContent = Math.round(value);
    bar.style.width = `${normalized}%`;

    // Update range display with detected bit depth
    const rangeText = bitDepth ? `0 - ${maxValue} (${bitDepth}-bit)` :
        (is16BitDevice ? '0 - 65535 (16-bit)' : '0 - 255 (8-bit)');
    if (rangeDisplay.textContent !== rangeText)
    {
        rangeDisplay.textContent = rangeText;
    }

    // Highlight if changed
    if (changed)
    {
        card.classList.add('updated');
        setTimeout(() => card.classList.remove('updated'), 500);
    }
}

function getAxisName(axisId)
{
    // Map common axis IDs to names (this may need adjustment based on device)
    const axisNames = {
        1: 'X',
        2: 'Y',
        3: 'Z',
        4: 'Rx (Rotation X)',
        5: 'Ry (Rotation Y)',
        6: 'Rz (Rotation Z)',
        7: 'Slider',
        8: 'Dial',
    };

    return axisNames[axisId] || `Axis ${axisId}`;
}

function updateAllAxisCardRanges()
{
    // Update the range text on all existing axis cards
    const rangeText = is16BitDevice ? '0 - 65535 (16-bit)' : '0 - 255 (8-bit)';

    document.querySelectorAll('.axis-card .axis-range').forEach(rangeDisplay =>
    {
        rangeDisplay.textContent = rangeText;
    });

    console.log(`Updated all axis cards to ${rangeText}`);
}

function addEventToStream(type, axisId, value)
{
    const eventEntry = document.createElement('div');
    eventEntry.className = `event-entry ${type.toLowerCase().replace('_', '-')}`;

    const now = new Date();
    const timestamp = now.toLocaleTimeString('en-US', {
        hour12: false,
        fractionalSecondDigits: 3
    });

    const axisName = getAxisName(axisId);

    eventEntry.innerHTML = `
        <span class="event-timestamp">${timestamp}</span>
        <span class="event-type">${type}</span>
        <span class="event-data">${axisName} = <span class="event-value">${Math.round(value)}</span></span>
    `;

    rawEventStream.insertBefore(eventEntry, rawEventStream.firstChild);

    // Limit to 200 events
    while (rawEventStream.children.length > 200)
    {
        rawEventStream.removeChild(rawEventStream.lastChild);
    }
}

async function updateGilrsComparison()
{
    try
    {
        // Poll Gilrs for the current device state
        // This would need a Rust function that reports current Gilrs axis values
        // For now, we'll simulate by tracking what we see in detection

        // Update Gilrs list
        if (gilrsAxes.size > 0)
        {
            gilrsAxisList.innerHTML = '';
            gilrsAxes.forEach(axis =>
            {
                const item = document.createElement('div');
                item.className = 'axis-list-item';
                item.innerHTML = `
                    <div class="axis-list-name">${axis}</div>
                    <div class="axis-list-value">Detected</div>
                `;
                gilrsAxisList.appendChild(item);
            });
        }

        // Update HID list
        if (hidAxes.size > 0)
        {
            hidAxisList.innerHTML = '';
            hidAxes.forEach(axis =>
            {
                const item = document.createElement('div');
                item.className = 'axis-list-item';
                item.innerHTML = `
                    <div class="axis-list-name">${axis}</div>
                    <div class="axis-list-value">Detected</div>
                `;
                hidAxisList.appendChild(item);
            });
        }

    } catch (error)
    {
        console.error('Error updating Gilrs comparison:', error);
    }
}

function updateComparison()
{
    // Find axes that are in HID but not in Gilrs
    const missingInGilrs = [...hidAxes].filter(axis => !gilrsAxes.has(axis));

    if (missingInGilrs.length > 0)
    {
        missingAxesAlert.style.display = 'block';
        missingAxesList.textContent = missingInGilrs.join(', ');
    } else
    {
        missingAxesAlert.style.display = 'none';
    }
}

// Expose for debugging
window.hidDebugger = {
    selectedDevice,
    lastAxisValues,
    gilrsAxes,
    hidAxes,
};
