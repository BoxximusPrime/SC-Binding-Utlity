# VKB Integration Usage Guide

## Overview

The VKB integration allows SC-Binding-Utility to understand VKB joystick layer configurations (SHIFT1/SHIFT2) and TEMPO modes (Short/Long/Double press), and automatically annotate Star Citizen keybindings with this information.

## Workflow

### 1. Import VKB Profile

**When:** Before or after loading SC keybindings
**Order:** Can be done in any order - if VKB profiles are loaded first, keybindings will be annotated when loaded; if keybindings are loaded first, they can be reloaded after importing VKB profiles

```javascript
import { invoke } from '@tauri-apps/api/core';
import { open } from '@tauri-apps/plugin-dialog';

// Let user select VKB .fp3 file
async function importVkbProfile() {
    try {
        // Open file dialog
        const filePath = await open({
            title: 'Select VKB Report (.fp3)',
            filters: [{
                name: 'VKB Report',
                extensions: ['fp3']
            }],
            multiple: false
        });

        if (!filePath) return;

        // Parse and store VKB profile
        // joystick_id should match the instance ID in SC keybindings (usually 1 or 2)
        const buttons = await invoke('parse_vkb_profile', {
            filePath: filePath,
            joystickId: 1  // Use 1 for first joystick, 2 for second, etc.
        });

        console.log(`✅ Imported ${buttons.length} VKB buttons`);

        // Show summary
        const byLayer = buttons.reduce((acc, btn) => {
            const key = btn.layer || 'Direct';
            acc[key] = (acc[key] || 0) + 1;
            return acc;
        }, {});

        console.log('Layer breakdown:', byLayer);

        // Reload keybindings to apply VKB annotations
        if (currentKeybindingsLoaded) {
            await reloadKeybindings();
        }

    } catch (error) {
        console.error('Failed to import VKB profile:', error);
        alert(`Error: ${error}`);
    }
}
```

### 2. Import Multiple Joysticks

For dual-stick setups (e.g., VKB Gladiator L + R):

```javascript
async function importBothJoysticks() {
    // Import right stick (usually instance 1)
    const rightPath = await open({
        title: 'Select Right Stick VKB Report',
        filters: [{ name: 'VKB Report', extensions: ['fp3'] }]
    });

    if (rightPath) {
        const rightButtons = await invoke('parse_vkb_profile', {
            filePath: rightPath,
            joystickId: 1
        });
        console.log(`Right stick: ${rightButtons.length} buttons`);
    }

    // Import left stick (usually instance 2)
    const leftPath = await open({
        title: 'Select Left Stick VKB Report',
        filters: [{ name: 'VKB Report', extensions: ['fp3'] }]
    });

    if (leftPath) {
        const leftButtons = await invoke('parse_vkb_profile', {
            filePath: leftPath,
            joystickId: 2
        });
        console.log(`Left stick: ${leftButtons.length} buttons`);
    }

    // Check total mappings stored
    const totalMappings = await invoke('get_vkb_mapping_count');
    console.log(`Total VKB mappings: ${totalMappings}`);

    // Reload keybindings to apply annotations
    await reloadKeybindings();
}
```

### 3. Load Keybindings (With Auto-Annotation)

Once VKB profiles are imported, loading keybindings will automatically annotate them:

```javascript
async function loadKeybindings(filePath) {
    try {
        const bindings = await invoke('load_keybindings', {
            filePath: filePath
        });

        // bindings now contain VKB layer information in each rebind
        displayBindings(bindings);

    } catch (error) {
        console.error('Failed to load keybindings:', error);
    }
}
```

### 4. Display VKB Layer Information

Access VKB information from the `Rebind` structure:

```javascript
function displayBinding(rebind) {
    // rebind structure:
    // {
    //   input: "js1_button105",
    //   multi_tap: null,
    //   activation_mode: "",
    //   vkb_layer: "SHIFT1",           // Optional: "SHIFT1", "SHIFT2", null
    //   vkb_tempo: null,                // Optional: "Short", "Long", "Double", null
    //   vkb_physical_id: 5,             // Optional: physical button ID
    //   vkb_physical_info: "(A2)"       // Optional: button description
    // }

    let displayText = rebind.input;

    // Add layer indicator
    if (rebind.vkb_layer) {
        displayText += ` <span class="layer-badge ${rebind.vkb_layer.toLowerCase()}">${rebind.vkb_layer}</span>`;
    }

    // Add tempo indicator
    if (rebind.vkb_tempo) {
        displayText += ` <span class="tempo-badge">[${rebind.vkb_tempo}]</span>`;
    }

    // Add physical button info
    if (rebind.vkb_physical_info) {
        displayText += ` <span class="physical-info">${rebind.vkb_physical_info}</span>`;
    }

    return displayText;
}

// Example CSS:
// .layer-badge.shift1 { background: #4a90e2; color: white; }
// .layer-badge.shift2 { background: #e67e22; color: white; }
// .tempo-badge { background: #9b59b6; color: white; }
```

### 5. Filter/Group by Layer

```javascript
function groupBindingsByLayer(actions) {
    const grouped = {};

    for (const action of actions) {
        for (const rebind of action.rebinds) {
            const layer = rebind.vkb_layer || 'Direct';

            if (!grouped[layer]) {
                grouped[layer] = [];
            }

            grouped[layer].push({
                action: action.name,
                input: rebind.input,
                physical_info: rebind.vkb_physical_info,
                tempo: rebind.vkb_tempo
            });
        }
    }

    return grouped;
}

// Usage:
const byLayer = groupBindingsByLayer(actionMaps);
// {
//   "Direct": [...],
//   "SHIFT1": [...],
//   "SHIFT2": [...]
// }
```

### 6. Clear VKB Mappings

```javascript
async function clearVkbMappings() {
    try {
        await invoke('clear_vkb_mappings');
        console.log('Cleared VKB mappings');

        // Reload keybindings to remove annotations
        await reloadKeybindings();
    } catch (error) {
        console.error('Failed to clear VKB mappings:', error);
    }
}
```

## Complete Example: VKB-Aware Binding List Component

```javascript
// Example Svelte component
<script>
import { invoke } from '@tauri-apps/api/core';
import { open } from '@tauri-apps/plugin-dialog';

let bindings = [];
let vkbMappingCount = 0;
let showLayerFilter = false;
let selectedLayer = 'all';

async function importVkb() {
    const filePath = await open({
        filters: [{ name: 'VKB Report', extensions: ['fp3'] }]
    });

    if (filePath) {
        await invoke('parse_vkb_profile', {
            filePath,
            joystickId: 1  // Adjust based on your setup
        });

        vkbMappingCount = await invoke('get_vkb_mapping_count');
        await loadBindings();  // Reload to apply annotations
    }
}

async function loadBindings() {
    const data = await invoke('load_keybindings', {
        filePath: currentKeybindingsPath
    });

    bindings = data.action_maps.flatMap(map =>
        map.actions.flatMap(action => ({
            action: action.name,
            ...action
        }))
    );

    showLayerFilter = vkbMappingCount > 0;
}

function getLayerBadgeClass(layer) {
    if (!layer) return 'badge-direct';
    if (layer === 'SHIFT1') return 'badge-shift1';
    if (layer === 'SHIFT2') return 'badge-shift2';
    return 'badge-other';
}

$: filteredBindings = selectedLayer === 'all'
    ? bindings
    : bindings.filter(b =>
        b.rebinds.some(r => (r.vkb_layer || 'Direct') === selectedLayer)
    );
</script>

<div class="binding-list">
    <div class="toolbar">
        <button on:click={importVkb}>Import VKB Profile</button>

        {#if vkbMappingCount > 0}
            <span class="mapping-count">
                {vkbMappingCount} VKB mappings loaded
            </span>

            <select bind:value={selectedLayer}>
                <option value="all">All Layers</option>
                <option value="Direct">Direct</option>
                <option value="SHIFT1">SHIFT1</option>
                <option value="SHIFT2">SHIFT2</option>
            </select>
        {/if}
    </div>

    <div class="bindings">
        {#each filteredBindings as binding}
            <div class="binding-row">
                <span class="action-name">{binding.action}</span>

                {#each binding.rebinds as rebind}
                    <div class="rebind">
                        <span class="input">{rebind.input}</span>

                        {#if rebind.vkb_layer}
                            <span class="badge {getLayerBadgeClass(rebind.vkb_layer)}">
                                {rebind.vkb_layer}
                            </span>
                        {/if}

                        {#if rebind.vkb_tempo}
                            <span class="badge badge-tempo">[{rebind.vkb_tempo}]</span>
                        {/if}

                        {#if rebind.vkb_physical_info}
                            <span class="physical-info">{rebind.vkb_physical_info}</span>
                        {/if}
                    </div>
                {/each}
            </div>
        {/each}
    </div>
</div>

<style>
.badge-direct { background: #95a5a6; }
.badge-shift1 { background: #4a90e2; }
.badge-shift2 { background: #e67e22; }
.badge-tempo { background: #9b59b6; }
.badge { padding: 2px 6px; border-radius: 3px; color: white; font-size: 0.85em; }
.physical-info { color: #7f8c8d; font-style: italic; }
</style>
```

## API Reference

### Tauri Commands

#### `parse_vkb_profile(filePath: string, joystickId: number)`
Parses a VKB .fp3 file and stores the mappings in application state.

**Parameters:**
- `filePath`: Path to .fp3 file
- `joystickId`: Joystick instance (1, 2, etc.) matching SC keybindings

**Returns:** `Array<VkbButtonInfo>`

**Example:**
```javascript
const buttons = await invoke('parse_vkb_profile', {
    filePath: '/path/to/vkb_report_R.fp3',
    joystickId: 1
});
```

#### `clear_vkb_mappings()`
Clears all stored VKB mappings from state.

**Returns:** `void`

**Example:**
```javascript
await invoke('clear_vkb_mappings');
```

#### `get_vkb_mapping_count()`
Returns the number of VKB button mappings currently stored.

**Returns:** `number`

**Example:**
```javascript
const count = await invoke('get_vkb_mapping_count');
console.log(`${count} VKB mappings loaded`);
```

### Data Structures

#### `VkbButtonInfo`
```typescript
interface VkbButtonInfo {
    virtual_id: number;        // Virtual button ID (what SC sees)
    physical_id: number;       // Physical button ID
    physical_info: string;     // Description (e.g., "(A2)", "(Fire 1)")
    layer: string | null;      // "SHIFT1", "SHIFT2", or null
    tempo: string | null;      // "Short", "Long", "Double", or null
}
```

#### `Rebind` (Extended)
```typescript
interface Rebind {
    input: string;                      // e.g., "js1_button105"
    multi_tap?: number;
    activation_mode: string;
    vkb_layer?: string;                 // VKB layer info (not saved to XML)
    vkb_tempo?: string;                 // TEMPO info (not saved to XML)
    vkb_physical_id?: number;           // Physical button ID
    vkb_physical_info?: string;         // Physical button description
}
```

## Notes

- VKB fields (`vkb_layer`, `vkb_tempo`, etc.) are **not serialized** to XML when exporting keybindings
- VKB mappings are stored in memory only; they're cleared when the app restarts
- Multiple joysticks can be imported; their mappings are merged in state
- Importing the same joystick twice will update/overwrite its mappings
