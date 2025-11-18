# HID Raw Data Debugger

## Overview

This is a new debugging tool designed to help diagnose and fix issues with missing axes (Rx, Ry, Slider) that aren't being detected by the current Gilrs-based input system.

## Problem Statement

Your friend reported:
- 2x potentiometers showing as "Rx" and "Ry" values (0-255 range)
- 1x slider showing as "slider" (0-255 range)
- These show up in USB HID reports but aren't being detected by our current Gilrs implementation
- The throttle Z axis (also 0-255) **IS** detected, so the device is partially working

## What We've Built

### 1. **HID Raw Data Debugger Page** (`hid-debugger.html`)

A comprehensive debugging interface with:
- Device selection from all connected controllers
- Real-time axis value monitoring
- Raw event stream showing all HID reports
- Side-by-side comparison of Gilrs vs HID detection
- Visual indicators for missing axes

### 2. **Integration with Main App**

- Added new tab switcher in the Input Debugger section
- Two modes:
  - **Basic Debugger**: The existing Gilrs-based detection
  - **HID Raw Data**: New direct HID access for troubleshooting

### 3. **Backend Preparation**

Added `hidapi` crate to `Cargo.toml` for direct USB HID communication.

## Next Steps to Complete Implementation

### Phase 1: Implement HID Reading in Rust

We need to add HID reading functions to `src-tauri/src/directinput.rs`:

```rust
use hidapi::{HidApi, HidDevice};

// New function to enumerate HID devices
pub fn list_hid_devices() -> Result<Vec<HidDeviceInfo>, String> {
    let api = HidApi::new().map_err(|e| e.to_string())?;
    
    let devices: Vec<HidDeviceInfo> = api
        .device_list()
        .map(|info| HidDeviceInfo {
            vendor_id: info.vendor_id(),
            product_id: info.product_id(),
            serial_number: info.serial_number().map(|s| s.to_string()),
            manufacturer: info.manufacturer_string().map(|s| s.to_string()),
            product: info.product_string().map(|s| s.to_string()),
            path: info.path().to_string_lossy().to_string(),
        })
        .collect();
    
    Ok(devices)
}

// New function to read raw HID report
pub fn read_hid_report(device_path: &str) -> Result<Vec<u8>, String> {
    let api = HidApi::new().map_err(|e| e.to_string())?;
    let device = api.open_path(device_path).map_err(|e| e.to_string())?;
    
    let mut buf = [0u8; 256];
    let len = device.read_timeout(&mut buf, 50).map_err(|e| e.to_string())?;
    
    Ok(buf[..len].to_vec())
}

// Parse HID report to extract axis values
pub fn parse_hid_axes(report: &[u8]) -> Result<HashMap<u32, u8>, String> {
    // This is device-specific and needs to be figured out by analyzing the reports
    // Common HID report structure for joysticks:
    // - First byte: Report ID
    // - Next bytes: Button states (packed bits)
    // - Following bytes: Axis values (1-2 bytes each)
    
    let mut axes = HashMap::new();
    
    // Example parsing (will need adjustment based on actual device):
    // Assuming axes start at byte 2, 1 byte each
    if report.len() >= 8 {
        axes.insert(1, report[2]); // X
        axes.insert(2, report[3]); // Y
        axes.insert(3, report[4]); // Z
        axes.insert(4, report[5]); // Rx
        axes.insert(5, report[6]); // Ry
        axes.insert(6, report[7]); // Slider
    }
    
    Ok(axes)
}
```

### Phase 2: Add Tauri Commands

In `src-tauri/src/lib.rs`, add:

```rust
#[tauri::command]
async fn list_hid_devices_cmd() -> Result<Vec<HidDeviceInfo>, String> {
    directinput::list_hid_devices()
}

#[tauri::command]
async fn poll_hid_device(device_path: String) -> Result<HashMap<u32, u8>, String> {
    let report = directinput::read_hid_report(&device_path)?;
    directinput::parse_hid_axes(&report)
}
```

### Phase 3: Update Frontend

The JavaScript in `hid-debugger.js` is already set up to call these functions:
- `list_hid_devices_cmd()` - Called when selecting device
- `poll_hid_device()` - Called in polling loop

### Phase 4: Testing & Analysis

1. **Run the HID debugger** and select your friend's device
2. **Watch the raw event stream** to see all axis values in real-time
3. **Move Rx, Ry, and Slider** to identify which bytes change
4. **Compare with Gilrs detection** to see what's missing
5. **Adjust parsing logic** based on observed patterns

### Phase 5: Integration

Once we know which axes are missing:

1. **Extend Gilrs detection** to also poll HID directly for missing axes
2. **Add fallback logic** in `detect_axis_movement_for_device()`:
   ```rust
   // Try Gilrs first
   let gilrs_result = poll_gilrs_for_axis(...);
   
   // If Gilrs doesn't detect it, try HID directly
   if gilrs_result.is_none() {
       let hid_result = poll_hid_for_axis(...);
       return hid_result;
   }
   ```

3. **Update axis mapping** to include Rx, Ry, Slider in the standard axis list

## Technical Notes

### Why Gilrs Might Miss Some Axes

Gilrs uses OS-level gamepad APIs which sometimes:
- Only expose "standard" gamepad axes (X, Y, Z, RotX, RotY, RotZ)
- Miss extended axes like Rx, Ry, extra sliders
- Depend on the device's HID descriptor being properly formatted

### HID Report Structure

USB HID joysticks typically report data as:
```
[Report ID] [Buttons...] [Axis1] [Axis2] ... [AxisN]
```

The exact structure depends on the device's HID descriptor. Common patterns:
- **8-bit axes**: 0-255 range (what your friend is seeing)
- **16-bit axes**: 0-65535 range (higher precision)
- **Packed buttons**: Multiple buttons in each byte (bitfield)

### Device Identification

Match devices between Gilrs and HID using:
- Vendor ID (VID)
- Product ID (PID)
- Serial number (if available)

## Files Created/Modified

### New Files
- `src/hid-debugger.html` - HID debugger UI
- `src/hid-debugger.css` - Styling for HID debugger
- `src/hid-debugger.js` - Frontend logic for HID debugging
- `HID_DEBUGGER_README.md` - This file

### Modified Files
- `src/index.html` - Added HID debugger tab switcher
- `src/main.js` - Added tab switching logic
- `src/styles.css` - Added button active state styling
- `src-tauri/Cargo.toml` - Added `hidapi` dependency

## Testing Checklist

- [ ] Compile with new `hidapi` dependency
- [ ] Implement HID reading functions in Rust
- [ ] Add Tauri commands to expose HID functions
- [ ] Test device enumeration
- [ ] Test raw report reading
- [ ] Identify Rx, Ry, Slider byte positions
- [ ] Update parsing logic
- [ ] Verify all axes detected
- [ ] Integrate with main detection system
- [ ] Test with various devices

## Resources

- **hidapi documentation**: https://docs.rs/hidapi/
- **USB HID Usage Tables**: https://usb.org/sites/default/files/hut1_21_0.pdf
- **Gilrs documentation**: https://docs.rs/gilrs/
- **HID Report Descriptor Tool**: https://eleccelerator.com/usbdescreqparser/

## Support

If you encounter issues:
1. Check the raw HID reports in the debugger
2. Compare with Gilrs detection side-by-side
3. Look for patterns when moving specific axes
4. Post findings on GitHub for collaborative debugging
