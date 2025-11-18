use hidapi::HidApi;
use serde::Serialize;
use std::collections::HashMap;
use std::ffi::CString;
use hut::Usage;
use hidreport::{ReportDescriptor, Field, Report};

#[derive(Serialize, Clone, Debug)]
pub struct HidDeviceListItem {
    pub vendor_id: u16,
    pub product_id: u16,
    pub serial_number: Option<String>,
    pub manufacturer: Option<String>,
    pub product: Option<String>,
    pub path: String,
    pub interface_number: i32,
}

#[derive(Serialize, Clone, Debug)]
pub struct HidAxisReport {
    pub axis_values: HashMap<u32, u16>, // axis_id -> raw value (0-65535 for 16-bit, 0-255 for 8-bit)
    pub axis_bit_depths: HashMap<u32, u8>, // axis_id -> detected bit depth (8, 10, 11, 12, 16, etc.)
    pub axis_names: HashMap<u32, String>, // axis_id -> HID usage name (e.g., "X", "Y", "Rz")
    pub timestamp_ms: u64,
    pub is_16bit: bool, // Indicates if values are 16-bit (true) or 8-bit (false)
}

/// List all HID devices that appear to be game controllers
pub fn list_hid_game_controllers() -> Result<Vec<HidDeviceListItem>, String> {
    let api = HidApi::new().map_err(|e| format!("Failed to initialize HID API: {}", e))?;
    
    let devices: Vec<HidDeviceListItem> = api
        .device_list()
        .filter(|info| {
            // Filter for devices that are likely game controllers
            // HID Usage Page 0x01 = Generic Desktop Controls
            // HID Usage 0x04 = Joystick, 0x05 = Gamepad
            let usage_page = info.usage_page();
            let usage = info.usage();
            
            // Check if it's a joystick (0x04) or gamepad (0x05)
            usage_page == 0x01 && (usage == 0x04 || usage == 0x05)
        })
        .map(|info| HidDeviceListItem {
            vendor_id: info.vendor_id(),
            product_id: info.product_id(),
            serial_number: info.serial_number().map(|s| s.to_string()),
            manufacturer: info.manufacturer_string().map(|s| s.to_string()),
            product: info.product_string().map(|s| s.to_string()),
            path: info.path().to_string_lossy().to_string(),
            interface_number: info.interface_number(),
        })
        .collect();
    
    eprintln!("[HID] Found {} game controller devices", devices.len());
    
    Ok(devices)
}

/// Open a HID device and read a single report
pub fn read_hid_report(device_path: &str, timeout_ms: i32) -> Result<Vec<u8>, String> {
    let api = HidApi::new().map_err(|e| format!("Failed to initialize HID API: {}", e))?;
    
    // Convert Rust string to CString for hidapi
    let c_path = CString::new(device_path)
        .map_err(|e| format!("Invalid device path (contains null byte): {}", e))?;
    
    let device = api
        .open_path(&c_path)
        .map_err(|e| format!("Failed to open HID device: {}", e))?;
    
    let mut buf = [0u8; 256];
    let len = device
        .read_timeout(&mut buf, timeout_ms)
        .map_err(|e| format!("Failed to read from HID device: {}", e))?;
    
    if len > 0 {
        eprintln!("[HID] Read {} bytes from device", len);
        eprintln!("[HID] Raw report: {:?}", &buf[..len]);
        
        // Print first 16 bytes with positions for easier analysis
        if len >= 16 {
            eprint!("[HID] Bytes 0-15:  ");
            for i in 0..16 {
                eprint!("{:02X} ", buf[i]);
            }
            eprintln!();
            eprint!("[HID] Positions:   ");
            for i in 0..16 {
                eprint!("{:2} ", i);
            }
            eprintln!();
        }
    }
    
    Ok(buf[..len].to_vec())
}

/// Parse a HID report to extract axis values using proper descriptor parsing
/// This version uses the hidreport crate for accurate field extraction
pub fn parse_hid_axes_with_descriptor(report: &[u8], device_path: &str) -> Result<HidAxisReport, String> {
    let api = HidApi::new().map_err(|e| format!("Failed to initialize HID API: {}", e))?;
    
    let c_path = CString::new(device_path)
        .map_err(|e| format!("Invalid device path: {}", e))?;
    
    let device = api.open_path(&c_path)
        .map_err(|e| format!("Failed to open device: {}", e))?;
    
    // Get the report descriptor
    let mut descriptor_buf = vec![0u8; 4096];
    let descriptor_len = device.get_report_descriptor(&mut descriptor_buf)
        .map_err(|e| format!("Failed to get report descriptor: {}", e))?;
    
    let descriptor = &descriptor_buf[..descriptor_len];
    
    // Parse the descriptor
    let rdesc = ReportDescriptor::try_from(descriptor)
        .map_err(|e| format!("Failed to parse report descriptor: {:?}", e))?;
    
    // Find the matching input report
    let input_report = rdesc.find_input_report(report)
        .ok_or("No matching input report found")?;
    
    let mut axis_values = HashMap::new();
    let mut axis_bit_depths = HashMap::new();
    let mut axis_names = HashMap::new();
    let mut axis_index: u32 = 1;
    let mut max_bits = 8;
    
    // Extract values from each field
    for field in input_report.fields() {
        match field {
            Field::Variable(var) => {
                // Extract the value based on bit size
                let bits = var.bits.end - var.bits.start; // Range to size
                max_bits = max_bits.max(bits);
                
                // The extract method returns a FieldValue which can be converted to u32
                match var.extract(report) {
                    Ok(field_value) => {
                        // FieldValue implements Into<u32>
                        let value: u32 = field_value.into();
                        let value_u16 = value.min(u16::MAX as u32) as u16;
                        axis_values.insert(axis_index, value_u16);
                        axis_bit_depths.insert(axis_index, bits as u8);
                        
                        let usage_u32: u32 = var.usage.into();
                        if let Some(name) = get_axis_name_from_usage(usage_u32) {
                            axis_names.insert(axis_index, name.clone());
                            eprintln!("[HID] Axis {}: {} = {} ({} bits)", axis_index, name, value, bits);
                        }
                        
                        axis_index += 1;
                    }
                    Err(e) => {
                        eprintln!("[HID] Failed to extract axis value: {:?}", e);
                    }
                }
            }
            Field::Array(_arr) => {
                // Array fields are typically buttons, skip for axis extraction
            }
            Field::Constant(_) => {
                // Padding, skip
            }
        }
    }
    
    let is_16bit = max_bits > 8;
    
    Ok(HidAxisReport {
        axis_values,
        axis_bit_depths,
        axis_names,
        timestamp_ms: current_time_ms(),
        is_16bit,
    })
}

/// Parse a HID report to extract axis values (LEGACY VERSION - simpler but less accurate)
/// This is a generic parser that attempts to identify axis data
/// The actual structure will vary by device
pub fn parse_hid_axes(report: &[u8], axis_names: &HashMap<u32, String>) -> Result<HidAxisReport, String> {
    let mut axis_values = HashMap::new();
    
    if report.is_empty() {
        return Ok(HidAxisReport {
            axis_values,
            axis_bit_depths: HashMap::new(),
            axis_names: axis_names.clone(),
            timestamp_ms: current_time_ms(),
            is_16bit: false,
        });
    }
    
    eprintln!("[HID] Parsing report of {} bytes", report.len());
    
    // Common HID joystick report structure:
    // Byte 0: Report ID (often 0x01 or 0x00)
    // Bytes 1-N: Button data (packed bits, typically 1-4 bytes)
    // Bytes N+1 onwards: Axis data (1 or 2 bytes per axis)
    
    let report_id = report[0];
    eprintln!("[HID] Report ID: 0x{:02X}", report_id);
    
    // For VKB devices, the structure appears to be:
    // Byte 0: Report ID
    // Bytes 1-62: Axis data (31 axes * 2 bytes each = 62 bytes)
    // Byte 63: Padding or buttons
    // So we use bytes 1-62 (62 bytes, which is even)
    let axis_start_offset = 1;
    let axis_end_offset = if report.len() >= 63 { 63 } else { report.len() };
    let remaining = &report[axis_start_offset..axis_end_offset];
    
    eprintln!("[HID] Axis data starts at offset {}, {} bytes remaining", axis_start_offset, remaining.len());
    eprintln!("[HID] Raw axis bytes: {:02X?}", remaining);
    
    // Try to detect if axes are 16-bit or 8-bit
    // VKB devices use 16-bit axes, so we need to detect this intelligently
    // Check if we have enough data and if interpreting as 16-bit makes sense
    let is_16bit = detect_16bit_axes(remaining);
    eprintln!("[HID] Detected mode: {}", if is_16bit { "16-bit" } else { "8-bit" });
    
    let mut axis_bit_depths = HashMap::new();
    
    if is_16bit {
        // Parse as 16-bit little-endian values
        for i in (0..remaining.len()).step_by(2) {
            if i + 1 < remaining.len() {
                let axis_id = (i / 2 + 1) as u32; // 1-based axis numbering
                let value = u16::from_le_bytes([remaining[i], remaining[i + 1]]);
                
                // Detect bit depth for this axis value
                let bit_depth = detect_bit_depth(value);
                
                // Include all axes in 16-bit mode
                axis_values.insert(axis_id, value);
                axis_bit_depths.insert(axis_id, bit_depth);
                eprintln!("[HID] Axis {} (16-bit): {} ({}-bit precision, bytes: 0x{:02X} 0x{:02X})", 
                         axis_id, value, bit_depth, remaining[i], remaining[i + 1]);
            }
        }
    } else {
        // Parse as 8-bit values
        for (i, &byte_value) in remaining.iter().enumerate() {
            let axis_id = (i + 1) as u32; // 1-based axis numbering
            
            // Only include axes that have non-zero or varying values in 8-bit mode
            if byte_value > 0 || i < 8 {
                axis_values.insert(axis_id, byte_value as u16);
                axis_bit_depths.insert(axis_id, 8);
                eprintln!("[HID] Axis {} (8-bit): {}", axis_id, byte_value);
            }
        }
    }
    
    Ok(HidAxisReport {
        axis_values,
        axis_bit_depths,
        axis_names: axis_names.clone(),
        timestamp_ms: current_time_ms(),
        is_16bit,
    })
}

/// Detect the effective bit depth of an axis value
/// This determines how many bits are actually being used
/// Examples: 
///   - Value 4095 (0x0FFF) = 12-bit
///   - Value 2047 (0x07FF) = 11-bit  
///   - Value 1023 (0x03FF) = 10-bit
fn detect_bit_depth(value: u16) -> u8 {
    // Find the position of the highest set bit
    if value == 0 {
        return 1; // Edge case: value is 0
    }
    
    // Count the number of bits needed to represent this value
    let bits_needed = 16 - value.leading_zeros() as u8;
    
    // Common bit depths for joystick axes are 8, 10, 11, 12, 16
    // Round up to the nearest common bit depth
    match bits_needed {
        0..=8 => 8,
        9..=10 => 10,
        11 => 11,
        12 => 12,
        13..=16 => 16,
        _ => 16,
    }
}

/// Detect if the remaining bytes should be interpreted as 16-bit or 8-bit axes
/// This checks for patterns that indicate 16-bit values
fn detect_16bit_axes(remaining: &[u8]) -> bool {
    // Need at least 4 bytes (2 axes worth) to make a determination
    if remaining.len() < 4 {
        eprintln!("[HID] Too few bytes ({}) for 16-bit detection, defaulting to 8-bit", remaining.len());
        return false;
    }
    
    // VKB devices and most modern joysticks use 16-bit axes
    // If we have 8 or more bytes of axis data and even count, assume 16-bit
    if remaining.len() >= 8 && remaining.len() % 2 == 0 {
        eprintln!("[HID] {} bytes (even count >= 8), assuming 16-bit", remaining.len());
        return true;
    }
    
    // Even if less than 8 bytes, if we have an even count >= 4, try 16-bit
    if remaining.len() >= 4 && remaining.len() % 2 == 0 {
        eprintln!("[HID] {} bytes (even count >= 4), assuming 16-bit", remaining.len());
        return true;
    }
    
    eprintln!("[HID] Odd byte count or too few bytes, defaulting to 8-bit");
    false
}
/// More intelligent axis parser that tracks changes over time
/// This helps identify which bytes are actually axes vs static data
pub struct HidAxisTracker {
    previous_report: Vec<u8>,
    axis_change_counts: HashMap<usize, u32>, // byte_index -> number of times it changed
}

impl HidAxisTracker {
    pub fn new() -> Self {
        HidAxisTracker {
            previous_report: Vec::new(),
            axis_change_counts: HashMap::new(),
        }
    }
    
    pub fn process_report(&mut self, report: &[u8]) -> Result<HidAxisReport, String> {
        let mut axis_values = HashMap::new();
        
        if !self.previous_report.is_empty() && report.len() == self.previous_report.len() {
            // Compare with previous report to identify changing bytes
            for (i, (&current, &previous)) in report.iter().zip(self.previous_report.iter()).enumerate() {
                if current != previous {
                    let count = self.axis_change_counts.entry(i).or_insert(0);
                    *count += 1;
                    
                    eprintln!("[HID] Byte {} changed: {} -> {} (changes: {})", 
                              i, previous, current, count);
                }
            }
        }
        
        // Identify bytes that have changed frequently (likely axes)
        // and extract their current values
        let axis_start = 5.min(report.len()); // Skip report ID and button bytes
        
        for i in axis_start..report.len() {
            let change_count = self.axis_change_counts.get(&i).unwrap_or(&0);
            
            // If this byte has changed at least once, consider it an axis
            if *change_count > 0 || i < axis_start + 8 {
                let axis_id = (i - axis_start + 1) as u32;
                axis_values.insert(axis_id, report[i] as u16);
            }
        }
        
        self.previous_report = report.to_vec();
        
        let mut axis_bit_depths = HashMap::new();
        for axis_id in axis_values.keys() {
            axis_bit_depths.insert(*axis_id, 8u8);
        }
        
        Ok(HidAxisReport {
            axis_values,
            axis_bit_depths,
            axis_names: HashMap::new(), // Tracker doesn't have access to descriptor
            timestamp_ms: current_time_ms(),
            is_16bit: false, // Tracker mode assumes 8-bit for change detection
        })
    }
    
    /// Get a report of which bytes have changed and how often
    pub fn get_change_report(&self) -> HashMap<usize, u32> {
        self.axis_change_counts.clone()
    }
}

fn current_time_ms() -> u64 {
    use std::time::{SystemTime, UNIX_EPOCH};
    
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_millis() as u64
}

/// Get HID axis names from the device's report descriptor using proper HID parsing libraries
/// Returns a mapping of axis index -> axis name (e.g., "X", "Y", "Rz", "Slider")
pub fn get_axis_names_from_descriptor(device_path: &str) -> Result<HashMap<u32, String>, String> {
    let api = HidApi::new().map_err(|e| format!("Failed to initialize HID API: {}", e))?;
    
    let c_path = CString::new(device_path)
        .map_err(|e| format!("Invalid device path: {}", e))?;
    
    let device = api.open_path(&c_path)
        .map_err(|e| format!("Failed to open device: {}", e))?;
    
    // Get the report descriptor (max 4096 bytes for HID descriptors)
    let mut descriptor_buf = vec![0u8; 4096];
    let descriptor_len = device.get_report_descriptor(&mut descriptor_buf)
        .map_err(|e| format!("Failed to get report descriptor: {}", e))?;
    
    let descriptor = &descriptor_buf[..descriptor_len];
    eprintln!("[HID] Report descriptor length: {} bytes", descriptor.len());
    
    // Parse the descriptor using the hidreport crate
    parse_hid_descriptor_with_library(descriptor)
}

/// Parse HID report descriptor using the hidreport crate to extract axis names
/// This replaces our manual parsing with proper library-based parsing
fn parse_hid_descriptor_with_library(descriptor: &[u8]) -> Result<HashMap<u32, String>, String> {
    let mut axis_names = HashMap::new();
    
    // Parse the report descriptor
    let rdesc = ReportDescriptor::try_from(descriptor)
        .map_err(|e| format!("Failed to parse report descriptor: {:?}", e))?;
    
    eprintln!("[HID] Successfully parsed report descriptor");
    eprintln!("[HID] Input reports: {}", rdesc.input_reports().len());
    eprintln!("[HID] Output reports: {}", rdesc.output_reports().len());
    eprintln!("[HID] Feature reports: {}", rdesc.feature_reports().len());
    
    let mut axis_index: u32 = 1;
    
    // Iterate through all input reports
    for report in rdesc.input_reports() {
        eprintln!("[HID] Processing input report with ID: {:?}", report.report_id());
        eprintln!("[HID] Report has {} fields", report.fields().len());
        
        // Iterate through all fields in the report
        for field in report.fields() {
            match field {
                Field::Variable(var) => {
                    // Variable fields are typically axes
                    let bits = var.bits.end - var.bits.start; // Range to size
                    let usage_u32: u32 = var.usage.into();
                    
                    eprintln!("[HID] Variable field: {} bits, usage: 0x{:08X}", bits, usage_u32);
                    
                    // Try to get a human-readable name for this usage
                    if let Some(axis_name) = get_axis_name_from_usage(usage_u32) {
                        eprintln!("[HID] Found axis {}: {} ({} bits)", axis_index, axis_name, bits);
                        axis_names.insert(axis_index, axis_name);
                        axis_index += 1;
                    } else {
                        eprintln!("[HID] Unknown usage: 0x{:08X}", usage_u32);
                    }
                }
                Field::Array(arr) => {
                    // Array fields are typically buttons
                    let bits = arr.bits.end - arr.bits.start;
                    eprintln!("[HID] Array field: {} bits", bits);
                }
                Field::Constant(_) => {
                    // Constant fields are padding
                    eprintln!("[HID] Constant (padding) field");
                }
            }
        }
    }
    
    eprintln!("[HID] Total axes found: {}", axis_names.len());
    
    Ok(axis_names)
}

/// Convert a HID usage to a human-readable axis name using the hut crate
fn get_axis_name_from_usage(usage: u32) -> Option<String> {
    // Try to parse the usage
    match Usage::try_from(usage) {
        Ok(Usage::GenericDesktop(gd)) => {
            // Use the Debug format which gives us the enum name
            let name = format!("{:?}", gd);
            Some(name)
        }
        Ok(Usage::Button(btn)) => {
            // Button usage
            Some(format!("Button {}", btn))
        }
        Ok(other) => {
            // Other usage types (no Simulation in hut v0.4)
            Some(format!("{:?}", other))
        }
        Err(_) => {
            // Unknown or vendor-specific usage
            let usage_page = (usage >> 16) as u16;
            let usage_id = usage as u16;
            Some(format!("Usage(0x{:04X}:0x{:04X})", usage_page, usage_id))
        }
    }
}

/// Parse HID report descriptor to extract axis names (LEGACY - kept for reference)
/// HID descriptors are a series of items with tags, types, and data
fn parse_hid_descriptor_legacy(descriptor: &[u8]) -> Result<HashMap<u32, String>, String> {
    let mut axis_names = HashMap::new();
    let mut usage_page: u16 = 0;
    let mut current_axis_index: u32 = 1; // Sequential axis numbering
    let mut in_input_collection = false;
    
    let mut i = 0;
    while i < descriptor.len() {
        let item = descriptor[i];
        
        // Parse item header
        let size = match item & 0x03 {
            0 => 0,
            1 => 1,
            2 => 2,
            3 => 4,
            _ => 0,
        };
        
        let tag = (item >> 4) & 0x0F;
        let item_type = (item >> 2) & 0x03;
        
        // Extract data if present
        let data = if size > 0 && i + size < descriptor.len() {
            let mut val: u32 = 0;
            for j in 0..size {
                val |= (descriptor[i + 1 + j] as u32) << (j * 8);
            }
            Some(val)
        } else {
            None
        };
        
        // Main items (type 0)
        if item_type == 0 {
            match tag {
                0x08 => { // Input
                    in_input_collection = true;
                }
                0x09 => { // Output
                    in_input_collection = false;
                }
                _ => {}
            }
        }
        // Global items (type 1)
        else if item_type == 1 {
            match tag {
                0x00 => { // Usage Page
                    if let Some(page) = data {
                        usage_page = page as u16;
                        eprintln!("[HID] Usage Page: 0x{:04X}", usage_page);
                    }
                }
                _ => {}
            }
        }
        // Local items (type 2)
        else if item_type == 2 {
            match tag {
                0x00 => { // Usage
                    if let Some(usage) = data {
                        if in_input_collection && usage_page == 0x01 { // Generic Desktop page
                            let axis_name = match usage {
                                0x30 => "X",
                                0x31 => "Y",
                                0x32 => "Z",
                                0x33 => "Rx",
                                0x34 => "Ry",
                                0x35 => "Rz",
                                0x36 => "Slider",
                                0x37 => "Dial",
                                0x38 => "Wheel",
                                0x39 => "Hat Switch",
                                _ => {
                                    eprintln!("[HID] Unknown usage: 0x{:02X}", usage);
                                    continue;
                                }
                            };
                            
                            eprintln!("[HID] Found axis {}: {}", current_axis_index, axis_name);
                            axis_names.insert(current_axis_index, axis_name.to_string());
                            current_axis_index += 1;
                        }
                    }
                }
                _ => {}
            }
        }
        
        i += 1 + size;
    }
    
    Ok(axis_names)
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_parse_simple_report() {
        // Simulate a simple HID report:
        // [ReportID, Buttons, X, Y, Z, Rx, Ry, Slider]
        let report = vec![0x01, 0x00, 128, 64, 255, 100, 200, 50];
        
        let result = parse_hid_axes(&report).unwrap();
        
        // Should extract axes starting from byte 2 (after report ID and buttons)
        assert!(result.axis_values.len() > 0);
    }
    
    #[test]
    fn test_axis_tracker() {
        let mut tracker = HidAxisTracker::new();
        
        let report1 = vec![0x01, 0x00, 128, 128, 128, 128, 128, 128];
        let report2 = vec![0x01, 0x00, 128, 128, 128, 150, 128, 128]; // Rx changed
        let report3 = vec![0x01, 0x00, 128, 128, 128, 150, 200, 128]; // Ry changed
        
        let _ = tracker.process_report(&report1);
        let _ = tracker.process_report(&report2);
        let result = tracker.process_report(&report3).unwrap();
        
        // Should detect that bytes at positions 5 (Rx) and 6 (Ry) are axes
        let changes = tracker.get_change_report();
        assert!(changes.get(&5).unwrap_or(&0) > &0); // Rx changed
        assert!(changes.get(&6).unwrap_or(&0) > &0); // Ry changed
    }
}
