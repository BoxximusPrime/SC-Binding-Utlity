use gilrs::{Button, EventType, Gilrs};
use once_cell::sync::Lazy;
use rusty_xinput::XInputHandle;
use serde::Serialize;
use std::sync::Mutex;
use std::thread;
use std::time::{Duration, Instant};
use tauri::Emitter;

use crate::device_database;

// Global Gilrs instance for axis detection to avoid recreating it on every poll
static GILRS_INSTANCE: Lazy<Mutex<Option<Gilrs>>> = Lazy::new(|| Mutex::new(None));

/// Determine if a device is a gamepad (Xbox-style controller) or a joystick (HOTAS/flight stick)
/// Based on the device name and button/axis count
fn get_friendly_device_name(gamepad: &gilrs::Gamepad) -> String {
    // Try to get SDL mapping name first (most friendly)
    if let Some(map_name) = gamepad.map_name() {
        return map_name.to_string();
    }
    
    // Try to look up device in the database by VID/PID
    if let (Some(vendor_id), Some(product_id)) = (gamepad.vendor_id(), gamepad.product_id()) {
        eprintln!("[DirectInput] Looking up device: VID=0x{:04x}, PID=0x{:04x}", vendor_id, product_id);
        if let Some(device_entry) = device_database::DeviceDatabase::lookup_device(vendor_id as u32, product_id as u32) {
            eprintln!("[DirectInput] Device found in database: {}", device_entry.name);
            return device_entry.name;
        }
        eprintln!("[DirectInput] Device not found in database, using fallback vendor lookup");


        // Fallback for unknown devices: generic name based on vendor
        match vendor_id {
            0x231d => {
                let os_name = gamepad.name();
                if os_name.to_lowercase().contains("hid") {
                    return format!("VKB Sim Device (PID: 0x{:04x})", product_id);
                }
                return os_name.to_string();
            }
            0x044f => {
                let os_name = gamepad.name();
                if os_name.to_lowercase().contains("hid") {
                    return format!("Thrustmaster (PID: 0x{:04x})", product_id);
                }
                return os_name.to_string();
            }
            0x06a3 => {
                let os_name = gamepad.name();
                if os_name.to_lowercase().contains("hid") {
                    return format!("Saitek/Mad Catz (PID: 0x{:04x})", product_id);
                }
                return os_name.to_string();
            }
            0x068e => {
                let os_name = gamepad.name();
                if os_name.to_lowercase().contains("hid") {
                    return format!("CH Products (PID: 0x{:04x})", product_id);
                }
                return os_name.to_string();
            }
            0x3344 => {
                let os_name = gamepad.name();
                if os_name.to_lowercase().contains("hid") {
                    return format!("VirPil (PID: 0x{:04x})", product_id);
                }
                return os_name.to_string();
            }
            _ => {}
        }
    }
    
    // Fall back to regular name (which may still be "HID-compliant game controller")
    gamepad.name().to_string()
}

fn is_gamepad(name: &str, _gamepad: &gilrs::Gamepad) -> bool {
    let name_lower = name.to_lowercase();

    eprintln!("is_gamepad: Checking device: '{}'", name);

    // Common joystick/HOTAS identifiers - CHECK THESE FIRST to avoid misidentification
    let joystick_indicators = [
        "joystick",
        "hotas",
        "throttle",
        "gladiator",
        "warthog",
        "t16000",
        "vkb",
        "vkbsim",
        "virpil",
        "thrustmaster",
        "saitek",
        "x52",
        "x56",
    ];

    // If it has a joystick indicator, it's definitely NOT a gamepad
    if joystick_indicators
        .iter()
        .any(|indicator| name_lower.contains(indicator))
    {
        eprintln!("is_gamepad: '{}' identified as JOYSTICK", name);
        return false;
    }

    // Common gamepad identifiers in device names
    let gamepad_indicators = [
        "xbox",
        "playstation",
        "dualshock",
        "dualsense",
        "ps3",
        "ps4",
        "ps5",
        "controller for windows", // Xbox 360/One Controller for Windows
        "gamepad",
        "xinput",
    ];

    // Check if name contains any gamepad indicators
    if gamepad_indicators
        .iter()
        .any(|indicator| name_lower.contains(indicator))
    {
        eprintln!("is_gamepad: '{}' identified as GAMEPAD", name);
        return true;
    }

    // Generic devices that don't match either pattern default to JOYSTICK
    eprintln!(
        "is_gamepad: '{}' defaulting to JOYSTICK (generic device)",
        name
    );
    false
}

// Get currently pressed modifiers using Windows API
#[cfg(windows)]
fn get_active_modifiers() -> Vec<String> {
    use windows::Win32::UI::Input::KeyboardAndMouse::{
        GetAsyncKeyState, VK_LCONTROL, VK_LMENU, VK_LSHIFT, VK_RCONTROL, VK_RMENU, VK_RSHIFT,
    };

    let mut modifiers = Vec::new();

    unsafe {
        // Check if the high-order bit is set (key is pressed)
        // GetAsyncKeyState returns a SHORT where the most significant bit indicates current state
        if GetAsyncKeyState(VK_LMENU.0 as i32) as u16 & 0x8000 != 0 {
            modifiers.push("LALT".to_string());
        }
        if GetAsyncKeyState(VK_RMENU.0 as i32) as u16 & 0x8000 != 0 {
            modifiers.push("RALT".to_string());
        }
        if GetAsyncKeyState(VK_LCONTROL.0 as i32) as u16 & 0x8000 != 0 {
            modifiers.push("LCTRL".to_string());
        }
        if GetAsyncKeyState(VK_RCONTROL.0 as i32) as u16 & 0x8000 != 0 {
            modifiers.push("RCTRL".to_string());
        }
        if GetAsyncKeyState(VK_LSHIFT.0 as i32) as u16 & 0x8000 != 0 {
            modifiers.push("LSHIFT".to_string());
        }
        if GetAsyncKeyState(VK_RSHIFT.0 as i32) as u16 & 0x8000 != 0 {
            modifiers.push("RSHIFT".to_string());
        }
    }

    modifiers
}

#[derive(Serialize, Clone, Debug)]
pub struct AxisMovement {
    pub axis_id: u32,
    pub value: f32,
}

// Stub for non-Windows platforms
#[cfg(not(windows))]
fn get_active_modifiers() -> Vec<String> {
    Vec::new()
}

#[derive(Serialize, Clone, Debug)]
pub struct DetectedInput {
    pub input_string: String, // Star Citizen format like "js1_button3", "js1_hat1_up", or "js1_axis1_positive"
    pub display_name: String,
    pub device_type: String,
    pub axis_value: Option<f32>,     // Raw axis value if applicable
    pub modifiers: Vec<String>,      // Active modifiers: LALT, RALT, LCTRL, RCTRL, LSHIFT, RSHIFT
    pub is_modifier: bool,           // True if this input itself is a modifier key
    pub session_id: String, // Session ID to track which detection session this input belongs to
    pub device_uuid: Option<String>, // Unique device identifier for persistent mapping

    // Extended debug information from gilrs
    pub raw_axis_code: Option<String>, // Raw axis code from gilrs (e.g., "Axis::LeftStickX")
    pub raw_button_code: Option<String>, // Raw button code from gilrs
    pub raw_code_index: Option<u32>,   // Raw index from the Code debug representation
    pub device_name: Option<String>,   // Full device name
    pub device_gilrs_id: Option<usize>, // Internal gilrs device ID
    pub device_power_info: Option<String>, // Battery/power information if available
    pub device_is_ff_supported: Option<bool>, // Force feedback support
    pub all_device_axes: Option<Vec<String>>, // All available axes on this device
    pub all_device_buttons: Option<Vec<String>>, // All available buttons (estimated)
}

#[derive(Serialize, Clone, Debug)]
pub struct DetectionComplete {
    pub session_id: String,
}

#[derive(Serialize)]
pub struct JoystickInfo {
    pub id: usize,
    pub name: String,
    pub is_connected: bool,
    pub button_count: usize,
    pub axis_count: usize,
    pub hat_count: usize,
    pub device_type: String,
}

#[derive(Serialize, Clone, Debug)]
pub struct DeviceInfo {
    pub uuid: String,
    pub name: String,
    pub axis_count: usize,
    pub button_count: usize,
    pub hat_count: usize,
    pub device_type: String,
    pub is_connected: bool,
}

fn resolve_device_uuid(gamepad: &gilrs::Gamepad, fallback_id: usize) -> String {
    let raw = gamepad.uuid();
    if raw.iter().all(|b| *b == 0) {
        return format!("{}_{}", gamepad.name(), fallback_id);
    }

    let mut encoded = String::with_capacity(32);
    for byte in raw.iter() {
        encoded.push_str(&format!("{:02x}", byte));
    }
    encoded
}

fn resolve_xinput_uuid(controller_id: u32) -> String {
    // Create a consistent UUID for XInput controllers based on their slot
    format!("xinput_{}", controller_id)
}

fn extract_index_from_code(code: &gilrs::ev::Code) -> Option<u32> {
    let code_str = format!("{:?}", code);
    if let Some(start) = code_str.find("index: ") {
        let rest = &code_str[start + 7..];
        if let Some(end) = rest.find(' ') {
            return rest[..end].parse::<u32>().ok().map(|n| n + 1);
        }
    }
    None
}

/// Extract both the kind (Button/Axis) and index from the Code debug representation
/// Returns (is_axis, index) where is_axis is true for axes, false for buttons
fn extract_code_info(code: &gilrs::ev::Code) -> Option<(bool, u32)> {
    let code_str = format!("{:?}", code);
    
    // Determine if this is an axis or button by checking the kind field
    let is_axis = code_str.contains("kind: Axis");
    
    // Extract the index
    if let Some(start) = code_str.find("index: ") {
        let rest = &code_str[start + 7..];
        if let Some(end) = rest.find(' ') {
            if let Ok(index) = rest[..end].parse::<u32>() {
                return Some((is_axis, index + 1)); // +1 for 1-based indexing
            }
        }
    }
    None
}

use std::collections::HashMap;

// Axis state tracking to prevent duplicate detections
struct AxisState {
    last_value: f32,
    last_triggered_direction: Option<bool>, // true = positive, false = negative
}

/// Wait for joystick input using gilrs with hat detection and axis direction support
pub fn wait_for_input(
    session_id: String,
    timeout_secs: u64,
) -> Result<Option<DetectedInput>, String> {
    let mut gilrs = Gilrs::new().map_err(|e| e.to_string())?;

    eprintln!(
        "wait_for_input: Starting input detection for {} seconds",
        timeout_secs
    );
    eprintln!(
        "wait_for_input: Connected gamepads: {}",
        gilrs.gamepads().count()
    );

    // Initialize XInput for Xbox controller support
    let xinput =
        XInputHandle::load_default().map_err(|e| format!("Failed to load XInput: {:?}", e))?;
    let mut xinput_prev_states = [None, None, None, None]; // Track previous state for 4 possible controllers

    // Track XInput axis states (controller_id, axis_index) -> last triggered direction
    let mut xinput_axis_states: HashMap<(u32, u32), AxisState> = HashMap::new();

    // Initialize XInput states
    for i in 0..4 {
        if let Ok(state) = xinput.get_state(i) {
            xinput_prev_states[i as usize] = Some(state);
            eprintln!("wait_for_input: XInput controller {} initialized", i);

            // Initialize axis states for this controller
            // Normalize XInput values to -1.0 to 1.0 range
            let left_x = (state.raw.Gamepad.sThumbLX as f32) / 32768.0;
            let left_y = (state.raw.Gamepad.sThumbLY as f32) / 32768.0;
            let right_x = (state.raw.Gamepad.sThumbRX as f32) / 32768.0;
            let right_y = (state.raw.Gamepad.sThumbRY as f32) / 32768.0;
            let left_trigger = (state.raw.Gamepad.bLeftTrigger as f32) / 255.0;
            let right_trigger = (state.raw.Gamepad.bRightTrigger as f32) / 255.0;

            xinput_axis_states.insert(
                (i, 1),
                AxisState {
                    last_value: left_x,
                    last_triggered_direction: None,
                },
            );
            xinput_axis_states.insert(
                (i, 2),
                AxisState {
                    last_value: left_y,
                    last_triggered_direction: None,
                },
            );
            xinput_axis_states.insert(
                (i, 3),
                AxisState {
                    last_value: right_x,
                    last_triggered_direction: None,
                },
            );
            xinput_axis_states.insert(
                (i, 4),
                AxisState {
                    last_value: right_y,
                    last_triggered_direction: None,
                },
            );
            xinput_axis_states.insert(
                (i, 5),
                AxisState {
                    last_value: left_trigger * 2.0 - 1.0,
                    last_triggered_direction: None,
                },
            );
            xinput_axis_states.insert(
                (i, 6),
                AxisState {
                    last_value: right_trigger * 2.0 - 1.0,
                    last_triggered_direction: None,
                },
            );
        }
    }

    // Track axis states to prevent duplicate triggers
    // We'll initialize states dynamically as axes are moved to support any device
    let mut axis_states: HashMap<(usize, u32), AxisState> = HashMap::new();

    let start = Instant::now();
    let timeout = Duration::from_secs(timeout_secs);

    // Axis detection thresholds
    const AXIS_TRIGGER_THRESHOLD: f32 = 0.5; // 50% deflection to trigger
    const AXIS_RESET_THRESHOLD: f32 = 0.3; // 30% to reset (hysteresis)

    while start.elapsed() < timeout {
        // Process all available gilrs events (non-blocking)
        while let Some(event) = gilrs.next_event() {
            eprintln!("wait_for_input: Received event: {:?}", event);
            match event.event {
                EventType::ButtonPressed(button, code) => {
                    let joystick_id: usize = event.id.into();
                    let sc_instance = joystick_id + 1; // 1-based indexing for Star Citizen

                    // Get the gamepad to check if it's a gamepad or joystick
                    let gamepad = gilrs.gamepad(event.id);
                    let device_name = get_friendly_device_name(&gamepad);
                    let is_gp = is_gamepad(&device_name, &gamepad);
                    let device_prefix = if is_gp { "gp" } else { "js" };
                    let device_type_name = if is_gp { "Gamepad" } else { "Joystick" };

                    // Collect extended debug info
                    let raw_button_code = format!("{:?}", button);
                    let code_str = format!("{:?}", code);
                    let raw_code_index = if let Some(start) = code_str.find("index: ") {
                        let rest = &code_str[start + 7..];
                        if let Some(end) = rest.find(' ') {
                            rest[..end].parse::<u32>().ok()
                        } else {
                            None
                        }
                    } else {
                        None
                    };
                    let power_info = format!("{:?}", gamepad.power_info());
                    let is_ff = gamepad.is_ff_supported();

                    // First check if this is a known DPad button
                    let (input_string, display_name) = match button {
                        Button::DPadUp => (
                            format!("{}{}_hat1_up", device_prefix, sc_instance),
                            format!("{} {} - Hat 1 UP", device_type_name, sc_instance),
                        ),
                        Button::DPadDown => (
                            format!("{}{}_hat1_down", device_prefix, sc_instance),
                            format!("{} {} - Hat 1 DOWN", device_type_name, sc_instance),
                        ),
                        Button::DPadLeft => (
                            format!("{}{}_hat1_left", device_prefix, sc_instance),
                            format!("{} {} - Hat 1 LEFT", device_type_name, sc_instance),
                        ),
                        Button::DPadRight => (
                            format!("{}{}_hat1_right", device_prefix, sc_instance),
                            format!("{} {} - Hat 1 RIGHT", device_type_name, sc_instance),
                        ),
                        _ => {
                            // Regular button - extract the button index from the Code
                            // The Code debug format is: Code(EvCode { kind: Button, index: N })
                            // We need to parse out just the index number
                            let code_str = format!("{:?}", code);

                            // Extract button number from debug string like "Code(EvCode { kind: Button, index: 1 })"
                            let button_index = if let Some(start) = code_str.find("index: ") {
                                let rest = &code_str[start + 7..];
                                if let Some(end) = rest.find(' ') {
                                    rest[..end].parse::<u32>().unwrap_or(0) + 1 // +1 for 1-based indexing
                                } else {
                                    0
                                }
                            } else {
                                0
                            };

                            if button_index > 0 {
                                (
                                    format!(
                                        "{}{}_button{}",
                                        device_prefix, sc_instance, button_index
                                    ),
                                    format!(
                                        "{} {} - Button {}",
                                        device_type_name, sc_instance, button_index
                                    ),
                                )
                            } else {
                                // Fallback if parsing fails
                                (
                                    format!("{}{}_button_unknown", device_prefix, sc_instance),
                                    format!(
                                        "{} {} - Button Unknown",
                                        device_type_name, sc_instance
                                    ),
                                )
                            }
                        }
                    };

                    // Get device UUID for persistent mapping
                    let device_uuid = resolve_device_uuid(&gamepad, joystick_id);

                    return Ok(Some(DetectedInput {
                        input_string,
                        display_name,
                        device_type: device_type_name.to_string(),
                        axis_value: None,
                        modifiers: get_active_modifiers(),
                        is_modifier: false,
                        session_id: session_id.clone(),
                        device_uuid: Some(device_uuid.clone()),
                        raw_axis_code: None,
                        raw_button_code: Some(raw_button_code),
                        raw_code_index,
                        device_name: Some(device_name.to_string()),
                        device_gilrs_id: Some(joystick_id),
                        device_power_info: Some(power_info),
                        device_is_ff_supported: Some(is_ff),
                        all_device_axes: None,
                        all_device_buttons: None,
                    }));
                }
                EventType::AxisChanged(axis, value, code) => {
                    let joystick_id: usize = event.id.into();
                    let sc_instance = joystick_id + 1; // 1-based indexing for Star Citizen

                    // Get the gamepad to check if it's a gamepad or joystick
                    let gamepad = gilrs.gamepad(event.id);
                    let device_name = get_friendly_device_name(&gamepad);
                    let is_gp = is_gamepad(&device_name, &gamepad);
                    let device_prefix = if is_gp { "gp" } else { "js" };
                    let device_type_name = if is_gp { "Gamepad" } else { "Joystick" };

                    // Collect extended debug info
                    let raw_axis_code = format!("{:?}", axis);
                    let power_info = format!("{:?}", gamepad.power_info());
                    let is_ff = gamepad.is_ff_supported();

                    // Extract axis index and verify this is actually an axis event
                    if let Some((is_axis, axis_index)) = extract_code_info(&code) {
                        if !is_axis {
                            // Code says this is a button, not an axis - skip it
                            eprintln!("AxisChanged event but Code indicates Button (index {})", axis_index);
                            continue;
                        }
                    
                    if axis_index > 0 {
                        let axis_key = (joystick_id, axis_index);

                        // Get or create axis state
                        let state = axis_states.entry(axis_key).or_insert(AxisState {
                            last_value: value, // Initialize with current value instead of 0
                            last_triggered_direction: None,
                        });

                        // Calculate how much the axis has moved from its initial/last value
                        let movement_delta = (value - state.last_value).abs();

                        // Require significant movement from initial position (prevents false triggers)
                        const MOVEMENT_THRESHOLD: f32 = 0.3; // Require 30% movement from starting position

                        // Determine if this is a significant movement
                        let is_positive = value > AXIS_TRIGGER_THRESHOLD;
                        let is_negative = value < -AXIS_TRIGGER_THRESHOLD;
                        let is_centered = value.abs() < AXIS_RESET_THRESHOLD;
                        let has_moved_enough = movement_delta > MOVEMENT_THRESHOLD;

                        // Reset state if axis returns to center
                        if is_centered {
                            state.last_triggered_direction = None;
                            state.last_value = value;
                        }

                        // Only trigger if:
                        // 1. Axis moved beyond threshold
                        // 2. Axis has moved significantly from its starting position
                        // 3. This direction hasn't been triggered yet
                        let should_trigger_positive = is_positive
                            && has_moved_enough
                            && state.last_triggered_direction != Some(true);
                        let should_trigger_negative = is_negative
                            && has_moved_enough
                            && state.last_triggered_direction != Some(false);

                        if should_trigger_positive || should_trigger_negative {
                            let direction = if should_trigger_positive {
                                "positive"
                            } else {
                                "negative"
                            };
                            let direction_symbol = if should_trigger_positive { "+" } else { "-" };

                            // Update state
                            state.last_triggered_direction = Some(should_trigger_positive);
                            state.last_value = value;

                            // Get friendly axis name (using Star Citizen naming convention)
                            // Support more than 6 axes for advanced joysticks
                            let axis_name = match axis_index {
                                1 => "X",
                                2 => "Y",
                                3 => "RotX",
                                4 => "RotY",
                                5 => "Z",
                                6 => "RotZ",
                                _ => &format!("Axis{}", axis_index), // For axes beyond the standard 6
                            };

                            // Get device UUID
                            let device_uuid = resolve_device_uuid(&gamepad, joystick_id);

                            return Ok(Some(DetectedInput {
                                input_string: format!(
                                    "{}{}_axis{}_{}",
                                    device_prefix, sc_instance, axis_index, direction
                                ),
                                display_name: format!(
                                    "{} {} - {} {} (Axis {})",
                                    device_type_name,
                                    sc_instance,
                                    axis_name,
                                    direction_symbol,
                                    axis_index
                                ),
                                device_type: device_type_name.to_string(),
                                axis_value: Some(value),
                                modifiers: get_active_modifiers(),
                                is_modifier: false,
                                session_id: session_id.clone(),
                                device_uuid: Some(device_uuid),
                                raw_axis_code: Some(raw_axis_code),
                                raw_button_code: None,
                                raw_code_index: Some(axis_index),
                                device_name: Some(device_name.to_string()),
                                device_gilrs_id: Some(joystick_id),
                                device_power_info: Some(power_info),
                                device_is_ff_supported: Some(is_ff),
                                all_device_axes: None,
                                all_device_buttons: None,
                            }));
                        }
                    }
                    } // End of if let Some((is_axis, axis_index))
                }
                _ => {}
            }
        }

        // Poll XInput controllers for button presses and axis movements
        for controller_id in 0..4 {
            if let Ok(state) = xinput.get_state(controller_id) {
                if let Some(prev_state) = &xinput_prev_states[controller_id as usize] {
                    // Check if any button was newly pressed
                    let buttons_pressed =
                        state.raw.Gamepad.wButtons & !prev_state.raw.Gamepad.wButtons;

                    if buttons_pressed != 0 {
                        eprintln!(
                            "XInput controller {} button pressed: 0x{:04X}",
                            controller_id, buttons_pressed
                        );

                        // Find which button was pressed
                        let button_num = if buttons_pressed & 0x1000 != 0 {
                            1
                        }
                        // A
                        else if buttons_pressed & 0x2000 != 0 {
                            2
                        }
                        // B
                        else if buttons_pressed & 0x4000 != 0 {
                            3
                        }
                        // X
                        else if buttons_pressed & 0x8000 != 0 {
                            4
                        }
                        // Y
                        else if buttons_pressed & 0x0100 != 0 {
                            5
                        }
                        // LB
                        else if buttons_pressed & 0x0200 != 0 {
                            6
                        }
                        // RB
                        else if buttons_pressed & 0x0010 != 0 {
                            7
                        }
                        // Back
                        else if buttons_pressed & 0x0020 != 0 {
                            8
                        }
                        // Start
                        else if buttons_pressed & 0x0040 != 0 {
                            9
                        }
                        // LS (Left Stick Click)
                        else if buttons_pressed & 0x0080 != 0 {
                            10
                        }
                        // RS (Right Stick Click)
                        else if buttons_pressed & 0x0001 != 0 {
                            11
                        }
                        // DPad Up
                        else if buttons_pressed & 0x0002 != 0 {
                            12
                        }
                        // DPad Down
                        else if buttons_pressed & 0x0004 != 0 {
                            13
                        }
                        // DPad Left
                        else if buttons_pressed & 0x0008 != 0 {
                            14
                        }
                        // DPad Right
                        else {
                            0
                        };

                        if button_num > 0 {
                            let sc_instance = controller_id as usize + 1;
                            let device_uuid = resolve_xinput_uuid(controller_id);
                            // XInput is specifically for Xbox controllers, so use gp prefix
                            return Ok(Some(DetectedInput {
                                input_string: format!("gp{}_button{}", sc_instance, button_num),
                                display_name: format!(
                                    "Gamepad {} - Button {}",
                                    sc_instance, button_num
                                ),
                                device_type: "Gamepad".to_string(),
                                axis_value: None,
                                modifiers: get_active_modifiers(),
                                is_modifier: false,
                                session_id: session_id.clone(),
                                device_uuid: Some(device_uuid),
                                raw_axis_code: None,
                                raw_button_code: Some(format!(
                                    "XInput Button: 0x{:04X}",
                                    buttons_pressed
                                )),
                                raw_code_index: Some(button_num),
                                device_name: Some(format!("Xbox Controller (XInput {})", controller_id)),
                                device_gilrs_id: None,
                                device_power_info: None,
                                device_is_ff_supported: None,
                                all_device_axes: None,
                                all_device_buttons: None,
                            }));
                        }
                    }

                    // Check for axis movements
                    // Normalize XInput values to -1.0 to 1.0 range
                    let axes = [
                        (
                            1,
                            (state.raw.Gamepad.sThumbLX as f32) / 32768.0,
                            "Left Stick X",
                        ),
                        (
                            2,
                            (state.raw.Gamepad.sThumbLY as f32) / 32768.0,
                            "Left Stick Y",
                        ),
                        (
                            3,
                            (state.raw.Gamepad.sThumbRX as f32) / 32768.0,
                            "Right Stick X",
                        ),
                        (
                            4,
                            (state.raw.Gamepad.sThumbRY as f32) / 32768.0,
                            "Right Stick Y",
                        ),
                        (
                            5,
                            (state.raw.Gamepad.bLeftTrigger as f32) / 255.0 * 2.0 - 1.0,
                            "Left Trigger",
                        ),
                        (
                            6,
                            (state.raw.Gamepad.bRightTrigger as f32) / 255.0 * 2.0 - 1.0,
                            "Right Trigger",
                        ),
                    ];

                    for (axis_index, value, axis_name) in axes.iter() {
                        let axis_key = (controller_id, *axis_index);
                        let state_entry = xinput_axis_states.entry(axis_key).or_insert(AxisState {
                            last_value: *value,
                            last_triggered_direction: None,
                        });

                        let movement_delta = (value - state_entry.last_value).abs();
                        const MOVEMENT_THRESHOLD: f32 = 0.3;
                        const AXIS_TRIGGER_THRESHOLD: f32 = 0.5;
                        const AXIS_RESET_THRESHOLD: f32 = 0.3;

                        let is_positive = *value > AXIS_TRIGGER_THRESHOLD;
                        let is_negative = *value < -AXIS_TRIGGER_THRESHOLD;
                        let is_centered = value.abs() < AXIS_RESET_THRESHOLD;
                        let has_moved_enough = movement_delta > MOVEMENT_THRESHOLD;

                        if is_centered {
                            state_entry.last_triggered_direction = None;
                            state_entry.last_value = *value;
                        }

                        let should_trigger_positive = is_positive
                            && has_moved_enough
                            && state_entry.last_triggered_direction != Some(true);
                        let should_trigger_negative = is_negative
                            && has_moved_enough
                            && state_entry.last_triggered_direction != Some(false);

                        if should_trigger_positive || should_trigger_negative {
                            let direction = if should_trigger_positive {
                                "positive"
                            } else {
                                "negative"
                            };
                            let direction_symbol = if should_trigger_positive { "+" } else { "-" };

                            state_entry.last_triggered_direction = Some(should_trigger_positive);
                            state_entry.last_value = *value;

                            let sc_instance = controller_id as usize + 1;
                            let device_uuid = resolve_xinput_uuid(controller_id);
                            return Ok(Some(DetectedInput {
                                input_string: format!(
                                    "gp{}_axis{}_{}",
                                    sc_instance, axis_index, direction
                                ),
                                display_name: format!(
                                    "Gamepad {} - {} {} (Axis {})",
                                    sc_instance, axis_name, direction_symbol, axis_index
                                ),
                                device_type: "Gamepad".to_string(),
                                axis_value: Some(*value),
                                modifiers: get_active_modifiers(),
                                is_modifier: false,
                                session_id: session_id.clone(),
                                device_uuid: Some(device_uuid),
                                raw_axis_code: Some(format!("XInput {}", axis_name)),
                                raw_button_code: None,
                                raw_code_index: Some(*axis_index),
                                device_name: Some(format!("Xbox Controller (XInput {})", controller_id)),
                                device_gilrs_id: None,
                                device_power_info: None,
                                device_is_ff_supported: None,
                                all_device_axes: None,
                                all_device_buttons: None,
                            }));
                        }
                    }
                }

                // Update previous state
                xinput_prev_states[controller_id as usize] = Some(state);
            }
        }

        // Small sleep to prevent CPU spinning
        std::thread::sleep(Duration::from_millis(10));
    }

    Ok(None) // Timeout
}

/// Wait for multiple joystick inputs and collect them all
/// Continues listening for 2 seconds after the first input is detected
pub fn wait_for_multiple_inputs(
    session_id: String,
    initial_timeout_secs: u64,
    collect_duration_secs: u64,
) -> Result<Vec<DetectedInput>, String> {
    let mut gilrs = Gilrs::new().map_err(|e| e.to_string())?;

    // Track axis states to prevent duplicate triggers
    // Track axis states - dynamically initialized as axes are moved
    let mut axis_states: HashMap<(usize, u32), AxisState> = HashMap::new();

    let start = Instant::now();
    let initial_timeout = Duration::from_secs(initial_timeout_secs);
    let mut collected_inputs: Vec<DetectedInput> = Vec::new();
    let mut first_input_time: Option<Instant> = None;
    let collect_duration = Duration::from_secs(collect_duration_secs);

    // Axis detection thresholds
    const AXIS_TRIGGER_THRESHOLD: f32 = 0.5;
    const AXIS_RESET_THRESHOLD: f32 = 0.3;

    loop {
        // Check timeout conditions
        if first_input_time.is_none() {
            // Still waiting for first input
            if start.elapsed() >= initial_timeout {
                break; // Timeout reached, return what we have (might be empty)
            }
        } else {
            // First input detected, collect for additional duration
            if first_input_time.unwrap().elapsed() >= collect_duration {
                break; // Collection period complete
            }
        }

        while let Some(event) = gilrs.next_event_blocking(Some(Duration::from_millis(50))) {
            let detected_input = match event.event {
                EventType::ButtonPressed(button, code) => {
                    let joystick_id: usize = event.id.into();
                    let sc_instance = joystick_id + 1;

                    // Get the gamepad to check if it's a gamepad or joystick
                    let gamepad = gilrs.gamepad(event.id);
                    let device_name = get_friendly_device_name(&gamepad);
                    let is_gp = is_gamepad(&device_name, &gamepad);
                    let device_prefix = if is_gp { "gp" } else { "js" };
                    let device_type_name = if is_gp { "Gamepad" } else { "Joystick" };

                    let (input_string, display_name) = match button {
                        Button::DPadUp => (
                            format!("{}{}_hat1_up", device_prefix, sc_instance),
                            format!("{} {} - Hat 1 UP", device_type_name, sc_instance),
                        ),
                        Button::DPadDown => (
                            format!("{}{}_hat1_down", device_prefix, sc_instance),
                            format!("{} {} - Hat 1 DOWN", device_type_name, sc_instance),
                        ),
                        Button::DPadLeft => (
                            format!("{}{}_hat1_left", device_prefix, sc_instance),
                            format!("{} {} - Hat 1 LEFT", device_type_name, sc_instance),
                        ),
                        Button::DPadRight => (
                            format!("{}{}_hat1_right", device_prefix, sc_instance),
                            format!("{} {} - Hat 1 RIGHT", device_type_name, sc_instance),
                        ),
                        _ => {
                            let code_str = format!("{:?}", code);
                            let button_index = if let Some(start) = code_str.find("index: ") {
                                let rest = &code_str[start + 7..];
                                if let Some(end) = rest.find(' ') {
                                    rest[..end].parse::<u32>().unwrap_or(0) + 1
                                } else {
                                    0
                                }
                            } else {
                                0
                            };

                            if button_index > 0 {
                                (
                                    format!(
                                        "{}{}_button{}",
                                        device_prefix, sc_instance, button_index
                                    ),
                                    format!(
                                        "{} {} - Button {}",
                                        device_type_name, sc_instance, button_index
                                    ),
                                )
                            } else {
                                (
                                    format!("{}{}_button_unknown", device_prefix, sc_instance),
                                    format!(
                                        "{} {} - Button Unknown",
                                        device_type_name, sc_instance
                                    ),
                                )
                            }
                        }
                    };

                    // Collect extended debug info
                    let raw_button_code = format!("{:?}", button);
                    let code_str_for_index = format!("{:?}", code);
                    let raw_code_index = if let Some(start) = code_str_for_index.find("index: ") {
                        let rest = &code_str_for_index[start + 7..];
                        if let Some(end) = rest.find(' ') {
                            rest[..end].parse::<u32>().ok()
                        } else {
                            None
                        }
                    } else {
                        None
                    };
                    let power_info = format!("{:?}", gamepad.power_info());
                    let is_ff = gamepad.is_ff_supported();

                    Some(DetectedInput {
                        input_string,
                        display_name,
                        device_type: device_type_name.to_string(),
                        axis_value: None,
                        modifiers: get_active_modifiers(),
                        is_modifier: false,
                        session_id: session_id.clone(),
                        device_uuid: Some(format!("{:?}", gamepad.uuid())),
                        raw_axis_code: None,
                        raw_button_code: Some(raw_button_code),
                        raw_code_index,
                        device_name: Some(device_name.to_string()),
                        device_gilrs_id: Some(joystick_id),
                        device_power_info: Some(power_info),
                        device_is_ff_supported: Some(is_ff),
                        all_device_axes: None,
                        all_device_buttons: None,
                    })
                }
                EventType::AxisChanged(axis, value, code) => {
                    let joystick_id: usize = event.id.into();
                    let sc_instance = joystick_id + 1;

                    // Get the gamepad to check if it's a gamepad or joystick
                    let gamepad = gilrs.gamepad(event.id);
                    let device_name = get_friendly_device_name(&gamepad);
                    let is_gp = is_gamepad(&device_name, &gamepad);
                    let device_prefix = if is_gp { "gp" } else { "js" };
                    let device_type_name = if is_gp { "Gamepad" } else { "Joystick" };

                    // Extract axis index and verify this is an axis event
                    if let Some((is_axis, axis_index)) = extract_code_info(&code) {
                        if !is_axis {
                            // Code indicates this is a button, not an axis
                            continue;
                        }

                    if axis_index > 0 {
                        let axis_key = (joystick_id, axis_index);
                        let state = axis_states.entry(axis_key).or_insert(AxisState {
                            last_value: value,
                            last_triggered_direction: None,
                        });

                        let movement_delta = (value - state.last_value).abs();
                        const MOVEMENT_THRESHOLD: f32 = 0.3;

                        let is_positive = value > AXIS_TRIGGER_THRESHOLD;
                        let is_negative = value < -AXIS_TRIGGER_THRESHOLD;
                        let is_centered = value.abs() < AXIS_RESET_THRESHOLD;
                        let has_moved_enough = movement_delta > MOVEMENT_THRESHOLD;

                        if is_centered {
                            state.last_triggered_direction = None;
                            state.last_value = value;
                        }

                        let should_trigger_positive = is_positive
                            && has_moved_enough
                            && state.last_triggered_direction != Some(true);
                        let should_trigger_negative = is_negative
                            && has_moved_enough
                            && state.last_triggered_direction != Some(false);

                        if should_trigger_positive || should_trigger_negative {
                            let direction = if should_trigger_positive {
                                "positive"
                            } else {
                                "negative"
                            };
                            let direction_symbol = if should_trigger_positive { "+" } else { "-" };

                            state.last_triggered_direction = Some(should_trigger_positive);
                            state.last_value = value;

                            // Use index-based axis naming instead of gilrs::Axis enum
                            let axis_name = match axis_index {
                                1 => "X",
                                2 => "Y",
                                3 => "RotX",
                                4 => "RotY",
                                5 => "Z",
                                6 => "RotZ",
                                _ => &format!("Axis{}", axis_index),
                            };

                            // Collect extended debug info
                            let raw_axis_code = format!("{:?}", axis);
                            let power_info = format!("{:?}", gamepad.power_info());
                            let is_ff = gamepad.is_ff_supported();

                            Some(DetectedInput {
                                input_string: format!(
                                    "{}{}_axis{}_{}",
                                    device_prefix, sc_instance, axis_index, direction
                                ),
                                display_name: format!(
                                    "{} {} - {} {} (Axis {})",
                                    device_type_name,
                                    sc_instance,
                                    axis_name,
                                    direction_symbol,
                                    axis_index
                                ),
                                device_type: device_type_name.to_string(),
                                axis_value: Some(value),
                                modifiers: get_active_modifiers(),
                                is_modifier: false,
                                session_id: session_id.clone(),
                                device_uuid: Some(format!("{:?}", gamepad.uuid())),
                                raw_axis_code: Some(raw_axis_code),
                                raw_button_code: None,
                                raw_code_index: Some(axis_index),
                                device_name: Some(device_name.to_string()),
                                device_gilrs_id: Some(joystick_id),
                                device_power_info: Some(power_info),
                                device_is_ff_supported: Some(is_ff),
                                all_device_axes: None,
                                all_device_buttons: None,
                            })
                        } else {
                            None
                        }
                    } else {
                        None
                    }
                    } // End of if let Some((is_axis, axis_index))
                    else {
                        None
                    }
                }
                _ => None,
            };

            if let Some(input) = detected_input {
                // Check if this input is already in the list (avoid duplicates)
                if !collected_inputs
                    .iter()
                    .any(|i| i.input_string == input.input_string)
                {
                    collected_inputs.push(input);

                    // Mark the time when first input was detected
                    if first_input_time.is_none() {
                        first_input_time = Some(Instant::now());
                    }
                }
            }
        }
    }

    Ok(collected_inputs)
}

/// Wait for joystick inputs and emit events in real-time as they're detected
/// This version uses Tauri's event system to send updates to the frontend immediately
pub fn wait_for_inputs_with_events(
    window: tauri::Window,
    session_id: String,
    initial_timeout_secs: u64,
    collect_duration_secs: u64,
) -> Result<(), String> {
    use std::collections::HashMap;

    let mut gilrs = Gilrs::new().map_err(|e| e.to_string())?;

    // Initialize XInput for Xbox controller support
    let xinput =
        XInputHandle::load_default().map_err(|e| format!("Failed to load XInput: {:?}", e))?;
    let mut xinput_prev_states = [None, None, None, None]; // Track previous state for 4 possible controllers

    // Track XInput axis states (controller_id, axis_index) -> last triggered direction
    let mut xinput_axis_states: HashMap<(u32, u32), AxisState> = HashMap::new();

    // Initialize XInput states
    for i in 0..4 {
        if let Ok(state) = xinput.get_state(i) {
            xinput_prev_states[i as usize] = Some(state);

            // Initialize axis states for this controller
            // Normalize XInput values to -1.0 to 1.0 range
            let left_x = (state.raw.Gamepad.sThumbLX as f32) / 32768.0;
            let left_y = (state.raw.Gamepad.sThumbLY as f32) / 32768.0;
            let right_x = (state.raw.Gamepad.sThumbRX as f32) / 32768.0;
            let right_y = (state.raw.Gamepad.sThumbRY as f32) / 32768.0;
            let left_trigger = (state.raw.Gamepad.bLeftTrigger as f32) / 255.0;
            let right_trigger = (state.raw.Gamepad.bRightTrigger as f32) / 255.0;

            xinput_axis_states.insert(
                (i, 1),
                AxisState {
                    last_value: left_x,
                    last_triggered_direction: None,
                },
            );
            xinput_axis_states.insert(
                (i, 2),
                AxisState {
                    last_value: left_y,
                    last_triggered_direction: None,
                },
            );
            xinput_axis_states.insert(
                (i, 3),
                AxisState {
                    last_value: right_x,
                    last_triggered_direction: None,
                },
            );
            xinput_axis_states.insert(
                (i, 4),
                AxisState {
                    last_value: right_y,
                    last_triggered_direction: None,
                },
            );
            xinput_axis_states.insert(
                (i, 5),
                AxisState {
                    last_value: left_trigger * 2.0 - 1.0,
                    last_triggered_direction: None,
                },
            );
            xinput_axis_states.insert(
                (i, 6),
                AxisState {
                    last_value: right_trigger * 2.0 - 1.0,
                    last_triggered_direction: None,
                },
            );
        }
    }

    // Track axis states to prevent duplicate triggers
    // Track axis states - dynamically initialized as axes are moved
    let mut axis_states: HashMap<(usize, u32), AxisState> = HashMap::new();

    let start = Instant::now();
    let initial_timeout = Duration::from_secs(initial_timeout_secs);
    // Removed HashSet to allow duplicate inputs for double-tap detection
    let mut first_input_time: Option<Instant> = None;
    let collect_duration = Duration::from_secs(collect_duration_secs);

    const AXIS_TRIGGER_THRESHOLD: f32 = 0.5;
    const AXIS_RESET_THRESHOLD: f32 = 0.3;

    loop {
        // Check timeout conditions
        if first_input_time.is_none() {
            if start.elapsed() >= initial_timeout {
                // Emit completion event
                let _ = window.emit(
                    "input-detection-complete",
                    DetectionComplete {
                        session_id: session_id.clone(),
                    },
                );
                break;
            }
        } else {
            if first_input_time.unwrap().elapsed() >= collect_duration {
                // Emit completion event
                let _ = window.emit(
                    "input-detection-complete",
                    DetectionComplete {
                        session_id: session_id.clone(),
                    },
                );
                break;
            }
        }

        while let Some(event) = gilrs.next_event_blocking(Some(Duration::from_millis(50))) {
            let detected_input = match event.event {
                EventType::ButtonPressed(button, code) => {
                    let joystick_id: usize = event.id.into();
                    let sc_instance = joystick_id + 1;

                    // Get the gamepad to check if it's a gamepad or joystick
                    let gamepad = gilrs.gamepad(event.id);
                    let device_name = get_friendly_device_name(&gamepad);
                    let is_gp = is_gamepad(&device_name, &gamepad);
                    let device_prefix = if is_gp { "gp" } else { "js" };
                    let device_type_name = if is_gp { "Gamepad" } else { "Joystick" };

                    let (input_string, display_name) = match button {
                        Button::DPadUp => (
                            format!("{}{}_hat1_up", device_prefix, sc_instance),
                            format!("{} {} - Hat 1 UP", device_type_name, sc_instance),
                        ),
                        Button::DPadDown => (
                            format!("{}{}_hat1_down", device_prefix, sc_instance),
                            format!("{} {} - Hat 1 DOWN", device_type_name, sc_instance),
                        ),
                        Button::DPadLeft => (
                            format!("{}{}_hat1_left", device_prefix, sc_instance),
                            format!("{} {} - Hat 1 LEFT", device_type_name, sc_instance),
                        ),
                        Button::DPadRight => (
                            format!("{}{}_hat1_right", device_prefix, sc_instance),
                            format!("{} {} - Hat 1 RIGHT", device_type_name, sc_instance),
                        ),
                        _ => {
                            let code_str = format!("{:?}", code);
                            let button_index = if let Some(start) = code_str.find("index: ") {
                                let rest = &code_str[start + 7..];
                                if let Some(end) = rest.find(' ') {
                                    rest[..end].parse::<u32>().unwrap_or(0) + 1
                                } else {
                                    0
                                }
                            } else {
                                0
                            };

                            if button_index > 0 {
                                (
                                    format!(
                                        "{}{}_button{}",
                                        device_prefix, sc_instance, button_index
                                    ),
                                    format!(
                                        "{} {} - Button {}",
                                        device_type_name, sc_instance, button_index
                                    ),
                                )
                            } else {
                                continue;
                            }
                        }
                    };

                    // Collect extended debug info
                    let raw_button_code = format!("{:?}", button);
                    let code_str_for_index = format!("{:?}", code);
                    let raw_code_index = if let Some(start) = code_str_for_index.find("index: ") {
                        let rest = &code_str_for_index[start + 7..];
                        if let Some(end) = rest.find(' ') {
                            rest[..end].parse::<u32>().ok()
                        } else {
                            None
                        }
                    } else {
                        None
                    };
                    let power_info = format!("{:?}", gamepad.power_info());
                    let is_ff = gamepad.is_ff_supported();

                    Some(DetectedInput {
                        input_string,
                        display_name,
                        device_type: device_type_name.to_string(),
                        axis_value: None,
                        modifiers: get_active_modifiers(),
                        is_modifier: false,
                        session_id: session_id.clone(),
                        device_uuid: Some(format!("{:?}", gamepad.uuid())),
                        raw_axis_code: None,
                        raw_button_code: Some(raw_button_code),
                        raw_code_index,
                        device_name: Some(device_name.to_string()),
                        device_gilrs_id: Some(joystick_id),
                        device_power_info: Some(power_info),
                        device_is_ff_supported: Some(is_ff),
                        all_device_axes: None,
                        all_device_buttons: None,
                    })
                }
                EventType::AxisChanged(axis, value, code) => {
                    let joystick_id: usize = event.id.into();
                    let sc_instance = joystick_id + 1;

                    // Get the gamepad to check if it's a gamepad or joystick
                    let gamepad = gilrs.gamepad(event.id);
                    let device_name = get_friendly_device_name(&gamepad);
                    let is_gp = is_gamepad(&device_name, &gamepad);
                    let device_prefix = if is_gp { "gp" } else { "js" };
                    let device_type_name = if is_gp { "Gamepad" } else { "Joystick" };

                    // Extract axis index from Code instead of using gilrs::Axis enum
                    if let Some((is_axis, axis_index)) = extract_code_info(&code) {
                        if !is_axis {
                            // Code indicates button, not axis
                            continue;
                        }

                    if axis_index > 0 {
                        let axis_key = (joystick_id, axis_index);
                        let state = axis_states.entry(axis_key).or_insert(AxisState {
                            last_value: value, // Initialize with current value
                            last_triggered_direction: None,
                        });

                        let abs_value = value.abs();
                        let is_positive = value > 0.0;
                        let should_trigger = match state.last_triggered_direction {
                            None => abs_value >= AXIS_TRIGGER_THRESHOLD,
                            Some(last_positive) if last_positive == is_positive => false,
                            Some(_) => {
                                if abs_value >= AXIS_TRIGGER_THRESHOLD {
                                    true
                                } else {
                                    false
                                }
                            }
                        };

                        if abs_value < AXIS_RESET_THRESHOLD {
                            state.last_triggered_direction = None;
                        }

                        state.last_value = value;

                        if should_trigger {
                            state.last_triggered_direction = Some(is_positive);

                            let direction = if is_positive { "positive" } else { "negative" };
                            let direction_symbol = if is_positive { "+" } else { "-" };

                            // Use index-based axis naming
                            let axis_name = match axis_index {
                                1 => "X",
                                2 => "Y",
                                3 => "RotX",
                                4 => "RotY",
                                5 => "Z",
                                6 => "RotZ",
                                _ => &format!("Axis{}", axis_index),
                            };

                            // Collect extended debug info
                            let raw_axis_code = format!("{:?}", axis);
                            let power_info = format!("{:?}", gamepad.power_info());
                            let is_ff = gamepad.is_ff_supported();

                            Some(DetectedInput {
                                input_string: format!(
                                    "{}{}_axis{}_{}",
                                    device_prefix, sc_instance, axis_index, direction
                                ),
                                display_name: format!(
                                    "{} {} - {} {} (Axis {})",
                                    device_type_name,
                                    sc_instance,
                                    axis_name,
                                    direction_symbol,
                                    axis_index
                                ),
                                device_type: device_type_name.to_string(),
                                axis_value: Some(value),
                                modifiers: get_active_modifiers(),
                                is_modifier: false,
                                session_id: session_id.clone(),
                                device_uuid: Some(format!("{:?}", gamepad.uuid())),
                                raw_axis_code: Some(raw_axis_code),
                                raw_button_code: None,
                                raw_code_index: Some(axis_index),
                                device_name: Some(device_name.to_string()),
                                device_gilrs_id: Some(joystick_id),
                                device_power_info: Some(power_info),
                                device_is_ff_supported: Some(is_ff),
                                all_device_axes: None,
                                all_device_buttons: None,
                            })
                        } else {
                            None
                        }
                    } else {
                        None
                    }
                    } // End of if let Some((is_axis, axis_index))
                    else {
                        None
                    }
                }
                _ => None,
            };

            if let Some(input) = detected_input {
                // Emit event immediately (allow duplicates for double-tap detection)
                let _ = window.emit("input-detected", &input);

                // Mark the time when first input was detected
                if first_input_time.is_none() {
                    first_input_time = Some(Instant::now());
                }
            }
        }

        // Poll XInput controllers for button presses and axis movements
        for controller_id in 0..4 {
            if let Ok(state) = xinput.get_state(controller_id) {
                if let Some(prev_state) = &xinput_prev_states[controller_id as usize] {
                    // Check if any button was newly pressed
                    let buttons_pressed =
                        state.raw.Gamepad.wButtons & !prev_state.raw.Gamepad.wButtons;

                    if buttons_pressed != 0 {
                        // Find which button was pressed
                        let button_num = if buttons_pressed & 0x1000 != 0 {
                            1
                        }
                        // A
                        else if buttons_pressed & 0x2000 != 0 {
                            2
                        }
                        // B
                        else if buttons_pressed & 0x4000 != 0 {
                            3
                        }
                        // X
                        else if buttons_pressed & 0x8000 != 0 {
                            4
                        }
                        // Y
                        else if buttons_pressed & 0x0100 != 0 {
                            5
                        }
                        // LB
                        else if buttons_pressed & 0x0200 != 0 {
                            6
                        }
                        // RB
                        else if buttons_pressed & 0x0010 != 0 {
                            7
                        }
                        // Back
                        else if buttons_pressed & 0x0020 != 0 {
                            8
                        }
                        // Start
                        else if buttons_pressed & 0x0040 != 0 {
                            9
                        }
                        // LS (Left Stick Click)
                        else if buttons_pressed & 0x0080 != 0 {
                            10
                        }
                        // RS (Right Stick Click)
                        else if buttons_pressed & 0x0001 != 0 {
                            11
                        }
                        // DPad Up
                        else if buttons_pressed & 0x0002 != 0 {
                            12
                        }
                        // DPad Down
                        else if buttons_pressed & 0x0004 != 0 {
                            13
                        }
                        // DPad Left
                        else if buttons_pressed & 0x0008 != 0 {
                            14
                        }
                        // DPad Right
                        else {
                            0
                        };

                        if button_num > 0 {
                            let sc_instance = controller_id as usize + 1;
                            let device_uuid = resolve_xinput_uuid(controller_id);
                            let input_string = format!("gp{}_button{}", sc_instance, button_num);

                            // Emit event immediately (allow duplicates for double-tap detection)
                            let input = DetectedInput {
                                input_string,
                                display_name: format!(
                                    "Gamepad {} - Button {}",
                                    sc_instance, button_num
                                ),
                                device_type: "Gamepad".to_string(),
                                axis_value: None,
                                modifiers: get_active_modifiers(),
                                is_modifier: false,
                                session_id: session_id.clone(),
                                device_uuid: Some(device_uuid),
                                raw_axis_code: None,
                                raw_button_code: Some(format!(
                                    "XInput Button: 0x{:04X}",
                                    buttons_pressed
                                )),
                                raw_code_index: Some(button_num),
                                device_name: Some(format!("Xbox Controller (XInput {})", controller_id)),
                                device_gilrs_id: None,
                                device_power_info: None,
                                device_is_ff_supported: None,
                                all_device_axes: None,
                                all_device_buttons: None,
                            };

                            // Emit event immediately
                            let _ = window.emit("input-detected", &input);

                            // Mark the time when first input was detected
                            if first_input_time.is_none() {
                                first_input_time = Some(Instant::now());
                            }
                        }
                    }

                    // Check for axis movements
                    // Normalize XInput values to -1.0 to 1.0 range
                    let axes = [
                        (
                            1,
                            (state.raw.Gamepad.sThumbLX as f32) / 32768.0,
                            "Left Stick X",
                        ),
                        (
                            2,
                            (state.raw.Gamepad.sThumbLY as f32) / 32768.0,
                            "Left Stick Y",
                        ),
                        (
                            3,
                            (state.raw.Gamepad.sThumbRX as f32) / 32768.0,
                            "Right Stick X",
                        ),
                        (
                            4,
                            (state.raw.Gamepad.sThumbRY as f32) / 32768.0,
                            "Right Stick Y",
                        ),
                        (
                            5,
                            (state.raw.Gamepad.bLeftTrigger as f32) / 255.0 * 2.0 - 1.0,
                            "Left Trigger",
                        ),
                        (
                            6,
                            (state.raw.Gamepad.bRightTrigger as f32) / 255.0 * 2.0 - 1.0,
                            "Right Trigger",
                        ),
                    ];

                    for (axis_index, value, axis_name) in axes.iter() {
                        let axis_key = (controller_id, *axis_index);
                        let state_entry = xinput_axis_states.entry(axis_key).or_insert(AxisState {
                            last_value: *value,
                            last_triggered_direction: None,
                        });

                        let movement_delta = (value - state_entry.last_value).abs();
                        const MOVEMENT_THRESHOLD: f32 = 0.3;
                        const AXIS_TRIGGER_THRESHOLD: f32 = 0.5;
                        const AXIS_RESET_THRESHOLD: f32 = 0.3;

                        let is_positive = *value > AXIS_TRIGGER_THRESHOLD;
                        let is_negative = *value < -AXIS_TRIGGER_THRESHOLD;
                        let is_centered = value.abs() < AXIS_RESET_THRESHOLD;
                        let has_moved_enough = movement_delta > MOVEMENT_THRESHOLD;

                        if is_centered {
                            state_entry.last_triggered_direction = None;
                            state_entry.last_value = *value;
                        }

                        let should_trigger_positive = is_positive
                            && has_moved_enough
                            && state_entry.last_triggered_direction != Some(true);
                        let should_trigger_negative = is_negative
                            && has_moved_enough
                            && state_entry.last_triggered_direction != Some(false);

                        if should_trigger_positive || should_trigger_negative {
                            let direction = if should_trigger_positive {
                                "positive"
                            } else {
                                "negative"
                            };
                            let direction_symbol = if should_trigger_positive { "+" } else { "-" };

                            state_entry.last_triggered_direction = Some(should_trigger_positive);
                            state_entry.last_value = *value;

                            let sc_instance = controller_id as usize + 1;
                            let device_uuid = resolve_xinput_uuid(controller_id);
                            let input_string =
                                format!("gp{}_axis{}_{}", sc_instance, axis_index, direction);

                            // Emit event immediately (allow duplicates for double-tap detection)
                            let input = DetectedInput {
                                input_string,
                                display_name: format!(
                                    "Gamepad {} - {} {} (Axis {})",
                                    sc_instance, axis_name, direction_symbol, axis_index
                                ),
                                device_type: "Gamepad".to_string(),
                                axis_value: Some(*value),
                                modifiers: get_active_modifiers(),
                                is_modifier: false,
                                session_id: session_id.clone(),
                                device_uuid: Some(device_uuid),
                                raw_axis_code: Some(format!("XInput {}", axis_name)),
                                raw_button_code: None,
                                raw_code_index: Some(*axis_index),
                                device_name: Some(format!("Xbox Controller (XInput {})", controller_id)),
                                device_gilrs_id: None,
                                device_power_info: None,
                                device_is_ff_supported: None,
                                all_device_axes: None,
                                all_device_buttons: None,
                            };

                            // Emit event immediately
                            let _ = window.emit("input-detected", &input);

                            // Mark the time when first input was detected
                            if first_input_time.is_none() {
                                first_input_time = Some(Instant::now());
                            }
                        }
                    }
                }

                // Update previous state
                xinput_prev_states[controller_id as usize] = Some(state);
            }
        }
    }

    Ok(())
}

/// Get list of available joysticks
pub fn detect_joysticks() -> Result<Vec<JoystickInfo>, String> {
    let mut gilrs = Gilrs::new().map_err(|e| e.to_string())?;

    let mut joysticks = Vec::new();

    eprintln!("=== Gilrs Gamepad Detection ===");
    eprintln!("Gilrs version: {}", env!("CARGO_PKG_VERSION"));

    // Give gilrs a moment to initialize
    thread::sleep(Duration::from_millis(100));

    // Process any pending events to ensure device list is up to date
    while let Some(_event) = gilrs.next_event() {
        // Just drain the event queue
    }

    let gamepad_count = gilrs.gamepads().count();
    eprintln!(
        "Gilrs reports {} gamepads after event processing",
        gamepad_count
    );

    for (_id, gamepad) in gilrs.gamepads() {
        let name = get_friendly_device_name(&gamepad);
        let is_connected = gamepad.is_connected();
        let id = usize::from(gamepad.id());

        eprintln!(
            "Found gamepad ID {}: {} (connected: {})",
            id, name, is_connected
        );
        eprintln!("  Power info: {:?}", gamepad.power_info());
        eprintln!("  Is FF supported: {}", gamepad.is_ff_supported());
        eprintln!("  UUID: {:?}", gamepad.uuid());
        eprintln!("  Vendor ID: {:?}", gamepad.vendor_id());
        eprintln!("  Product ID: {:?}", gamepad.product_id());

        // Gilrs doesn't provide a way to query exact button/axis counts before they're used
        // Use reasonable defaults based on device type
        let is_gamepad_device = is_gamepad(&name, &gamepad);

        let (button_count, axis_count, hat_count) = if is_gamepad_device {
            // Standard gamepad: Xbox/PlayStation style
            (15, 6, 1) // A/B/X/Y, LB/RB, LT/RT, Back/Start, LS/RS, D-pad (4 buttons) | Left stick X/Y, Right stick X/Y, Triggers | D-pad as hat
        } else {
            // Flight stick/HOTAS: More buttons, fewer axes
            (32, 6, 1) // Many buttons typical on flight sticks | Fewer axes | Usually 1 hat switch
        };

        eprintln!(
            "  Estimated: {} buttons, {} axes, {} hats (type: {})",
            button_count,
            axis_count,
            hat_count,
            if is_gamepad_device {
                "gamepad"
            } else {
                "joystick"
            }
        );

        let device_type_name = if is_gamepad_device {
            "Gamepad"
        } else {
            "Joystick"
        };

        joysticks.push(JoystickInfo {
            id,
            name: name.clone(),
            is_connected,
            button_count,
            axis_count,
            hat_count,
            device_type: device_type_name.to_string(),
        });
    }
    eprintln!("=== Total gamepads found: {} ===", joysticks.len());

    Ok(joysticks)
}

/// Returns detailed information for all connected devices.
pub fn list_connected_devices() -> Result<Vec<DeviceInfo>, String> {
    let mut gilrs = Gilrs::new().map_err(|e| e.to_string())?;

    // Drain events so gilrs updates its internal cache
    while let Some(_event) = gilrs.next_event() {
        // no-op
    }

    let mut devices = Vec::new();

    for (_id, gamepad) in gilrs.gamepads() {
        let name = get_friendly_device_name(&gamepad);

        // Skip Xbox controllers in Gilrs if we're on Windows, as we'll add them via XInput
        // This prevents duplicates and ensures we use the XInput UUIDs that match our input detection
        if cfg!(windows) && (name.to_lowercase().contains("xbox") || name.to_lowercase().contains("xinput")) {
            continue;
        }

        let is_connected = gamepad.is_connected();
        let id = usize::from(gamepad.id());
        let uuid = resolve_device_uuid(&gamepad, id);
        let is_gamepad_device = is_gamepad(&name, &gamepad);

        let (button_count, axis_count, hat_count) = if is_gamepad_device {
            (15, 6, 1)
        } else {
            (32, 7, 1)
        };

        devices.push(DeviceInfo {
            uuid,
            name,
            axis_count,
            button_count,
            hat_count,
            device_type: if is_gamepad_device {
                "gamepad"
            } else {
                "joystick"
            }
            .to_string(),
            is_connected,
        });
    }

    // Add XInput devices explicitly
    // This ensures that if wait_for_input detects via XInput fallback, we have a matching device in the list
    if let Ok(xinput) = XInputHandle::load_default() {
        for i in 0..4 {
            if xinput.get_state(i).is_ok() {
                let uuid = resolve_xinput_uuid(i);
                
                // Only add if not already present (though UUIDs will likely differ from Gilrs)
                if !devices.iter().any(|d| d.uuid == uuid) {
                    devices.push(DeviceInfo {
                        uuid,
                        name: format!("Xbox Controller (XInput {})", i),
                        axis_count: 6,
                        button_count: 15,
                        hat_count: 1,
                        device_type: "Gamepad".to_string(),
                        is_connected: true,
                    });
                }
            }
        }
    }

    Ok(devices)
}

/// Waits for the user to move an axis on the specified device and returns the raw axis index.
pub fn detect_axis_movement_for_device(
    target_uuid: &str,
    timeout_millis: u64,
) -> Result<Option<AxisMovement>, String> {
    let timeout = Duration::from_millis(timeout_millis);
    let start = Instant::now();

    // Process any pending events within the timeout window
    while start.elapsed() < timeout {
        // Get or create the shared Gilrs instance in each iteration
        let mut gilrs_lock = GILRS_INSTANCE.lock().map_err(|e| e.to_string())?;
        if gilrs_lock.is_none() {
            *gilrs_lock = Some(Gilrs::new().map_err(|e| e.to_string())?);
        }
        
        // Drain all pending events and track the most recent axis movement
        let mut latest_movement: Option<AxisMovement> = None;
        
        if let Some(gilrs) = gilrs_lock.as_mut() {
            // Process ALL pending events to clear the queue
            while let Some(event) = gilrs.next_event() {
                if let EventType::AxisChanged(_axis, value, code) = event.event {
                    let gamepad = gilrs.gamepad(event.id);
                    let uuid = resolve_device_uuid(&gamepad, usize::from(event.id));

                    if uuid != target_uuid {
                        continue;
                    }

                    // Only track axis movement outside deadzone (±0.15) to filter noise/drift
                    if value.abs() > 0.15 {
                        if let Some(index) = extract_index_from_code(&code) {
                            // Keep updating to get the most recent value
                            latest_movement = Some(AxisMovement {
                                axis_id: index,
                                value,
                            });
                        }
                    }
                }
            }
        }
        
        // Return the most recent movement if we found one
        if latest_movement.is_some() {
            return Ok(latest_movement);
        }
        
        // Release lock before sleeping
        drop(gilrs_lock);
        thread::sleep(Duration::from_millis(5));
    }

    Ok(None)
}
