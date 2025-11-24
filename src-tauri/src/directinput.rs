use crate::hid_reader;
use rusty_xinput::{XInputHandle, XInputState};
use serde::Serialize;
use std::collections::HashMap;
use std::thread;
use std::time::{Duration, Instant};
use tauri::Emitter;

// Constants for detection thresholds
const AXIS_TRIGGER_THRESHOLD: f32 = 0.5;
const AXIS_RESET_THRESHOLD: f32 = 0.3;
const MOVEMENT_THRESHOLD: f32 = 0.3;

fn is_gamepad(name: &str) -> bool {
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

    // Extended debug information
    pub raw_button_code: Option<String>, // Raw button code
    pub raw_code_index: Option<u32>,     // Raw index from the code
    pub device_name: Option<String>,     // Full device name

    // HID-specific axis information (from device descriptor)
    pub hid_usage_id: Option<u32>, // HID usage ID for this axis (e.g., 48 for X, 53 for Rz)
    pub hid_axis_name: Option<String>, // HID axis name from descriptor (e.g., "X", "Y", "Rz")
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
    pub uuid: Option<String>, // Hardware UUID (vendor_id:product_id format)
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

fn resolve_xinput_uuid(controller_id: u32) -> String {
    // Create a consistent UUID for XInput controllers based on their slot
    format!("xinput_{}", controller_id)
}

// Axis state tracking to prevent duplicate detections
struct AxisState {
    last_value: f32,
    last_triggered_direction: Option<bool>, // true = positive, false = negative
}

/// Handles the state and polling logic for input detection
struct InputDetector {
    session_id: String,
    xinput: Option<XInputHandle>,
    xinput_prev_states: [Option<XInputState>; 4],
    xinput_axis_states: HashMap<(u32, u32), AxisState>,
    hid_devices: Vec<hid_reader::HidDeviceListItem>,
    device_descriptors: HashMap<String, Vec<u8>>,
    device_instances: HashMap<String, usize>,
    device_hid_to_axis_maps: HashMap<String, HashMap<u32, u32>>,
    prev_hid_reports: HashMap<String, hid_reader::HidFullReport>,
    opened_devices: HashMap<String, hid_reader::OpenedHidDevice>,
}

impl InputDetector {
    fn new(session_id: String) -> Self {
        eprintln!("InputDetector: Initializing...");

        // Initialize XInput
        let xinput = XInputHandle::load_default().ok();
        if xinput.is_none() {
            eprintln!("InputDetector: Failed to load XInput (or not on Windows)");
        }

        let mut xinput_prev_states = [None, None, None, None];
        let mut xinput_axis_states = HashMap::new();

        if let Some(ref xinput) = xinput {
            for i in 0..4 {
                if let Ok(state) = xinput.get_state(i) {
                    xinput_prev_states[i as usize] = Some(state);
                    eprintln!("InputDetector: XInput controller {} initialized", i);

                    // Initialize axis states for this controller
                    let left_x = (state.raw.Gamepad.sThumbLX as f32) / 32768.0;
                    let left_y = (state.raw.Gamepad.sThumbLY as f32) / 32768.0;
                    let right_x = (state.raw.Gamepad.sThumbRX as f32) / 32768.0;
                    let right_y = (state.raw.Gamepad.sThumbRY as f32) / 32768.0;
                    let left_trigger = (state.raw.Gamepad.bLeftTrigger as f32) / 255.0;
                    let right_trigger = (state.raw.Gamepad.bRightTrigger as f32) / 255.0;

                    for (axis_idx, value) in [
                        (1, left_x),
                        (2, left_y),
                        (3, right_x),
                        (4, right_y),
                        (5, left_trigger * 2.0 - 1.0),
                        (6, right_trigger * 2.0 - 1.0),
                    ] {
                        xinput_axis_states.insert(
                            (i, axis_idx),
                            AxisState {
                                last_value: value,
                                last_triggered_direction: None,
                            },
                        );
                    }
                }
            }
        }

        // Initialize HID devices
        let hid_devices = hid_reader::list_hid_game_controllers().unwrap_or_default();
        let mut device_descriptors = HashMap::new();
        let mut device_instances = HashMap::new();
        let mut device_hid_to_axis_maps = HashMap::new();
        let mut opened_devices = HashMap::new();

        for (idx, device) in hid_devices.iter().enumerate() {
            // Try to open the device persistently
            match hid_reader::OpenedHidDevice::open(&device.path) {
                Ok(opened_dev) => {
                    opened_devices.insert(device.path.clone(), opened_dev);
                }
                Err(e) => {
                    eprintln!(
                        "InputDetector: Failed to open device {}: {}",
                        device.path, e
                    );
                    continue;
                }
            }

            if let Ok(descriptor) = hid_reader::get_hid_descriptor_bytes(&device.path) {
                device_descriptors.insert(device.path.clone(), descriptor);
                device_instances.insert(device.path.clone(), idx + 1);

                // Get the DirectInput-to-HID mapping and invert it (HID usage ID -> DirectInput index)
                if let Ok(di_to_hid) = hid_reader::get_directinput_to_hid_axis_mapping(&device.path)
                {
                    let mut hid_to_axis: HashMap<u32, u32> = HashMap::new();
                    for (axis_idx, hid_usage_id) in di_to_hid {
                        hid_to_axis.insert(hid_usage_id, axis_idx);
                    }
                    device_hid_to_axis_maps.insert(device.path.clone(), hid_to_axis);
                }
            }
        }

        eprintln!(
            "InputDetector: Monitoring {} HID devices and 4 XInput slots",
            hid_devices.len()
        );

        Self {
            session_id,
            xinput,
            xinput_prev_states,
            xinput_axis_states,
            hid_devices,
            device_descriptors,
            device_instances,
            device_hid_to_axis_maps,
            prev_hid_reports: HashMap::new(),
            opened_devices,
        }
    }

    fn poll(&mut self) -> Vec<DetectedInput> {
        let mut detected_inputs = Vec::new();

        // Poll XInput
        if let Some(ref xinput) = self.xinput {
            for controller_id in 0..4 {
                if let Ok(state) = xinput.get_state(controller_id) {
                    if let Some(prev_state) = &self.xinput_prev_states[controller_id as usize] {
                        // Check buttons
                        let buttons_pressed =
                            state.raw.Gamepad.wButtons & !prev_state.raw.Gamepad.wButtons;

                        if buttons_pressed != 0 {
                            let button_num = match buttons_pressed {
                                0x1000 => Some(1),  // A
                                0x2000 => Some(2),  // B
                                0x4000 => Some(3),  // X
                                0x8000 => Some(4),  // Y
                                0x0100 => Some(5),  // LB
                                0x0200 => Some(6),  // RB
                                0x0010 => Some(7),  // Back
                                0x0020 => Some(8),  // Start
                                0x0040 => Some(9),  // LS
                                0x0080 => Some(10), // RS
                                0x0001 => Some(11), // DPad Up
                                0x0002 => Some(12), // DPad Down
                                0x0004 => Some(13), // DPad Left
                                0x0008 => Some(14), // DPad Right
                                _ => None,
                            };

                            if let Some(btn) = button_num {
                                let sc_instance = controller_id as usize + 1;
                                detected_inputs.push(DetectedInput {
                                    input_string: format!("gp{}_button{}", sc_instance, btn),
                                    display_name: format!(
                                        "Gamepad {} - Button {}",
                                        sc_instance, btn
                                    ),
                                    device_type: "Gamepad".to_string(),
                                    axis_value: None,
                                    modifiers: get_active_modifiers(),
                                    is_modifier: false,
                                    session_id: self.session_id.clone(),
                                    device_uuid: Some(resolve_xinput_uuid(controller_id)),
                                    raw_button_code: Some(format!(
                                        "XInput 0x{:04X}",
                                        buttons_pressed
                                    )),
                                    raw_code_index: Some(btn),
                                    device_name: Some(format!(
                                        "Xbox Controller (XInput {})",
                                        controller_id
                                    )),
                                    hid_usage_id: None,
                                    hid_axis_name: None,
                                });
                            }
                        }

                        // Check axes
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
                            let state_entry =
                                self.xinput_axis_states
                                    .entry(axis_key)
                                    .or_insert(AxisState {
                                        last_value: *value,
                                        last_triggered_direction: None,
                                    });

                            let movement_delta = (value - state_entry.last_value).abs();
                            let is_positive = *value > AXIS_TRIGGER_THRESHOLD;
                            let is_negative = *value < -AXIS_TRIGGER_THRESHOLD;
                            let is_centered = value.abs() < AXIS_RESET_THRESHOLD;
                            let has_moved_enough = movement_delta > MOVEMENT_THRESHOLD;

                            if is_centered {
                                state_entry.last_triggered_direction = None;
                                state_entry.last_value = *value;
                            }

                            let should_trigger = (is_positive
                                && has_moved_enough
                                && state_entry.last_triggered_direction != Some(true))
                                || (is_negative
                                    && has_moved_enough
                                    && state_entry.last_triggered_direction != Some(false));

                            if should_trigger {
                                let direction = if is_positive { "positive" } else { "negative" };
                                let direction_symbol = if is_positive { "+" } else { "-" };
                                state_entry.last_triggered_direction = Some(is_positive);
                                state_entry.last_value = *value;

                                let sc_instance = controller_id as usize + 1;
                                detected_inputs.push(DetectedInput {
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
                                    session_id: self.session_id.clone(),
                                    device_uuid: Some(resolve_xinput_uuid(controller_id)),
                                    raw_button_code: None,
                                    raw_code_index: Some(*axis_index),
                                    device_name: Some(format!(
                                        "Xbox Controller (XInput {})",
                                        controller_id
                                    )),
                                    hid_usage_id: None,
                                    hid_axis_name: None,
                                });
                            }
                        }
                    }
                    self.xinput_prev_states[controller_id as usize] = Some(state);
                }
            }
        }

        // Poll HID devices
        for device in &self.hid_devices {
            let Some(descriptor) = self.device_descriptors.get(&device.path) else {
                continue;
            };
            let Some(opened_device) = self.opened_devices.get(&device.path) else {
                continue;
            };
            let device_instance = self.device_instances[&device.path];

            // Drain the HID queue (up to 10 reports per poll to avoid blocking)
            let mut reports_processed = 0;
            loop {
                if reports_processed >= 10 {
                    break;
                }

                // Use 0 timeout for non-blocking read
                let report_bytes = match opened_device.read(0) {
                    Ok(bytes) if !bytes.is_empty() => bytes,
                    _ => break, // No more reports or error
                };

                reports_processed += 1;

                let current_report =
                    match hid_reader::parse_hid_full_report(&report_bytes, descriptor) {
                        Ok(report) => report,
                        Err(_) => continue,
                    };

                // Check buttons - only detect NEW button presses (not held buttons)
                // Skip detection on the very first poll for this device (baseline establishment)
                let prev_buttons = self
                    .prev_hid_reports
                    .get(&device.path)
                    .map(|r| &r.pressed_buttons)
                    .cloned();

                if let Some(prev_buttons) = prev_buttons {
                    // We have a previous state, so detect new button presses
                    let newly_pressed: Vec<u32> = current_report
                        .pressed_buttons
                        .iter()
                        .filter(|&&btn| !prev_buttons.contains(&btn))
                        .copied()
                        .collect();

                    // Debug logging for button detection
                    if !current_report.pressed_buttons.is_empty() || !prev_buttons.is_empty() {
                        eprintln!(
                        "Device {}: Current buttons: {:?}, Prev buttons: {:?}, Newly pressed: {:?}",
                        device_instance,
                        current_report.pressed_buttons,
                        prev_buttons,
                        newly_pressed
                    );
                    }

                    if !newly_pressed.is_empty() {
                        let button_num = newly_pressed[0];
                        let device_name = device.product.as_deref().unwrap_or("Unknown Device");

                        detected_inputs.push(DetectedInput {
                            input_string: format!("js{}_button{}", device_instance, button_num),
                            display_name: format!(
                                "Joystick {} - Button {}",
                                device_instance, button_num
                            ),
                            device_type: "Joystick".to_string(),
                            axis_value: None,
                            modifiers: get_active_modifiers(),
                            is_modifier: false,
                            session_id: self.session_id.clone(),
                            device_uuid: Some(format!(
                                "{:04x}:{:04x}",
                                device.vendor_id, device.product_id
                            )),
                            raw_button_code: Some(format!("HID Button {}", button_num)),
                            raw_code_index: Some(button_num),
                            device_name: Some(device_name.to_string()),
                            hid_usage_id: None,
                            hid_axis_name: None,
                        });
                    }
                } // else: First poll for this device - just establish baseline, don't detect anything

                // Check axes
                if let Some(prev_report) = self.prev_hid_reports.get(&device.path) {
                    for (&axis_id, &current_value) in &current_report.axis_values {
                        let prev_value =
                            prev_report.axis_values.get(&axis_id).copied().unwrap_or(0);

                        let (logical_min, logical_max) = current_report
                            .axis_ranges
                            .get(&axis_id)
                            .copied()
                            .unwrap_or((0, 65535));

                        let range = (logical_max - logical_min) as f32;
                        if range <= 0.0 {
                            continue;
                        }

                        let change_abs = (current_value as i32 - prev_value as i32).abs() as f32;

                        // Hat switch check (HID Usage ID 0x39 = 57)
                        // Hat switches need special handling - they should detect any value change
                        let is_hat_switch = axis_id == 0x39;

                        // For hat switches, trigger on any non-zero change
                        // For regular axes, use absolute threshold
                        const AXIS_CHANGE_THRESHOLD: f32 = 50.0; // Absolute value change needed

                        let should_detect = if is_hat_switch {
                            change_abs > 0.0 // Any change triggers detection for hat switches
                        } else {
                            change_abs >= AXIS_CHANGE_THRESHOLD
                        };

                        if should_detect {
                            // Normalize to -1.0 to 1.0 for all checks
                            let normalized =
                                ((current_value as i32 - logical_min) as f32 / range * 2.0) - 1.0;

                            // Hat switch check (HID Usage ID 0x39 = 57)
                            if is_hat_switch {
                                let axis_name = current_report
                                    .axis_names
                                    .get(&axis_id)
                                    .map(|s| s.as_str())
                                    .unwrap_or("Unknown");
                                let device_name =
                                    device.product.as_deref().unwrap_or("Unknown Device");

                                let axis_index = self
                                    .device_hid_to_axis_maps
                                    .get(&device.path)
                                    .and_then(|map| map.get(&axis_id).copied())
                                    .unwrap_or_else(|| {
                                        let mut sorted_axes: Vec<_> =
                                            current_report.axis_values.keys().copied().collect();
                                        sorted_axes.sort();
                                        sorted_axes
                                            .iter()
                                            .position(|&id| id == axis_id)
                                            .map(|pos| pos as u32 + 1)
                                            .unwrap_or(1)
                                    });

                                let hat_direction = match current_value {
                                    0 => Some("up"),
                                    1 => Some("up"),
                                    2 => Some("right"),
                                    3 => Some("right"),
                                    4 => Some("down"),
                                    5 => Some("down"),
                                    6 => Some("left"),
                                    7 => Some("left"),
                                    8 | 15 => None, // Centered
                                    _ => None,
                                };

                                if let Some(direction) = hat_direction {
                                    detected_inputs.push(DetectedInput {
                                        input_string: format!(
                                            "js{}_hat1_{}",
                                            device_instance, direction
                                        ),
                                        display_name: format!(
                                            "Joystick {} - Hat 1 {}",
                                            device_instance,
                                            direction.to_uppercase()
                                        ),
                                        device_type: "Joystick".to_string(),
                                        axis_value: Some(normalized),
                                        modifiers: get_active_modifiers(),
                                        is_modifier: false,
                                        session_id: self.session_id.clone(),
                                        device_uuid: Some(format!(
                                            "{:04x}:{:04x}",
                                            device.vendor_id, device.product_id
                                        )),
                                        raw_button_code: None,
                                        raw_code_index: Some(axis_index),
                                        device_name: Some(device_name.to_string()),
                                        hid_usage_id: Some(axis_id),
                                        hid_axis_name: Some(axis_name.to_string()),
                                    });
                                }
                                continue;
                            }

                            // Regular axis - detect any movement past threshold regardless of position
                            let direction = if normalized > 0.0 {
                                "positive"
                            } else {
                                "negative"
                            };
                            let direction_symbol = if normalized > 0.0 { "+" } else { "-" };
                            let axis_name = current_report
                                .axis_names
                                .get(&axis_id)
                                .map(|s| s.as_str())
                                .unwrap_or("Unknown");
                            let device_name = device.product.as_deref().unwrap_or("Unknown Device");

                            let axis_index = self
                                .device_hid_to_axis_maps
                                .get(&device.path)
                                .and_then(|map| map.get(&axis_id).copied())
                                .unwrap_or_else(|| {
                                    let mut sorted_axes: Vec<_> =
                                        current_report.axis_values.keys().copied().collect();
                                    sorted_axes.sort();
                                    sorted_axes
                                        .iter()
                                        .position(|&id| id == axis_id)
                                        .map(|pos| pos as u32 + 1)
                                        .unwrap_or(1)
                                });

                            detected_inputs.push(DetectedInput {
                                input_string: format!(
                                    "js{}_axis{}_{}",
                                    device_instance, axis_index, direction
                                ),
                                display_name: format!(
                                    "Joystick {} - {} {} (Axis {})",
                                    device_instance, axis_name, direction_symbol, axis_index
                                ),
                                device_type: "Joystick".to_string(),
                                axis_value: Some(normalized),
                                modifiers: get_active_modifiers(),
                                is_modifier: false,
                                session_id: self.session_id.clone(),
                                device_uuid: Some(format!(
                                    "{:04x}:{:04x}",
                                    device.vendor_id, device.product_id
                                )),
                                raw_button_code: None,
                                raw_code_index: Some(axis_index),
                                device_name: Some(device_name.to_string()),
                                hid_usage_id: Some(axis_id),
                                hid_axis_name: Some(axis_name.to_string()),
                            });
                        }
                    }
                }
                self.prev_hid_reports
                    .insert(device.path.clone(), current_report);
            }
        }

        detected_inputs
    }
}

/// Wait for input from any game controller (hybrid approach)
pub fn wait_for_input(
    session_id: String,
    timeout_secs: u64,
) -> Result<Option<DetectedInput>, String> {
    let start = Instant::now();
    let timeout = Duration::from_secs(timeout_secs);

    eprintln!(
        "wait_for_input: Starting hybrid input detection for {} seconds",
        timeout_secs
    );

    let mut detector = InputDetector::new(session_id);

    while start.elapsed() < timeout {
        let inputs = detector.poll();
        if let Some(input) = inputs.into_iter().next() {
            return Ok(Some(input));
        }
        thread::sleep(Duration::from_millis(5));
    }

    Ok(None)
}

/// Wait for joystick inputs and emit events in real-time
pub fn wait_for_inputs_with_events(
    window: tauri::Window,
    session_id: String,
    initial_timeout_secs: u64,
    collect_duration_secs: u64,
) -> Result<(), String> {
    eprintln!("wait_for_inputs_with_events: Starting hybrid input detection");

    let mut detector = InputDetector::new(session_id.clone());

    let start = Instant::now();
    let initial_timeout = Duration::from_secs(initial_timeout_secs);
    let mut first_input_time: Option<Instant> = None;
    let collect_duration = Duration::from_secs(collect_duration_secs);

    loop {
        // Check timeout conditions
        if let Some(first_time) = first_input_time {
            if first_time.elapsed() >= collect_duration {
                let _ = window.emit(
                    "input-detection-complete",
                    DetectionComplete {
                        session_id: session_id.clone(),
                    },
                );
                break;
            }
        } else if start.elapsed() >= initial_timeout {
            let _ = window.emit(
                "input-detection-complete",
                DetectionComplete {
                    session_id: session_id.clone(),
                },
            );
            break;
        }

        let inputs = detector.poll();
        for input in inputs {
            let _ = window.emit("input-detected", &input);
            if first_input_time.is_none() {
                first_input_time = Some(Instant::now());
            }
        }

        thread::sleep(Duration::from_millis(5));
    }

    Ok(())
}

/// Get list of available joysticks using hybrid approach (HID + XInput)
pub fn detect_joysticks() -> Result<Vec<JoystickInfo>, String> {
    let mut joysticks = Vec::new();

    eprintln!("=== Hybrid Device Detection (HID + XInput) ===");

    // First, list HID game controllers (joysticks/HOTAS)
    match hid_reader::list_hid_game_controllers() {
        Ok(hid_devices) => {
            eprintln!("Found {} HID game controllers", hid_devices.len());

            for (idx, device) in hid_devices.iter().enumerate() {
                let device_name = device.product.as_deref().unwrap_or("Unknown HID Device");
                let manufacturer = device.manufacturer.as_deref().unwrap_or("");
                let full_name = if !manufacturer.is_empty() {
                    format!("{} {}", manufacturer, device_name)
                } else {
                    device_name.to_string()
                };

                // Skip Xbox controllers as they'll be added via XInput for better support
                if cfg!(windows)
                    && (full_name.to_lowercase().contains("xbox")
                        || full_name.to_lowercase().contains("xinput")
                        || full_name.to_lowercase().contains("controller for windows"))
                {
                    eprintln!(
                        "HID Device {}: {} - SKIPPING (will use XInput instead)",
                        idx + 1,
                        full_name
                    );
                    continue;
                }

                eprintln!(
                    "HID Device {}: {} (VID: 0x{:04x}, PID: 0x{:04x})",
                    idx + 1,
                    full_name,
                    device.vendor_id,
                    device.product_id
                );

                // Try to get axis count from descriptor
                let (button_count, axis_count, hat_count) =
                    match hid_reader::get_axis_names_from_descriptor(&device.path) {
                        Ok(axis_names) => {
                            let axes = axis_names.len();
                            eprintln!("  Detected {} axes from HID descriptor", axes);
                            (32, axes, 1)
                        }
                        Err(e) => {
                            eprintln!("  Could not read descriptor: {}", e);
                            (32, 6, 1) // Defaults
                        }
                    };

                // Determine device type
                let device_type = if is_gamepad(&full_name) {
                    "Gamepad"
                } else {
                    "Joystick"
                };

                // Create UUID from vendor_id:product_id
                let uuid = format!("{:04x}:{:04x}", device.vendor_id, device.product_id);

                joysticks.push(JoystickInfo {
                    id: joysticks.len() + 1,
                    name: full_name,
                    is_connected: true,
                    button_count,
                    axis_count,
                    hat_count,
                    device_type: device_type.to_string(),
                    uuid: Some(uuid),
                });
            }
        }
        Err(e) => {
            eprintln!("Failed to list HID devices: {}", e);
        }
    }

    // Then, check for XInput controllers (Xbox gamepads)
    if let Ok(xinput) = XInputHandle::load_default() {
        for controller_id in 0..4 {
            if xinput.get_state(controller_id).is_ok() {
                eprintln!("XInput slot {} active (Xbox Controller)", controller_id);

                joysticks.push(JoystickInfo {
                    id: joysticks.len() + 1, // Continue numbering after HID devices
                    name: format!("Xbox Controller (XInput {})", controller_id),
                    is_connected: true,
                    button_count: 15,
                    axis_count: 6,
                    hat_count: 1,
                    device_type: "Gamepad".to_string(),
                    uuid: None, // XInput devices don't have hardware UUIDs
                });
            }
        }
    }

    eprintln!("=== Total devices found: {} ===", joysticks.len());

    Ok(joysticks)
}

/// Returns detailed information for all connected devices.
pub fn list_connected_devices() -> Result<Vec<DeviceInfo>, String> {
    let mut devices = Vec::new();

    // List HID game controllers (joysticks/HOTAS)
    let hid_devices = hid_reader::list_hid_game_controllers().unwrap_or_default();

    for device in hid_devices {
        let name = device
            .product
            .as_deref()
            .unwrap_or("Unknown Device")
            .to_string();

        // Skip Xbox controllers as they'll be added via XInput
        if cfg!(windows)
            && (name.to_lowercase().contains("xbox") || name.to_lowercase().contains("xinput"))
        {
            continue;
        }

        let uuid = format!("{:04x}:{:04x}", device.vendor_id, device.product_id);
        let is_gamepad_device = is_gamepad(&name);

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
            is_connected: true,
        });
    }

    // Add XInput devices explicitly
    if let Ok(xinput) = XInputHandle::load_default() {
        for i in 0..4 {
            if xinput.get_state(i).is_ok() {
                let uuid = resolve_xinput_uuid(i);

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

    // Check if this is an XInput device
    if target_uuid.starts_with("xinput_") {
        let controller_id: u32 = target_uuid
            .strip_prefix("xinput_")
            .and_then(|s| s.parse().ok())
            .ok_or("Invalid XInput UUID")?;

        let xinput =
            XInputHandle::load_default().map_err(|e| format!("Failed to load XInput: {:?}", e))?;

        let initial_state = xinput
            .get_state(controller_id)
            .map_err(|e| format!("XInput error: {:?}", e))?;

        while start.elapsed() < timeout {
            if let Ok(state) = xinput.get_state(controller_id) {
                let axes = [
                    (
                        1,
                        (state.raw.Gamepad.sThumbLX as f32) / 32768.0,
                        (initial_state.raw.Gamepad.sThumbLX as f32) / 32768.0,
                    ),
                    (
                        2,
                        (state.raw.Gamepad.sThumbLY as f32) / 32768.0,
                        (initial_state.raw.Gamepad.sThumbLY as f32) / 32768.0,
                    ),
                    (
                        3,
                        (state.raw.Gamepad.sThumbRX as f32) / 32768.0,
                        (initial_state.raw.Gamepad.sThumbRX as f32) / 32768.0,
                    ),
                    (
                        4,
                        (state.raw.Gamepad.sThumbRY as f32) / 32768.0,
                        (initial_state.raw.Gamepad.sThumbRY as f32) / 32768.0,
                    ),
                    (
                        5,
                        (state.raw.Gamepad.bLeftTrigger as f32) / 255.0 * 2.0 - 1.0,
                        (initial_state.raw.Gamepad.bLeftTrigger as f32) / 255.0 * 2.0 - 1.0,
                    ),
                    (
                        6,
                        (state.raw.Gamepad.bRightTrigger as f32) / 255.0 * 2.0 - 1.0,
                        (initial_state.raw.Gamepad.bRightTrigger as f32) / 255.0 * 2.0 - 1.0,
                    ),
                ];

                for (axis_id, value, initial_value) in axes {
                    let delta = (value - initial_value).abs();
                    if delta > 0.15 && value.abs() > 0.15 {
                        return Ok(Some(AxisMovement { axis_id, value }));
                    }
                }
            }
            thread::sleep(Duration::from_millis(5));
        }
        return Ok(None);
    }

    // Handle HID devices
    let hid_devices = hid_reader::list_hid_game_controllers().unwrap_or_default();

    let target_device = hid_devices.iter().find(|d| {
        let uuid = format!("{:04x}:{:04x}", d.vendor_id, d.product_id);
        uuid == target_uuid
    });

    let Some(device) = target_device else {
        return Err(format!("Device not found: {}", target_uuid));
    };

    let descriptor = hid_reader::get_hid_descriptor_bytes(&device.path)
        .map_err(|e| format!("Failed to get descriptor: {}", e))?;

    let initial_bytes = hid_reader::read_hid_report(&device.path, 10)
        .map_err(|e| format!("Failed to read initial report: {}", e))?;

    let initial_report = hid_reader::parse_hid_full_report(&initial_bytes, &descriptor)
        .map_err(|e| format!("Failed to parse initial report: {}", e))?;

    while start.elapsed() < timeout {
        let report_bytes = match hid_reader::read_hid_report(&device.path, 10) {
            Ok(bytes) if !bytes.is_empty() => bytes,
            _ => {
                thread::sleep(Duration::from_millis(5));
                continue;
            }
        };

        let current_report = match hid_reader::parse_hid_full_report(&report_bytes, &descriptor) {
            Ok(report) => report,
            Err(_) => {
                thread::sleep(Duration::from_millis(5));
                continue;
            }
        };

        for (&axis_id, &current_value) in &current_report.axis_values {
            let initial_value = initial_report
                .axis_values
                .get(&axis_id)
                .copied()
                .unwrap_or(0);

            let (logical_min, logical_max) = current_report
                .axis_ranges
                .get(&axis_id)
                .copied()
                .unwrap_or((0, 65535));

            let range = (logical_max - logical_min) as f32;
            if range <= 0.0 {
                continue;
            }

            let change_abs = (current_value as i32 - initial_value as i32).abs() as f32;
            let change_percent = change_abs / range;

            if change_percent >= 0.05 {
                let normalized = ((current_value as i32 - logical_min) as f32 / range * 2.0) - 1.0;

                if normalized.abs() > 0.15 {
                    if let Ok(di_to_hid) =
                        hid_reader::get_directinput_to_hid_axis_mapping(&device.path)
                    {
                        for (axis_index, hid_usage_id) in di_to_hid {
                            if hid_usage_id == axis_id {
                                return Ok(Some(AxisMovement {
                                    axis_id: axis_index,
                                    value: normalized,
                                }));
                            }
                        }
                    }
                }
            }
        }

        thread::sleep(Duration::from_millis(5));
    }

    Ok(None)
}
