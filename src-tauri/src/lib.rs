use log::{error, info};
use std::collections::HashMap;
use std::sync::Mutex;
use tauri::Manager;
use tauri_plugin_opener::OpenerExt;

mod device_database;
mod device_profiles;
mod directinput;
mod keybindings;
mod hid_reader;

use keybindings::{Action, ActionMap, ActionMaps, AllBinds, MergedBindings, OrganizedKeybindings};

// Resources subfolder name - change this to customize the bundled resources folder
// Note: Tauri automatically names this "_up_" in the bundle, so this must match that name
const RESOURCES_SUBFOLDER: &str = "_up_";

// Command to get the app version from Cargo.toml
#[tauri::command]
fn get_app_version() -> String {
    env!("CARGO_PKG_VERSION").to_string()
}

// Struct for returning conflicting binding information
#[derive(serde::Serialize)]
struct ConflictingBinding {
    action_map_name: String,
    action_map_label: String,
    action_name: String,
    action_label: String,
}

// Struct for Star Citizen installation information
#[derive(serde::Serialize)]
struct ScInstallation {
    name: String,
    path: String,
}

// Struct for character file information
#[derive(serde::Serialize, Clone)]
struct CharacterFile {
    name: String,
    path: String,
    size: u64,
    modified: u64, // Unix timestamp in seconds
}

// Global state to hold the current keybindings
struct AppState {
    current_bindings: Option<ActionMaps>,
    all_binds: Option<AllBinds>,
    current_file_name: Option<String>,
}

impl AppState {
    fn new() -> Self {
        AppState {
            current_bindings: None,
            all_binds: None,
            current_file_name: None,
        }
    }
}

// Learn more about Tauri commands at https://tauri.app/develop/calling-rust/
#[tauri::command]
fn greet(name: &str) -> String {
    format!("Hello, {}! You've been greeted from Rust!", name)
}

#[tauri::command]
fn detect_joysticks() -> Result<Vec<directinput::JoystickInfo>, String> {
    directinput::detect_joysticks()
}

#[tauri::command]
fn get_connected_devices() -> Result<Vec<directinput::DeviceInfo>, String> {
    directinput::list_connected_devices()
}

#[tauri::command]
fn get_device_axis_mapping(device_uuid: String) -> Result<HashMap<u32, String>, String> {
    let devices = directinput::list_connected_devices()?;
    let device = devices
        .into_iter()
        .find(|d| d.uuid == device_uuid)
        .ok_or_else(|| format!("Device with UUID {} not found", device_uuid))?;

    let (_profile_name, profile) = device_profiles::profile_for_device(&device.name);
    Ok(device_profiles::invert_profile(&profile))
}

#[tauri::command]
fn detect_axis_movement(
    device_uuid: String,
    timeout_millis: Option<u64>,
) -> Result<Option<directinput::AxisMovement>, String> {
    let timeout = timeout_millis.unwrap_or(100); // Default 100ms for polling
    directinput::detect_axis_movement_for_device(&device_uuid, timeout)
}

#[tauri::command]
/// DEPRECATED: Use get_axis_names_for_device() instead
/// This function returns hardcoded axis profiles from device-database.json
/// For accurate axis detection, use the HID descriptor parsing functions
fn get_axis_profiles() -> HashMap<String, HashMap<String, u32>> {
    device_profiles::profiles_snapshot()
}

#[tauri::command]
async fn wait_for_input_binding(
    session_id: String,
    timeout_secs: u64,
) -> Result<Option<directinput::DetectedInput>, String> {
    // Run the blocking operation in a separate thread to avoid freezing the UI
    tokio::task::spawn_blocking(move || directinput::wait_for_input(session_id, timeout_secs))
        .await
        .map_err(|e| format!("Task join error: {}", e))?
}

#[tauri::command]
async fn wait_for_multiple_inputs(
    session_id: String,
    initial_timeout_secs: u64,
    collect_duration_secs: u64,
) -> Result<Vec<directinput::DetectedInput>, String> {
    // Run the blocking operation in a separate thread to avoid freezing the UI
    tokio::task::spawn_blocking(move || {
        directinput::wait_for_multiple_inputs(
            session_id,
            initial_timeout_secs,
            collect_duration_secs,
        )
    })
    .await
    .map_err(|e| format!("Task join error: {}", e))?
}

#[tauri::command]
async fn wait_for_inputs_with_events(
    window: tauri::Window,
    session_id: String,
    initial_timeout_secs: u64,
    collect_duration_secs: u64,
) -> Result<(), String> {
    // Run the blocking operation in a separate thread to avoid freezing the UI
    tokio::task::spawn_blocking(move || {
        directinput::wait_for_inputs_with_events(
            window,
            session_id,
            initial_timeout_secs,
            collect_duration_secs,
        )
    })
    .await
    .map_err(|e| format!("Task join error: {}", e))?
}

#[tauri::command]
fn load_keybindings(
    file_path: String,
    state: tauri::State<Mutex<AppState>>,
) -> Result<OrganizedKeybindings, String> {
    // Read the XML file
    let xml_content =
        std::fs::read_to_string(&file_path).map_err(|e| format!("Failed to read file: {}", e))?;

    // Parse the XML
    let action_maps = ActionMaps::from_xml(&xml_content)?;

    // Extract filename from path
    let file_name = std::path::Path::new(&file_path)
        .file_name()
        .and_then(|s| s.to_str())
        .unwrap_or("layout_exported.xml")
        .to_string();

    // Store in state
    let mut app_state = state.lock().unwrap();
    app_state.current_bindings = Some(action_maps.clone());
    app_state.current_file_name = Some(file_name);

    // Organize the data for the UI
    Ok(action_maps.organize())
}

#[tauri::command]
fn update_binding(
    action_map_name: String,
    action_name: String,
    new_input: String,
    multi_tap: Option<u32>,
    activation_mode: Option<String>,
    state: tauri::State<Mutex<AppState>>,
) -> Result<(), String> {
    eprintln!("update_binding called with:");
    eprintln!("  action_map_name: '{}'", action_map_name);
    eprintln!("  action_name: '{}'", action_name);
    eprintln!("  new_input: '{}'", new_input);
    eprintln!("  multi_tap: {:?}", multi_tap);
    eprintln!("  activation_mode: {:?}", activation_mode);

    let mut app_state = state.lock().unwrap();

    if let Some(ref mut bindings) = app_state.current_bindings {
        eprintln!("Current bindings available, checking action maps...");
        eprintln!(
            "Available action maps: {:?}",
            bindings
                .action_maps
                .iter()
                .map(|am| &am.name)
                .collect::<Vec<_>>()
        );

        // Find the action map
        if let Some(action_map) = bindings
            .action_maps
            .iter_mut()
            .find(|am| am.name == action_map_name)
        {
            eprintln!("Found action map: '{}'", action_map_name);
            eprintln!(
                "Available actions: {:?}",
                action_map
                    .actions
                    .iter()
                    .map(|a| &a.name)
                    .collect::<Vec<_>>()
            );

            // Find the action
            if let Some(action) = action_map
                .actions
                .iter_mut()
                .find(|a| a.name == action_name)
            {
                eprintln!("Found action: '{}'", action_name);

                // Create the new rebind
                let new_rebind = keybindings::Rebind {
                    input: new_input.clone(),
                    multi_tap,
                    activation_mode: activation_mode.unwrap_or_default(),
                };
                eprintln!(
                    "New rebind: input='{}', multi_tap={:?}, activation_mode='{}'",
                    new_rebind.input, new_rebind.multi_tap, new_rebind.activation_mode
                );

                let new_input_type = new_rebind.get_input_type();

                // Extract device instance from the new input (e.g., "js1" from "js1_button3")
                let new_device_instance = if let Some(underscore_pos) = new_input.find('_') {
                    new_input[..underscore_pos].to_string()
                } else {
                    new_input.clone()
                };

                // Remove any existing binding from the same device instance
                // This ensures we only have one binding per device (js1, js2, kb1, mouse1, etc.)
                action.rebinds.retain(|r| {
                    let existing_device_instance = if let Some(underscore_pos) = r.input.find('_') {
                        r.input[..underscore_pos].to_string()
                    } else {
                        r.input.clone()
                    };
                    existing_device_instance != new_device_instance
                });

                // Add the new binding
                action.rebinds.push(new_rebind);

                eprintln!("Successfully updated binding");
                return Ok(());
            } else {
                eprintln!("Action '{}' not found in action map", action_name);
            }
        } else {
            eprintln!("Action map '{}' not found", action_map_name);
        }
    } else {
        eprintln!("No current bindings loaded in state");
    }

    // If we couldn't find it in current_bindings, try to create the structure from all_binds
    eprintln!("Attempting to use all_binds as template...");
    if let Some(ref all_binds) = app_state.all_binds {
        eprintln!("AllBinds available, looking for action...");

        // Find the action in all_binds to verify it exists
        let found = all_binds.action_maps.iter().any(|am| {
            am.name == action_map_name && am.actions.iter().any(|a| a.name == action_name)
        });

        if found {
            eprintln!("Action found in all_binds, creating user binding entry");

            // Initialize or update current_bindings from all_binds structure
            if app_state.current_bindings.is_none() {
                eprintln!("Creating new current_bindings structure");
                app_state.current_bindings = Some(ActionMaps {
                    profile_name: "User Customizations".to_string(),
                    action_maps: Vec::new(),
                    categories: Vec::new(),
                    devices: keybindings::DeviceInfo {
                        keyboards: Vec::new(),
                        mice: Vec::new(),
                        joysticks: Vec::new(),
                    },
                });
            }

            if let Some(ref mut bindings) = app_state.current_bindings {
                // Find or create the action map
                if let Some(action_map) = bindings
                    .action_maps
                    .iter_mut()
                    .find(|am| am.name == action_map_name)
                {
                    // Find or create the action
                    if let Some(action) = action_map
                        .actions
                        .iter_mut()
                        .find(|a| a.name == action_name)
                    {
                        // Update existing action
                        let new_rebind = keybindings::Rebind {
                            input: new_input.clone(),
                            multi_tap,
                            activation_mode: activation_mode.clone().unwrap_or_default(),
                        };

                        // Extract device instance from the new input (e.g., "js1" from "js1_button3")
                        let new_device_instance = if let Some(underscore_pos) = new_input.find('_')
                        {
                            new_input[..underscore_pos].to_string()
                        } else {
                            new_input.clone()
                        };

                        // Remove any existing binding from the same device instance
                        action.rebinds.retain(|r| {
                            let existing_device_instance =
                                if let Some(underscore_pos) = r.input.find('_') {
                                    r.input[..underscore_pos].to_string()
                                } else {
                                    r.input.clone()
                                };
                            existing_device_instance != new_device_instance
                        });

                        // Add the new binding
                        action.rebinds.push(new_rebind);
                        eprintln!("Successfully updated binding (existing action, replaced same device instance)");
                        return Ok(());
                    } else {
                        // Create new action
                        let new_action = Action {
                            name: action_name.clone(),
                            rebinds: vec![keybindings::Rebind {
                                input: new_input,
                                multi_tap,
                                activation_mode: activation_mode.clone().unwrap_or_default(),
                            }],
                        };
                        action_map.actions.push(new_action);
                        eprintln!("Successfully updated binding (new action)");
                        return Ok(());
                    }
                } else {
                    // Create new action map
                    let new_action = Action {
                        name: action_name.clone(),
                        rebinds: vec![keybindings::Rebind {
                            input: new_input,
                            multi_tap,
                            activation_mode: activation_mode.unwrap_or_default(),
                        }],
                    };
                    let new_action_map =
                        ActionMaps::new_empty_action_map(action_map_name.clone(), vec![new_action]);
                    bindings.action_maps.push(new_action_map);
                    eprintln!("Successfully updated binding (new action map)");
                    return Ok(());
                }
            }
        } else {
            eprintln!("Action not found in all_binds either - invalid action");
        }
    } else {
        eprintln!("AllBinds not available");
    }

    Err("Action not found".to_string())
}

#[tauri::command]
fn reset_binding(
    action_map_name: String,
    action_name: String,
    state: tauri::State<Mutex<AppState>>,
) -> Result<(), String> {
    let mut app_state = state.lock().unwrap();

    eprintln!(
        "Resetting binding for action: {} in map: {}",
        action_name, action_map_name
    );

    // Remove the custom binding from current_bindings
    // This will cause the merged view to show defaults from AllBinds again
    if let Some(ref mut bindings) = app_state.current_bindings {
        if let Some(action_map) = bindings
            .action_maps
            .iter_mut()
            .find(|am| am.name == action_map_name)
        {
            // Remove the action entirely
            action_map.actions.retain(|a| a.name != action_name);
            eprintln!("Removed custom binding for action: {}", action_name);

            // If the action map is now empty, optionally remove it
            // (keeping empty action maps shouldn't cause issues)
        }
        Ok(())
    } else {
        Err("No bindings loaded".to_string())
    }
}

#[tauri::command]
fn get_current_bindings(
    state: tauri::State<Mutex<AppState>>,
) -> Result<OrganizedKeybindings, String> {
    let app_state = state.lock().unwrap();

    if let Some(ref bindings) = app_state.current_bindings {
        Ok(bindings.organize())
    } else {
        Err("No bindings loaded".to_string())
    }
}

#[tauri::command]
fn export_keybindings(
    file_path: String,
    state: tauri::State<Mutex<AppState>>,
) -> Result<(), String> {
    let mut app_state = state.lock().unwrap();

    if let Some(ref mut bindings) = app_state.current_bindings {
        // Extract filename from path (without extension)
        let mut file_name = std::path::Path::new(&file_path)
            .file_stem()
            .and_then(|s| s.to_str())
            .unwrap_or("Profile")
            .to_string();

        // Remove "_exported" suffix if present
        if file_name.ends_with("_exported") {
            file_name.truncate(file_name.len() - 9); // Remove "_exported" (9 chars)
        }

        // Update profile name to match the filename
        bindings.profile_name = file_name;
    }

    // Drop the mutable borrow before creating immutable borrow
    if let Some(ref bindings) = app_state.current_bindings {
        // Get AllBinds for category mapping
        let all_binds = app_state.all_binds.as_ref();

        // Serialize to XML with category information
        let xml_content = bindings.to_xml_with_categories(all_binds);

        // Write to file
        std::fs::write(&file_path, xml_content)
            .map_err(|e| format!("Failed to write keybindings file: {}", e))?;

        Ok(())
    } else {
        Err("No keybindings loaded to export".to_string())
    }
}

// Template management commands
#[tauri::command]
fn save_template(file_path: String, template_json: String) -> Result<(), String> {
    std::fs::write(&file_path, template_json)
        .map_err(|e| format!("Failed to save template: {}", e))?;
    Ok(())
}

#[tauri::command]
fn load_template(file_path: String) -> Result<String, String> {
    std::fs::read_to_string(&file_path).map_err(|e| format!("Failed to load template: {}", e))
}

#[tauri::command]
fn load_all_binds(
    state: tauri::State<Mutex<AppState>>,
    app_handle: tauri::AppHandle,
) -> Result<(), String> {
    // Load AllBinds.xml from resources
    let all_binds_path = if cfg!(debug_assertions) {
        // Development: look in project root
        let exe_path =
            std::env::current_exe().map_err(|e| format!("Failed to get exe path: {}", e))?;
        let exe_dir = exe_path
            .parent()
            .ok_or_else(|| "Failed to get exe directory".to_string())?;
        exe_dir
            .parent()
            .and_then(|p| p.parent())
            .and_then(|p| p.parent())
            .ok_or_else(|| "Failed to find project root".to_string())?
            .join("AllBinds.xml")
    } else {
        // Production: use Tauri's resource resolver
        // File is in the resources subfolder within resources
        app_handle
            .path()
            .resource_dir()
            .map_err(|e| format!("Failed to get resource dir: {}", e))?
            .join(RESOURCES_SUBFOLDER)
            .join("AllBinds.xml")
    };

    // Read the XML file
    let xml_content = std::fs::read_to_string(&all_binds_path)
        .map_err(|e| format!("Failed to read AllBinds.xml at {:?}: {}", all_binds_path, e))?;

    // Parse the XML
    let all_binds = AllBinds::from_xml(&xml_content)?;

    // Store in state
    let mut app_state = state.lock().unwrap();
    app_state.all_binds = Some(all_binds);

    Ok(())
}

#[tauri::command]
fn get_merged_bindings(state: tauri::State<Mutex<AppState>>) -> Result<MergedBindings, String> {
    let app_state = state.lock().unwrap();

    if let Some(ref all_binds) = app_state.all_binds {
        // Merge with user bindings if they exist
        let user_bindings = app_state.current_bindings.as_ref();
        Ok(all_binds.merge_with_user_bindings(user_bindings))
    } else {
        Err("AllBinds.xml not loaded. Please restart the application.".to_string())
    }
}

#[tauri::command]
fn get_user_customizations(
    state: tauri::State<Mutex<AppState>>,
) -> Result<Option<ActionMaps>, String> {
    let app_state = state.lock().unwrap();

    eprintln!("get_user_customizations called");
    eprintln!(
        "  has_current_bindings: {}",
        app_state.current_bindings.is_some()
    );
    if let Some(ref bindings) = app_state.current_bindings {
        eprintln!("  action_maps_count: {}", bindings.action_maps.len());
        eprintln!("  profile_name: {}", bindings.profile_name);
    }

    // Return a clone of the user's customizations (delta only)
    // This is what gets cached and is much smaller than the full merged view
    Ok(app_state.current_bindings.clone())
}

#[tauri::command]
fn restore_user_customizations(
    customizations: Option<ActionMaps>,
    state: tauri::State<Mutex<AppState>>,
) -> Result<(), String> {
    eprintln!("restore_user_customizations called");
    eprintln!("  has_data: {}", customizations.is_some());
    if let Some(ref c) = customizations {
        eprintln!("  action_maps_count: {}", c.action_maps.len());
        eprintln!("  profile_name: {}", c.profile_name);
    }

    let mut app_state = state.lock().unwrap();

    // Restore the cached user customizations (delta) to backend state
    // This allows us to preserve unsaved work across app restarts
    app_state.current_bindings = customizations;

    eprintln!("restore_user_customizations completed successfully");
    Ok(())
}

#[tauri::command]
fn find_conflicting_bindings(
    input: String,
    exclude_action_map: String,
    exclude_action: String,
    state: tauri::State<Mutex<AppState>>,
) -> Result<Vec<ConflictingBinding>, String> {
    let app_state = state.lock().unwrap();
    let mut conflicts = Vec::new();

    // Check in current bindings
    if let Some(ref bindings) = app_state.current_bindings {
        for action_map in &bindings.action_maps {
            for action in &action_map.actions {
                // Skip the action we're trying to bind
                if action_map.name == exclude_action_map && action.name == exclude_action {
                    continue;
                }

                // Check if this action has the same input bound
                for rebind in &action.rebinds {
                    if rebind.input == input {
                        conflicts.push(ConflictingBinding {
                            action_map_name: action_map.name.clone(),
                            action_map_label: action_map.name.clone(), // Will be enhanced with UI label
                            action_name: action.name.clone(),
                            action_label: action.name.clone(), // Will be enhanced with UI label
                        });
                        break; // Only add once per action
                    }
                }
            }
        }
    }

    // Enhance with UI labels from AllBinds
    if let Some(ref all_binds) = app_state.all_binds {
        for conflict in &mut conflicts {
            if let Some(all_binds_map) = all_binds
                .action_maps
                .iter()
                .find(|am| am.name == conflict.action_map_name)
            {
                conflict.action_map_label = all_binds_map.ui_label.clone();

                if let Some(all_binds_action) = all_binds_map
                    .actions
                    .iter()
                    .find(|a| a.name == conflict.action_name)
                {
                    conflict.action_label = all_binds_action.ui_label.clone();
                }
            }
        }
    }

    Ok(conflicts)
}

#[tauri::command]
fn clear_specific_binding(
    action_map_name: String,
    action_name: String,
    input_to_clear: String,
    state: tauri::State<Mutex<AppState>>,
) -> Result<(), String> {
    eprintln!("clear_specific_binding called with:");
    eprintln!("  action_map_name: '{}'", action_map_name);
    eprintln!("  action_name: '{}'", action_name);
    eprintln!("  input_to_clear: '{}'", input_to_clear);

    let mut app_state = state.lock().unwrap();

    // Determine the input type of the binding to clear
    let clear_rebind = keybindings::Rebind {
        input: input_to_clear.clone(),
        multi_tap: None,
        activation_mode: String::new(),
    };
    let input_type = clear_rebind.get_input_type();
    eprintln!("Input type to clear: {:?}", input_type);

    // Extract the joystick instance number if it's a joystick binding
    let js_instance = if matches!(input_type, keybindings::InputType::Joystick) {
        if let Some(js_part) = input_to_clear.split('_').next() {
            if js_part.starts_with("js") {
                js_part.get(2..).and_then(|s| s.parse::<u8>().ok())
            } else {
                None
            }
        } else {
            None
        }
    } else {
        None
    };

    // Check if this action has a default binding for this input type in AllBinds.xml
    let has_default_binding = if let Some(ref all_binds) = app_state.all_binds {
        all_binds.action_maps.iter().any(|am| {
            am.name == action_map_name
                && am.actions.iter().any(|a| {
                    if a.name != action_name {
                        return false;
                    }

                    // Check if there's a non-empty default binding for this input type
                    match input_type {
                        keybindings::InputType::Joystick => {
                            !a.default_joystick.is_empty() && a.default_joystick.trim() != ""
                        }
                        keybindings::InputType::Keyboard => {
                            !a.default_keyboard.is_empty() && a.default_keyboard.trim() != ""
                        }
                        keybindings::InputType::Mouse => {
                            !a.default_mouse.is_empty() && a.default_mouse.trim() != ""
                        }
                        keybindings::InputType::Gamepad => {
                            !a.default_gamepad.is_empty() && a.default_gamepad.trim() != ""
                        }
                        keybindings::InputType::Unknown => false,
                    }
                })
        })
    } else {
        false
    };

    eprintln!("Has default binding: {}", has_default_binding);

    // Only create a cleared binding if there's a default to override
    let cleared_input = if has_default_binding {
        match input_type {
            keybindings::InputType::Joystick => {
                if let Some(instance) = js_instance {
                    format!("js{}_ ", instance)
                } else {
                    "js1_ ".to_string()
                }
            }
            keybindings::InputType::Keyboard => "kb1_ ".to_string(),
            keybindings::InputType::Mouse => "mouse1_ ".to_string(),
            keybindings::InputType::Gamepad => "gp1_ ".to_string(),
            keybindings::InputType::Unknown => return Err("Unknown input type".to_string()),
        }
    } else {
        // No default binding, so we can just remove it entirely
        String::new()
    };

    eprintln!("Cleared input string: '{}'", cleared_input);

    // If there's no default binding and we're just removing, we can delete the entire action if it becomes empty
    if cleared_input.is_empty() {
        eprintln!("No default binding, removing the binding entirely");

        if let Some(ref mut bindings) = app_state.current_bindings {
            if let Some(action_map) = bindings
                .action_maps
                .iter_mut()
                .find(|am| am.name == action_map_name)
            {
                if let Some(action) = action_map
                    .actions
                    .iter_mut()
                    .find(|a| a.name == action_name)
                {
                    // Remove only the specific binding that matches input_to_clear
                    action.rebinds.retain(|r| r.input != input_to_clear);
                    eprintln!("Removed binding without adding cleared entry");
                }
            }
        }
        return Ok(());
    }

    // Initialize current_bindings if it doesn't exist
    if app_state.current_bindings.is_none() {
        eprintln!("Creating new current_bindings structure");
        app_state.current_bindings = Some(ActionMaps {
            profile_name: "User Customizations".to_string(),
            action_maps: Vec::new(),
            categories: Vec::new(),
            devices: keybindings::DeviceInfo {
                keyboards: Vec::new(),
                mice: Vec::new(),
                joysticks: Vec::new(),
            },
        });
    }

    if let Some(ref mut bindings) = app_state.current_bindings {
        // Find or create the action map
        let action_map = if let Some(am) = bindings
            .action_maps
            .iter_mut()
            .find(|am| am.name == action_map_name)
        {
            am
        } else {
            // Create new action map
            bindings.action_maps.push(ActionMap {
                name: action_map_name.clone(),
                actions: Vec::new(),
            });
            bindings.action_maps.last_mut().unwrap()
        };

        // Find or create the action
        let action = if let Some(a) = action_map
            .actions
            .iter_mut()
            .find(|a| a.name == action_name)
        {
            a
        } else {
            // Create new action
            action_map.actions.push(Action {
                name: action_name.clone(),
                rebinds: Vec::new(),
            });
            action_map.actions.last_mut().unwrap()
        };

        // Remove only the specific binding that matches input_to_clear
        action.rebinds.retain(|r| r.input != input_to_clear);

        // Add the cleared binding (with trailing space to indicate it's explicitly unbound)
        action.rebinds.push(keybindings::Rebind {
            input: cleared_input,
            multi_tap: None,
            activation_mode: String::new(),
        });

        eprintln!("Successfully cleared binding with explicit unbind entry");
        Ok(())
    } else {
        Err("Failed to initialize bindings".to_string())
    }
}

#[tauri::command]
fn clear_custom_bindings(state: tauri::State<Mutex<AppState>>) -> Result<(), String> {
    let mut app_state = state.lock().unwrap();
    app_state.current_bindings = None;
    app_state.current_file_name = None;
    Ok(())
}

#[tauri::command]
fn scan_sc_installations(base_path: String) -> Result<Vec<ScInstallation>, String> {
    use std::path::Path;

    let base = Path::new(&base_path);

    // Check if the base path exists
    if !base.exists() {
        return Err("Directory does not exist".to_string());
    }

    if !base.is_dir() {
        return Err("Path is not a directory".to_string());
    }

    let mut installations = Vec::new();

    // Common Star Citizen installation folder names
    let sc_folders = ["LIVE", "PTU", "EPTU", "TECH-PREVIEW"];

    // Scan for each potential installation
    for folder_name in &sc_folders {
        let folder_path = base.join(folder_name);

        // Check if this folder exists
        if !folder_path.exists() || !folder_path.is_dir() {
            continue;
        }

        // Check for data.p4k in the Data folder
        let data_p4k_path = folder_path.join("data.p4k");

        if data_p4k_path.exists() && data_p4k_path.is_file() {
            installations.push(ScInstallation {
                name: folder_name.to_string(),
                path: folder_path.to_string_lossy().to_string(),
            });
        }
    }

    Ok(installations)
}

#[tauri::command]
fn get_current_file_name(state: tauri::State<Mutex<AppState>>) -> Result<String, String> {
    let app_state = state.lock().unwrap();

    if let Some(ref file_name) = app_state.current_file_name {
        Ok(file_name.clone())
    } else {
        Err("No keybindings file loaded".to_string())
    }
}

#[tauri::command]
fn save_bindings_to_install(
    installation_path: String,
    state: tauri::State<Mutex<AppState>>,
) -> Result<(), String> {
    use std::path::Path;

    let app_state = state.lock().unwrap();

    // Get the current bindings
    let bindings = app_state
        .current_bindings
        .as_ref()
        .ok_or_else(|| "No keybindings loaded".to_string())?;

    // Get the filename
    let file_name = app_state
        .current_file_name
        .as_ref()
        .ok_or_else(|| "No filename stored".to_string())?;

    // Build the target path: INSTALL\user\client\0\controls\mappings
    let target_dir = Path::new(&installation_path)
        .join("user")
        .join("client")
        .join("0")
        .join("controls")
        .join("mappings");

    // Create the directory structure if it doesn't exist
    std::fs::create_dir_all(&target_dir)
        .map_err(|e| format!("Failed to create directory structure: {}", e))?;

    // Full path to the target file
    let target_file = target_dir.join(file_name);

    // Get AllBinds for category mapping
    let all_binds = app_state.all_binds.as_ref();

    // Serialize to XML with category information
    let xml_content = bindings.to_xml_with_categories(all_binds);

    // Write to the target location
    std::fs::write(&target_file, xml_content)
        .map_err(|e| format!("Failed to write keybindings file: {}", e))?;

    Ok(())
}

#[tauri::command]
fn write_binary_file(path: String, contents: Vec<u8>) -> Result<(), String> {
    std::fs::write(&path, contents).map_err(|e| format!("Failed to write file: {}", e))
}

#[tauri::command]
fn log_error(message: String, stack: Option<String>) -> Result<(), String> {
    if let Some(stack_trace) = stack {
        error!("JavaScript Error: {}\nStack: {}", message, stack_trace);
    } else {
        error!("JavaScript Error: {}", message);
    }
    Ok(())
}

#[tauri::command]
fn log_info(message: String) -> Result<(), String> {
    info!("{}", message);
    Ok(())
}

#[tauri::command]
fn get_log_file_path(app_handle: tauri::AppHandle) -> Result<String, String> {
    let log_dir = app_handle
        .path()
        .app_log_dir()
        .map_err(|e| format!("Failed to get log directory: {}", e))?;

    let log_file = log_dir.join("sc-joy-mapper.log");
    Ok(log_file.to_string_lossy().to_string())
}

#[tauri::command]
fn get_resource_dir(app_handle: tauri::AppHandle) -> Result<String, String> {
    let resource_dir = if cfg!(debug_assertions) {
        // Development: look in project root
        let exe_path =
            std::env::current_exe().map_err(|e| format!("Failed to get exe path: {}", e))?;
        let exe_dir = exe_path
            .parent()
            .ok_or_else(|| "Failed to get exe directory".to_string())?;
        exe_dir
            .parent()
            .and_then(|p| p.parent())
            .and_then(|p| p.parent())
            .ok_or_else(|| "Failed to find project root".to_string())?
            .to_path_buf()
    } else {
        // Production: use Tauri's resource resolver
        app_handle
            .path()
            .resource_dir()
            .map_err(|e| format!("Failed to get resource dir: {}", e))?
            .join(RESOURCES_SUBFOLDER)
    };

    Ok(resource_dir.to_string_lossy().to_string())
}

#[tauri::command]
async fn open_url(app_handle: tauri::AppHandle, url: String) -> Result<(), String> {
    app_handle
        .opener()
        .open_url(&url, None::<&str>)
        .map_err(|e| format!("Failed to open URL: {}", e))
}

fn setup_logging(app_handle: &tauri::AppHandle) -> Result<(), Box<dyn std::error::Error>> {
    use std::fs::OpenOptions;
    use std::io::Write;

    // Get log directory
    let log_dir = app_handle.path().app_log_dir()?;
    std::fs::create_dir_all(&log_dir)?;

    let log_file = log_dir.join("sc-joy-mapper.log");

    // Set up file logging with env_logger
    let target = Box::new(
        OpenOptions::new()
            .create(true)
            .append(true)
            .open(&log_file)?,
    );

    env_logger::Builder::from_default_env()
        .target(env_logger::Target::Pipe(target))
        .format(|buf, record| {
            writeln!(
                buf,
                "[{}] {} - {}",
                chrono::Local::now().format("%Y-%m-%d %H:%M:%S"),
                record.level(),
                record.args()
            )
        })
        .init();

    info!("=== SC Joy Mapper Started ===");
    info!("Version: {}", env!("CARGO_PKG_VERSION"));
    info!("Log file: {:?}", log_file);

    Ok(())
}

// Struct for unbind profile generation result
#[derive(serde::Serialize)]
struct UnbindProfileResult {
    saved_locations: Vec<String>,
}

// Struct for unbind profile removal result
#[derive(serde::Serialize)]
struct RemoveUnbindResult {
    removed_count: usize,
}

#[tauri::command]
fn generate_unbind_profile(
    devices: keybindings::DeviceSelection,
    base_path: String,
    state: tauri::State<Mutex<AppState>>,
) -> Result<UnbindProfileResult, String> {
    use std::fs;

    info!(
        "Generating unbind profile for devices: keyboard={}, mouse={}, gamepad={}, js1={}, js2={}",
        devices.keyboard, devices.mouse, devices.gamepad, devices.joystick1, devices.joystick2
    );
    info!("Using base path: {}", base_path);

    // Get AllBinds from state
    let app_state = state
        .lock()
        .map_err(|e| format!("Failed to lock state: {}", e))?;
    let all_binds = app_state
        .all_binds
        .as_ref()
        .ok_or("AllBinds not loaded. Please load the keybindings first.")?;

    // Generate the unbind XML
    let unbind_xml = keybindings::generate_unbind_xml(all_binds, &devices)?;

    info!("Generated unbind XML, length: {} bytes", unbind_xml.len());

    // Try to save to SC installation directories
    let mut saved_locations = Vec::new();

    // Get SC installations
    match scan_sc_installations(base_path.clone()) {
        Ok(installations) => {
            info!("Found {} SC installations", installations.len());
            for install in installations {
                info!(
                    "Processing installation: {} at {}",
                    install.name, install.path
                );
                let mappings_dir = format!("{}\\user\\client\\0\\controls\\mappings", install.path);

                // Create directory if it doesn't exist
                if let Err(e) = fs::create_dir_all(&mappings_dir) {
                    error!(
                        "Failed to create mappings directory {}: {}",
                        mappings_dir, e
                    );
                    continue;
                }

                let file_path = format!("{}\\UNBIND_ALL.xml", mappings_dir);
                info!("Attempting to write to: {}", file_path);
                match fs::write(&file_path, &unbind_xml) {
                    Ok(_) => {
                        info!("Successfully saved unbind profile to: {}", file_path);
                        saved_locations.push(file_path);
                    }
                    Err(e) => error!("Failed to write to {}: {}", file_path, e),
                }
            }
        }
        Err(e) => {
            error!(
                "Failed to scan SC installations from base path '{}': {}",
                base_path, e
            );
        }
    }

    // If no installations found, save to current directory as fallback
    if saved_locations.is_empty() {
        let fallback_path = "UNBIND_ALL.xml";
        fs::write(fallback_path, &unbind_xml)
            .map_err(|e| format!("Failed to write unbind profile: {}", e))?;
        saved_locations.push(fallback_path.to_string());
        info!(
            "Saved unbind profile to current directory: {}",
            fallback_path
        );
    }

    Ok(UnbindProfileResult { saved_locations })
}

#[tauri::command]
fn remove_unbind_profile() -> Result<RemoveUnbindResult, String> {
    use std::fs;

    info!("Removing unbind profile files");

    let mut removed_count = 0;

    // Get base path for SC installations
    let base_path = "C:\\Program Files\\Roberts Space Industries\\StarCitizen".to_string();

    // Get SC installations
    match scan_sc_installations(base_path) {
        Ok(installations) => {
            for install in installations {
                let file_path = format!(
                    "{}\\user\\client\\0\\controls\\mappings\\UNBIND_ALL.xml",
                    install.path
                );

                if fs::metadata(&file_path).is_ok() {
                    match fs::remove_file(&file_path) {
                        Ok(_) => {
                            info!("Removed unbind profile from: {}", file_path);
                            removed_count += 1;
                        }
                        Err(e) => error!("Failed to remove {}: {}", file_path, e),
                    }
                }
            }
        }
        Err(e) => {
            error!("Failed to scan SC installations: {}", e);
        }
    }

    // Also try to remove from current directory
    let fallback_path = "UNBIND_ALL.xml";
    if fs::metadata(fallback_path).is_ok() {
        match fs::remove_file(fallback_path) {
            Ok(_) => {
                info!("Removed unbind profile from current directory");
                removed_count += 1;
            }
            Err(e) => error!("Failed to remove {}: {}", fallback_path, e),
        }
    }

    Ok(RemoveUnbindResult { removed_count })
}

#[tauri::command]
fn scan_character_files(directory_path: String) -> Result<Vec<CharacterFile>, String> {
    use std::fs;
    use std::time::UNIX_EPOCH;

    let dir_path = std::path::Path::new(&directory_path);

    // Check if directory exists
    if !dir_path.exists() {
        // Return empty list instead of error if directory doesn't exist
        return Ok(Vec::new());
    }

    if !dir_path.is_dir() {
        return Err("Path is not a directory".to_string());
    }

    let mut characters = Vec::new();

    // Read directory entries
    let entries = fs::read_dir(dir_path)
        .map_err(|e| format!("Failed to read directory: {}", e))?;

    for entry in entries {
        let entry = entry.map_err(|e| format!("Failed to read entry: {}", e))?;
        let path = entry.path();

        // Only process .chf files
        if path.is_file() {
            if let Some(ext) = path.extension() {
                if ext == "chf" {
                    if let Some(file_name) = path.file_name() {
                        if let Some(name_str) = file_name.to_str() {
                            // Get file metadata
                            let metadata = fs::metadata(&path)
                                .map_err(|e| format!("Failed to read metadata: {}", e))?;

                            let size = metadata.len();
                            let modified = metadata
                                .modified()
                                .map_err(|e| format!("Failed to get modified time: {}", e))?
                                .duration_since(UNIX_EPOCH)
                                .map_err(|e| format!("Time error: {}", e))?
                                .as_secs();

                            characters.push(CharacterFile {
                                name: name_str.to_string(),
                                path: path.to_string_lossy().to_string(),
                                size,
                                modified,
                            });
                        }
                    }
                }
            }
        }
    }

    // Sort by name
    characters.sort_by(|a, b| a.name.cmp(&b.name));

    Ok(characters)
}

#[tauri::command]
fn deploy_character_to_installation(
    character_name: String,
    library_path: String,
    installation_path: String,
) -> Result<(), String> {
    use std::fs;
    use std::path::Path;

    let source_path = Path::new(&library_path).join(&character_name);
    let target_dir = Path::new(&installation_path)
        .join("user")
        .join("client")
        .join("0")
        .join("customcharacters");

    // Create target directory if it doesn't exist
    fs::create_dir_all(&target_dir)
        .map_err(|e| format!("Failed to create target directory: {}", e))?;

    let target_path = target_dir.join(&character_name);

    // Copy the file
    fs::copy(&source_path, &target_path)
        .map_err(|e| format!("Failed to copy character file: {}", e))?;

    info!(
        "Deployed character {} from {} to {}",
        character_name,
        source_path.display(),
        target_path.display()
    );

    Ok(())
}

#[tauri::command]
fn import_character_to_library(
    character_name: String,
    installation_path: String,
    library_path: String,
) -> Result<(), String> {
    use std::fs;
    use std::path::Path;

    let source_path = Path::new(&installation_path)
        .join("user")
        .join("client")
        .join("0")
        .join("customcharacters")
        .join(&character_name);

    let target_dir = Path::new(&library_path);

    // Create library directory if it doesn't exist
    fs::create_dir_all(&target_dir)
        .map_err(|e| format!("Failed to create library directory: {}", e))?;

    let target_path = target_dir.join(&character_name);

    // Copy the file
    fs::copy(&source_path, &target_path)
        .map_err(|e| format!("Failed to copy character file: {}", e))?;

    info!(
        "Imported character {} from {} to {}",
        character_name,
        source_path.display(),
        target_path.display()
    );

    Ok(())
}

// ===== HID Debug Commands =====

#[tauri::command]
fn list_hid_devices() -> Result<Vec<hid_reader::HidDeviceListItem>, String> {
    hid_reader::list_hid_game_controllers()
}

#[tauri::command]
fn read_hid_device_report(device_path: String, timeout_ms: Option<i32>) -> Result<Vec<u8>, String> {
    let timeout = timeout_ms.unwrap_or(50);
    hid_reader::read_hid_report(&device_path, timeout)
}

#[tauri::command]
fn parse_hid_report(report: Vec<u8>) -> Result<hid_reader::HidAxisReport, String> {
    // Don't read descriptor on every report - that causes device conflicts
    // Axis names should be fetched once at startup via get_hid_axis_names
    let empty_names = std::collections::HashMap::new();
    hid_reader::parse_hid_axes(&report, &empty_names)
}

#[tauri::command]
fn get_hid_axis_names(device_path: String) -> Result<std::collections::HashMap<u32, String>, String> {
    hid_reader::get_axis_names_from_descriptor(&device_path)
}

#[tauri::command]
fn get_axis_names_for_device(device_name: String) -> Result<std::collections::HashMap<u32, String>, String> {
    // Try to find a matching HID device by name
    // This helps bridge the gap between DirectInput devices and HID devices
    
    let hid_devices = hid_reader::list_hid_game_controllers()
        .map_err(|e| format!("Failed to list HID devices: {}", e))?;
    
    // Try to find a device with a matching name
    let matching_device = hid_devices.iter().find(|dev| {
        if let Some(product) = &dev.product {
            // Simple case-insensitive substring match
            product.to_lowercase().contains(&device_name.to_lowercase()) ||
            device_name.to_lowercase().contains(&product.to_lowercase())
        } else {
            false
        }
    });
    
    if let Some(device) = matching_device {
        eprintln!("[Axis Names] Found HID device for '{}': {:?}", device_name, device.product);
        hid_reader::get_axis_names_from_descriptor(&device.path)
    } else {
        eprintln!("[Axis Names] No matching HID device found for '{}'", device_name);
        Err(format!("No HID device found matching name: {}", device_name))
    }
}

// ===== End HID Debug Commands =====

#[tauri::command]
fn delete_character_from_library(
    character_name: String,
    library_path: String,
) -> Result<(), String> {
    use std::fs;
    use std::path::Path;

    let file_path = Path::new(&library_path).join(&character_name);

    // Delete the file
    fs::remove_file(&file_path).map_err(|e| format!("Failed to delete character file: {}", e))?;

    info!("Deleted character {} from library", character_name);

    Ok(())
}

#[tauri::command]
fn delete_character_from_installation(
    character_name: String,
    installation_path: String,
) -> Result<(), String> {
    use std::fs;
    use std::path::Path;

    // Build path to character file in installation
    // Path format: {install}\user\client\0\customcharacters\{character_name}
    let char_file_path = Path::new(&installation_path)
        .join("user")
        .join("client")
        .join("0")
        .join("customcharacters")
        .join(&character_name);

    // Delete the file
    fs::remove_file(&char_file_path).map_err(|e| format!("Failed to delete character file: {}", e))?;

    info!("Deleted character {} from installation", character_name);

    Ok(())
}

#[cfg_attr(mobile, tauri::mobile_entry_point)]
pub fn run() {
    let app = tauri::Builder::default()
        .plugin(tauri_plugin_opener::init())
        .plugin(tauri_plugin_dialog::init())
        .manage(Mutex::new(AppState::new()))
        .invoke_handler(tauri::generate_handler![
            get_app_version,
            greet,
            detect_joysticks,
            get_connected_devices,
            get_device_axis_mapping,
            detect_axis_movement,
            get_axis_profiles,
            wait_for_input_binding,
            wait_for_multiple_inputs,
            wait_for_inputs_with_events,
            load_keybindings,
            update_binding,
            reset_binding,
            get_current_bindings,
            export_keybindings,
            save_template,
            load_template,
            load_all_binds,
            get_merged_bindings,
            get_user_customizations,
            restore_user_customizations,
            find_conflicting_bindings,
            clear_specific_binding,
            clear_custom_bindings,
            scan_sc_installations,
            get_current_file_name,
            save_bindings_to_install,
            write_binary_file,
            log_error,
            log_info,
            get_log_file_path,
            get_resource_dir,
            open_url,
            generate_unbind_profile,
            remove_unbind_profile,
            scan_character_files,
            deploy_character_to_installation,
            import_character_to_library,
            delete_character_from_library,
            delete_character_from_installation,
            list_hid_devices,
            read_hid_device_report,
            parse_hid_report,
            get_hid_axis_names,
            get_axis_names_for_device
        ])
        .setup(|app| {
            // Set up logging
            if let Err(e) = setup_logging(app.handle()) {
                eprintln!("Failed to set up logging: {}", e);
            }

            // Initialize device database
            let db_path = if cfg!(debug_assertions) {
                // Development: look in src-tauri directory
                let exe_path =
                    std::env::current_exe().map_err(|e| format!("Failed to get exe path: {}", e))?;
                let exe_dir = exe_path
                    .parent()
                    .ok_or_else(|| "Failed to get exe directory".to_string())?;
                exe_dir
                    .parent()
                    .and_then(|p| p.parent())
                    .and_then(|p| p.parent())
                    .ok_or_else(|| "Failed to find project root".to_string())?
                    .join("src-tauri")
                    .join("device-database.json")
            } else {
                // Production: look in resources
                let resource_dir = app
                    .path()
                    .resource_dir()
                    .map_err(|e| format!("Failed to get resource dir: {}", e))?;
                resource_dir
                    .join(RESOURCES_SUBFOLDER)
                    .join("device-database.json")
            };

            eprintln!("Attempting to load device database from: {:?}", db_path);
            eprintln!("Database exists: {}", db_path.exists());
            
            if let Err(e) = device_database::DeviceDatabase::init(&db_path) {
                eprintln!("Warning: Failed to initialize device database: {}", e);
                eprintln!("Device lookup will fall back to OS device names");
                // Don't fail startup if database fails to load
            } else {
                info!("Device database initialized successfully");
                eprintln!("Device database loaded successfully!");
            }

            Ok(())
        })
        .build(tauri::generate_context!())
        .expect("error while running tauri application")
        .run(|_app_handle, event| {
            if let tauri::RunEvent::ExitRequested { .. } = event {
                info!("=== SC Joy Mapper Shutting Down ===");
            }
        });
}
