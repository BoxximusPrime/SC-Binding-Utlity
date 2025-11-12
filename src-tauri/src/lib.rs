use std::sync::Mutex;
use tauri::Manager;
use log::{info, error};

mod keybindings;
mod directinput;

use keybindings::{ActionMaps, OrganizedKeybindings, AllBinds, MergedBindings, ActionMap, Action};

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
async fn wait_for_input_binding(session_id: String, timeout_secs: u64) -> Result<Option<directinput::DetectedInput>, String> {
    // Run the blocking operation in a separate thread to avoid freezing the UI
    tokio::task::spawn_blocking(move || {
        directinput::wait_for_input(session_id, timeout_secs)
    })
    .await
    .map_err(|e| format!("Task join error: {}", e))?
}

#[tauri::command]
async fn wait_for_multiple_inputs(session_id: String, initial_timeout_secs: u64, collect_duration_secs: u64) -> Result<Vec<directinput::DetectedInput>, String> {
    // Run the blocking operation in a separate thread to avoid freezing the UI
    tokio::task::spawn_blocking(move || {
        directinput::wait_for_multiple_inputs(session_id, initial_timeout_secs, collect_duration_secs)
    })
    .await
    .map_err(|e| format!("Task join error: {}", e))?
}

#[tauri::command]
async fn wait_for_inputs_with_events(window: tauri::Window, session_id: String, initial_timeout_secs: u64, collect_duration_secs: u64) -> Result<(), String> {
    // Run the blocking operation in a separate thread to avoid freezing the UI
    tokio::task::spawn_blocking(move || {
        directinput::wait_for_inputs_with_events(window, session_id, initial_timeout_secs, collect_duration_secs)
    })
    .await
    .map_err(|e| format!("Task join error: {}", e))?
}

#[tauri::command]
fn load_keybindings(file_path: String, state: tauri::State<Mutex<AppState>>) -> Result<OrganizedKeybindings, String> {
    // Read the XML file
    let xml_content = std::fs::read_to_string(&file_path)
        .map_err(|e| format!("Failed to read file: {}", e))?;
    
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
    state: tauri::State<Mutex<AppState>>
) -> Result<(), String> {
    eprintln!("update_binding called with:");
    eprintln!("  action_map_name: '{}'", action_map_name);
    eprintln!("  action_name: '{}'", action_name);
    eprintln!("  new_input: '{}'", new_input);
    
    let mut app_state = state.lock().unwrap();
    
    if let Some(ref mut bindings) = app_state.current_bindings {
        eprintln!("Current bindings available, checking action maps...");
        eprintln!("Available action maps: {:?}", bindings.action_maps.iter().map(|am| &am.name).collect::<Vec<_>>());
        
        // Find the action map
        if let Some(action_map) = bindings.action_maps.iter_mut().find(|am| am.name == action_map_name) {
            eprintln!("Found action map: '{}'", action_map_name);
            eprintln!("Available actions: {:?}", action_map.actions.iter().map(|a| &a.name).collect::<Vec<_>>());
            
            // Find the action
            if let Some(action) = action_map.actions.iter_mut().find(|a| a.name == action_name) {
                eprintln!("Found action: '{}'", action_name);
                
                // Determine the input type of the new binding
                let new_rebind = keybindings::Rebind {
                    input: new_input.clone(),
                };
                let new_input_type = new_rebind.get_input_type();
                eprintln!("New binding input type: {:?}", new_input_type);
                
                // Remove any existing rebinds of the same input type
                action.rebinds.retain(|r| r.get_input_type() != new_input_type);
                
                // Add the new binding
                action.rebinds.push(new_rebind);
                eprintln!("Successfully updated binding (replaced same input type)");
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
        let found = all_binds.action_maps.iter()
            .any(|am| am.name == action_map_name && 
                      am.actions.iter().any(|a| a.name == action_name));
        
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
                if let Some(action_map) = bindings.action_maps.iter_mut().find(|am| am.name == action_map_name) {
                    // Find or create the action
                    if let Some(action) = action_map.actions.iter_mut().find(|a| a.name == action_name) {
                        // Update existing action - replace bindings of the same input type
                        let new_rebind = keybindings::Rebind {
                            input: new_input.clone(),
                        };
                        let new_input_type = new_rebind.get_input_type();
                        
                        // Remove any existing rebinds of the same input type
                        action.rebinds.retain(|r| r.get_input_type() != new_input_type);
                        
                        // Add the new binding
                        action.rebinds.push(new_rebind);
                        eprintln!("Successfully updated binding (existing action, replaced same input type)");
                        return Ok(());
                    } else {
                        // Create new action
                        let new_action = Action {
                            name: action_name.clone(),
                            rebinds: vec![keybindings::Rebind {
                                input: new_input,
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
                        }],
                    };
                    let new_action_map = ActionMaps::new_empty_action_map(action_map_name.clone(), vec![new_action]);
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
    state: tauri::State<Mutex<AppState>>
) -> Result<(), String> {
    let mut app_state = state.lock().unwrap();
    
    eprintln!("Resetting binding for action: {} in map: {}", action_name, action_map_name);
    
    // Remove the custom binding from current_bindings
    // This will cause the merged view to show defaults from AllBinds again
    if let Some(ref mut bindings) = app_state.current_bindings {
        if let Some(action_map) = bindings.action_maps.iter_mut().find(|am| am.name == action_map_name) {
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
fn get_current_bindings(state: tauri::State<Mutex<AppState>>) -> Result<OrganizedKeybindings, String> {
    let app_state = state.lock().unwrap();
    
    if let Some(ref bindings) = app_state.current_bindings {
        Ok(bindings.organize())
    } else {
        Err("No bindings loaded".to_string())
    }
}

#[tauri::command]
fn export_keybindings(file_path: String, state: tauri::State<Mutex<AppState>>) -> Result<(), String> {
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
    std::fs::read_to_string(&file_path)
        .map_err(|e| format!("Failed to load template: {}", e))
}

#[tauri::command]
fn load_all_binds(state: tauri::State<Mutex<AppState>>, app_handle: tauri::AppHandle) -> Result<(), String> {
    // Load AllBinds.xml from resources
    let all_binds_path = if cfg!(debug_assertions) {
        // Development: look in project root
        let exe_path = std::env::current_exe()
            .map_err(|e| format!("Failed to get exe path: {}", e))?;
        let exe_dir = exe_path.parent()
            .ok_or_else(|| "Failed to get exe directory".to_string())?;
        exe_dir.parent()
            .and_then(|p| p.parent())
            .and_then(|p| p.parent())
            .ok_or_else(|| "Failed to find project root".to_string())?
            .join("AllBinds.xml")
    } else {
        // Production: use Tauri's resource resolver
        // File is in the _up_ subfolder within resources
        app_handle.path()
            .resource_dir()
            .map_err(|e| format!("Failed to get resource dir: {}", e))?
            .join("_up_")
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
fn find_conflicting_bindings(
    input: String,
    exclude_action_map: String,
    exclude_action: String,
    state: tauri::State<Mutex<AppState>>
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
            if let Some(all_binds_map) = all_binds.action_maps.iter()
                .find(|am| am.name == conflict.action_map_name) {
                conflict.action_map_label = all_binds_map.ui_label.clone();
                
                if let Some(all_binds_action) = all_binds_map.actions.iter()
                    .find(|a| a.name == conflict.action_name) {
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
    state: tauri::State<Mutex<AppState>>
) -> Result<(), String> {
    eprintln!("clear_specific_binding called with:");
    eprintln!("  action_map_name: '{}'", action_map_name);
    eprintln!("  action_name: '{}'", action_name);
    eprintln!("  input_to_clear: '{}'", input_to_clear);
    
    let mut app_state = state.lock().unwrap();
    
    // Determine the input type of the binding to clear
    let clear_rebind = keybindings::Rebind {
        input: input_to_clear.clone(),
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
        all_binds.action_maps.iter()
            .any(|am| am.name == action_map_name && 
                am.actions.iter().any(|a| {
                    if a.name != action_name {
                        return false;
                    }
                    
                    // Check if there's a non-empty default binding for this input type
                    match input_type {
                        keybindings::InputType::Joystick => {
                            !a.default_joystick.is_empty() && a.default_joystick.trim() != ""
                        },
                        keybindings::InputType::Keyboard => {
                            !a.default_keyboard.is_empty() && a.default_keyboard.trim() != ""
                        },
                        keybindings::InputType::Mouse => {
                            !a.default_mouse.is_empty() && a.default_mouse.trim() != ""
                        },
                        keybindings::InputType::Gamepad => {
                            !a.default_gamepad.is_empty() && a.default_gamepad.trim() != ""
                        },
                        keybindings::InputType::Unknown => false,
                    }
                }))
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
            },
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
            if let Some(action_map) = bindings.action_maps.iter_mut().find(|am| am.name == action_map_name) {
                if let Some(action) = action_map.actions.iter_mut().find(|a| a.name == action_name) {
                    // Remove existing bindings of this input type
                    action.rebinds.retain(|r| r.get_input_type() != input_type);
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
        let action_map = if let Some(am) = bindings.action_maps.iter_mut().find(|am| am.name == action_map_name) {
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
        let action = if let Some(a) = action_map.actions.iter_mut().find(|a| a.name == action_name) {
            a
        } else {
            // Create new action
            action_map.actions.push(Action {
                name: action_name.clone(),
                rebinds: Vec::new(),
            });
            action_map.actions.last_mut().unwrap()
        };
        
        // Remove existing bindings of this input type
        action.rebinds.retain(|r| r.get_input_type() != input_type);
        
        // Add the cleared binding (with trailing space to indicate it's explicitly unbound)
        action.rebinds.push(keybindings::Rebind {
            input: cleared_input,
        });
        
        eprintln!("Successfully cleared binding with explicit unbind entry");
        Ok(())
    } else {
        Err("Failed to initialize bindings".to_string())
    }
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
    state: tauri::State<Mutex<AppState>>
) -> Result<(), String> {
    use std::path::Path;
    
    let app_state = state.lock().unwrap();
    
    // Get the current bindings
    let bindings = app_state.current_bindings.as_ref()
        .ok_or_else(|| "No keybindings loaded".to_string())?;
    
    // Get the filename
    let file_name = app_state.current_file_name.as_ref()
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
    std::fs::write(&path, contents)
        .map_err(|e| format!("Failed to write file: {}", e))
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
    let log_dir = app_handle.path()
        .app_log_dir()
        .map_err(|e| format!("Failed to get log directory: {}", e))?;
    
    let log_file = log_dir.join("sc-joy-mapper.log");
    Ok(log_file.to_string_lossy().to_string())
}

fn setup_logging(app_handle: &tauri::AppHandle) -> Result<(), Box<dyn std::error::Error>> {
    use std::fs::OpenOptions;
    use std::io::Write;
    
    // Get log directory
    let log_dir = app_handle.path().app_log_dir()?;
    std::fs::create_dir_all(&log_dir)?;
    
    let log_file = log_dir.join("sc-joy-mapper.log");
    
    // Set up file logging with env_logger
    let target = Box::new(OpenOptions::new()
        .create(true)
        .append(true)
        .open(&log_file)?);
    
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
            find_conflicting_bindings,
            clear_specific_binding,
            scan_sc_installations,
            get_current_file_name,
            save_bindings_to_install,
            write_binary_file,
            log_error,
            log_info,
            get_log_file_path
        ])
        .setup(|app| {
            // Set up logging
            if let Err(e) = setup_logging(app.handle()) {
                eprintln!("Failed to set up logging: {}", e);
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

