use serde::Deserialize;
use serde::Serialize;
use std::collections::HashMap;

/// Represents the entire Star Citizen keybinding file
#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct ActionMaps {
    pub profile_name: String,
    pub action_maps: Vec<ActionMap>,
    pub categories: Vec<Category>,
    pub devices: DeviceInfo,
}

/// Represents the AllBinds.xml master file with all available actions
#[derive(Debug, Serialize, Clone)]
pub struct AllBinds {
    pub action_maps: Vec<AllBindsActionMap>,
}

/// Action map from AllBinds.xml with UI metadata
#[derive(Debug, Serialize, Clone)]
pub struct AllBindsActionMap {
    pub name: String,
    pub version: String,
    pub ui_label: String,
    pub ui_category: String,
    pub actions: Vec<AllBindsAction>,
}

/// Action from AllBinds.xml with default bindings and UI metadata
#[derive(Debug, Serialize, Clone)]
pub struct AllBindsAction {
    pub name: String,
    pub ui_label: String,
    pub ui_description: String,
    pub category: String,
    pub activation_mode: String,
    pub on_hold: bool,
    pub default_keyboard: String,
    pub default_mouse: String,
    pub default_gamepad: String,
    pub default_joystick: String,
}

/// A single control option (child of <options> element)
/// e.g., <flight_move_pitch invert="1"/>
#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct ControlOption {
    pub name: String,
    pub attributes: Vec<(String, String)>,
}

/// Device options entry that preserves control settings
#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct DeviceOptions {
    pub device_type: String,
    pub instance: String,
    pub product: String,
    pub control_options: Vec<ControlOption>,
}

/// UI header containing metadata about devices and categories
#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct DeviceInfo {
    pub keyboards: Vec<String>,
    pub mice: Vec<String>,
    pub joysticks: Vec<String>,
    #[serde(default)]
    pub device_options: Vec<DeviceOptions>,
}

/// A single category
#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct Category {
    pub label: String,
}

/// A group of actions (e.g., "seat_general", "spaceship_general")
#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct ActionMap {
    pub name: String,
    pub actions: Vec<Action>,
}

/// A single action that can be bound to inputs
#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct Action {
    pub name: String,
    pub rebinds: Vec<Rebind>,
}

/// A keybinding for an action
#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct Rebind {
    pub input: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    #[serde(default)]
    pub multi_tap: Option<u32>,
    #[serde(skip_serializing_if = "String::is_empty")]
    #[serde(default)]
    pub activation_mode: String,
    // VKB layer information (not serialized to XML)
    #[serde(skip)]
    pub vkb_layer: Option<String>,
    #[serde(skip)]
    pub vkb_tempo: Option<String>,
    #[serde(skip)]
    pub vkb_physical_id: Option<u8>,
    #[serde(skip)]
    pub vkb_physical_info: Option<String>,
}

/// Parsed input type for easier filtering
#[derive(Debug, Serialize, Deserialize, Clone, PartialEq)]
pub enum InputType {
    Keyboard,
    Mouse,
    Joystick,
    Gamepad,
    Unknown,
}

impl Rebind {
    /// Parse the input type from the rebind string
    /// Examples: "kb_space", "js1_button3", "js2_button15", "mouse1_left", "LALT+js1_button3", "kb_u+lshift"
    pub fn get_input_type(&self) -> InputType {
        let input = self.input.trim();

        // Check if this is an unbound placeholder (device prefix followed by underscore and only whitespace)
        // Examples: "kb_ ", "kb_", "js1_ ", "mouse1_", "gp1_ "
        if let Some(after_underscore_pos) = input.find('_') {
            let after_underscore = &input[after_underscore_pos + 1..];

            // If everything after the underscore is whitespace or empty, it's unbound
            if after_underscore.trim().is_empty() {
                return InputType::Unknown;
            }
        }

        // Check all parts for device prefixes (handles modifiers in any position)
        if input.contains('+') {
            // Split and check each part for device prefix
            for part in input.split('+') {
                let part = part.trim();
                if part.starts_with("kb") {
                    return InputType::Keyboard;
                } else if part.starts_with("mouse") {
                    return InputType::Mouse;
                } else if part.starts_with("js") {
                    return InputType::Joystick;
                } else if part.starts_with("gp") {
                    return InputType::Gamepad;
                }
            }
        } else {
            // No modifiers, check the whole string
            if input.starts_with("kb") {
                return InputType::Keyboard;
            } else if input.starts_with("mouse") {
                return InputType::Mouse;
            } else if input.starts_with("js") {
                return InputType::Joystick;
            } else if input.starts_with("gp") {
                return InputType::Gamepad;
            }
        }

        InputType::Unknown
    }

    /// Get the device type from the input string, even for cleared/unbound bindings
    /// This is used for deduplication - SC only allows ONE binding per device TYPE per action
    /// Examples: "js1_ " -> Joystick, "kb_ " -> Keyboard, "js2_button5" -> Joystick
    pub fn get_device_type(&self) -> InputType {
        let input = self.input.trim();

        // Check the device prefix regardless of whether there's an actual binding
        if input.contains('+') {
            // Split and check each part for device prefix
            for part in input.split('+') {
                let part = part.trim();
                if part.starts_with("kb") {
                    return InputType::Keyboard;
                } else if part.starts_with("mouse") {
                    return InputType::Mouse;
                } else if part.starts_with("js") {
                    return InputType::Joystick;
                } else if part.starts_with("gp") {
                    return InputType::Gamepad;
                }
            }
        } else {
            // No modifiers, check the whole string
            if input.starts_with("kb") {
                return InputType::Keyboard;
            } else if input.starts_with("mouse") {
                return InputType::Mouse;
            } else if input.starts_with("js") {
                return InputType::Joystick;
            } else if input.starts_with("gp") {
                return InputType::Gamepad;
            }
        }

        InputType::Unknown
    }

    /// Get a human-readable description of the binding
    pub fn get_display_name(&self) -> String {
        let input = self.input.trim();

        // Check if unbound - empty or ends with underscore and space(s)
        if input.is_empty()
            || input.ends_with("_ ")
            || input.ends_with("_  ")
            || input.ends_with("_")
        {
            return "Unbound".to_string();
        }

        // Check for modifier prefixes (e.g., "lalt+rctrl+js1_button3" or "LALT+RCTRL+js1_button3")
        let mut modifiers = Vec::new();
        let mut remaining_input = input;

        // Extract all modifiers from the beginning (case-insensitive)
        while let Some((prefix, rest)) = remaining_input.split_once('+') {
            let prefix_upper = prefix.trim().to_uppercase();
            if matches!(
                prefix_upper.as_str(),
                "LALT" | "RALT" | "LCTRL" | "RCTRL" | "LSHIFT" | "RSHIFT"
            ) {
                modifiers.push(prefix_upper);
                remaining_input = rest;
            } else {
                break;
            }
        }

        // Parse the remaining input string (the actual binding)
        let base_display = if let Some((device, binding)) = remaining_input.split_once('_') {
            let binding = binding.trim();
            if binding.is_empty() {
                "Unbound".to_string()
            } else {
                match &device[..2.min(device.len())] {
                    "kb" => format!("Keyboard - {}", Self::format_binding(binding)),
                    "js" => {
                        let instance = device.get(2..3).unwrap_or("1");
                        format!("Joystick {} - {}", instance, Self::format_binding(binding))
                    }
                    "mo" => format!("Mouse - {}", Self::format_binding(binding)),
                    "gp" => format!("Gamepad - {}", Self::format_binding(binding)),
                    _ => format!("{} - {}", device, Self::format_binding(binding)),
                }
            }
        } else {
            remaining_input.to_string()
        };

        // Format modifiers for display
        if !modifiers.is_empty() {
            let modifier_display: Vec<String> = modifiers
                .iter()
                .map(|m| match m.as_str() {
                    "LALT" => "Left Alt".to_string(),
                    "RALT" => "Right Alt".to_string(),
                    "LCTRL" => "Left Ctrl".to_string(),
                    "RCTRL" => "Right Ctrl".to_string(),
                    "LSHIFT" => "Left Shift".to_string(),
                    "RSHIFT" => "Right Shift".to_string(),
                    _ => m.clone(),
                })
                .collect();

            format!("{} + {}", modifier_display.join(" + "), base_display)
        } else {
            base_display
        }
    }

    /// Format binding name to be more readable
    fn format_binding(binding: &str) -> String {
        let clean = binding.trim();

        // Handle button inputs: "button3" -> "Button 3"
        if clean.starts_with("button") {
            if let Some(num) = clean.strip_prefix("button") {
                return format!("Button {}", num.trim());
            }
        }

        // Handle hat switch inputs: "hat1_up" -> "Hat 1 Up"
        if clean.starts_with("hat") {
            // Extract hat number and direction
            if let Some(rest) = clean.strip_prefix("hat") {
                if let Some((num, direction)) = rest.split_once('_') {
                    let direction_cap =
                        direction.chars().next().unwrap().to_uppercase().to_string()
                            + &direction[1..];
                    return format!("Hat {} {}", num.trim(), direction_cap);
                }
            }
        }

        clean.replace('_', " ").to_uppercase()
    }
}

/// Helper struct for organizing keybindings by category for the UI
#[derive(Debug, Serialize, Clone)]
pub struct OrganizedKeybindings {
    pub profile_name: String,
    pub categories: Vec<String>,
    pub devices: DeviceInfo,
    pub action_maps: Vec<OrganizedActionMap>,
}

#[derive(Debug, Serialize, Clone)]
pub struct OrganizedActionMap {
    pub name: String,
    pub display_name: String,
    pub actions: Vec<OrganizedAction>,
}

#[derive(Debug, Serialize, Clone)]
pub struct OrganizedAction {
    pub name: String,
    pub display_name: String,
    pub bindings: Vec<BindingInfo>,
}

#[derive(Debug, Serialize, Clone)]
pub struct BindingInfo {
    pub input: String,
    pub input_type: String,
    pub display_name: String,
    pub activation_mode: String,
    pub multi_tap: Option<u32>,
}

impl ActionMaps {
    /// Check if there are any customized keyboard bindings
    pub fn has_keyboard_bindings(&self) -> bool {
        self.action_maps.iter().any(|action_map| {
            action_map.actions.iter().any(|action| {
                action
                    .rebinds
                    .iter()
                    .any(|rebind| rebind.get_input_type() == InputType::Keyboard)
            })
        })
    }

    /// Check if there are any customized mouse bindings
    pub fn has_mouse_bindings(&self) -> bool {
        self.action_maps.iter().any(|action_map| {
            action_map.actions.iter().any(|action| {
                action
                    .rebinds
                    .iter()
                    .any(|rebind| rebind.get_input_type() == InputType::Mouse)
            })
        })
    }

    /// Parse XML file into ActionMaps structure using event-based parser
    pub fn from_xml(xml: &str) -> Result<Self, String> {
        let mut profile_name = String::new();
        let mut action_maps = Vec::new();
        let mut categories = Vec::new();
        let mut devices = DeviceInfo {
            keyboards: Vec::new(),
            mice: Vec::new(),
            joysticks: Vec::new(),
            device_options: Vec::new(),
        };

        // Use quick-xml's Reader
        let mut reader = quick_xml::Reader::from_str(xml);
        let mut buf = vec![];
        let mut current_action_map: Option<ActionMap> = None;
        let mut current_action: Option<Action> = None;
        let mut current_device_options: Option<DeviceOptions> = None;

        loop {
            match reader.read_event_into(&mut buf) {
                Ok(quick_xml::events::Event::Start(ref e)) => {
                    match e.name().as_ref() {
                        b"ActionMaps" => {
                            // Get profile name
                            for attr in e.attributes().flatten() {
                                if attr.key.as_ref() == b"profileName" {
                                    profile_name =
                                        String::from_utf8(attr.value.to_vec()).unwrap_or_default();
                                }
                            }
                        }
                        b"options" => {
                            let mut device_type = String::new();
                            let mut product = String::new();
                            let mut instance = String::new();

                            for attr in e.attributes().flatten() {
                                match attr.key.as_ref() {
                                    b"type" => {
                                        device_type = String::from_utf8(attr.value.to_vec())
                                            .unwrap_or_default()
                                    }
                                    b"Product" => {
                                        product = String::from_utf8(attr.value.to_vec())
                                            .unwrap_or_default()
                                    }
                                    b"instance" => {
                                        instance = String::from_utf8(attr.value.to_vec())
                                            .unwrap_or_default()
                                    }
                                    _ => {}
                                }
                            }

                            // Add to legacy device lists for backward compatibility
                            if !product.is_empty() {
                                match device_type.as_str() {
                                    "keyboard" => devices.keyboards.push(product.clone()),
                                    "mouse" => devices.mice.push(product.clone()),
                                    "joystick" => devices.joysticks.push(product.clone()),
                                    _ => {}
                                }
                            }

                            // Start event - this options tag has children (control options)
                            current_device_options = Some(DeviceOptions {
                                device_type,
                                instance,
                                product,
                                control_options: Vec::new(),
                            });
                        }
                        b"actionmap" => {
                            let mut name = String::new();
                            for attr in e.attributes().flatten() {
                                if attr.key.as_ref() == b"name" {
                                    name =
                                        String::from_utf8(attr.value.to_vec()).unwrap_or_default();
                                }
                            }
                            current_action_map = Some(ActionMap {
                                name,
                                actions: Vec::new(),
                            });
                        }
                        b"action" => {
                            let mut name = String::new();
                            for attr in e.attributes().flatten() {
                                if attr.key.as_ref() == b"name" {
                                    name =
                                        String::from_utf8(attr.value.to_vec()).unwrap_or_default();
                                }
                            }
                            current_action = Some(Action {
                                name,
                                rebinds: Vec::new(),
                            });
                        }
                        _ => {
                            // If we're inside an <options> element, capture child elements as control options
                            if let Some(ref mut device_opts) = current_device_options {
                                let name = String::from_utf8(e.name().as_ref().to_vec())
                                    .unwrap_or_default();
                                let mut attributes = Vec::new();
                                for attr in e.attributes().flatten() {
                                    let key = String::from_utf8(attr.key.as_ref().to_vec())
                                        .unwrap_or_default();
                                    let value =
                                        String::from_utf8(attr.value.to_vec()).unwrap_or_default();
                                    attributes.push((key, value));
                                }
                                device_opts
                                    .control_options
                                    .push(ControlOption { name, attributes });
                            }
                        }
                    }
                }
                Ok(quick_xml::events::Event::Empty(ref e)) => {
                    match e.name().as_ref() {
                        b"category" => {
                            // Get category label
                            for attr in e.attributes().flatten() {
                                if attr.key.as_ref() == b"label" {
                                    let label =
                                        String::from_utf8(attr.value.to_vec()).unwrap_or_default();
                                    if !label.is_empty() {
                                        categories.push(Category { label });
                                    }
                                }
                            }
                        }
                        b"options" => {
                            let mut device_type = String::new();
                            let mut product = String::new();
                            let mut instance = String::new();

                            for attr in e.attributes().flatten() {
                                match attr.key.as_ref() {
                                    b"type" => {
                                        device_type = String::from_utf8(attr.value.to_vec())
                                            .unwrap_or_default()
                                    }
                                    b"Product" => {
                                        product = String::from_utf8(attr.value.to_vec())
                                            .unwrap_or_default()
                                    }
                                    b"instance" => {
                                        instance = String::from_utf8(attr.value.to_vec())
                                            .unwrap_or_default()
                                    }
                                    _ => {}
                                }
                            }

                            // Add to legacy device lists for backward compatibility
                            if !product.is_empty() {
                                match device_type.as_str() {
                                    "keyboard" => devices.keyboards.push(product.clone()),
                                    "mouse" => devices.mice.push(product.clone()),
                                    "joystick" => devices.joysticks.push(product.clone()),
                                    _ => {}
                                }
                            }

                            // Empty (self-closing) tag - no children
                            devices.device_options.push(DeviceOptions {
                                device_type,
                                instance,
                                product,
                                control_options: Vec::new(),
                            });
                        }
                        b"rebind" => {
                            let mut input = String::new();
                            let mut multi_tap: Option<u32> = None;
                            let mut activation_mode_attr = String::new();
                            for attr in e.attributes().flatten() {
                                match attr.key.as_ref() {
                                    b"input" => {
                                        input = String::from_utf8(attr.value.to_vec())
                                            .unwrap_or_default()
                                    }
                                    b"multiTap" => {
                                        if let Ok(s) = String::from_utf8(attr.value.to_vec()) {
                                            multi_tap = s.parse::<u32>().ok();
                                        }
                                    }
                                    b"activationMode" => {
                                        activation_mode_attr =
                                            String::from_utf8(attr.value.to_vec())
                                                .unwrap_or_default()
                                    }
                                    _ => {}
                                }
                            }
                            if let Some(ref mut action) = current_action {
                                // Create a temporary rebind to get its device type
                                let new_rebind = Rebind {
                                    input: input.clone(),
                                    multi_tap,
                                    activation_mode: activation_mode_attr.clone(),
                                    vkb_layer: None,
                                    vkb_tempo: None,
                                    vkb_physical_id: None,
                                    vkb_physical_info: None,
                                };
                                let new_device_type = new_rebind.get_device_type();

                                // Star Citizen only allows ONE rebind per device TYPE per action
                                // (e.g., one joystick binding total, not one per js1/js2)
                                // Remove any existing rebind from the same device type
                                action
                                    .rebinds
                                    .retain(|r| r.get_device_type() != new_device_type);

                                action.rebinds.push(new_rebind);
                            }
                        }
                        _ => {
                            // If we're inside an <options> element, capture child elements as control options
                            if let Some(ref mut device_opts) = current_device_options {
                                let name = String::from_utf8(e.name().as_ref().to_vec())
                                    .unwrap_or_default();
                                let mut attributes = Vec::new();
                                for attr in e.attributes().flatten() {
                                    let key = String::from_utf8(attr.key.as_ref().to_vec())
                                        .unwrap_or_default();
                                    let value =
                                        String::from_utf8(attr.value.to_vec()).unwrap_or_default();
                                    attributes.push((key, value));
                                }
                                device_opts
                                    .control_options
                                    .push(ControlOption { name, attributes });
                            }
                        }
                    }
                }
                Ok(quick_xml::events::Event::End(ref e)) => match e.name().as_ref() {
                    b"options" => {
                        // Finalize the current device options when we hit </options>
                        if let Some(device_opts) = current_device_options.take() {
                            devices.device_options.push(device_opts);
                        }
                    }
                    b"action" => {
                        if let (Some(action), Some(ref mut action_map)) =
                            (current_action.take(), &mut current_action_map)
                        {
                            action_map.actions.push(action);
                        }
                    }
                    b"actionmap" => {
                        if let Some(action_map) = current_action_map.take() {
                            action_maps.push(action_map);
                        }
                    }
                    _ => {}
                },
                Ok(quick_xml::events::Event::Eof) => break,
                Err(e) => {
                    return Err(format!("XML parsing error: {}", e));
                }
                _ => {}
            }
            buf.clear();
        }

        Ok(ActionMaps {
            profile_name,
            action_maps,
            categories,
            devices,
        })
    }

    /// Enhanced export that determines categories from actionmaps with custom bindings
    /// and preserves the order from AllBinds.xml
    pub fn to_xml_with_categories(&self, all_binds: Option<&AllBinds>) -> String {
        use std::collections::{HashMap, HashSet};

        let mut xml = String::new();

        // XML declaration
        xml.push_str("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");

        // Root ActionMaps element
        xml.push_str(&format!(
            "<ActionMaps version=\"1\" optionsVersion=\"2\" rebindVersion=\"2\" profileName=\"{}\">\n",
            self.profile_name
        ));

        // Build a map of actionmap names to their categories and order
        let mut actionmap_to_category: HashMap<String, (String, usize)> = HashMap::new();
        if let Some(all_binds) = all_binds {
            for (idx, all_binds_map) in all_binds.action_maps.iter().enumerate() {
                actionmap_to_category.insert(
                    all_binds_map.name.clone(),
                    (all_binds_map.ui_category.clone(), idx),
                );
            }
        }

        // Determine which actionmaps have custom bindings
        // NOTE: We include actionmaps with cleared bindings (ending with "_ ")
        // because these are used to override defaults in Star Citizen
        let mut actionmaps_with_bindings = Vec::new();
        for action_map in &self.action_maps {
            let has_custom_bindings = action_map
                .actions
                .iter()
                .any(|action| !action.rebinds.is_empty());

            if has_custom_bindings {
                actionmaps_with_bindings.push(action_map.name.clone());
            }
        }

        // Collect unique categories from actionmaps with bindings, preserving order
        let mut categories_ordered = Vec::new();
        let mut seen_categories = HashSet::new();

        // First pass: collect categories in the order they appear in AllBinds
        if let Some(all_binds) = all_binds {
            for all_binds_map in &all_binds.action_maps {
                if actionmaps_with_bindings.contains(&all_binds_map.name)
                    && !all_binds_map.ui_category.is_empty()
                    && !seen_categories.contains(&all_binds_map.ui_category)
                {
                    categories_ordered.push(all_binds_map.ui_category.clone());
                    seen_categories.insert(all_binds_map.ui_category.clone());
                }
            }
        }

        // Write CustomisationUIHeader
        xml.push_str(" <CustomisationUIHeader label=\"");
        xml.push_str(&self.profile_name);
        xml.push_str("\" description=\"\" image=\"\">\n");

        // Check if we have keyboard or mouse customizations
        let has_keyboard = self.has_keyboard_bindings();
        let has_mouse = self.has_mouse_bindings();

        // Write devices section - order matters!
        xml.push_str("  <devices>\n");
        if has_keyboard {
            xml.push_str("   <keyboard instance=\"1\"/>\n");
        }
        if has_mouse {
            xml.push_str("   <mouse instance=\"1\"/>\n");
        }
        // Write joystick instances
        for i in 1..=self.devices.joysticks.len().max(2) {
            xml.push_str(&format!("   <joystick instance=\"{}\"/>\n", i));
        }
        xml.push_str("  </devices>\n");

        // Write categories section with proper ordering
        // if !categories_ordered.is_empty() {
        //     xml.push_str("  <categories>\n");
        //     for category in &categories_ordered {
        //         xml.push_str("   <category label=\"");
        //         xml.push_str(category);
        //         xml.push_str("\"/>\n");
        //     }
        //     xml.push_str("  </categories>\n");
        // }

        xml.push_str(" </CustomisationUIHeader>\n");

        // Write options for each device type - preserving control settings if present
        // First, check if we have preserved device_options with control settings
        if !self.devices.device_options.is_empty() {
            // Use preserved device options which include control settings
            for device_opts in &self.devices.device_options {
                // Only write if it's a device type we're using
                let should_write = match device_opts.device_type.as_str() {
                    "keyboard" => has_keyboard,
                    "mouse" => has_mouse,
                    "joystick" => true, // Always include joysticks if present
                    _ => true,
                };

                if should_write {
                    if device_opts.control_options.is_empty() {
                        // Self-closing tag if no control options
                        xml.push_str(&format!(
                            " <options type=\"{}\" instance=\"{}\" Product=\"{}\"/>\n",
                            device_opts.device_type, device_opts.instance, device_opts.product
                        ));
                    } else {
                        // Tag with children for control options
                        xml.push_str(&format!(
                            " <options type=\"{}\" instance=\"{}\" Product=\"{}\">\n",
                            device_opts.device_type, device_opts.instance, device_opts.product
                        ));
                        for ctrl_opt in &device_opts.control_options {
                            xml.push_str(&format!("  <{}", ctrl_opt.name));
                            for (key, value) in &ctrl_opt.attributes {
                                xml.push_str(&format!(" {}=\"{}\"", key, value));
                            }
                            xml.push_str("/>\n");
                        }
                        xml.push_str(" </options>\n");
                    }
                }
            }
        } else {
            // Fallback to legacy device lists (for backward compatibility)
            // Write options for each device type - order matters!
            // Keyboard options first (if we have keyboard bindings)
            if has_keyboard {
                // Use a default keyboard product ID if the devices list is empty
                if !self.devices.keyboards.is_empty() {
                    for keyboard in &self.devices.keyboards {
                        xml.push_str(" <options type=\"keyboard\" instance=\"1\" Product=\"");
                        xml.push_str(keyboard);
                        xml.push_str("\"/>\n");
                    }
                } else {
                    // Add default keyboard product ID
                    xml.push_str(" <options type=\"keyboard\" instance=\"1\" Product=\"Keyboard  {6F1D2B61-D5A0-11CF-BFC7-444553540000}\"/>\n");
                }
            }

            // Mouse options second (if we have mouse bindings)
            if has_mouse {
                // Use a default mouse product ID if the devices list is empty
                if !self.devices.mice.is_empty() {
                    for mouse in &self.devices.mice {
                        xml.push_str(" <options type=\"mouse\" instance=\"1\" Product=\"");
                        xml.push_str(mouse);
                        xml.push_str("\"/>\n");
                    }
                } else {
                    // Add default mouse product ID
                    xml.push_str(" <options type=\"mouse\" instance=\"1\" Product=\"Mouse  {6F1D2B62-D5A0-11CF-BFC7-444553540000}\"/>\n");
                }
            }

            // Joystick options last
            if !self.devices.joysticks.is_empty() {
                for (idx, joystick) in self.devices.joysticks.iter().enumerate() {
                    let instance = idx + 1;
                    xml.push_str(&format!(
                        " <options type=\"joystick\" instance=\"{}\" Product=\"",
                        instance
                    ));
                    xml.push_str(joystick);
                    xml.push_str("\"/>\n");
                }
            }
        }

        xml.push_str(" <modifiers />\n");

        // Sort actionmaps according to AllBinds.xml order
        let mut sorted_actionmaps_with_bindings: Vec<_> = actionmaps_with_bindings
            .iter()
            .map(|name| {
                let order = actionmap_to_category
                    .get(name)
                    .map(|(_, idx)| *idx)
                    .unwrap_or(usize::MAX);
                (name, order)
            })
            .collect();
        sorted_actionmaps_with_bindings.sort_by_key(|(_, order)| *order);

        // Write actionmaps in the proper order
        for (actionmap_name, _) in sorted_actionmaps_with_bindings {
            if let Some(action_map) = self
                .action_maps
                .iter()
                .find(|am| &am.name == actionmap_name)
            {
                // Include ALL actions with rebinds, even cleared ones (ending with "_ ")
                // Cleared bindings need to be written to override Star Citizen defaults
                let actions_with_rebinds: Vec<&Action> = action_map
                    .actions
                    .iter()
                    .filter(|action| !action.rebinds.is_empty())
                    .collect();

                if !actions_with_rebinds.is_empty() {
                    xml.push_str(" <actionmap name=\"");
                    xml.push_str(&action_map.name);
                    xml.push_str("\">\n");

                    for action in actions_with_rebinds {
                        xml.push_str("  <action name=\"");
                        xml.push_str(&action.name);
                        xml.push_str("\">\n");

                        // Deduplicate rebinds by device TYPE to ensure we only write one per type
                        // Star Citizen only accepts one rebind per device TYPE (Joystick, Keyboard, etc.)
                        // Keep the LAST rebind for each device type (most recent binding)
                        let mut type_to_rebind: std::collections::HashMap<String, &Rebind> =
                            std::collections::HashMap::new();
                        for rebind in &action.rebinds {
                            let device_type = format!("{:?}", rebind.get_device_type());
                            type_to_rebind.insert(device_type, rebind);
                        }

                        // Write deduplicated rebinds
                        for rebind in type_to_rebind.values() {
                            xml.push_str("   <rebind input=\"");
                            xml.push_str(&rebind.input);
                            xml.push_str("\"");
                            // Add multiTap attribute if present
                            if let Some(tap_count) = rebind.multi_tap {
                                xml.push_str(&format!(" multiTap=\"{}\"", tap_count));
                            }
                            // Add activationMode attribute if present
                            if !rebind.activation_mode.is_empty() {
                                xml.push_str(&format!(
                                    " activationMode=\"{}\"",
                                    rebind.activation_mode
                                ));
                            }
                            xml.push_str("/>\n");
                        }

                        xml.push_str("  </action>\n");
                    }

                    xml.push_str(" </actionmap>\n");
                }
            }
        }

        xml.push_str("</ActionMaps>\n");

        xml
    }

    /// Organize keybindings for easier UI display
    pub fn organize(&self) -> OrganizedKeybindings {
        // Get categories
        let categories: Vec<String> = self
            .categories
            .iter()
            .map(|c| c.label.clone())
            .filter(|label| !label.is_empty())
            .collect();

        // Organize action maps
        let organized_maps: Vec<OrganizedActionMap> = self
            .action_maps
            .iter()
            .map(|action_map| {
                let organized_actions: Vec<OrganizedAction> = action_map
                    .actions
                    .iter()
                    .map(|action| {
                        let bindings: Vec<BindingInfo> = action
                            .rebinds
                            .iter()
                            .map(|rebind| {
                                let input_type = rebind.get_input_type();
                                BindingInfo {
                                    input: rebind.input.clone(),
                                    input_type: format!("{:?}", input_type),
                                    display_name: rebind.get_display_name(),
                                    activation_mode: rebind.activation_mode.clone(),
                                    multi_tap: rebind.multi_tap,
                                }
                            })
                            .collect();

                        OrganizedAction {
                            name: action.name.clone(),
                            display_name: Self::format_action_name(&action.name),
                            bindings,
                        }
                    })
                    .collect();

                OrganizedActionMap {
                    name: action_map.name.clone(),
                    display_name: Self::format_action_map_name(&action_map.name),
                    actions: organized_actions,
                }
            })
            .collect();

        OrganizedKeybindings {
            profile_name: self.profile_name.clone(),
            categories,
            devices: self.devices.clone(),
            action_maps: organized_maps,
        }
    }

    /// Create a new empty action map with the given name and actions
    pub fn new_empty_action_map(name: String, actions: Vec<Action>) -> ActionMap {
        ActionMap { name, actions }
    }

    /// Convert action_map name to display name
    fn format_action_map_name(name: &str) -> String {
        format_display_name(name)
    }

    /// Convert action name to display name
    fn format_action_name(name: &str) -> String {
        format_display_name(name)
    }
}

/// Comprehensive display name formatter for action names
/// Handles various naming patterns:
/// - v_action_name -> Action Name
/// - pc_action_name -> Action Name  
/// - PascalCase -> Pascal Case
/// - camelCase -> Camel Case
/// - snake_case -> Snake Case
/// - turret_gyromode -> Turret Gyro Mode (splits compounds)
pub fn format_display_name(name: &str) -> String {
    if name.is_empty() {
        return String::new();
    }

    // Remove common prefixes
    let cleaned = name
        .trim_start_matches("v_")
        .trim_start_matches("pc_")
        .trim_start_matches("ui_")
        .trim_start_matches("spectate_");

    // Handle special abbreviations and technical terms
    let result = cleaned
        .replace("mfd", "MFD")
        .replace("fps", "FPS")
        .replace("esp", "ESP")
        .replace("vjoy", "VJoy")
        .replace("mgv", "MGV")
        .replace("hud", "HUD")
        .replace("ifcs", "IFCS")
        .replace("vtol", "VTOL")
        .replace("ptu", "PTU")
        .replace("atc", "ATC");

    // Split on underscores first
    let parts: Vec<&str> = result.split('_').collect();

    let formatted_parts: Vec<String> = parts
        .iter()
        .map(|part| {
            if part.is_empty() {
                return String::new();
            }

            // Check if it's all caps (abbreviation) - preserve it
            if part.chars().all(|c| c.is_uppercase() || c.is_numeric()) {
                return part.to_string();
            }

            // Check if it's a mix of uppercase and lowercase (PascalCase or camelCase)
            if part.chars().any(|c| c.is_uppercase()) && part.chars().any(|c| c.is_lowercase()) {
                split_camel_case(part)
            } else {
                // Regular word - capitalize first letter
                capitalize_word(part)
            }
        })
        .collect();

    formatted_parts.join(" ").trim().to_string()
}

/// Split PascalCase or camelCase into separate words
fn split_camel_case(s: &str) -> String {
    let mut result = String::new();
    let mut prev_was_lower = false;
    let mut prev_was_upper = false;

    for (i, c) in s.chars().enumerate() {
        if i == 0 {
            result.push(c.to_ascii_uppercase());
            prev_was_upper = c.is_uppercase();
            prev_was_lower = c.is_lowercase();
        } else if c.is_uppercase() {
            // Add space before uppercase if previous was lowercase
            // or if this is start of a new word (e.g., "HTMLParser" -> "HTML Parser")
            if prev_was_lower {
                result.push(' ');
            } else if prev_was_upper && i + 1 < s.len() {
                // Check if next char is lowercase (indicates start of new word)
                if let Some(next_c) = s.chars().nth(i + 1) {
                    if next_c.is_lowercase() {
                        result.push(' ');
                    }
                }
            }
            result.push(c);
            prev_was_upper = true;
            prev_was_lower = false;
        } else {
            result.push(c);
            prev_was_upper = false;
            prev_was_lower = true;
        }
    }

    result
}

/// Capitalize the first letter of a word
fn capitalize_word(word: &str) -> String {
    let mut chars = word.chars();
    match chars.next() {
        None => String::new(),
        Some(first) => {
            let rest: String = chars.collect();
            // Handle special cases
            match word.to_lowercase().as_str() {
                "up" | "down" | "left" | "right" => word.to_uppercase(),
                "x" | "y" | "z" => word.to_uppercase(),
                "1to1" => "1:1".to_string(),
                _ => first.to_uppercase().collect::<String>() + &rest.to_lowercase(),
            }
        }
    }
}

impl AllBinds {
    /// Parse AllBinds.xml file into AllBinds structure
    pub fn from_xml(xml: &str) -> Result<Self, String> {
        let mut action_maps = Vec::new();
        let mut reader = quick_xml::Reader::from_str(xml);
        let mut buf = vec![];

        let mut current_action_map: Option<AllBindsActionMap> = None;

        loop {
            match reader.read_event_into(&mut buf) {
                Ok(quick_xml::events::Event::Start(ref e))
                | Ok(quick_xml::events::Event::Empty(ref e)) => {
                    match e.name().as_ref() {
                        b"actionmap" => {
                            let mut name = String::new();
                            let mut version = String::new();
                            let mut ui_label = String::new();
                            let mut ui_category = String::new();

                            for attr in e.attributes().flatten() {
                                match attr.key.as_ref() {
                                    b"name" => {
                                        name = String::from_utf8(attr.value.to_vec())
                                            .unwrap_or_default()
                                    }
                                    b"version" => {
                                        version = String::from_utf8(attr.value.to_vec())
                                            .unwrap_or_default()
                                    }
                                    b"UILabel" => {
                                        ui_label = String::from_utf8(attr.value.to_vec())
                                            .unwrap_or_default()
                                    }
                                    b"UICategory" => {
                                        ui_category = String::from_utf8(attr.value.to_vec())
                                            .unwrap_or_default()
                                    }
                                    _ => {}
                                }
                            }

                            current_action_map = Some(AllBindsActionMap {
                                name,
                                version,
                                ui_label,
                                ui_category,
                                actions: Vec::new(),
                            });
                        }
                        b"action" => {
                            if let Some(ref mut action_map) = current_action_map {
                                let mut name = String::new();
                                let mut ui_label = String::new();
                                let mut ui_description = String::new();
                                let mut category = String::new();
                                let mut activation_mode = String::new();
                                let mut on_hold = false;
                                let mut keyboard = String::new();
                                let mut mouse = String::new();
                                let mut gamepad = String::new();
                                let mut joystick = String::new();

                                for attr in e.attributes().flatten() {
                                    match attr.key.as_ref() {
                                        b"name" => {
                                            name = String::from_utf8(attr.value.to_vec())
                                                .unwrap_or_default()
                                        }
                                        b"UILabel" => {
                                            ui_label = String::from_utf8(attr.value.to_vec())
                                                .unwrap_or_default()
                                        }
                                        b"UIDescription" => {
                                            ui_description = String::from_utf8(attr.value.to_vec())
                                                .unwrap_or_default()
                                        }
                                        b"Category" => {
                                            category = String::from_utf8(attr.value.to_vec())
                                                .unwrap_or_default()
                                        }
                                        b"activationMode" => {
                                            activation_mode = String::from_utf8(attr.value.to_vec())
                                                .unwrap_or_default()
                                        }
                                        b"onHold" => {
                                            let val = String::from_utf8(attr.value.to_vec())
                                                .unwrap_or_default();
                                            on_hold = val == "1";
                                        }
                                        b"keyboard" => {
                                            keyboard = String::from_utf8(attr.value.to_vec())
                                                .unwrap_or_default()
                                        }
                                        b"mouse" => {
                                            mouse = String::from_utf8(attr.value.to_vec())
                                                .unwrap_or_default()
                                        }
                                        b"gamepad" => {
                                            gamepad = String::from_utf8(attr.value.to_vec())
                                                .unwrap_or_default()
                                        }
                                        b"joystick" => {
                                            joystick = String::from_utf8(attr.value.to_vec())
                                                .unwrap_or_default()
                                        }
                                        _ => {}
                                    }
                                }

                                // Debug: log quantum actions
                                if name.contains("quantum") {
                                    println!("Parsing action: {} with kb='{}', mouse='{}', gamepad='{}', js='{}'", 
                                        name, keyboard, mouse, gamepad, joystick);
                                }

                                action_map.actions.push(AllBindsAction {
                                    name,
                                    ui_label,
                                    ui_description,
                                    category,
                                    activation_mode,
                                    on_hold,
                                    default_keyboard: keyboard,
                                    default_mouse: mouse,
                                    default_gamepad: gamepad,
                                    default_joystick: joystick,
                                });
                            }
                        }
                        _ => {}
                    }
                }
                Ok(quick_xml::events::Event::End(ref e)) => {
                    if e.name().as_ref() == b"actionmap" {
                        if let Some(action_map) = current_action_map.take() {
                            action_maps.push(action_map);
                        }
                    }
                }
                Ok(quick_xml::events::Event::Eof) => break,
                Err(e) => {
                    return Err(format!("XML parsing error: {}", e));
                }
                _ => {}
            }
            buf.clear();
        }

        Ok(AllBinds { action_maps })
    }
}

/// Merged view of AllBinds with user customizations
#[derive(Debug, Serialize, Clone)]
pub struct MergedBindings {
    pub action_maps: Vec<MergedActionMap>,
}

#[derive(Debug, Serialize, Clone)]
pub struct MergedActionMap {
    pub name: String,
    pub ui_label: String,
    pub ui_category: String,
    pub actions: Vec<MergedAction>,
}

#[derive(Debug, Serialize, Clone)]
pub struct MergedAction {
    pub name: String,
    pub ui_label: String,
    pub ui_description: String,
    pub category: String,
    pub is_customized: bool,
    pub on_hold: bool,
    pub bindings: Vec<MergedBinding>,
}

#[derive(Debug, Serialize, Clone)]
pub struct MergedBinding {
    pub input: String,
    pub display_name: String,
    pub input_type: String,
    pub is_default: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub multi_tap: Option<u32>,
    pub activation_mode: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub original_default: Option<String>, // For cleared bindings, store the original default binding text
}

impl AllBinds {
    /// Merge AllBinds with user customizations from ActionMaps
    pub fn merge_with_user_bindings(&self, user_bindings: Option<&ActionMaps>) -> MergedBindings {
        // Build a lookup map for user bindings
        let mut user_actions: HashMap<(String, String), Vec<Rebind>> = HashMap::new();

        if let Some(user_maps) = user_bindings {
            for action_map in &user_maps.action_maps {
                for action in &action_map.actions {
                    user_actions.insert(
                        (action_map.name.clone(), action.name.clone()),
                        action.rebinds.clone(),
                    );
                }
            }
        }

        let merged_maps: Vec<MergedActionMap> = self
            .action_maps
            .iter()
            .map(|all_binds_map| {
                // Debug: log quantum actionmap
                if all_binds_map.name.contains("quantum") || all_binds_map.name == "seat_general" {
                    let quantum_actions: Vec<_> = all_binds_map
                        .actions
                        .iter()
                        .filter(|a| a.name.contains("quantum"))
                        .map(|a| &a.name)
                        .collect();
                    if !quantum_actions.is_empty() {
                        println!(
                            "ActionMap '{}' has quantum actions: {:?}",
                            all_binds_map.name, quantum_actions
                        );
                    }
                }

                let merged_actions: Vec<MergedAction> = all_binds_map
                    .actions
                    .iter()
                    .map(|all_binds_action| {
                        // Check if user has custom bindings for this action
                        let user_rebinds = user_actions
                            .get(&(all_binds_map.name.clone(), all_binds_action.name.clone()));

                        // Only consider it customized if there are actual non-empty rebinds
                        let is_customized = if let Some(rebinds) = user_rebinds {
                            // Action is customized if there are ANY rebinds from the user
                            // This includes cleared bindings (e.g., "js1_ ") which represent explicit user action
                            !rebinds.is_empty()
                        } else {
                            false
                        };

                        let bindings: Vec<MergedBinding> = if let Some(rebinds) = user_rebinds {
                            // User has custom bindings - include them plus defaults for other input types
                            let mut all_bindings: Vec<MergedBinding> = rebinds
                                .iter()
                                .map(|rebind| {
                                    let input_type = rebind.get_input_type();
                                    let input_trimmed = rebind.input.trim();

                                    // Check if this is a cleared binding (e.g., "js1_ ", "kb_ ", etc.)
                                    // Pattern: (js|kb|mouse|gp)\d+_\s* (device + number + underscore + optional spaces)
                                    let is_cleared_binding = if input_trimmed.len() >= 3 {
                                        let parts: Vec<&str> = input_trimmed.split('_').collect();
                                        parts.len() == 2 && parts[1].trim().is_empty()
                                    } else {
                                        false
                                    };

                                    // If it's a cleared binding, mark it as is_default: true
                                    // because it's clearing/overriding a default binding
                                    let is_default_flag = is_cleared_binding;

                                    // For cleared bindings, get the original default value
                                    let original_default = if is_cleared_binding {
                                        match input_type {
                                            InputType::Keyboard => {
                                                let default_value =
                                                    all_binds_action.default_keyboard.trim();
                                                if default_value.is_empty() {
                                                    None
                                                } else {
                                                    let rebind = Rebind {
                                                        input: format!("kb_{}", default_value),
                                                        multi_tap: None,
                                                        activation_mode: String::new(),
                                                        vkb_layer: None,
                                                        vkb_tempo: None,
                                                        vkb_physical_id: None,
                                                        vkb_physical_info: None,
                                                    };
                                                    Some(rebind.get_display_name())
                                                }
                                            }
                                            InputType::Mouse => {
                                                let default_value =
                                                    all_binds_action.default_mouse.trim();
                                                if default_value.is_empty() {
                                                    None
                                                } else {
                                                    let rebind = Rebind {
                                                        input: format!("mouse1_{}", default_value),
                                                        multi_tap: None,
                                                        activation_mode: String::new(),
                                                        vkb_layer: None,
                                                        vkb_tempo: None,
                                                        vkb_physical_id: None,
                                                        vkb_physical_info: None,
                                                    };
                                                    Some(rebind.get_display_name())
                                                }
                                            }
                                            InputType::Joystick => {
                                                let default_value =
                                                    all_binds_action.default_joystick.trim();
                                                if default_value.is_empty() {
                                                    None
                                                } else {
                                                    let rebind = Rebind {
                                                        input: format!("js1_{}", default_value),
                                                        multi_tap: None,
                                                        activation_mode: String::new(),
                                                        vkb_layer: None,
                                                        vkb_tempo: None,
                                                        vkb_physical_id: None,
                                                        vkb_physical_info: None,
                                                    };
                                                    Some(rebind.get_display_name())
                                                }
                                            }
                                            InputType::Gamepad => {
                                                let default_value =
                                                    all_binds_action.default_gamepad.trim();
                                                if default_value.is_empty() {
                                                    None
                                                } else {
                                                    let rebind = Rebind {
                                                        input: format!("gp1_{}", default_value),
                                                        multi_tap: None,
                                                        activation_mode: String::new(),
                                                        vkb_layer: None,
                                                        vkb_tempo: None,
                                                        vkb_physical_id: None,
                                                        vkb_physical_info: None,
                                                    };
                                                    Some(rebind.get_display_name())
                                                }
                                            }
                                            InputType::Unknown => None,
                                        }
                                    } else {
                                        None
                                    };

                                    MergedBinding {
                                        input: rebind.input.clone(),
                                        display_name: if is_cleared_binding {
                                            original_default
                                                .clone()
                                                .unwrap_or_else(|| "Unbound".to_string())
                                        } else {
                                            rebind.get_display_name()
                                        },
                                        input_type: format!("{:?}", input_type),
                                        is_default: is_default_flag,
                                        multi_tap: rebind.multi_tap,
                                        activation_mode: rebind.activation_mode.clone(),
                                        original_default,
                                    }
                                })
                                .collect();

                            // Build a set of input types we already have from user bindings
                            use std::collections::HashSet;
                            let custom_input_types: HashSet<String> = all_bindings
                                .iter()
                                .filter_map(|b| {
                                    // For cleared bindings (input_type is "Unknown"), determine the actual input type from the input string
                                    if b.input_type == "Unknown" && b.is_default {
                                        // This is a cleared binding - extract the actual input type from the input prefix
                                        let input_trimmed = b.input.trim();
                                        if input_trimmed.starts_with("kb") {
                                            Some("Keyboard".to_string())
                                        } else if input_trimmed.starts_with("mouse") {
                                            Some("Mouse".to_string())
                                        } else if input_trimmed.starts_with("js") {
                                            Some("Joystick".to_string())
                                        } else if input_trimmed.starts_with("gp") {
                                            Some("Gamepad".to_string())
                                        } else {
                                            Some(b.input_type.clone())
                                        }
                                    } else {
                                        Some(b.input_type.clone())
                                    }
                                })
                                .collect();

                            // Add keyboard default if not customized
                            if !custom_input_types.contains("Keyboard") {
                                let kb_trimmed = all_binds_action.default_keyboard.trim();
                                // Only add if there's an actual binding (not just space)
                                if !kb_trimmed.is_empty() {
                                    let input = format!("kb_{}", kb_trimmed);
                                    let rebind = Rebind {
                                        input: input.clone(),
                                        multi_tap: None,
                                        activation_mode: String::new(),
                                        vkb_layer: None,
                                        vkb_tempo: None,
                                        vkb_physical_id: None,
                                        vkb_physical_info: None,
                                    };
                                    let input_type = rebind.get_input_type();
                                    all_bindings.push(MergedBinding {
                                        input: rebind.input.clone(),
                                        display_name: rebind.get_display_name(),
                                        input_type: format!("{:?}", input_type),
                                        is_default: true,
                                        multi_tap: None,
                                        activation_mode: String::new(),
                                        original_default: None,
                                    });
                                }
                            }

                            // Add gamepad default if not customized
                            if !custom_input_types.contains("Gamepad") {
                                let gp_trimmed = all_binds_action.default_gamepad.trim();
                                // Only add if there's an actual binding (not just space)
                                if !gp_trimmed.is_empty() {
                                    let input = format!("gp1_{}", gp_trimmed);
                                    let rebind = Rebind {
                                        input: input.clone(),
                                        multi_tap: None,
                                        activation_mode: String::new(),
                                        vkb_layer: None,
                                        vkb_tempo: None,
                                        vkb_physical_id: None,
                                        vkb_physical_info: None,
                                    };
                                    let input_type = rebind.get_input_type();
                                    all_bindings.push(MergedBinding {
                                        input: rebind.input.clone(),
                                        display_name: rebind.get_display_name(),
                                        input_type: format!("{:?}", input_type),
                                        is_default: true,
                                        multi_tap: None,
                                        activation_mode: String::new(),
                                        original_default: None,
                                    });
                                }
                            }

                            // Add joystick default if not customized
                            if !custom_input_types.contains("Joystick") {
                                let js_trimmed = all_binds_action.default_joystick.trim();
                                // Only add if there's an actual binding (not just space)
                                if !js_trimmed.is_empty() {
                                    let input = format!("js1_{}", js_trimmed);
                                    let rebind = Rebind {
                                        input: input.clone(),
                                        multi_tap: None,
                                        activation_mode: String::new(),
                                        vkb_layer: None,
                                        vkb_tempo: None,
                                        vkb_physical_id: None,
                                        vkb_physical_info: None,
                                    };
                                    let input_type = rebind.get_input_type();
                                    all_bindings.push(MergedBinding {
                                        input: rebind.input.clone(),
                                        display_name: rebind.get_display_name(),
                                        input_type: format!("{:?}", input_type),
                                        is_default: true,
                                        multi_tap: None,
                                        activation_mode: String::new(),
                                        original_default: None,
                                    });
                                }
                            }

                            // Add mouse default if not customized
                            if !custom_input_types.contains("Mouse") {
                                let mouse_trimmed = all_binds_action.default_mouse.trim();
                                // Only add if there's an actual binding (not just space)
                                if !mouse_trimmed.is_empty() {
                                    let input = format!("mouse1_{}", mouse_trimmed);
                                    let rebind = Rebind {
                                        input: input.clone(),
                                        multi_tap: None,
                                        activation_mode: String::new(),
                                        vkb_layer: None,
                                        vkb_tempo: None,
                                        vkb_physical_id: None,
                                        vkb_physical_info: None,
                                    };
                                    let input_type = rebind.get_input_type();
                                    all_bindings.push(MergedBinding {
                                        input: rebind.input.clone(),
                                        display_name: rebind.get_display_name(),
                                        input_type: format!("{:?}", input_type),
                                        is_default: true,
                                        multi_tap: None,
                                        activation_mode: String::new(),
                                        original_default: None,
                                    });
                                }
                            }

                            // If we ended up with no bindings at all (all defaults were spaces),
                            // add an empty keyboard binding so the action still appears in the UI
                            if all_bindings.is_empty() {
                                all_bindings.push(MergedBinding {
                                    input: "kb_ ".to_string(),
                                    display_name: "Unbound".to_string(),
                                    input_type: "Unknown".to_string(),
                                    is_default: true,
                                    multi_tap: None,
                                    activation_mode: String::new(),
                                    original_default: None,
                                });
                            }

                            all_bindings
                        } else {
                            // No custom bindings, use defaults from AllBinds
                            let mut default_bindings = Vec::new();

                            // Add keyboard default if exists
                            let kb_trimmed = all_binds_action.default_keyboard.trim();
                            // Only add if there's an actual binding (not just space)
                            if !kb_trimmed.is_empty() {
                                let input = format!("kb_{}", kb_trimmed);
                                let rebind = Rebind {
                                    input: input.clone(),
                                    multi_tap: None,
                                    activation_mode: String::new(),
                                    vkb_layer: None,
                                    vkb_tempo: None,
                                    vkb_physical_id: None,
                                    vkb_physical_info: None,
                                };
                                let input_type = rebind.get_input_type();
                                default_bindings.push(MergedBinding {
                                    input: rebind.input.clone(),
                                    display_name: rebind.get_display_name(),
                                    input_type: format!("{:?}", input_type),
                                    is_default: true,
                                    multi_tap: None,
                                    activation_mode: String::new(),
                                    original_default: None,
                                });
                            }

                            // Add gamepad default if exists
                            let gp_trimmed = all_binds_action.default_gamepad.trim();
                            // Only add if there's an actual binding (not just space)
                            if !gp_trimmed.is_empty() {
                                let input = format!("gp1_{}", gp_trimmed);
                                let rebind = Rebind {
                                    input: input.clone(),
                                    multi_tap: None,
                                    activation_mode: String::new(),
                                    vkb_layer: None,
                                    vkb_tempo: None,
                                    vkb_physical_id: None,
                                    vkb_physical_info: None,
                                };
                                let input_type = rebind.get_input_type();
                                default_bindings.push(MergedBinding {
                                    input: rebind.input.clone(),
                                    display_name: rebind.get_display_name(),
                                    input_type: format!("{:?}", input_type),
                                    is_default: true,
                                    multi_tap: None,
                                    activation_mode: String::new(),
                                    original_default: None,
                                });
                            }

                            // Add joystick default if exists
                            let js_trimmed = all_binds_action.default_joystick.trim();
                            // Only add if there's an actual binding (not just space)
                            if !js_trimmed.is_empty() {
                                let input = format!("js1_{}", js_trimmed);
                                let rebind = Rebind {
                                    input: input.clone(),
                                    multi_tap: None,
                                    activation_mode: String::new(),
                                    vkb_layer: None,
                                    vkb_tempo: None,
                                    vkb_physical_id: None,
                                    vkb_physical_info: None,
                                };
                                let input_type = rebind.get_input_type();
                                default_bindings.push(MergedBinding {
                                    input: rebind.input.clone(),
                                    display_name: rebind.get_display_name(),
                                    input_type: format!("{:?}", input_type),
                                    is_default: true,
                                    multi_tap: None,
                                    activation_mode: String::new(),
                                    original_default: None,
                                });
                            }

                            // Add mouse default if exists
                            let mouse_trimmed = all_binds_action.default_mouse.trim();
                            // Only add if there's an actual binding (not just space)
                            if !mouse_trimmed.is_empty() {
                                let input = format!("mouse1_{}", mouse_trimmed);
                                let rebind = Rebind {
                                    input: input.clone(),
                                    multi_tap: None,
                                    activation_mode: String::new(),
                                    vkb_layer: None,
                                    vkb_tempo: None,
                                    vkb_physical_id: None,
                                    vkb_physical_info: None,
                                };
                                let input_type = rebind.get_input_type();
                                default_bindings.push(MergedBinding {
                                    input: rebind.input.clone(),
                                    display_name: rebind.get_display_name(),
                                    input_type: format!("{:?}", input_type),
                                    is_default: true,
                                    multi_tap: None,
                                    activation_mode: String::new(),
                                    original_default: None,
                                });
                            }

                            // If we ended up with no bindings at all (all defaults were spaces),
                            // add an empty keyboard binding so the action still appears in the UI
                            if default_bindings.is_empty() {
                                default_bindings.push(MergedBinding {
                                    input: "kb_ ".to_string(),
                                    display_name: "Unbound".to_string(),
                                    input_type: "Unknown".to_string(),
                                    is_default: true,
                                    multi_tap: None,
                                    activation_mode: String::new(),
                                    original_default: None,
                                });
                            }

                            default_bindings
                        };

                        // Use UILabel if available and not a localization key, otherwise format the name
                        let display_label = if !all_binds_action.ui_label.is_empty()
                            && !all_binds_action.ui_label.starts_with('@')
                        {
                            all_binds_action.ui_label.clone()
                        } else {
                            format_display_name(&all_binds_action.name)
                        };

                        // Debug: log quantum actions in merge
                        if all_binds_action.name.contains("quantum") {
                            println!(
                                "Merging action: {} with {} bindings, display_label: {}",
                                all_binds_action.name,
                                bindings.len(),
                                display_label
                            );
                            for (i, binding) in bindings.iter().enumerate() {
                                println!(
                                    "  Binding {}: input='{}', type={}, is_default={}",
                                    i, binding.input, binding.input_type, binding.is_default
                                );
                            }
                        }

                        MergedAction {
                            name: all_binds_action.name.clone(),
                            ui_label: display_label,
                            ui_description: all_binds_action.ui_description.clone(),
                            category: all_binds_action.category.clone(),
                            is_customized,
                            on_hold: all_binds_action.on_hold,
                            bindings,
                        }
                    })
                    .collect();

                // Use UILabel if available and not a localization key, otherwise format the name
                let display_map_label = if !all_binds_map.ui_label.is_empty()
                    && !all_binds_map.ui_label.starts_with('@')
                {
                    all_binds_map.ui_label.clone()
                } else {
                    format_display_name(&all_binds_map.name)
                };

                MergedActionMap {
                    name: all_binds_map.name.clone(),
                    ui_label: display_map_label,
                    ui_category: all_binds_map.ui_category.clone(),
                    actions: merged_actions,
                }
            })
            .collect();

        MergedBindings {
            action_maps: merged_maps,
        }
    }
}

// Device selection struct for unbind profile generation
#[derive(serde::Deserialize)]
pub struct DeviceSelection {
    pub keyboard: bool,
    pub mouse: bool,
    pub gamepad: bool,
    pub joystick1: bool,
    pub joystick2: bool,
}

/// Generate an unbind profile XML that clears all bindings for selected devices
pub fn generate_unbind_xml(
    all_binds: &AllBinds,
    devices: &DeviceSelection,
) -> Result<String, String> {
    use std::collections::HashSet;

    let mut xml = String::new();

    // XML declaration
    xml.push_str("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");

    // Root ActionMaps element
    xml.push_str("<ActionMaps version=\"1\" optionsVersion=\"2\" rebindVersion=\"2\" profileName=\"UNBIND_ALL_DEVICES\">\n");

    // Collect all unique categories in order
    let mut categories_ordered = Vec::new();
    let mut seen_categories = HashSet::new();
    for action_map in &all_binds.action_maps {
        if !action_map.ui_category.is_empty() && !seen_categories.contains(&action_map.ui_category)
        {
            categories_ordered.push(action_map.ui_category.clone());
            seen_categories.insert(action_map.ui_category.clone());
        }
    }

    // Write CustomisationUIHeader with proper attributes
    xml.push_str(" <CustomisationUIHeader label=\"UNBIND_ALL_DEVICES\" description=\"Clears all bindings for selected devices\" image=\"\">\n");

    // Write devices section - order matters!
    xml.push_str("  <devices>\n");
    if devices.keyboard {
        xml.push_str("   <keyboard instance=\"1\"/>\n");
    }
    if devices.mouse {
        xml.push_str("   <mouse instance=\"1\"/>\n");
    }
    if devices.joystick1 {
        xml.push_str("   <joystick instance=\"1\"/>\n");
    }
    if devices.joystick2 {
        xml.push_str("   <joystick instance=\"2\"/>\n");
    }
    xml.push_str("  </devices>\n");

    // Write categories section
    // if !categories_ordered.is_empty() {
    //     xml.push_str("  <categories>\n");
    //     for category in &categories_ordered {
    //         xml.push_str("   <category label=\"");
    //         xml.push_str(category);
    //         xml.push_str("\"/>\n");
    //     }
    //     xml.push_str("  </categories>\n");
    // }

    xml.push_str(" </CustomisationUIHeader>\n");

    // Write options for each device type with default Product IDs
    if devices.keyboard {
        xml.push_str(" <options type=\"keyboard\" instance=\"1\" Product=\"Keyboard  {6F1D2B61-D5A0-11CF-BFC7-444553540000}\"/>\n");
    }
    if devices.mouse {
        xml.push_str(" <options type=\"mouse\" instance=\"1\" Product=\"Mouse  {6F1D2B62-D5A0-11CF-BFC7-444553540000}\"/>\n");
    }
    if devices.joystick1 {
        xml.push_str(" <options type=\"joystick\" instance=\"1\" Product=\"\"/>\n");
    }
    if devices.joystick2 {
        xml.push_str(" <options type=\"joystick\" instance=\"2\" Product=\"\"/>\n");
    }
    if devices.gamepad {
        xml.push_str(" <options type=\"gamepad\" instance=\"1\" Product=\"\"/>\n");
    }

    // Empty modifiers
    xml.push_str(" <modifiers />\n");

    // Write actionmaps with blank rebinds
    for action_map in &all_binds.action_maps {
        xml.push_str(" <actionmap name=\"");
        xml.push_str(&action_map.name);
        xml.push_str("\">\n");

        for action in &action_map.actions {
            xml.push_str("  <action name=\"");
            xml.push_str(&action.name);
            xml.push_str("\">\n");

            // Special handling for critical UI actions to ensure they always work
            if action.name == "ui_toggle_pause" || action.name == "ui_back" {
                // Set escape key explicitly for menu navigation
                xml.push_str("   <rebind input=\"kb1_escape\" activationMode=\"press\"/>\n");
            } else {
                // Skip mouse axis actions to preserve view controls (rotateyaw, rotatepitch, etc.)
                // These are critical for camera/view control and SC handles them differently
                let is_mouse_axis_action = action.name.contains("yaw")
                    || action.name.contains("pitch")
                    || action.name.contains("roll")
                    || action.default_mouse.contains("maxis_");

                // Add blank rebinds for each selected device for all other actions
                if devices.keyboard {
                    xml.push_str("   <rebind input=\"kb1_ \"/>\n");
                }
                if devices.mouse && !is_mouse_axis_action {
                    xml.push_str("   <rebind input=\"mouse1_ \"/>\n");
                }
                if devices.gamepad {
                    xml.push_str("   <rebind input=\"gp1_ \"/>\n");
                }
                if devices.joystick1 {
                    xml.push_str("   <rebind input=\"js1_ \"/>\n");
                }
                if devices.joystick2 {
                    xml.push_str("   <rebind input=\"js2_ \"/>\n");
                }
            }

            xml.push_str("  </action>\n");
        }

        xml.push_str(" </actionmap>\n");
    }

    xml.push_str("</ActionMaps>\n");

    Ok(xml)
}

/// Generate a restore defaults profile XML with only default bindings for selected devices
pub fn generate_restore_defaults_xml(
    all_binds: &AllBinds,
    devices: &DeviceSelection,
) -> Result<String, String> {
    let mut xml = String::new();

    // XML declaration
    xml.push_str("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");

    // Root ActionMaps element
    xml.push_str("<ActionMaps version=\"1\" optionsVersion=\"2\" rebindVersion=\"2\" profileName=\"RESTORE_DEFAULTS\">\n");

    // Write CustomisationUIHeader with proper attributes
    xml.push_str(" <CustomisationUIHeader label=\"RESTORE_DEFAULTS\" description=\"Restores default bindings for selected devices\" image=\"\">\n");

    // Write devices section - order matters!
    xml.push_str("  <devices>\n");
    if devices.keyboard {
        xml.push_str("   <keyboard instance=\"1\"/>\n");
    }
    if devices.mouse {
        xml.push_str("   <mouse instance=\"1\"/>\n");
    }
    if devices.gamepad {
        xml.push_str("   <gamepad instance=\"1\"/>\n");
    }
    if devices.joystick1 {
        xml.push_str("   <joystick instance=\"1\"/>\n");
    }
    if devices.joystick2 {
        xml.push_str("   <joystick instance=\"2\"/>\n");
    }
    xml.push_str("  </devices>\n");

    xml.push_str(" </CustomisationUIHeader>\n");

    // Write options for each device type with default Product IDs
    if devices.keyboard {
        xml.push_str(" <options type=\"keyboard\" instance=\"1\" Product=\"Keyboard  {6F1D2B61-D5A0-11CF-BFC7-444553540000}\"/>\n");
    }
    if devices.mouse {
        xml.push_str(" <options type=\"mouse\" instance=\"1\" Product=\"Mouse  {6F1D2B62-D5A0-11CF-BFC7-444553540000}\"/>\n");
    }
    if devices.joystick1 {
        xml.push_str(" <options type=\"joystick\" instance=\"1\" Product=\"\"/>\n");
    }
    if devices.joystick2 {
        xml.push_str(" <options type=\"joystick\" instance=\"2\" Product=\"\"/>\n");
    }
    if devices.gamepad {
        xml.push_str(" <options type=\"gamepad\" instance=\"1\" Product=\"\"/>\n");
    }

    // Empty modifiers
    xml.push_str(" <modifiers />\n");

    // Write actionmaps with default bindings only (no customizations)
    for action_map in &all_binds.action_maps {
        let mut has_default_bindings = false;

        // Check if this action map has any default bindings for selected devices
        for action in &action_map.actions {
            if (devices.keyboard && !action.default_keyboard.trim().is_empty())
                || (devices.mouse && !action.default_mouse.trim().is_empty())
                || (devices.gamepad && !action.default_gamepad.trim().is_empty())
                || (devices.joystick1 && !action.default_joystick.trim().is_empty())
                || (devices.joystick2 && !action.default_joystick.trim().is_empty())
            {
                has_default_bindings = true;
                break;
            }
        }

        // Only write the action map if it has default bindings for selected devices
        if !has_default_bindings {
            continue;
        }

        xml.push_str(" <actionmap name=\"");
        xml.push_str(&action_map.name);
        xml.push_str("\">\n");

        for action in &action_map.actions {
            let mut default_inputs: Vec<String> = Vec::new();

            // Collect default bindings for selected devices
            if devices.keyboard && !action.default_keyboard.trim().is_empty() {
                default_inputs.push(format!("kb1_{}", action.default_keyboard.trim()));
            }
            if devices.mouse && !action.default_mouse.trim().is_empty() {
                default_inputs.push(format!("mouse1_{}", action.default_mouse.trim()));
            }
            if devices.gamepad && !action.default_gamepad.trim().is_empty() {
                default_inputs.push(format!("gp1_{}", action.default_gamepad.trim()));
            }
            if devices.joystick1 && !action.default_joystick.trim().is_empty() {
                default_inputs.push(format!("js1_{}", action.default_joystick.trim()));
            }
            if devices.joystick2 && !action.default_joystick.trim().is_empty() {
                default_inputs.push(format!("js2_{}", action.default_joystick.trim()));
            }

            // Only write the action if it has default bindings
            if default_inputs.is_empty() {
                continue;
            }

            xml.push_str("  <action name=\"");
            xml.push_str(&action.name);
            xml.push_str("\">\n");

            // Write each default binding
            for input in default_inputs {
                xml.push_str("   <rebind input=\"");
                xml.push_str(&input);
                xml.push_str("\"");

                // Add activation mode if present and not default "press"
                if !action.activation_mode.is_empty() && action.activation_mode != "press" {
                    xml.push_str(" activationMode=\"");
                    xml.push_str(&action.activation_mode);
                    xml.push_str("\"");
                }

                xml.push_str("/>\n");
            }

            xml.push_str("  </action>\n");
        }

        xml.push_str(" </actionmap>\n");
    }

    xml.push_str("</ActionMaps>\n");

    Ok(xml)
}
