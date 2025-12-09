use sc_keymap::button::{PhysicalButtonKind, ShiftKind, TempoKind};
use serde::Serialize;
use std::path::PathBuf;

/// Simplified button info for frontend
#[derive(Serialize, Clone, Debug)]
pub struct VkbButtonInfo {
    pub virtual_id: u8,
    pub physical_id: u8,
    pub physical_info: String, // e.g., "(A2)", "(Fire 1)"
    pub layer: Option<String>, // None, "SHIFT1", "SHIFT2"
    pub tempo: Option<String>, // None, "Short", "Long", "Double"
}

/// Build a lookup map from SC button IDs to VKB button info
///
/// # Arguments
/// * `buttons` - VKB button info from parsing
/// * `joystick_id` - Joystick instance number (1, 2, etc.)
///
/// # Returns
/// HashMap mapping "js1_button105" -> VkbButtonInfo
pub fn build_sc_button_lookup(
    buttons: &[VkbButtonInfo],
    joystick_id: u8,
) -> std::collections::HashMap<String, VkbButtonInfo> {
    let mut lookup = std::collections::HashMap::new();
    for button in buttons {
        let sc_button_id = format!("js{}_button{}", joystick_id, button.virtual_id);
        lookup.insert(sc_button_id, button.clone());
    }
    lookup
}

/// Parse a VKB .fp3 file and return button mappings
///
/// # Errors
/// Returns error string if parsing fails
pub fn parse_vkb_fp3(file_path: PathBuf) -> Result<Vec<VkbButtonInfo>, String> {
    let mapping = sc_keymap::vkb::parse_and_check_vkb_both_sticks(file_path, &None)
        .map_err(|e| format!("VKB parsing error: {e:?}"))?;

    let mut button_infos = Vec::new();

    // Extract virtual -> physical mappings
    for (virtual_id, physical_buttons) in &mapping.map_virtual_button_id_to_parent_physical_buttons
    {
        for physical_button in physical_buttons {
            let (layer, tempo) = determine_layer_and_tempo(physical_button.get_kind(), *virtual_id);

            button_infos.push(VkbButtonInfo {
                virtual_id: *virtual_id,
                physical_id: *physical_button.get_id(),
                physical_info: physical_button.get_info().clone(),
                layer,
                tempo,
            });
        }
    }

    log::info!(
        "Parsed {} virtual buttons from VKB report",
        button_infos.len()
    );

    Ok(button_infos)
}

/// Determine layer and tempo information from button kind
fn determine_layer_and_tempo(
    kind: &PhysicalButtonKind,
    virtual_id: u8,
) -> (Option<String>, Option<String>) {
    let layer = match kind {
        PhysicalButtonKind::Momentary { shift: Some(shift) } => match shift {
            ShiftKind::Shift1 { button_id_shift1 } if virtual_id == *button_id_shift1 => {
                Some("SHIFT1".to_string())
            }
            ShiftKind::Shift2 { button_id_shift2 } if virtual_id == *button_id_shift2 => {
                Some("SHIFT2".to_string())
            }
            ShiftKind::Shift12 {
                button_id_shift1,
                button_id_shift2,
            } => {
                if virtual_id == *button_id_shift1 {
                    Some("SHIFT1".to_string())
                } else if virtual_id == *button_id_shift2 {
                    Some("SHIFT2".to_string())
                } else {
                    None
                }
            }
            _ => None,
        },
        PhysicalButtonKind::Shift1 => Some("SHIFT1_MODIFIER".to_string()),
        PhysicalButtonKind::Shift2 => Some("SHIFT2_MODIFIER".to_string()),
        _ => None,
    };

    let tempo = match kind {
        PhysicalButtonKind::Tempo(tempo_kind) => match tempo_kind {
            TempoKind::Tempo2 {
                button_id_short,
                button_id_long,
            } => {
                if virtual_id == *button_id_short {
                    Some("Short".to_string())
                } else if virtual_id == *button_id_long {
                    Some("Long".to_string())
                } else {
                    None
                }
            }
            TempoKind::Tempo3 {
                button_id_short,
                button_id_long,
                button_id_double,
            } => {
                if virtual_id == *button_id_short {
                    Some("Short".to_string())
                } else if virtual_id == *button_id_long {
                    Some("Long".to_string())
                } else if virtual_id == *button_id_double {
                    Some("Double".to_string())
                } else {
                    None
                }
            }
            _ => None,
        },
        _ => None,
    };

    (layer, tempo)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_layer_detection_shift1() {
        let kind = PhysicalButtonKind::Momentary {
            shift: Some(ShiftKind::Shift1 {
                button_id_shift1: 105,
            }),
        };

        let (layer, tempo) = determine_layer_and_tempo(&kind, 105);
        assert_eq!(layer, Some("SHIFT1".to_string()));
        assert_eq!(tempo, None);
    }

    #[test]
    fn test_tempo_detection_short() {
        let kind = PhysicalButtonKind::Tempo(TempoKind::Tempo2 {
            button_id_short: 50,
            button_id_long: 51,
        });

        let (layer, tempo) = determine_layer_and_tempo(&kind, 50);
        assert_eq!(layer, None);
        assert_eq!(tempo, Some("Short".to_string()));
    }

    #[test]
    fn test_tempo_detection_long() {
        let kind = PhysicalButtonKind::Tempo(TempoKind::Tempo2 {
            button_id_short: 50,
            button_id_long: 51,
        });

        let (layer, tempo) = determine_layer_and_tempo(&kind, 51);
        assert_eq!(layer, None);
        assert_eq!(tempo, Some("Long".to_string()));
    }
}
