# Star Citizen Binding Utility

<img width="1502" height="1156" alt="sc-joy-mapper_uK1072xT1b" src="https://github.com/user-attachments/assets/d93c0709-602d-42c0-8a11-716c1efc043c" />
<img width="1502" height="1032" alt="sc-joy-mapper_vncPl1X5MI" src="https://github.com/user-attachments/assets/9f30bd34-36b4-4a0c-b67f-94a0a998ae49" />

A desktop application for managing joystick, keyboard, and mouse key bindings for Star Citizen outside of the game. Built with Tauri 2.0 and Rust, this tool provides a powerful interface to organize, customize, and debug your control scheme.

## Overview

Star Citizen Joystick Mapper simplifies the complex task of configuring control inputs. Whether you're mapping a single joystick or dual-stick setup, this application provides visual feedback, comprehensive organization, and direct integration with your Star Citizen installation.

## Core Features

### Binding Management
- **Keyboard/Mouse/Joystick Binding Page**: Comprehensive UI for viewing and modifying all key bindings
- **Sorted Categories**: Automatically organized by major action categories (spaceships, fps, vehicles, etc.) for easy navigation
- **Advanced Filtering**: Quickly find specific actions or bindings
- **Multi-Joystick Support**: Configure up to two joysticks per profile with dedicated left and right view modes

### Visual Joystick Viewer
- **Interactive Visualization**: See your joystick layout and all assigned actions at a glance
- **Left/Right View**: Toggle between individual stick views or compare configurations side-by-side
- **Visual Indicators**: Identify customized bindings and default overrides
- **Customizable Display**: Hide default keys and focus only on personalized bindings

### Template Editor
- **Custom Template Builder**: Create your own joystick input templates without leaving the app
- **Button Management**: Add, remove, and reposition buttons on your template
- **Image Upload**: Add custom images and button graphics to your templates
- **Mirror Support**: Easily mirror button layouts for dual-stick configurations
- **Template Persistence**: Save and manage multiple template profiles

### Input Debugging
- **Button ID Detector**: Automatically identify button IDs from joystick input
- **Real-time Input Monitoring**: Watch button presses register in real-time
- **Troubleshooting Helper**: Quickly determine unknown button identifiers from your hardware

### Auto-Save & Integration
- **Automatic Binding Updates**: Select your Star Citizen installation directory and the app automatically updates your actionmaps
- **Direct File Integration**: Seamlessly sync with Star Citizen keybinding files
- **Profile Management**: Support for multiple joystick profiles (VKB, Thrustmaster, etc.)
- **Change Detection**: Auto-save captures and applies configuration changes

## Advanced Features

- **Modifier Key Support**: Bind actions with Ctrl, Alt, and Shift modifiers
- **Binding Conflict Detection**: Visual alerts when multiple actions share the same binding
- **Visual Export**: Export your joystick layout to an image
- **Cleared Binding Display**: Visual distinction between cleared overrides and default bindings
- **State Persistence**: Application remembers your visual view settings (pan, zoom, active view) between sessions
- **Checks for Updates**: Automatically checks github for any updates

## Technical Stack

- **Frontend**: HTML, CSS, JavaScript with Vite
- **Backend**: Rust with Tauri 2.0
- **Platform**: Windows Desktop
- **Input Handling**: DirectInput integration for joystick hardware detection

## Usage

1. **Launch the Application**: Start the Joystick Mapper
2. **Load or Create a Profile**: Select an existing keybinding .xml file or create a new one
   - When saving a key binding file, name is "layout_NAME_exported.xml" replacing NAME with your bindings name.
4. **Customize Bindings**: 
   - Use the Binding Page to set up keyboard, mouse, and joystick inputs
   - Use the Visual View to see your joystick layout
5. **Apply Changes**: Auto-save will update your Star Citizen installation automatically when you save your bindings
6. 6. While in-game, go to your advanced key bindings, and import your saved profile (sometimes may need to import twice, idk why)

## Tips & Tricks

- **Template Editor And Visual View**: Use middle-click and drag to pan the view, double-click buttons on the template view to edit them.
- **Profile Management**: Store multiple joystick profiles and switch between them easily
- **Visual Debugging**: Use the Visual View to spot binding gaps or conflicts
- **Template Sharing**: Export and share custom templates with other players
- **Batch Updates**: The auto-save system means changes sync instantly to your game, including multiple installs

## Discord
Feel free to join us on Discord! https://discord.gg/73uBrHtX

## AI Disclaimer
- This utility was created with vibes and some old fashioned analog brain.

## License

This project is designed for personal use with Star Citizen by Cloud Imperium Games.

---

Made with Rust and web technologies for the Star Citizen community.
