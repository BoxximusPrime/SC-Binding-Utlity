# Boxxy Binder

A desktop application for managing joystick (including button boxes, etc), keyboard, and mouse key bindings for Star Citizen outside of the game. Built with Tauri 2.0 and Rust, this tool provides a powerful interface to organize, customize, and debug your control scheme.

## Core Features

### Binding Management

<img width="1602" height="1032" alt="image" src="https://github.com/user-attachments/assets/979b32ce-3954-4a89-b179-ac8583888389" />

- **Keyboard/Mouse/Joystick Binding Page**: Comprehensive UI for viewing and modifying all key bindings
- **Sorted Categories**: Organized by major action categories (spaceships, fps, vehicles, etc.) for easy navigation
- **Advanced Filtering**: Quickly find specific actions or bindings
- **Multi-Device Support**: Configure all kinds of devices, like joysticks, button boxes, etc

### Visual Device Viewer

<img width="1602" height="1032" alt="image" src="https://github.com/user-attachments/assets/f8ccd3a6-3e5c-44d2-b679-ae53dfe28c59" />

- **Interactive Visualization**: See your device layout and all assigned actions at a glance
- **Device Stacking**: Can add multiple templates and view all devices within them
- **Visual Indicators**: Identify customized bindings and default overrides
- **Customizable Display**: Hide default binds, customize font size and button sizes, and more

### Template Editor

<img width="1602" height="1032" alt="image" src="https://github.com/user-attachments/assets/47ae769c-286e-4c6e-af6b-94f9da918a2c" />

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

- **Automatic Deployment**: Save, and the app will automatically update your actionmaps for all installations
- **Near Realtime Updates**: Provides a console command to quickly update your binds in-game without having to log out
- **Master Binds File**: Auto-deploy your binds to installs even if your binds file isn't even in the star citizen folder (great if you're constanlly installing PTU, Live, etc)

### Character appearance manager

<img width="1602" height="1032" alt="image" src="https://github.com/user-attachments/assets/26ea855b-14d4-4485-84b4-0e6bd977ea3c" />

- **Backup/Save Your Characters**: Managing multiple installs can make saving and updating your characters a pain - use this to sync them between all installs

## Technical Stack

- **Frontend**: HTML, CSS, JavaScript with Vite
- **Backend**: Rust with Tauri 2.0
- **Platform**: Windows Desktop
- **Input Handling**: DirectInput integration for joystick hardware detection

## To Install

1. **Check the Releases on this repo**: You can find all releases and installer binaries in the releases section on this github repo (right side of page).

## Discord

Feel free to join us on Discord! https://discord.gg/cjEtkeBG

- We have a user-submitted template section with different kinds of devices and combos. Feel free to join and grab them - or submit your own! As well as bug reports, etc.

## AI Disclaimer

- I used copilot and my own brain to make the app, and ComfyUI to make the logo.

## Finding this usefull and want to support?

[![ko-fi](https://ko-fi.com/img/githubbutton_sm.svg)](https://ko-fi.com/C0C51OF2NM)
