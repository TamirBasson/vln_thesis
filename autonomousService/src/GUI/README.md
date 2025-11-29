# Robot Navigation GUI System

A graphical user interface for controlling the autonomous robot navigation system with natural language commands.

## Overview

This GUI system allows you to:
- Send natural language navigation commands to the robot
- Set the environment context (Warehouse, Home, Office, etc.)
- View real-time status updates from the robot
- See the complete conversation history

## Features

✨ **Natural Language Commands**: "Move to the table", "Go near the chair", etc.

🗺️ **Intelligent Path Planning**: Uses D* Lite algorithm to find alternative routes when objects are blocked

🖥️ **Beautiful Interface**: Clean, modern chat-style interface with gradient design

📡 **Real-time Updates**: See robot status and navigation progress in real-time

## Quick Start

### Option 1: Using the Launcher (Recommended)

```bash
cd /home/tamir/thesis/autonomousService/src/GUI
python3 launch_robot_gui.py
```

This will automatically start both:
1. The autonomous service node
2. The GUI interface

### Option 2: Manual Launch

Terminal 1 - Start the service:
```bash
cd /home/tamir/thesis/autonomousService/src/detect_vl/detect_vl
python3 start_service.py
```

Terminal 2 - Start the GUI:
```bash
cd /home/tamir/thesis/autonomousService/src/GUI
python3 robot_chat_gui.py
```

## Usage Guide

### 1. Set Environment Context

When the GUI opens:
1. Look for the "Environment:" field at the top
2. Enter your environment type (e.g., "Home", "Warehouse", "Office")
3. Click "Set Context"

Default: "Home"

### 2. Send Navigation Commands

Type natural language commands in the message box at the bottom:

**Example Commands:**
- `Move to the table`
- `Go near the chair and stop`
- `Navigate to the bed, turn right, and stop`
- `Move to the exercise ball, turn right, and stop near the bed`

### 3. Monitor Progress

The chat window will show:
- ✅ Command received confirmation
- 🤔 Command processing status
- 🎯 Navigation target coordinates
- 🗺️ Motion planner activation (when needed)
- ✅ Goal reached notifications
- ❌ Error messages with suggestions

## How It Works

### Architecture

```
┌─────────────────┐         /service_question        ┌──────────────────┐
│                 │────────────────────────────────>│                  │
│  GUI Interface  │                                  │  Service Node    │
│                 │<────────────────────────────────│                  │
└─────────────────┘         /robot_status           └──────────────────┘
        │                                                     │
        │ /environment_context                               │
        └────────────────────────────────────────────────────┘
```

### ROS2 Topics

| Topic | Direction | Purpose |
|-------|-----------|---------|
| `/service_question` | GUI → Service | Navigation commands |
| `/environment_context` | GUI → Service | Environment type |
| `/robot_status` | Service → GUI | Status updates |

### Navigation Flow

1. **User sends command** via GUI
2. **Command queued** in service node
3. **GPT-4o processes** command → extracts objects and actions
4. **Vision system detects** objects in camera view
5. **If object not visible** → Motion planner finds alternative route
6. **Robot navigates** to target
7. **Status updates** sent back to GUI

## Motion Planner Integration

The system includes an intelligent fallback mechanism:

1. **Direct Detection**: First tries to detect object directly in camera view
2. **Motion Planning**: If blocked, searches `memory.yaml` for object location
3. **Path Planning**: Uses D* Lite algorithm to find optimal path
4. **Room Navigation**: Navigates through intermediate rooms to reach target
5. **Final Approach**: Once in correct room, attempts direct navigation

**Example Scenario:**
```
Command: "Move to the bed"

❌ Bed not visible (blocked)
🗺️ Motion planner activated
🔍 Found bed in "living room"
📍 Current location: "home gym"
🛤️ Path: home gym → living room
🚶 Navigating to living room...
✅ Arrived at living room
🎯 Navigating to bed...
✅ Goal reached!
```

## GUI Features

### Chat Display
- **User messages**: Green bubbles (right-aligned)
- **Robot messages**: Gray bubbles (left-aligned)
- **Auto-scroll**: Always shows latest message

### Environment Context
- Set once at startup
- Can be changed at any time
- Used by GPT for better understanding

### Status Icons
- 🤖 Robot/System messages
- 📝 Command received
- 🤔 Processing
- 🎯 Navigating
- ✅ Success
- ❌ Error
- 💡 Suggestions
- 🗺️ Motion planner active

## Troubleshooting

### GUI doesn't open
- Check PyQt5 is installed: `pip install PyQt5 PyQtWebEngine`
- Check ROS2 is sourced

### Commands not working
1. Check service node is running
2. Verify camera data is being received
3. Check ROS2 topics: `ros2 topic list`

### Object not found
1. Make sure object is visible in camera view
2. Check object is in `memory.yaml` (for motion planner)
3. Try more descriptive terms (e.g., "blue chair" instead of "chair")

### Connection Issues
```bash
# Check if topics are active
ros2 topic echo /robot_status
ros2 topic echo /service_question
```

## Dependencies

```bash
# Python packages
pip install PyQt5 PyQtWebEngine rclpy opencv-python pillow numpy

# ROS2 packages
# (Already installed in your workspace)
```

## Configuration

### Memory File Location
The motion planner uses: `/home/tamir/thesis/autonomousService/src/memory.yaml`

To use a different environment:
- Switch to `memory_house.yaml` by modifying the path in `start_service.py`

### Camera Settings
- Simulation mode is enabled by default
- Camera parameters are configured in `start_service.py`

## Advanced Usage

### Custom Commands
The system supports various command formats:
- Simple: `"Go to the table"`
- With relation: `"Move near the chair"`
- Sequential: `"Go to the door, turn right, stop at the bed"`

### Environment Contexts
Different contexts help GPT understand the scene better:
- **Home**: Bedrooms, living room, kitchen
- **Warehouse**: Shelves, pallets, loading docks
- **Office**: Desks, meeting rooms, cubicles
- **Gym**: Equipment, mats, weights

## Files

- `robot_chat_gui.py`: Main GUI application
- `launch_robot_gui.py`: Launcher script for both GUI and service
- `README.md`: This file

## Support

For issues or questions:
1. Check the terminal output for detailed logs
2. Review ROS2 topics for message flow
3. Check `memory.yaml` for available locations

## Future Improvements

- [ ] Map visualization in GUI
- [ ] Voice commands
- [ ] Multi-robot support
- [ ] Command history with re-send
- [ ] Waypoint planning interface




