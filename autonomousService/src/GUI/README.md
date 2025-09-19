# Robot Navigation Chat Interface

A modern, WhatsApp-style chat interface for autonomous robot navigation using natural language commands.

## Features
- WhatsApp-inspired chat interface with message bubbles
- Real-time robot status feedback
- Natural language command processing
- ROS 2 integration for command and status communication
- Modern material design UI
- Message timestamps and status indicators
- Responsive and user-friendly interface

## Screenshot
![Robot Navigation Chat Interface](screenshot.png)

## Prerequisites
- Python 3.8 or higher
- ROS 2 (Foxy or newer)
- PyQt5
- rclpy
- qt-material

## Installation
1. Clone this repository
2. Install the required dependencies:
   ```bash
   pip install -r requirements.txt
   ```
3. Make sure ROS 2 is properly sourced in your environment:
   ```bash
   source /opt/ros/<ros2-distro>/setup.bash
   ```

## Usage
1. Start the chat interface:
   ```bash
   python main.py
   ```
2. Type natural language commands in the chat input (e.g., "Go to the kitchen" or "Turn right at the couch")
3. Monitor robot status and responses in the chat window
4. The interface supports:
   - Sending commands by pressing Enter or clicking the send button
   - Viewing message timestamps
   - Seeing robot status updates in real-time
   - Auto-scrolling to new messages

## ROS 2 Topics
- Command input: `/robot/commands`
- Status feedback: `/robot/status`

## Interface Features
- **Chat Window**: Displays the conversation history with the robot
- **Status Bar**: Shows current robot status
- **Command Input**: Type and send commands to the robot
- **Message Bubbles**: Different colors for user and robot messages
- **Timestamps**: Shows when messages were sent/received
- **Status Indicators**: Shows message delivery status

## Customization
The interface can be customized by modifying the following:
- Color scheme in `main.py`
- Font sizes and styles
- Message bubble appearance
- Window size and layout

## Contributing
Feel free to submit issues and enhancement requests!

## License
This project is licensed under the MIT License - see the LICENSE file for details. 