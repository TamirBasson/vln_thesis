import sys
import json
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QLineEdit,
    QPushButton, QLabel, QHBoxLayout, QFrame
)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QFont, QColor, QLinearGradient, QPainter
from PyQt5.QtWebEngineWidgets import QWebEngineView
from threading import Thread
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class GradientFrame(QFrame):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setSizePolicy(QFrame.Expanding, QFrame.Expanding)

    def paintEvent(self, event):
        painter = QPainter(self)
        gradient = QLinearGradient(0, 0, 0, self.height())
        gradient.setColorAt(0, QColor("#43cea2"))
        gradient.setColorAt(1, QColor("#185a9d"))
        painter.fillRect(self.rect(), gradient)

class RobotChatNode(Node):
    def __init__(self):
        super().__init__('robot_chat_node')
        self.nav_pub = self.create_publisher(String, 'navigation_command', 10)
        self.question_pub = self.create_publisher(String, 'service_question', 10)
        self.subscription = self.create_subscription(
            String, 'robot_status', self.listener_callback, 10)
        self.last_status_msg = ""
        self.get_logger().info("Robot ready for command.")
        self.last_status_msg = "🤖 Robot: Ready for your command."

    def send_nav(self, command: str):
        msg = String()
        msg.data = command
        self.nav_pub.publish(msg)
        self.get_logger().info(f'Sent navigation command: "{command}"')

    def send_question(self, question: str):
        msg = String()
        msg.data = question
        self.question_pub.publish(msg)
        self.get_logger().info(f'Sent service question: "{question}"')

    def listener_callback(self, msg: String):
        if "reached" in msg.data.lower():
            self.last_status_msg = "🤖 Robot: ✅ Destination reached."
        else:
            self.last_status_msg = f"🤖 Robot: {msg.data}"

class RobotChatGUI(QWidget):
    def __init__(self, ros_node: RobotChatNode):
        super().__init__()
        self.ros_node = ros_node
        self.init_ui()

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_status)
        self.timer.start(500)

    def init_ui(self):
        self.setWindowTitle('Autonomous Robot Navigation Chat')
        self.setMinimumSize(800, 600)
        layout = QVBoxLayout()
        self.setLayout(layout)

        header = QLabel("Autonomous Robot Navigation Chat")
        header.setStyleSheet(
            "background-color: #4CAF50; color: white; font-size: 24px; padding: 15px; text-align: center;"
        )
        header.setAlignment(Qt.AlignCenter)
        layout.addWidget(header)

        self.chat_display = QWebEngineView()
        self.chat_html = """
        <html><head><style>
        body { background-color: #f0f0f0; font-family: Arial; padding: 10px; }
        .user { background-color: #dcf8c6; padding: 10px; border-radius: 10px; margin: 10px 50px 10px auto; text-align: right; max-width: 70%; }
        .robot { background-color: #e5e5ea; padding: 10px; border-radius: 10px; margin: 10px auto 10px 50px; text-align: left; max-width: 70%; }
        .sender { font-weight: bold; display: block; margin-bottom: 4px; color: #555; }
        </style></head><body>
        """
        self.chat_display.setHtml(self.chat_html + "</body></html>")
        layout.addWidget(self.chat_display)

        input_layout = QHBoxLayout()
        self.input_line = QLineEdit()
        self.input_line.setPlaceholderText("Type your message here...")
        self.send_button = QPushButton("Send")
        self.send_button.clicked.connect(self.on_send)
        self.input_line.returnPressed.connect(self.on_send)
        input_layout.addWidget(self.input_line)
        input_layout.addWidget(self.send_button)
        layout.addLayout(input_layout)

    def append_chat(self, sender_class: str, sender_name: str, text: str):
        safe_text = text.replace('<', '&lt;').replace('>', '&gt;')
        self.chat_html += f'<div class="{sender_class}"><span class="sender">{sender_name}</span>{safe_text}</div>'
        self.chat_display.setHtml(self.chat_html + "</body></html>")
        self.chat_display.page().runJavaScript("window.scrollTo(0, document.body.scrollHeight);")

    def on_send(self):
        text = self.input_line.text().strip()
        if not text:
            return
        self.ros_node.send_nav(text)
        self.ros_node.send_question(text)
        self.append_chat("user", "You:", text)
        self.input_line.clear()

    def update_status(self):
        status = self.ros_node.last_status_msg
        if status:
            display = status.replace("🤖 Robot: ", "")
            self.append_chat("robot", "🤖 Robot:", display)
            self.ros_node.last_status_msg = ""


def main(args=None):
    rclpy.init(args=args)
    ros_node = RobotChatNode()

    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    app.setFont(QFont("Segoe UI", 10))

    gui = RobotChatGUI(ros_node)
    gui.show()

    ros_thread = Thread(target=lambda: rclpy.spin(ros_node), daemon=True)
    ros_thread.start()

    sys.exit(app.exec_())
    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()