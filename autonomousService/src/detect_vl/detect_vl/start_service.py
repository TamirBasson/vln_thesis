"""
###########################
# Autonomous Navigation Service Node
# LLM: GPT-4o | VLM: Grounding DINO
#
# Main Functions:
# 1. Understand task requirements and extract key features
# 2. Extract targets for VLM, obtain distance from depth map, solve world coordinate offset
# 3. Publish coordinates_diff for navigation
############################
"""

import rclpy
from rclpy.node import Node
import threading
import time
import os

# Vision packages
from cv_bridge import CvBridge
import cv2
from PIL import Image as PILImage
import numpy as np

# ROS2 messages
from common_interface.msg import RectDepth, Camera2map
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Int32MultiArray, Float32MultiArray, String
from std_srvs.srv import Trigger

# Custom modules
from scripts.service_vl import GroundingDINOInfer
import scripts.service_lm as lm
import scripts.ans2json as ans2json
from scripts.memory_builder import MemoryBuilder
from scripts.motion_planner import DStarLitePlanner
import yaml

# Set environment variable for display
os.environ["QT_QPA_PLATFORM"] = "xcb"


class ServiceNode(Node):
    def __init__(self):
        super().__init__('detect_vl_node')
        
        # Initialize core components
        self.bridge = CvBridge()
        self.VL = GroundingDINOInfer()
        
        # Configure for simulation mode (lower thresholds for better detection)
        self.VL.set_simulation_mode()
        self.get_logger().info("🎮 Configured GroundingDINO for simulation mode")
        
        self.memory_builder = MemoryBuilder()
        self.motion_planner = None  # Will be initialized after memory file is set
        
        # Image data
        self.rgb_image = None
        self.depth_image = None
        
        # Task data
        self.obj_list = None
        self.turn_list = None
        self.relation_list = None
        self.rect: list[int] | None = None
        
        # State variables
        self.robot_state = "navigating"
        self.environment_context = ""
        self.current_room = None
        self.room_pose = [0.0, 0.0, 0.0]
        
        # Control flags
        self.update_flag = 1
        self.suppress_background_activity = False
        
        # GUI command queue
        self.command_queue = []
        self.processing_command = False
        
        # Camera2map monitoring
        self.last_camera2map_time = 0
        self.camera2map_warning_threshold = 5.0
        self.camera2map_warning_sent = False
        
        # Display control
        self.last_display_update: float = 0.0
        self.display_update_interval = 0.1
        
        # File paths
        self.memory_file = "/src/memory.yaml"
        
        # Initialize motion planner with workspace path
        workspace_root = self._find_workspace_root()
        memory_path = os.path.join(workspace_root, "autonomousService", "src", "memory.yaml")
        if os.path.exists(memory_path):
            self.motion_planner = DStarLitePlanner(memory_path)
            self.get_logger().info(f"🗺️ Motion planner initialized with: {memory_path}")
        else:
            self.get_logger().warn(f"⚠️ Memory file not found at {memory_path}, motion planner not initialized")
        
        self._setup_subscriptions_and_publishers()
        self._setup_timers()
        
        self.get_logger().info("ServiceNode started - waiting for images and camera2map messages...")
    
    def _find_workspace_root(self):
        """Find the workspace root directory"""
        current_dir = os.path.dirname(os.path.abspath(__file__))
        
        # Look for workspace root by going up directories
        while current_dir != os.path.dirname(current_dir):  # Stop at filesystem root
            # Check if this looks like a workspace root
            if os.path.exists(os.path.join(current_dir, 'autonomousService')):
                return current_dir
            current_dir = os.path.dirname(current_dir)
        
        # Default to /home/tamir/thesis if not found
        return "/home/tamir/thesis"

    def _setup_subscriptions_and_publishers(self):
        """Initialize ROS2 subscriptions and publishers"""
        # Subscriptions
        # self.create_subscription(
        #     CompressedImage, 
        #     '/camera/camera/color/image_raw/compressed', 
        #     self.rgb_callback, 10
        # )
        # self.create_subscription(
        #     Image, 
        #     '/camera/camera/depth/image_rect_raw', 
        #     self.depth_callback, 10
        # )

        #For Simulation only --------------------------------
        self.create_subscription(
            Image, 
            '/camera/camera/image_raw', 
            self.rgb_callback, 10
        )
        self.create_subscription(
            Image, 
            '/camera/camera/depth/image_raw', 
            self.depth_callback, 10
        )
          #For Simulation only --------------------------------
          
        self.create_subscription(
            Camera2map, 
            '/camera2map', 
            self.camera2map_callback, 10
        )
        self.create_subscription(
            String, 
            '/robot_state', 
            self.robot_state_update_callback, 10
        )
        self.create_subscription(
            String,
            '/service_question',
            self.gui_command_callback, 10
        )
        self.create_subscription(
            String,
            '/environment_context',
            self.environment_context_callback, 10
        )
        
        # Publishers
        self.target_pub = self.create_publisher(RectDepth, 'task/rect_depth', 10)
        self.status_pub = self.create_publisher(String, '/robot_status', 10)

    def _setup_timers(self):
        """Initialize ROS2 timers"""
        self.update_memory_map = self.create_timer(1.0, self.update_map)
        self.camera2map_monitor_timer = self.create_timer(2.0, self.monitor_camera2map_topic)
        self.command_processor_timer = self.create_timer(0.5, self.process_command_queue)

    def suppress_background_logging(self, suppress=True):
        """Enable or disable background activity logging"""
        self.suppress_background_activity = suppress
        if suppress:
            self.get_logger().info("🔇 Background activity logging suppressed")
        else:
            self.get_logger().info("🔊 Background activity logging enabled")

    # ============================================================================
    # CALLBACK FUNCTIONS
    # ============================================================================

    def rgb_callback(self, msg):
        """Handle RGB image messages"""
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.update_flag = 1
        except Exception as e:
            self.get_logger().error(f"Error processing RGB image: {e}")

    def depth_callback(self, msg):
        """Handle depth image messages"""
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            
            if self.depth_image is not None and self.depth_image.size != 0:
                # Normalize depth for visualization
                depth_normalized = np.zeros_like(self.depth_image)
                cv2.normalize(self.depth_image, depth_normalized, 0, 255, cv2.NORM_MINMAX)
                depth_normalized = np.uint8(depth_normalized)
        except Exception as e:
            self.get_logger().error(f"Depth image failed to transfer: {e}")

    def camera2map_callback(self, msg):
        """Handle camera to map transformation updates"""
        try:
            self.last_camera2map_time = time.time()
            self.camera2map_warning_sent = False
            
            wx, wy, yaw = msg.coordinate.data
            
            if not self.suppress_background_activity:
                self.memory_builder.update_camera_pose(wx, wy, yaw, self.get_logger())
            else:
                self.memory_builder.update_camera_pose(wx, wy, yaw, None)
        except Exception as e:
            self.get_logger().error(f"Error processing camera pose: {e}")

    def robot_state_update_callback(self, msg):
        """Update robot state from navigation node"""
        self.robot_state = msg.data
        if not self.suppress_background_activity:
            self.get_logger().info(f"🤖 Robot state updated to: {self.robot_state}")
        
        # Publish status to GUI
        if self.robot_state == "reachGoal":
            self._publish_status("✅ Reached goal!")
    
    def gui_command_callback(self, msg):
        """Handle commands from GUI"""
        command = msg.data.strip()
        if command:
            self.get_logger().info(f"📨 Received GUI command: {command}")
            self.command_queue.append(command)
            self._publish_status(f"📝 Command received: {command}")
    
    def environment_context_callback(self, msg):
        """Handle environment context updates from GUI"""
        context = msg.data.strip()
        if context:
            self.environment_context = context
            self.get_logger().info(f"🌍 Environment context updated: {context}")
            self._publish_status(f"✅ Environment set to: {context}")
    
    def _publish_status(self, message: str):
        """Publish status message to GUI"""
        msg = String()
        msg.data = message
        self.status_pub.publish(msg)
        self.get_logger().info(f"📤 Status to GUI: {message}")

    # ============================================================================
    # TIMER FUNCTIONS
    # ============================================================================

    def update_map(self):
        """Update semantic map with detected features"""
        if self.suppress_background_activity or not self.update_flag or self.rgb_image is None:
            return
            
        self.update_flag = 0
        pil_image = PILImage.fromarray(cv2.cvtColor(self.rgb_image, cv2.COLOR_BGR2RGB))
        
        try:
            if self.environment_context:
                self.get_logger().info(f"🗺️ Using environment context: {self.environment_context}")
            
            map_analysis = lm.gpt_map_build(pil_image, self.environment_context)
            map_analysis = ans2json.ans2json(map_analysis)
            self.get_logger().info(f"Map Analysis: {map_analysis}")
            
        except Exception as e:
            self.get_logger().error(f"OpenAI API error in map building: {e}")
            self.get_logger().warn("Skipping map update due to API timeout/error")
            return
        
        features_with_coords = self._process_map_features(map_analysis)
        self._save_room_classification(map_analysis, features_with_coords)
        
        self.get_logger().info("Updated features in map!")

    def _process_map_features(self, map_analysis):
        """Process detected features and calculate coordinates"""
        features_with_coords = []
        
        if map_analysis and "features" in map_analysis:
            for feature in map_analysis["features"]:
                if "object" in feature:
                    obj_name = feature["object"]
                    img_detect, rect, center = self.VL.infer(self.rgb_image, obj_name + ".")
                    
                    if rect is not None and center is not None:
                        dis, wx, wy = self.memory_builder.pix2camera_frame(
                            center, self.depth_image, self.get_logger()
                        )
                        
                        if dis is not None and dis > 0:
                            features_with_coords.append({
                                "object": obj_name,
                                "Coordinate relative to the camera frame": [wx, wy]
                            })
        
        return features_with_coords

    def _save_room_classification(self, map_analysis, features_with_coords):
        """Save room classification and features to memory"""
        if not features_with_coords or "room_type" not in map_analysis:
            return
            
        proposed_room_type = map_analysis["room_type"]
        room_pose = self.memory_builder.camera_pose if self.memory_builder.camera_pose else [0.0, 0.0, 0.0]
        
        if self.memory_builder.can_classify_new_room(proposed_room_type, self.get_logger()):
            self.memory_builder.save_to_memory(proposed_room_type, features_with_coords, room_pose)
            self.get_logger().info(f"✅ Room '{proposed_room_type}' classified and saved to memory at pose {room_pose}")
        else:
            if self.memory_builder.last_room_type:
                self.memory_builder.save_to_memory(self.memory_builder.last_room_type, features_with_coords, room_pose)
                self.get_logger().info(f"📝 Features saved to existing room '{self.memory_builder.last_room_type}' (no new room classification)")
            else:
                self.get_logger().warn("⚠️ No room type available for feature storage")



    def monitor_camera2map_topic(self):
        """Monitor camera2map topic and warn if no messages received"""
        if self.suppress_background_activity:
            return
            
        current_time = time.time()
        time_since_last_message = current_time - self.last_camera2map_time
        
        if time_since_last_message > self.camera2map_warning_threshold and not self.camera2map_warning_sent:
            self.get_logger().warn(f"⚠️ No /camera2map messages received for {time_since_last_message:.1f} seconds. Camera pose might not be updated.")
            self.camera2map_warning_sent = True
        elif time_since_last_message <= self.camera2map_warning_threshold and self.camera2map_warning_sent:
            self.get_logger().info("✅ /camera2map topic is now receiving messages again.")
            self.camera2map_warning_sent = False

    # ============================================================================
    # NAVIGATION EXECUTION
    # ============================================================================

    def _process_object_navigation(self, obj, relation, idx):
        """Process navigation to detected object"""
        img_detect, rect, center = self.VL.infer(self.rgb_image, obj + ".")
        self.rect = rect
        
        if img_detect is not None:
            cv2.imshow("VLM Detection", img_detect)
            cv2.waitKey(1)

        if rect is None:
            self._publish_status(f"❌ Object '{obj}' not visible")
            self._publish_status("💡 Trying motion planner for alternative route...")
            
            # Try using motion planner to find alternative route
            if self.motion_planner:
                success = self._navigate_with_motion_planner(obj, relation)
                if success:
                    return True
            
            # If motion planner also fails, show troubleshooting suggestions
            self._publish_status(f"❌ Could not find '{obj}' in view or memory")
            self._publish_status("💡 Make sure the object is visible or recorded in memory")
            return False
        
        dis, wx, wy = self.memory_builder.pix2camera_frame(center, self.depth_image, self.get_logger())
        if dis is None or dis == 0:
            return False
        
        # Adjust distance based on spatial relation
        orig_dis = dis
        dis = self._adjust_distance_by_relation(dis, relation)
        if dis < 0.0:
            dis = 0.0

        # Recompute coordinates along the same camera ray to match adjusted distance
        if orig_dis and orig_dis > 0.0:
            scale = dis / orig_dis
            wx *= scale
            wy *= scale
        
        # Create and publish navigation message
        msg = self._create_navigation_message(rect, center, dis, wx, wy)
        self.target_pub.publish(msg)
        
        status_msg = f"🎯 Navigating to '{obj}' at ({wx:.2f}, {wy:.2f}), distance: {dis:.2f}m"
        print(status_msg)
        self._publish_status(status_msg)
        return True

    def _adjust_distance_by_relation(self, distance, relation):
        """Adjust target distance based on spatial relation"""
        if relation == 'near':
            return distance - 1.0
        elif relation == 'through':
            return distance + 0.5
        elif relation == 'at':
            return distance
        return distance

    def _create_navigation_message(self, rect, center, dis, wx, wy, theta=0.0):
        """Create RectDepth message for navigation"""
        msg = RectDepth()
        
        msg.rect = Int32MultiArray()
        msg.rect.data = rect
        
        msg.center = Int32MultiArray()
        msg.center.data = center
        
        msg.frame = time.time()
        msg.depth = dis
        msg.theta = theta
        
        msg.coordinate_diff = Float32MultiArray()
        msg.coordinate_diff.data = [wx, wy]
        
        return msg

    def _wait_for_goal_completion(self):
        """Wait for robot to reach the current goal"""
        print("Waiting for robot to reach goal...")
        timeout = 20.0  # 20 second timeout
        start_time = time.time()
        
        while self.robot_state != "reachGoal":
            elapsed = time.time() - start_time
            
            # Check for timeout
            if elapsed > timeout:
                print(f"⚠️ Warning: Goal timeout after {timeout}s. Current state: {self.robot_state}")
                self._publish_status(f"⚠️ Navigation timeout - assuming goal reached")
                break
            
            # Print status less frequently (every 2 seconds instead of 0.5)
            if int(elapsed * 2) % 4 == 0:  # Print every 2 seconds
                print(f"Robot state: {self.robot_state} (elapsed: {elapsed:.1f}s)")
            
            time.sleep(0.5)
        
        if self.robot_state == "reachGoal":
            print("✅ Robot reached goal!")
            self._publish_status("✅ Reached goal!")
        
        # Reset state for next goal
        time.sleep(0.5)  # Small delay to ensure message is sent

    def _process_turn_action(self, act):
        """Process turn action"""
        msg = self._create_navigation_message([], [], 0.0, 0.0, 0.0, float(act))
        msg.coordinate_diff = Float32MultiArray()
        msg.coordinate_diff.data = [0.0, 0.0]
        
        self.target_pub.publish(msg)
        print(f"Turn command: {msg.theta}")
        print("Waiting for turn completion...")
        time.sleep(1)
        
        while self.robot_state != "reachGoal":
            print(f"Robot state: {self.robot_state}")
            time.sleep(0.5)
        print("Turn completed!")
    
    # ============================================================================
    # MOTION PLANNING FOR BLOCKED PATHS
    # ============================================================================
    
    def _find_object_in_memory(self, obj_name: str):
        """
        Find an object in memory.yaml and return its room and coordinates
        
        Args:
            obj_name: Name of the object to find
            
        Returns:
            tuple: (room_name, coordinates, node_id) or (None, None, None) if not found
        """
        if not self.motion_planner:
            return None, None, None
        
        # Search through all nodes in the motion planner
        for node_id, node in self.motion_planner.nodes.items():
            # Check if this is an object node and matches our target
            if node.node_type.value == "object" and node.features:
                for feature in node.features:
                    if 'object' in feature and obj_name.lower() in feature['object'].lower():
                        # Extract room name from node_id (format: room_object_idx)
                        room_name = node_id.split('_')[0]
                        coords = feature.get('Coordinate relative to the world frame', None)
                        self.get_logger().info(f"🔍 Found '{obj_name}' in room '{room_name}' at coordinates {coords}")
                        return room_name, coords, node_id
        
        return None, None, None
    
    def _navigate_to_room(self, room_name: str):
        """
        Navigate to a room using its stored coordinates
        
        Args:
            room_name: Name of the room to navigate to
            
        Returns:
            bool: True if navigation was successful, False otherwise
        """
        if not self.motion_planner:
            return False
        
        # Find room node in motion planner
        room_node = None
        for node_id, node in self.motion_planner.nodes.items():
            if node.node_type.value == "room" and node_id.lower() == room_name.lower():
                room_node = node
                break
        
        if not room_node:
            self.get_logger().warn(f"⚠️ Room '{room_name}' not found in memory")
            return False
        
        # Get room position (in world frame)
        wx, wy = room_node.position.x, room_node.position.y
        
        # Convert world coordinates to camera frame coordinates
        # For room navigation, we'll use a fixed distance approach
        current_pose = self.memory_builder.camera_pose
        if current_pose:
            cx, cy, cyaw = current_pose
            # Calculate relative position
            dx = wx - cx
            dy = wy - cy
            
            # Calculate distance
            distance = math.sqrt(dx**2 + dy**2)
            
            # Transform to camera frame
            # Rotate by -yaw to get camera frame coordinates
            cam_x = dx * math.cos(-cyaw) - dy * math.sin(-cyaw)
            cam_y = dx * math.sin(-cyaw) + dy * math.cos(-cyaw)
            
            self.get_logger().info(f"🎯 Navigating to room '{room_name}' at camera frame coords ({cam_x:.2f}, {cam_y:.2f})")
            
            # Create and publish navigation message
            msg = self._create_navigation_message([], [], distance, cam_x, cam_y)
            self.target_pub.publish(msg)
            
            print(f"📍 Sent goal for room '{room_name}' at coordinates ({cam_x:.2f}, {cam_y:.2f}), distance: {distance:.2f}m")
            return True
        else:
            self.get_logger().warn("⚠️ Current camera pose not available")
            return False
    
    def _navigate_with_motion_planner(self, obj_name: str, relation: str):
        """
        Use motion planner to find alternative path when object is not directly visible
        
        Args:
            obj_name: Name of the target object
            relation: Spatial relation (near, at, through)
            
        Returns:
            bool: True if navigation was successful, False otherwise
        """
        self.get_logger().info(f"🗺️ Object '{obj_name}' not visible. Using motion planner to find alternative route...")
        
        # Find object in memory
        room_name, coords, node_id = self._find_object_in_memory(obj_name)
        
        if not room_name or not coords:
            self.get_logger().warn(f"⚠️ Object '{obj_name}' not found in memory.yaml")
            return False
        
        # Determine current location (use last known room or estimate)
        current_location = self.memory_builder.last_room_type if self.memory_builder.last_room_type else "entrance hall"
        
        self.get_logger().info(f"📍 Current location: {current_location}")
        self.get_logger().info(f"🎯 Target: {obj_name} in room '{room_name}'")
        
        # Plan path using motion planner
        path = self.motion_planner.plan_path(current_location, room_name)
        
        if not path or len(path) == 0:
            self.get_logger().warn(f"⚠️ No path found from '{current_location}' to '{room_name}'")
            return False
        
        self.get_logger().info(f"🛤️ Planned path: {' -> '.join(path)}")
        
        # Navigate through the path (skip first node if it's current location)
        start_idx = 1 if len(path) > 1 and path[0].lower() == current_location.lower() else 0
        
        for i in range(start_idx, len(path)):
            waypoint = path[i]
            node = self.motion_planner.nodes.get(waypoint)
            
            if not node:
                continue
            
            # Check if this is a room node
            if node.node_type.value == "room":
                self.get_logger().info(f"🚶 Navigating to intermediate room: {waypoint}")
                if self._navigate_to_room(waypoint):
                    self._wait_for_goal_completion()
                    time.sleep(1)
        
        # Now try to detect the object directly since we're in the right room
        self.get_logger().info(f"✅ Arrived at room '{room_name}'. Attempting to detect '{obj_name}'...")
        time.sleep(2)  # Wait for camera to stabilize
        
        # Try to detect the object now
        img_detect, rect, center = self.VL.infer(self.rgb_image, obj_name + ".")
        
        if rect is not None:
            self.get_logger().info(f"✅ Object '{obj_name}' now visible! Navigating to it...")
            dis, wx, wy = self.memory_builder.pix2camera_frame(center, self.depth_image, self.get_logger())
            
            if dis and dis > 0:
                # Adjust distance based on relation
                dis = self._adjust_distance_by_relation(dis, relation)
                if dis < 0.0:
                    dis = 0.0
                
                # Create and publish navigation message
                msg = self._create_navigation_message(rect, center, dis, wx, wy)
                self.target_pub.publish(msg)
                print(f"Goal sent for object '{obj_name}' at coordinates ({wx:.2f}, {wy:.2f}), distance: {dis:.2f}m")
                return True
        else:
            # If still not visible, navigate to stored coordinates
            self.get_logger().info(f"🎯 Object still not visible. Navigating to stored coordinates...")
            if len(coords) >= 2:
                # Navigate to stored world coordinates
                current_pose = self.memory_builder.camera_pose
                if current_pose:
                    cx, cy, cyaw = current_pose
                    dx = coords[0] - cx
                    dy = coords[1] - cy
                    distance = math.sqrt(dx**2 + dy**2)
                    
                    # Adjust distance based on relation
                    distance = self._adjust_distance_by_relation(distance, relation)
                    if distance < 0.0:
                        distance = 0.0
                    
                    # Transform to camera frame
                    cam_x = dx * math.cos(-cyaw) - dy * math.sin(-cyaw)
                    cam_y = dx * math.sin(-cyaw) + dy * math.cos(-cyaw)
                    
                    msg = self._create_navigation_message([], [], distance, cam_x, cam_y)
                    self.target_pub.publish(msg)
                    print(f"Goal sent for object '{obj_name}' using stored coordinates")
                    return True
        
        return False
    
    # ============================================================================
    # COMMAND PROCESSING
    # ============================================================================
    
    def process_command_queue(self):
        """Timer callback to process commands from the queue"""
        if not self.processing_command and len(self.command_queue) > 0:
            command = self.command_queue.pop(0)
            # Run in a separate thread to avoid blocking the timer
            threading.Thread(target=self.process_navigation_command, args=(command,), daemon=True).start()
    
    def process_navigation_command(self, question: str):
        """
        Process a navigation command from the GUI
        
        Args:
            question: Navigation command from user
        """
        if self.processing_command:
            self._publish_status("⚠️ Already processing a command, please wait...")
            return
        
        self.processing_command = True
        
        try:
            # Wait for camera data
            if self.rgb_image is None:
                self._publish_status("⚠️ Waiting for camera data...")
                retry_count = 0
                while self.rgb_image is None and retry_count < 10:
                    time.sleep(0.5)
                    retry_count += 1
                
                if self.rgb_image is None:
                    self._publish_status("❌ Camera data not available")
                    self.processing_command = False
                    return
            
            self._publish_status(f"🤔 Processing command: {question}")
            
            # Process command with GPT
            answer = ans2json.ans2json(lm.ask_gpt_ll(question))
            self._publish_status(f"✅ Command understood")
            self.get_logger().info(f"GPT-4o answer: {answer}")
            
            self.turn_list = answer["turn"]
            self.obj_list = answer["objects"]
            self.relation_list = answer["relative"]
            
            # Execute navigation sequence
            idx = 0
            goal_sent = False
            
            while idx < len(self.turn_list):
                obj = self.obj_list[idx]
                act = self.turn_list[idx]
                relation = self.relation_list[idx]
                
                if obj and obj.lower() != "null":
                    if not goal_sent:
                        if self._process_object_navigation(obj, relation, idx):
                            goal_sent = True
                        else:
                            self._publish_status(f"⚠️ Skipping object '{obj}'")
                            idx += 1
                            continue
                    
                    if self.robot_state == "navigating" or goal_sent:
                        self._wait_for_goal_completion()
                        idx += 1
                        goal_sent = False
                        time.sleep(1)
                        
                elif act and act.lower() != "null":
                    self._process_turn_action(act)
                    idx += 1
                    
                else:
                    self.get_logger().warn("Both object and action are null, skipping...")
                    idx += 1
            
            self._publish_status(f"✅ Navigation sequence completed! ({idx} steps)")
            self.get_logger().info("Navigation sequence completed successfully!")
            self.robot_state = "reachGoal"
            
        except Exception as e:
            self._publish_status(f"❌ Error: {str(e)}")
            self.get_logger().error(f"Error processing command: {e}")
        finally:
            self.processing_command = False


def main(args=None):
    """
    Main entry point for the service node (GUI mode)
    
    This version is designed to work with the robot_chat_gui.py interface.
    Commands are received via the /service_question topic and status updates
    are sent to /robot_status topic.
    """
    rclpy.init(args=args)
    node = ServiceNode()
    
    # Publish initial status
    time.sleep(1)  # Wait for publishers to be ready
    node._publish_status("🤖 Robot service ready! Send commands via GUI.")
    node.get_logger().info("=" * 60)
    node.get_logger().info("🎮 GUI MODE: Waiting for commands from robot_chat_gui.py")
    node.get_logger().info("📡 Subscribed to: /service_question")
    node.get_logger().info("📤 Publishing to: /robot_status")
    node.get_logger().info("=" * 60)
    
    try:
        # Spin the node to process callbacks
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        node.get_logger().info("⛔ Shutting down due to KeyboardInterrupt")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()