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
import math

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
        
        # Navigation tracking for obstacle detection
        self.current_path = []
        self.current_path_index = 0
        self.navigation_retries = {}  # Track retries per edge
        self.max_retries_per_edge = 1  # Only try each edge once before removing
        
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
        self.create_subscription(
            String,
            '/initial_location',
            self.initial_location_callback, 10
        )
        
        # Publishers
        self.target_pub = self.create_publisher(RectDepth, 'task/rect_depth', 10)
        self.status_pub = self.create_publisher(String, '/robot_status', 10)

    def _setup_timers(self):
        """Initialize ROS2 timers"""
        self.update_memory_map = self.create_timer(1.0, self.update_map)
        self.camera2map_monitor_timer = self.create_timer(2.0, self.monitor_camera2map_topic)
        self.command_processor_timer = self.create_timer(0.5, self.process_command_queue)
        # Dedicated timer for GUI updates (30 Hz) to keep window responsive
        self.display_timer = self.create_timer(0.033, self.update_display)

    def update_display(self):
        """Handle GUI updates independent of camera rate"""
        if self.rgb_image is not None:
            try:
                # Show image
                cv2.imshow("Robot Camera View", self.rgb_image)
                cv2.waitKey(1)
            except Exception as e:
                self.get_logger().warn(f"Display error: {e}")

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
    
    def initial_location_callback(self, msg):
        """Handle initial location updates from GUI - sets the robot's current room in the graph"""
        location = msg.data.strip()
        if location:
            self.memory_builder.last_room_type = location
            self.get_logger().info(f"📍 Robot initial location set to: {location}")
            self._publish_status(f"📍 Robot is now in: {location}")
    
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


        if rect is None:
            # STRICT RULE: If object not visible, assume route is blocked
            self.get_logger().warn(f"🚫 Object '{obj}' not detected in camera view")
            self._publish_status(f"❌ Vision failed: Cannot see '{obj}'")
            self._publish_status(f"🤔 Analysis: Direct route assumed BLOCKED by obstacle")
            self._publish_status(f"🧠 Strategy: Switching to memory-based navigation...")
            
            # Try using motion planner to find alternative route through memory.yaml
            if self.motion_planner:
                self.get_logger().info(f"🗺️ Planning alternative route to '{obj}' using memory.yaml")
                self._publish_status(f"🗺️ Querying memory map for alternative path to '{obj}'...")
                
                # Force motion planner to find path
                success = self._navigate_with_motion_planner(obj, relation)
                
                if success:
                    # Update current room after successful motion planner navigation
                    room_name, _, _ = self._find_object_in_memory(obj)
                    if room_name:
                        self.memory_builder.last_room_type = room_name
                        self.get_logger().info(f"📍 Location updated to: {room_name}")
                    
                    self._publish_status(f"✅ Success: Found and initiated alternative route via memory!")
                    return True
                else:
                    self._publish_status(f"❌ Failure: Memory search yielded no valid path to '{obj}'")
                    return False
            else:
                self._publish_status(f"❌ Critical: Motion planner unavailable - cannot reroute")
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
        
        # Update current location based on object detection
        room_name, _, _ = self._find_object_in_memory(obj)
        if room_name:
            self.memory_builder.last_room_type = room_name
            self.get_logger().info(f"📍 Detected '{obj}' in room: {room_name}")
        
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
        timeout = 15.0  # 15 second timeout 
        start_time = time.time()
        
        # Reset robot state to avoid using stale state from previous goal
        initial_state = self.robot_state
        self.get_logger().info(f"Waiting for goal completion, initial state: {initial_state}")
        
        while self.robot_state != "reachGoal" and self.robot_state != "failed":
            elapsed = time.time() - start_time
            
            # Check for timeout
            if elapsed > timeout:
                print(f"⚠️ Warning: Goal timeout after {timeout}s. Current state: {self.robot_state}")
                self.get_logger().warn(f"Goal timeout. State remained: {self.robot_state}")
                self._publish_status(f"⚠️ Navigation timeout after {timeout}s")
                # Don't automatically mark as failed - the robot might still be navigating
                return False
            
            # Print status less frequently (every 2 seconds instead of 0.5)
            if int(elapsed * 2) % 4 == 0:  # Print every 2 seconds
                print(f"Robot state: {self.robot_state} (elapsed: {elapsed:.1f}s)")
            
            time.sleep(0.5)
        
        if self.robot_state == "reachGoal":
            print("✅ Robot reached goal!")
            self._publish_status("✅ Reached goal!")
            # Don't reset state here - let next goal set it
            return True
        elif self.robot_state == "failed":
            print("❌ Navigation failed!")
            self.get_logger().error(f"Navigation failed with state: {self.robot_state}")
            self._publish_status("❌ Navigation failed - obstacle detected")
            return False
        
        # This should not be reached, but return True as fallback
        self.get_logger().warn("Goal completion finished with unexpected state")
        return True

    def _process_turn_action(self, act):
        """Process turn action"""
        msg = self._create_navigation_message([], [], 0.0, 0.0, 0.0, float(act))
        msg.coordinate_diff = Float32MultiArray()
        msg.coordinate_diff.data = [0.0, 0.0]
        
        # Reset robot state before publishing turn command
        self.robot_state = "navigating"
        self.target_pub.publish(msg)
        print(f"Turn command: {msg.theta}")
        print("Waiting for turn completion...")
        time.sleep(0.5)  # Brief delay for message propagation
        
        timeout = 7.0  # 7 second timeout for turns
        start_time = time.time()
        
        while self.robot_state != "reachGoal":
            elapsed = time.time() - start_time
            if elapsed > timeout:
                print(f"⚠️ Warning: Turn timeout after {timeout}s")
                self.get_logger().warn(f"Turn action timeout after {timeout}s")
                break
            print(f"Robot state: {self.robot_state}")
            time.sleep(0.5)
        print("Turn completed!")
    
    # ============================================================================
    # MOTION PLANNING FOR BLOCKED PATHS
    # ============================================================================
    
    def _remove_blocked_path_from_memory(self, from_room: str, to_room: str):
        """
        Remove a blocked path between two rooms from memory.yaml
        
        Args:
            from_room: Source room name
            to_room: Destination room name
        """
        if not self.motion_planner:
            self.get_logger().warn("Motion planner not available, cannot remove edge")
            return
        
        self.get_logger().info(f"🚫 Removing blocked path: {from_room} ↔ {to_room}")
        self._publish_status(f"🔄 Updating map - removing blocked path...")
        
        # Use motion planner to remove the edge
        removed = self.motion_planner.remove_edge(from_room, to_room)
        if removed:
            # Also remove from YAML file
            self.motion_planner.remove_edge_from_yaml(from_room, to_room)
            self._publish_status(f"✅ Map updated - path removed from {from_room} to {to_room}")
        else:
            self._publish_status(f"⚠️ Could not remove path (may not exist in map)")
    
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
        
        # Clean up object name (remove articles like "the", "a", "an")
        clean_obj_name = obj_name.lower().strip()
        for article in ["the ", "a ", "an "]:
            if clean_obj_name.startswith(article):
                clean_obj_name = clean_obj_name[len(article):]
                break
        
        self.get_logger().info(f"🔍 Searching for '{clean_obj_name}' (original: '{obj_name}')")
        
        # Search through all nodes in the motion planner
        for node_id, node in self.motion_planner.nodes.items():
            # Check if this is an object node and matches our target
            if node.node_type.value == "object" and node.features:
                for feature in node.features:
                    if 'object' in feature:
                        feature_obj = feature['object'].lower().strip()
                        # Check if object names match (bidirectional substring match)
                        if clean_obj_name in feature_obj or feature_obj in clean_obj_name:
                            # Extract room name from node_id
                            # Handle formats: "room_object_idx" or "room name_object_idx"
                            parts = node_id.rsplit('_', 2)  # Split from right
                            if len(parts) >= 3:
                                room_name = '_'.join(parts[:-2])
                            elif len(parts) == 2:
                                room_name = parts[0]
                            else:
                                room_name = node_id
                            
                            coords = feature.get('Coordinate relative to the world frame', None)
                            self.get_logger().info(f"✅ Found '{clean_obj_name}' in room '{room_name}' at {coords}")
                            return room_name, coords, node_id
        
        self.get_logger().warn(f"⚠️ Object '{obj_name}' not found in memory.yaml")
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
    
    def _navigate_with_motion_planner(self, obj_name: str, relation: str, retry_count: int = 0, last_failed_from: str = None):
        """
        Use motion planner to find alternative path when object is not directly visible
        
        Args:
            obj_name: Name of the target object
            relation: Spatial relation (near, at, through)
            retry_count: Number of replanning attempts (to prevent infinite loops)
            last_failed_from: The room where the last navigation attempt started from
            
        Returns:
            bool: True ONLY if a navigation goal was published for the object, False otherwise
        """
        if retry_count > 3:
            self.get_logger().error(f"❌ Max replanning attempts ({retry_count}) reached. Cannot reach '{obj_name}'")
            self._publish_status(f"❌ Cannot find any path to '{obj_name}' - all routes blocked")
            return False
        
        self.get_logger().info(f"🗺️ Motion planner activated for '{obj_name}'")
        self._publish_status(f"🗺️ Using memory.yaml to navigate to '{obj_name}'...")
        
        # SAFETY: Rebuild graph to ensure edges are up to date
        self.motion_planner.build_graph()
        
        if retry_count > 0:
            self.get_logger().info(f"🔄 Replanning attempt #{retry_count}")
            self._publish_status(f"🔄 Attempt #{retry_count} to find alternative route...")
        
        # Track whether we actually send a navigation goal
        goal_published = False
        
        # Find object in memory
        room_name, coords, node_id = self._find_object_in_memory(obj_name)
        
        if not room_name or not coords:
            self.get_logger().error(f"❌ Object '{obj_name}' NOT FOUND in memory.yaml")
            self._publish_status(f"❌ '{obj_name}' not in memory - cannot plan route")
            return False
        
        self.get_logger().info(f"✅ Found '{obj_name}' in room '{room_name}' at {coords}")
        self._publish_status(f"📍 Target identified: '{obj_name}' is in '{room_name}'")
        
        # Determine current location
        if self.memory_builder.last_room_type:
            current_location = self.memory_builder.last_room_type
        else:
            # Try to infer from camera pose or use default
            current_location = "home gym"
            self._publish_status(f"⚠️ Unknown start location, assuming '{current_location}'")
        
        self.get_logger().info(f"📍 Current location: {current_location}")
        
        # If this is a retry and we know where we failed from, remove that edge
        if retry_count > 0 and last_failed_from and room_name:
            self.get_logger().info(f"🚫 Previous attempt failed from '{last_failed_from}' to '{room_name}'")
            self._remove_blocked_path_from_memory(last_failed_from, room_name)
            current_location = last_failed_from  # Stay at failed location
        
        self.get_logger().info(f"📍 Current location: {current_location}")
        self.get_logger().info(f"🎯 Target: {obj_name} in room '{room_name}'")
        
        # Plan path using motion planner
        self._publish_status(f"🧠 Calculating path from '{current_location}' to '{room_name}'...")
        path = self.motion_planner.plan_path(current_location, room_name)
        
        if not path or len(path) == 0:
            self.get_logger().warn(f"⚠️ No path found from '{current_location}' to '{room_name}'")
            self._publish_status(f"❌ No valid path found between '{current_location}' and '{room_name}'")
            return False
        
        self.get_logger().info(f"🛤️ Planned path: {' -> '.join(path)}")
        self._publish_status(f"✅ Path found: {' → '.join(path)}")
        self.current_path = path
        
        # Navigate through the path (skip first node if it's current location)
        start_idx = 1 if len(path) > 1 and path[0].lower() == current_location.lower() else 0
        
        for i in range(start_idx, len(path)):
            waypoint = path[i]
            node = self.motion_planner.nodes.get(waypoint)
            
            if not node:
                continue
            
            # Check if this is a room node
            if node.node_type.value == "room":
                self.get_logger().info(f"🚶 Navigating to waypoint: {waypoint}")
                self._publish_status(f"🚶 Moving to waypoint {i}/{len(path)-1}: {waypoint}")
                self.current_path_index = i
                
                # Navigate to room using world coordinates from memory.yaml
                if self._navigate_to_room(waypoint):
                    # Wait for navigation to complete and check for failures
                    success = self._wait_for_goal_completion()
                    
                    if not success:
                        # Navigation failed - obstacle detected
                        self.get_logger().warn(f"🚫 Failed to reach '{waypoint}' from previous location")
                        
                        # Determine the blocked edge
                        if i > 0:
                            prev_waypoint = path[i - 1]
                            blocked_edge = (prev_waypoint, waypoint)
                            self.get_logger().info(f"🔍 Identified blocked edge: {prev_waypoint} ↔ {waypoint}")
                            
                            # Remove the blocked edge and replan
                            self._publish_status(f"🚧 Obstacle detected: Path blocked between {prev_waypoint} and {waypoint}")
                            self._publish_status(f"🚫 Marking edge as impassable in memory map")
                            self._publish_status(f"🔄 Recalculating optimal path to '{obj_name}'...")
                            
                            # Trigger replanning with edge removal
                            new_path = self.motion_planner.replan_after_edge_removal(
                                current_location, room_name, prev_waypoint, waypoint
                            )
                            
                            if new_path:
                                # Recursive call with new path
                                self.get_logger().info("✅ Alternative route found! Retrying...")
                                self._publish_status(f"✅ New strategy: Route via {' → '.join(new_path)}")
                                time.sleep(2)
                                # Reset robot state for retry
                                self.robot_state = "navigating"
                                return self._navigate_with_motion_planner(obj_name, relation, retry_count + 1, prev_waypoint)
                            else:
                                self.get_logger().error("❌ No alternative path exists")
                                self._publish_status("❌ Failure: All potential routes to target are blocked")
                                return False
                        else:
                            self.get_logger().error("❌ Failed at first waypoint")
                            self._publish_status("❌ Failed to leave starting location")
                            return False
                    else:
                        # Success - update current location
                        self.memory_builder.last_room_type = waypoint
                        self.get_logger().info(f"✅ Reached: {waypoint}")
                        self._publish_status(f"✅ Arrived at waypoint: {waypoint}")
                        time.sleep(1)
        
        # Now try to detect the object directly since we're in the right room
        self.get_logger().info(f"✅ Arrived at room '{room_name}'. Attempting to detect '{obj_name}'...")
        self._publish_status(f"✅ Arrived at '{room_name}' - Scanning for '{obj_name}'...")
        time.sleep(2)  # Wait for camera to stabilize
        
        # Try to detect the object now
        img_detect, rect, center = self.VL.infer(self.rgb_image, obj_name + ".")
        
        if rect is not None:
            self.get_logger().info(f"✅ Object '{obj_name}' now visible! Navigating to it...")
            self._publish_status(f"👀 Visual Confirmation: '{obj_name}' DETECTED!")
            self._publish_status(f"🎯 Final Approach: Navigating to object...")
            dis, wx, wy = self.memory_builder.pix2camera_frame(center, self.depth_image, self.get_logger())
            
            if dis and dis > 0:
                # Adjust distance based on relation
                orig_dis = dis
                dis = self._adjust_distance_by_relation(dis, relation)
                if dis < 0.0:
                    dis = 0.0
                
                # Recompute coordinates along the same camera ray
                if orig_dis and orig_dis > 0.0:
                    scale = dis / orig_dis
                    wx *= scale
                    wy *= scale
                
                # CRITICAL: Publish status BEFORE sending goal
                status_msg = f"🎯 Navigating to '{obj_name}' at ({wx:.2f}, {wy:.2f}), distance: {dis:.2f}m"
                self._publish_status(status_msg)
                self.get_logger().info(status_msg)
                
                # Create and publish navigation message
                msg = self._create_navigation_message(rect, center, dis, wx, wy)
                self.target_pub.publish(msg)
                goal_published = True
                
                # Prevent race condition: Reset state immediately
                self.robot_state = "navigating"
                time.sleep(0.5)  # Allow time for message propagation
                
                print(f"✅ GOAL SENT for '{obj_name}'")
                self.get_logger().info(f"✅ Navigation goal published for '{obj_name}'")
                return goal_published
            else:
                self.get_logger().error(f"❌ Cannot compute distance to '{obj_name}'")
                self._publish_status(f"❌ Failure: Could not calculate distance to object")
                return False
        else:
            # If still not visible, navigate to stored coordinates
            self.get_logger().warn(f"⚠️ Object '{obj_name}' still not visible after reaching room")
            self._publish_status(f"⚠️ Warning: '{obj_name}' not visible in room")
            self._publish_status(f"🗺️ Strategy: Falling back to stored coordinates in memory")
            
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
                    
                    # CRITICAL: Publish status BEFORE sending goal
                    status_msg = f"🎯 Navigating to '{obj_name}' at stored coordinates ({coords[0]:.2f}, {coords[1]:.2f})"
                    self._publish_status(status_msg)
                    self.get_logger().info(status_msg)
                    
                    msg = self._create_navigation_message([], [], distance, cam_x, cam_y)
                    self.target_pub.publish(msg)
                    goal_published = True
                    
                    # Prevent race condition: Reset state immediately
                    self.robot_state = "navigating"
                    time.sleep(0.5)  # Allow time for message propagation
                    
                    print(f"✅ GOAL SENT for '{obj_name}' using stored coordinates")
                    self.get_logger().info(f"✅ Navigation goal published for '{obj_name}' (stored coords)")
                    return goal_published
                else:
                    self.get_logger().error(f"❌ Camera pose not available for '{obj_name}'")
                    self._publish_status(f"❌ Failure: Cannot execute fallback (no camera pose)")
                    return False
            else:
                self.get_logger().error(f"❌ No coordinates available for '{obj_name}'")
                self._publish_status(f"❌ Failure: No fallback coordinates in memory")
                return False
        
        # Should never reach here - no goal was published
        if not goal_published:
            self.get_logger().error(f"❌ CRITICAL: Motion planner did NOT publish navigation goal for '{obj_name}'")
            self._publish_status(f"❌ Critical Error: Navigation sequence failed unexpectedly")
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
                            self.get_logger().info(f"✅ Navigation goal sent for '{obj}'")
                        else:
                            # Object not detected - strict blocked route assumption
                            self.get_logger().warn(f"🚫 '{obj}' not detected - switching to memory navigation")
                            self._publish_status(f"🚧 Route to '{obj}' blocked - finding alternative...")
                            
                            # Use motion planner to find alternative route
                            if self.motion_planner:
                                if self._navigate_with_motion_planner(obj, relation, retry_count=0):
                                    # Motion planner will handle the navigation and waiting
                                    self._wait_for_goal_completion()
                                    self._publish_status(f"✅ Navigation goal completed via alternative route")
                                else:
                                    self._publish_status(f"❌ Failed to find path to '{obj}'")
                            else:
                                self._publish_status(f"❌ Motion planner unavailable")
                            idx += 1
                            continue
                    
                    if self.robot_state == "navigating" or goal_sent:
                        self.get_logger().info(f"Waiting for navigation to '{obj}' to complete...")
                        success = self._wait_for_goal_completion()
                        
                        if not success:
                            # Vision-based navigation failed/timed out
                            # Only retry with motion planner if it was a real failure, not timeout
                            self.get_logger().warn(f"Navigation to '{obj}' did not complete successfully")
                            self._publish_status(f"⚠️ Navigation to '{obj}' incomplete, trying alternative...")
                            
                            # Use motion planner as fallback ONLY if state is "failed", not just timeout
                            if self.motion_planner and self.robot_state == "failed":
                                self.get_logger().info("Attempting alternative route via motion planner...")
                                target_room, _, _ = self._find_object_in_memory(obj)
                                current_room = self.memory_builder.last_room_type if self.memory_builder.last_room_type else "home gym"
                                
                                if target_room and current_room:
                                    self.get_logger().info(f"🔍 Trying path from '{current_room}' to '{target_room}'")
                                    
                                    if self._navigate_with_motion_planner(obj, relation, retry_count=1, last_failed_from=current_room):
                                        self._wait_for_goal_completion()
                                        self._publish_status(f"✅ Reached '{obj}' via alternative route!")
                                    else:
                                        self._publish_status(f"❌ Cannot find alternative route to '{obj}'")
                            elif not success and self.robot_state != "failed":
                                # Timeout but not failed - assume robot is still moving, proceed to next step
                                self._publish_status(f"⏱️ Continuing to next step (navigation may still be in progress)")
                        
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
            self._publish_status(f"❌ System Error: {str(e)}")
            self._publish_status(f"⚠️ Please check logs for details.")
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