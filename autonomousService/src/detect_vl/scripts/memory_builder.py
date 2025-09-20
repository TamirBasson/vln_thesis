import yaml
import os
import math
import numpy as np
import time
import cv2
from typing import List, Dict, Any, Tuple

class MemoryBuilder:
    def __init__(self, memory_file: str = "src/memory.yaml", use_simulation: bool = True):
        self.memory_file = memory_file
        self.use_simulation = use_simulation  # True for simulation, False for real hardware
        self.memory_data = {
            "nodes": [],
            "edges": []
        }
        self._load_memory()
        self.camera_pose = None  # [x, y, yaw]
        
        # Room tracking
        self.last_room_type = None

        if not os.path.exists(self.memory_file):
            # create a new file
            with open(self.memory_file, 'w') as f:
                pass

    def _load_memory(self):
        """Load existing memory if it exists"""
        if os.path.exists(self.memory_file):
            try:
                with open(self.memory_file, 'r') as f:
                    self.memory_data = yaml.safe_load(f) or {"nodes": [], "edges": []}
            except yaml.YAMLError as e:
                print(f"Error reading memory file: {e}")
                self.memory_data = {"nodes": [], "edges": []}

    def _calculate_distance(self, pose1: List[float], pose2: List[float]) -> float:
        """Calculate Euclidean distance between two poses"""
        return math.sqrt((pose1[0] - pose2[0])**2 + (pose1[1] - pose2[1])**2)

    def _transform_to_map_frame(self, camera_coords: List[float]) -> List[float]:
        """Transform coordinates from camera frame to map frame using the camera pose"""
        print(f"DEBUG: _transform_to_map_frame called with camera_coords: {camera_coords}")
        print(f"DEBUG: Current camera_pose: {self.camera_pose}")
        
        if self.camera_pose is None:
            print("Warning: No camera pose available, returning camera coordinates as-is")
            return camera_coords
        
        cam_x, cam_y, cam_yaw = self.camera_pose
        diff_x, diff_y = camera_coords[0], camera_coords[1]
        
        # Apply the transformation equations
        # camera_x (forward) becomes map_x, camera_y (left) becomes map_y
        target_x = cam_x + diff_x * math.cos(cam_yaw) - diff_y * math.sin(cam_yaw)
        target_y = cam_y + diff_x * math.sin(cam_yaw) + diff_y * math.cos(cam_yaw)
        
        print(f"Camera coords: ({diff_x:.3f}, {diff_y:.3f}) -> Map coords: ({target_x:.3f}, {target_y:.3f})")
        return [target_x, target_y]  

    def _is_feature_unique(self, new_feature: Dict[str, Any], existing_features: List[Dict[str, Any]]) -> bool:
        """Check if a feature is unique based on its type and name"""
        for existing_feature in existing_features:
            if (existing_feature.get('object') == new_feature.get('object')):
                return False
        return True

    def _filter_features_by_distance(self, features: List[Dict[str, Any]], room_pose: List[float]) -> List[Dict[str, Any]]:
        """Filter features that are within 4 meters of the room pose"""
        filtered_features = []
        for feature in features:
            if 'Coordinate relative to the world frame' in feature:
                feature_coords = feature['Coordinate relative to the world frame']
                distance = self._calculate_distance(room_pose[:2], feature_coords[:2])  # Only use x,y coordinates
                if distance <= 4.0:  # 4 meters threshold
                    filtered_features.append(feature)
        return filtered_features

    def _update_edges(self):
        """Update edges between directly accessible rooms"""
        self.memory_data["edges"] = []
        nodes = self.memory_data["nodes"]
        
        if len(nodes) < 2:
            # Need at least 2 rooms to create edges
            return
        
        # Create edges between rooms based on proximity and accessibility
        for i, node1 in enumerate(nodes):
            for node2 in nodes[i+1:]:
                distance = self._calculate_distance(node1["pose"], node2["pose"])
                
                # Debug: print distance between rooms
                print(f"🔍 Distance between {node1['name']} and {node2['name']}: {distance:.2f}m")
                
                # Create edge if rooms are within reasonable indoor distance
                # and check if they should be directly connected
                if distance < 10.0:
                    directly_connected = self._are_rooms_directly_connected(node1, node2, distance)
                    print(f"  → Directly connected: {directly_connected}")
                    
                    if directly_connected:
                        # Create bidirectional edges
                        edge1 = {
                        "from": node1["name"],
                        "to": node2["name"],
                        "cost": round(distance, 2)
                    }
                        edge2 = {
                            "from": node2["name"],
                            "to": node1["name"],
                            "cost": round(distance, 2)
                        }
                        self.memory_data["edges"].append(edge1)
                        self.memory_data["edges"].append(edge2)
                        
                        # Debug: print when edge is created
                        print(f"✅ Created edge: {node1['name']} ↔ {node2['name']} (distance: {distance:.2f}m)")
                else:
                    print(f"  → Too far for direct connection (>{10.0}m)")
        
        # Print total edges created
        print(f"📊 Total edges created: {len(self.memory_data['edges'])}")
    
    def _are_rooms_directly_connected(self, room1, room2, distance):
        """Check if two rooms are directly accessible without going through other rooms"""
        # Simple heuristic: rooms are directly connected if:
        # 1. They are close enough (already checked)
        # 2. No other room is positioned between them
        
        nodes = self.memory_data["nodes"]
        room1_pose = room1["pose"]
        room2_pose = room2["pose"]
        
        print(f"    🔍 Checking direct connection between {room1['name']} and {room2['name']}")
        
        # Check if any other room is positioned between these two rooms
        for node in nodes:
            if node["name"] == room1["name"] or node["name"] == room2["name"]:
                continue
                
            node_pose = node["pose"]
            
            # Calculate if this room is roughly between room1 and room2
            dist_to_room1 = self._calculate_distance(node_pose, room1_pose)
            dist_to_room2 = self._calculate_distance(node_pose, room2_pose)
            
            print(f"      • Checking {node['name']}: dist to {room1['name']}={dist_to_room1:.2f}m, dist to {room2['name']}={dist_to_room2:.2f}m")
            
            # If this room is very close to the line between room1 and room2
            # and the total distance through this room is not much longer,
            # then room1 and room2 might not be directly connected
            if (dist_to_room1 < distance * 0.8 and dist_to_room2 < distance * 0.8 and
                dist_to_room1 + dist_to_room2 < distance * 1.5):
                # There's likely a room between them
                print(f"      ❌ {node['name']} is blocking the direct path!")
                return False
        
        # Default: assume rooms are directly connected if no blocking room found
        print(f"    ✅ No blocking rooms found - direct connection possible")
        return True
    
    def force_edge_update(self, logger=None):
        """Force an immediate update of all edges and save to file"""
        if logger:
            logger.info("🔄 Forcing edge update...")
        else:
            print("🔄 Forcing edge update...")
            
        self._update_edges()
        
        # Save updated edges to file
        try:
            with open(self.memory_file, 'w') as f:
                safe_data = convert_numpy(self.memory_data)
                yaml.dump(safe_data, f, default_flow_style=False, sort_keys=False)
            
            message = f"✅ Edge update complete! Created {len(self.memory_data['edges'])} edges between {len(self.memory_data['nodes'])} rooms"
            if logger:
                logger.info(message)
            else:
                print(message)
                
        except Exception as e:
            error_msg = f"Error saving edges to memory file: {e}"
            if logger:
                logger.error(error_msg)
            else:
                print(error_msg)

    def update_camera_pose(self, wx: float, wy: float, yaw: float, logger=None):
        """Update the camera pose in the map frame"""
        self.camera_pose = [wx, wy, yaw]
        if logger:
            logger.info(f"Updated camera pose: wx={wx:.3f}, wy={wy:.3f}, yaw={yaw:.3f}")

    def save_to_memory(self, room_type: str, features_with_coords: List[Dict[str, Any]], room_pose: List[float] = [0.0, 0.0, 0.0]):
        """Save or update room features in memory"""
        # Room pose should already be in map frame, don't transform it
        map_room_pose = room_pose
        
        # Transform feature coordinates from camera frame to map frame
        transformed_features = []
        for feature in features_with_coords:
            print(f"DEBUG: Processing feature: {feature}")
            if 'Coordinate relative to the camera frame' in feature:
                print(f"DEBUG: Found camera coordinates in feature: {feature['object']}")
                camera_coords = feature['Coordinate relative to the camera frame']
                map_coords = self._transform_to_map_frame(camera_coords)
                # Create a copy of the feature with map coordinates
                transformed_feature = feature.copy()
                transformed_feature['Coordinate relative to the world frame'] = map_coords
                transformed_features.append(transformed_feature)
            else:
                print(f"DEBUG: No camera coordinates found in feature: {feature.get('object', 'unknown')}")
                # If no camera coordinates, keep the feature as-is
                transformed_features.append(feature)

        # Filter features by distance
        filtered_features = self._filter_features_by_distance(transformed_features, map_room_pose)
        
        # Check if room already exists
        room_exists = False
        for node in self.memory_data["nodes"]:
            if node["name"] == room_type:
                # Update existing room with unique features
                existing_features = node["features"]
                for feature in filtered_features:
                    if self._is_feature_unique(feature, existing_features):
                        existing_features.append(feature)
                node["features"] = existing_features
                node["pose"] = map_room_pose
                room_exists = True
                break

        if not room_exists:
            # Create new room node with filtered features
            new_room = {
                "name": room_type,
                "pose": map_room_pose,
                "features": filtered_features
            }
            self.memory_data["nodes"].append(new_room)

        # Update edges after room update
        self._update_edges()

        # Save to file
        try:
            with open(self.memory_file, 'w') as f:
                safe_data = convert_numpy(self.memory_data)
                yaml.safe_dump(safe_data, f, default_flow_style=False)
            print(f"Updated memory file with {room_type} features and edges")
        except Exception as e:
            print(f"Error saving to memory file: {e}")

    def get_room_features(self, room_type: str) -> List[Dict[str, Any]]:
        """Get features for a specific room"""
        for node in self.memory_data["nodes"]:
            if node["name"] == room_type:
                return node["features"]
        return []

    def get_all_rooms(self) -> List[str]:
        """Get list of all room names"""
        return [node["name"] for node in self.memory_data["nodes"]]

    def get_connected_rooms(self, room_type: str) -> List[Tuple[str, float]]:
        """Get list of rooms connected to the specified room and their costs"""
        connected = []
        for edge in self.memory_data["edges"]:
            if edge["from"] == room_type:
                connected.append((edge["to"], edge["cost"]))
            elif edge["to"] == room_type:
                connected.append((edge["from"], edge["cost"]))
        return connected

    def update_room_pose(self, room_type: str, new_pose: List[float]):
        """Update the pose of a specific room (pose should already be in map frame)"""
        for node in self.memory_data["nodes"]:
            if node["name"] == room_type:
                node["pose"] = new_pose
                self._update_edges()
                self.save_to_memory(room_type, node["features"], new_pose)
                break

    # ===== MOVED FUNCTIONS FROM start_service.py =====

    def can_classify_new_room(self, proposed_room_type: str, logger=None) -> bool:
        """Check if we can classify a new room type"""
        # If this is the first room classification, allow it
        if self.last_room_type is None:
            self.last_room_type = proposed_room_type
            return True
            
        # If the proposed room type is the same as the last one, allow it
        if proposed_room_type == self.last_room_type:
            return True
            
        # Allow new room classification without door transition dependency
        if logger:
            logger.info(f"✅ Classifying new room: {proposed_room_type}")
        self.last_room_type = proposed_room_type
        return True


    def get_room_transition_status(self) -> Dict[str, Any]:
        """Get current room status for debugging"""
        return {
            "current_room": self.last_room_type
        }
    
    def print_memory_status(self, logger=None):
        """Print current memory status for debugging"""
        nodes = self.memory_data["nodes"]
        edges = self.memory_data["edges"]
        
        message = f"""
🗺️ MEMORY MAP STATUS:
📍 Rooms ({len(nodes)} total):"""
        
        for node in nodes:
            pose = node.get("pose", [0, 0, 0])
            features_count = len(node.get("features", []))
            message += f"\n  • {node['name']}: pose=({pose[0]:.2f}, {pose[1]:.2f}, {pose[2]:.2f}), features={features_count}"
        
        message += f"\n🔗 Edges ({len(edges)} total):"
        if edges:
            for edge in edges:
                message += f"\n  • {edge['from']} → {edge['to']} (cost: {edge['cost']}m)"
        else:
            message += "\n  • No edges found!"
        
        if logger:
            logger.info(message)
        else:
            print(message)

    # def detect_doors(self, rgb_image, depth_image, vlm_model, logger=None):
    #     """Detect doors in the current image and track door transitions"""
    #     if rgb_image is None:
    #         return
            
    #     current_time = time.time()
        
    #     # Detect doors using VLM
    #     img_detect, rect, center = vlm_model.infer(rgb_image, "door.")
        
    #     if rect is not None and center is not None:
    #         # Calculate distance to door
    #         result = self.pix2camera_frame(center, depth_image, logger)
    #         # Handle return format based on simulation vs hardware
            
    #         if dis is not None and dis > 0:
    #             if logger:
    #                 logger.info(f"🚪 Door detected at distance {dis:.2f}m")
                
    #             # Check if we're close enough to consider passing through
    #             if dis < self.door_detection_distance_threshold:
    #                 if not self.door_detected:
    #                     self.door_detected = True
    #                     self.last_door_detection_time = current_time
    #                     if logger:
    #                         logger.info("🚪 Door proximity detected - preparing for room transition")
    #             else:
    #                 # If we were previously near a door and now we're far, consider it a transition
    #                 if self.door_detected and (current_time - self.last_door_detection_time) > self.door_detection_threshold:
    #                     self.room_transition_detected = True
    #                     self.door_detected = False
    #                     if logger:
    #                         logger.info("✅ Room transition detected - door passed through")
    #     else:
    #         # No door detected, reset door detection if enough time has passed
    #         if self.door_detected and (current_time - self.last_door_detection_time) > self.door_detection_threshold:
    #             self.door_detected = False

    def pix2camera_frame(self, pix_xy, depth_image, logger=None, use_simulation=None):
        """Convert pixel coordinates to camera frame coordinates for navigation"""
        if depth_image is None:
            if logger:
                logger.warn("⚠️ Depth image not yet received.")
            return None, None, None

        depth_raw = depth_image[pix_xy[1], pix_xy[0]]  # Access depth image using (row, col)
        
        # ============================================================================
        # DEPTH VALUE CONVERSION - SIMULATION vs REAL HARDWARE
        # ============================================================================
        # Use instance configuration or explicit parameter
        if use_simulation is None:
            use_simulation = self.use_simulation  # Use instance setting
        
        # Debug: Log depth image properties
        if logger:
            logger.info(f"🔍 DEBUG - Depth image dtype: {depth_image.dtype}, raw value: {depth_raw}")
        
        if use_simulation:
            # SIMULATION: Check if Gazebo provides depth in mm or meters
            # Large values (>100) suggest millimeters, small values suggest meters
            if depth_raw > 100:  # Likely millimeters
                center_depth_m = float(depth_raw) / 1000.0  # Convert mm to meters
                if logger:
                    logger.info(f"🎮 SIMULATION depth at ({pix_xy[0]},{pix_xy[1]}): {center_depth_m:.3f} m (converted from mm: {depth_raw})")
            else:  # Likely already in meters
                center_depth_m = float(depth_raw)  # Already in meters
                if logger:
                    logger.info(f"🎮 SIMULATION depth at ({pix_xy[0]},{pix_xy[1]}): {center_depth_m:.3f} m (no conversion)")
        else:
            # REAL HARDWARE: RealSense provides depth in millimeters (uint16) 
            center_depth_m = float(depth_raw) / 1000.0  # Convert mm to meters
            if logger:
                logger.info(f"🔧 HARDWARE depth at ({pix_xy[0]},{pix_xy[1]}): {center_depth_m:.3f} m (converted from mm)")
        
        # ============================================================================
        # CAMERA INTRINSIC PARAMETERS - ALREADY DETERMINED ABOVE
        # ============================================================================
        
        if use_simulation:
            # SIMULATION CAMERA PARAMETERS (from Gazebo URDF)
            # horizontal_fov = 1.089 radians, width = 640, height = 480
            cx, cy = 320.0, 240.0  # Image center
            fx = fy = 521.74  # Calculated from FOV: fx = (640/2) / tan(1.089/2)
            if logger:
                logger.info("🎮 Using SIMULATION camera parameters")
        else:
            # REAL HARDWARE CAMERA PARAMETERS (RealSense)
            cx, cy = 211.73, 123.504  # Principal point  
            fx, fy = 307.767, 307.816  # Focal lengths
            if logger:
                logger.info("🔧 Using REAL HARDWARE camera parameters")
        
        pix_x, pix_y = pix_xy 
        
        if use_simulation:
            # SIMULATION: Full 3D coordinate calculation
            # X = d, Y = -(u-cx)*d/fx, Z = -(v-cy)*d/fy
            wx = center_depth_m                              # Forward distance (X-axis)
            wy = -(pix_x - cx) * center_depth_m / fx        # Lateral offset (Y-axis)  
            wz = -(pix_y - cy) * center_depth_m / fy        # Vertical offset (Z-axis)
            
            if logger:
                logger.info(f"🎮 SIMULATION coordinates: wx={wx:.3f}, wy={wy:.3f}, wz={wz:.3f}")
            return center_depth_m, wx, wy, wz
            
        else:
            # REAL HARDWARE: Keep the working 2D calculation
            wx = center_depth_m                              # Forward distance (X-axis)
            wy = -(pix_x - cx) * center_depth_m / fx        # Lateral offset (Y-axis)
            
            if logger:
                logger.info(f"🔧 HARDWARE coordinates: wx={wx:.3f}, wy={wy:.3f}")
            return center_depth_m, wx, wy


def convert_numpy(obj):
    """from numpy to Python type"""
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    elif isinstance(obj, np.generic):
        return obj.item()
    elif isinstance(obj, dict):
        return {k: convert_numpy(v) for k, v in obj.items()}
    elif isinstance(obj, list):
        return [convert_numpy(i) for i in obj]
    else:
        return obj