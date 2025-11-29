# 🗺️ Waypoint Navigation with memory.yaml

## Your Scenario: "Go to the exercise ball, turn right and stop near the bed"

### Expected Behavior:

1. **Robot navigates to exercise ball** ✅
   - Detects exercise ball in view
   - Navigates to it
   - **Updates location: "home gym"** (because exercise ball is in home gym)

2. **Robot turns right** ✅

3. **Robot tries to navigate to bed** 
   - Tries to detect bed in view ❌ (not visible)
   - Tries direct navigation ❌ (blocked by obstacle)
   - **Triggers motion planner** 🗺️

4. **Motion Planner Activates:**
   ```
   📍 Current location: "home gym" (from step 1)
   🎯 Target: "bed" in "bedroom2" (from memory.yaml)
   
   🔍 Searching memory.yaml for path...
   ✅ Found path: home gym → living room → bedroom2
   ```

5. **Waypoint Navigation:**
   ```
   🚶 Waypoint 1: living room
      - Uses world coordinates from memory.yaml
      - Navigates using Nav2
      - ✅ Arrived at living room
   
   🚶 Waypoint 2: bedroom2
      - Uses world coordinates from memory.yaml
      - Navigates using Nav2
      - ✅ Arrived at bedroom2
   
   👀 Detects bed in camera view
   🎯 Navigates to bed
   ✅ Success!
   ```

## How It Works

### Step-by-Step Flow

#### 1. **Location Tracking**
Every time the robot reaches an object, it updates its current location:

```python
# After reaching exercise ball
self.memory_builder.last_room_type = "home gym"
self.get_logger().info("📍 Detected 'exercise ball' in room: home gym")
```

#### 2. **Object Not Visible → Motion Planner**
When bed is not visible, the system searches memory.yaml:

```python
# Search for bed in memory
room_name, coords, node_id = self._find_object_in_memory("bed")
# Result: room_name = "bedroom2", coords = [-4.207, 0.5576]
```

#### 3. **Path Planning**
Using D* Lite algorithm on topological graph:

```python
current_location = "home gym"  # From last_room_type
target_location = "bedroom2"   # From memory search

path = self.motion_planner.plan_path(current_location, target_location)
# Result: ["home gym", "living room", "bedroom2"]
```

#### 4. **Waypoint Navigation**
Navigate through each room using world coordinates:

```python
for waypoint in path:
    node = self.motion_planner.nodes.get(waypoint)
    wx, wy = node.position.x, node.position.y  # World coordinates
    
    # Navigate to waypoint
    self._navigate_to_room(waypoint)
    self._wait_for_goal_completion()
    
    # Update current location
    self.memory_builder.last_room_type = waypoint
```

#### 5. **Final Object Detection**
At destination room, detect and navigate to target:

```python
# Now in bedroom2
img_detect, rect, center = self.VL.infer(self.rgb_image, "bed")
if rect is not None:
    # Bed now visible!
    self._navigate_to_object(center, distance, relation)
```

## memory.yaml Structure

Your memory.yaml contains:

### Edges (Connections):
```yaml
edges:
- cost: 0.63
  from: living room
  to: home gym
- cost: 0.63
  from: living room
  to: fitness room
# ... more connections
```

### Nodes (Rooms & Objects):
```yaml
nodes:
- name: home gym
  pose: [0.9706, -0.3286, -2.8126]  # World coordinates
  features:
    - object: exercise ball
      Coordinate relative to the world frame: [-1.8420, -2.6024]

- name: bedroom2
  pose: [-4.5237, 0.8572, 0.2613]  # World coordinates
  features:
    - object: bed
      Coordinate relative to the world frame: [-4.207, 0.5576]

- name: living room
  pose: [1.3703, 0.1598, -2.6666]  # World coordinates
  features:
    - object: chair
    - object: table
```

## Obstacle Handling

### If Path is Blocked

If navigation from "home gym" to "living room" fails:

```
1. 🚧 Navigation failed - obstacle detected
2. 🔍 Identifies blocked edge: home gym ↔ living room
3. 🗑️ Removes from memory.yaml:
   DELETE: cost: 0.63, from: living room, to: home gym
4. 🔄 Replans path avoiding blocked edge
5. ✅ Finds alternative: home gym → fitness room → hallway → living room → bedroom2
6. 🚶 Navigates via new route
```

## GUI Messages You'll See

```
✅ Environment set to: Home
📝 Command received: go to the exercise ball, turn right and stop near the bed
🤔 Processing command...
✅ Command understood
🎯 Navigating to 'exercise ball' at (4.00, -1.05), distance: 4.00m
✅ Reached goal!
📍 Detected 'exercise ball' in room: home gym
🔄 Turn command: 1.57 rad
✅ Reached goal!
❌ Object 'bed' not visible
💡 Trying motion planner for alternative route...
🗺️ Route: home gym → living room → bedroom2
🚶 Going to: living room
✅ Reached: living room
🚶 Going to: bedroom2
✅ Reached: bedroom2
✅ Reached bedroom2, searching for bed...
👀 Found bed! Navigating...
🎯 Navigating to bed at 2.00m
✅ Reached goal!
✅ Navigation sequence completed! (3 steps)
```

## Key Features

### ✅ Automatic Location Detection
- Tracks current room based on objects reached
- Uses last known location for planning

### ✅ World Coordinate Navigation  
- All room positions stored in memory.yaml
- Nav2 handles actual path execution
- Robust to robot pose updates

### ✅ Multi-Waypoint Support
- Can navigate through multiple rooms
- Each waypoint uses world coordinates
- Waits for completion before next waypoint

### ✅ Obstacle Avoidance
- Detects blocked paths
- Removes blocked edges from graph
- Automatically finds alternative routes

### ✅ Fallback Strategies
1. Try direct vision-based navigation
2. If blocked, use motion planner
3. Navigate to target room via waypoints
4. Try detecting object again at destination
5. If still not visible, navigate to stored coordinates

## Testing Your Scenario

### Setup:
1. Robot starts in home gym area
2. Exercise ball is visible
3. Path from home gym to bedroom2 is blocked
4. Alternative route via living room is clear

### Command:
```
"go to the exercise ball, turn right and stop near the bed"
```

### Expected Result:
```
✅ Reaches exercise ball
✅ Turns right  
🗺️ Plans alternative route (home gym → living room → bedroom2)
✅ Navigates through living room
✅ Reaches bedroom2
✅ Finds and approaches bed
```

## Configuration

### Current Location Defaults
```python
# If no previous location known
current_location = "home gym"  # Default starting point
```

### Timeout Settings
```python
timeout = 20.0  # seconds per navigation goal
```

### Max Replanning Attempts
```python
if retry_count > 3:
    # Stop and report failure
```

## Debug Information

### Check Current Location
```python
current_room = self.memory_builder.last_room_type
print(f"Current location: {current_room}")
```

### Verify Path Planning
```python
path = planner.plan_path("home gym", "bedroom2")
print(f"Planned path: {' -> '.join(path)}")
```

### Monitor Navigation State
```bash
ros2 topic echo /robot_state
# Should show: navigating, reachGoal, or failed
```

## Troubleshooting

### "Cannot reach object - all routes blocked"
- Check if edges exist in memory.yaml between rooms
- Verify room nodes have correct world coordinates
- Ensure motion planner initialized with correct file

### Robot doesn't update location
- Verify object exists in memory.yaml
- Check `_find_object_in_memory()` is finding the object
- Ensure room name matches between object and room nodes

### Waypoint navigation fails
- Check camera pose is being published to `/camera2map`
- Verify Nav2 is running and accepting goals
- Check world coordinates are in correct frame

---

**Status:** ✅ Fully Implemented
**Tested With:** memory.yaml (Home environment)
**Navigation:** Topological graph + Nav2 waypoints



