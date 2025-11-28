# 🚧 Dynamic Obstacle Avoidance & Path Replanning

## Overview

The robot now automatically detects blocked paths during navigation and dynamically replans alternative routes. When an obstacle prevents the robot from reaching its goal, it:

1. ✅ **Detects the navigation failure** (timeout or "failed" state)
2. ✅ **Identifies the blocked path** between current and target rooms
3. ✅ **Removes the blocked edge** from memory.yaml
4. ✅ **Finds an alternative route** using the topological graph
5. ✅ **Continues navigation** via the new path

## How It Works

### Scenario: "Go to the exercise ball, turn right and stop near the bed"

#### Without Obstacle
```
home gym (exercise ball) → turn right → bedroom2 (bed) ✅
```

#### With Obstacle Blocking Direct Path
```
1. Robot attempts: home gym → bedroom2
   ❌ Navigation fails - obstacle detected!

2. System response:
   🚫 Removes edge: home gym ↔ bedroom2 from memory.yaml
   🗺️  Searches for alternative path
   
3. Alternative route found:
   home gym → living room → bedroom2 ✅
   
4. Robot navigates via living room and reaches the bed! ✅
```

## Key Features

### 1. **Automatic Failure Detection**
- Monitors `/robot_state` topic for "failed" status
- Detects navigation timeouts (20 seconds)
- No manual intervention required

### 2. **Intelligent Edge Removal**
- Identifies blocked path between current and target rooms
- Removes bidirectional edges from graph
- Persists changes to memory.yaml file

### 3. **D* Lite Path Replanning**
- Uses D* Lite algorithm for efficient replanning
- Considers all alternative paths in topological graph
- Optimizes for shortest path cost

### 4. **Retry Prevention**
- Tracks retry attempts per edge (max 3 attempts)
- Prevents infinite loops
- Fails gracefully when no path exists

## Implementation Details

### Modified Files

#### 1. `motion_planner.py`
**New Methods:**
- `remove_edge(from_node, to_node)` - Remove edge from graph
- `remove_edge_from_yaml(from_node, to_node)` - Persist removal to file
- `replan_after_edge_removal(...)` - Complete replanning workflow

```python
def replan_after_edge_removal(self, start_location, goal_location, 
                              blocked_from, blocked_to):
    """Remove blocked edge and find alternative path"""
    # Remove from graph
    self.remove_edge(blocked_from, blocked_to)
    # Remove from YAML
    self.remove_edge_from_yaml(blocked_from, blocked_to)
    # Replan
    return self.plan_path(start_location, goal_location)
```

#### 2. `start_service.py`
**Enhanced Features:**
- `_wait_for_goal_completion()` - Returns success/failure status
- `_navigate_with_motion_planner()` - Handles replanning with failure info
- `_remove_blocked_path_from_memory()` - Wrapper for edge removal
- Navigation loop - Detects failures and triggers replanning

**New State Tracking:**
```python
# Navigation tracking for obstacle detection
self.current_path = []
self.current_path_index = 0
self.navigation_retries = {}
self.max_retries_per_edge = 1
```

### Navigation Flow with Failure Handling

```python
# In process_navigation_command()
while idx < len(self.turn_list):
    obj = self.obj_list[idx]
    
    if self._process_object_navigation(obj, relation, idx):
        success = self._wait_for_goal_completion()
        
        if not success:
            # 🚧 Navigation failed!
            target_room, _, _ = self._find_object_in_memory(obj)
            current_room = self.memory_builder.last_room_type
            
            # 🔄 Try alternative route
            self._navigate_with_motion_planner(
                obj, relation, 
                retry_count=1, 
                last_failed_from=current_room
            )
```

## User Experience

### GUI Feedback Messages

| Status | Message |
|--------|---------|
| 🚧 | Navigation failed - obstacle detected |
| 🔍 | Failed navigation from 'home gym' toward 'bedroom2' |
| 🚫 | Removing blocked path: home gym ↔ bedroom2 |
| 🔄 | Updating map - removing blocked path... |
| ✅ | Map updated - path removed |
| 🗺️ | Searching for alternative route... |
| ✅ | Found alternative path: home gym → living room → bedroom2 |
| 🚶 | Navigating to intermediate room: living room |
| ✅ | Reached 'bed' via alternative route! |

## Testing

### Manual Test Scenario

1. **Setup:** Place obstacle between home gym and bedroom2
2. **Command:** "go to the exercise ball, turn right and stop near the bed"
3. **Expected Behavior:**
   - Robot goes to exercise ball ✅
   - Turns right ✅
   - Attempts to reach bed ❌ (blocked)
   - Detects failure ✅
   - Finds alternative route via living room ✅
   - Successfully reaches bed ✅

### Verification

Check `memory.yaml` after failed navigation:
```yaml
edges:
# Before failure:
- cost: 2.4
  from: home gym
  to: bedroom2  # This edge exists

# After failure:
# Edge is removed from file ✅
```

## Configuration

### Timeout Settings
```python
# In _wait_for_goal_completion()
timeout = 20.0  # seconds
```

### Max Replanning Attempts
```python
# In _navigate_with_motion_planner()
if retry_count > 3:
    # Stop trying
```

## Benefits

1. **Autonomous Recovery** - No human intervention needed
2. **Persistent Learning** - Blocked paths stay removed
3. **Flexible Navigation** - Always finds alternative if available
4. **Robust Operation** - Handles dynamic environments
5. **User Friendly** - Clear status updates in GUI

## Future Enhancements

- [ ] Time-based edge re-enablement (temporary obstacles)
- [ ] Confidence scoring for blocked edges
- [ ] Local obstacle avoidance for minor obstacles
- [ ] Learning from successful alternative routes

## Related Files

- `/autonomousService/src/detect_vl/scripts/motion_planner.py`
- `/autonomousService/src/detect_vl/detect_vl/start_service.py`
- `/autonomousService/src/memory.yaml` (topological graph)
- `/autonomousPro/src/my_bringup/my_bringup/my_nav_goalpose.py` (Nav2 interface)

---

**Status:** ✅ Feature Complete & Tested
**Version:** 1.0
**Date:** November 2024


