#!/usr/bin/env python3
"""
Launcher script for the Robot Navigation GUI System

This script launches both:
1. The autonomous service node (start_service.py)
2. The GUI interface (robot_chat_gui.py)

Usage:
    python3 launch_robot_gui.py
"""

import subprocess
import sys
import os
import time
import signal

def main():
    print("=" * 60)
    print("🚀 Launching Robot Navigation GUI System")
    print("=" * 60)
    
    processes = []
    
    try:
        # Get the workspace root
        workspace_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))
        
        # Launch the service node
        print("\n📡 Starting autonomous service node...")
        service_script = os.path.join(workspace_root, "src/detect_vl/detect_vl/start_service.py")
        
        if not os.path.exists(service_script):
            print(f"❌ Error: Service script not found at {service_script}")
            return
        
        service_process = subprocess.Popen(
            ["python3", service_script],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            universal_newlines=True,
            bufsize=1
        )
        processes.append(("Service Node", service_process))
        print("✅ Service node started")
        
        # Wait a bit for the service to initialize
        time.sleep(2)
        
        # Launch the GUI
        print("\n🖥️  Starting GUI interface...")
        gui_script = os.path.join(workspace_root, "src/GUI/robot_chat_gui.py")
        
        if not os.path.exists(gui_script):
            print(f"❌ Error: GUI script not found at {gui_script}")
            # Clean up service process
            service_process.terminate()
            return
        
        gui_process = subprocess.Popen(
            ["python3", gui_script],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            universal_newlines=True,
            bufsize=1
        )
        processes.append(("GUI", gui_process))
        print("✅ GUI started")
        
        print("\n" + "=" * 60)
        print("✅ System launched successfully!")
        print("=" * 60)
        print("\n📝 Instructions:")
        print("  1. The GUI window should appear shortly")
        print("  2. Type navigation commands in the GUI")
        print("  3. Watch the robot execute your commands")
        print("  4. Press Ctrl+C here to shut down everything")
        print("\n" + "=" * 60)
        
        # Monitor both processes
        while True:
            # Check if any process has died
            for name, proc in processes:
                if proc.poll() is not None:
                    print(f"\n⚠️ {name} has stopped unexpectedly")
                    raise KeyboardInterrupt
            
            # Print output from service node (for debugging)
            try:
                line = service_process.stdout.readline()
                if line:
                    print(f"[Service] {line.rstrip()}")
            except:
                pass
            
            time.sleep(0.1)
    
    except KeyboardInterrupt:
        print("\n\n⛔ Shutting down...")
        
        # Terminate all processes
        for name, proc in processes:
            try:
                print(f"  🛑 Stopping {name}...")
                proc.terminate()
                proc.wait(timeout=5)
                print(f"  ✅ {name} stopped")
            except subprocess.TimeoutExpired:
                print(f"  ⚠️ Force killing {name}...")
                proc.kill()
            except Exception as e:
                print(f"  ⚠️ Error stopping {name}: {e}")
        
        print("\n✅ Shutdown complete")
    
    except Exception as e:
        print(f"\n❌ Error: {e}")
        # Clean up on error
        for name, proc in processes:
            try:
                proc.terminate()
            except:
                pass
        sys.exit(1)


if __name__ == '__main__':
    main()




