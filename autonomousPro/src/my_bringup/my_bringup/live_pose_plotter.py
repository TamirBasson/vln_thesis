import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import networkx as nx
import os
import sys
import numpy as np
from scipy.spatial.distance import pdist, squareform
from collections import defaultdict


# CSV path relative to the script's directory
CSV_FILE = os.path.join('/home/tamir/autonomousPro', 'poses.csv')

def read_csv():
    try:
        if os.path.exists(CSV_FILE):
            df = pd.read_csv(CSV_FILE)
            # Validate required columns
            required_columns = ['x', 'y', 'theta_deg']
            if not all(col in df.columns for col in required_columns):
                print(f"Error: CSV file is missing required columns. Expected: {required_columns}")
                return pd.DataFrame(columns=required_columns)
            return df
        else:
            print(f"Warning: CSV file not found at {CSV_FILE}")
            return pd.DataFrame(columns=['x', 'y', 'theta_deg'])
    except Exception as e:
        print(f"Error reading CSV file: {e}")
        return pd.DataFrame(columns=['x', 'y', 'theta_deg'])

class TopologicalMap:
    def __init__(self):
        self.graph = nx.Graph()
        self.regions = defaultdict(list)  # Store nodes by region
        self.connection_threshold = 0.3  # meters
        self.region_threshold = 1.0  # meters for region clustering
        
    def add_node(self, node_id, position, orientation):
        """Add a node to the topological map with its position and orientation."""
        self.graph.add_node(node_id, 
                          pos=position,
                          orientation=orientation)
        
        # Assign to a region based on position
        region_id = self._find_or_create_region(position)
        self.regions[region_id].append(node_id)
        
    def _find_or_create_region(self, position):
        """Find existing region or create new one based on position."""
        for region_id, nodes in self.regions.items():
            # Get the first node in the region as reference
            ref_node = nodes[0]
            ref_pos = self.graph.nodes[ref_node]['pos']
            if np.linalg.norm(np.array(position) - np.array(ref_pos)) < self.region_threshold:
                return region_id
        # If no nearby region found, create new one
        new_region = f"Region_{len(self.regions)}"
        return new_region
    
    def update_connections(self):
        """Update connections between nodes based on distance and regions."""
        # Clear existing edges
        self.graph.clear_edges()
        
        # Connect nodes within the same region
        for region_nodes in self.regions.values():
            if len(region_nodes) > 1:
                # Calculate distances between all nodes in the region
                positions = [self.graph.nodes[n]['pos'] for n in region_nodes]
                distances = squareform(pdist(positions))
                
                # Connect nodes that are within threshold
                for i, node1 in enumerate(region_nodes):
                    for j, node2 in enumerate(region_nodes[i+1:], i+1):
                        if distances[i, j] <= self.connection_threshold:
                            self.graph.add_edge(node1, node2, 
                                              weight=distances[i, j],
                                              type='intra-region')
        
        # Connect regions through their closest nodes
        region_centers = {}
        for region_id, nodes in self.regions.items():
            positions = [self.graph.nodes[n]['pos'] for n in nodes]
            center = np.mean(positions, axis=0)
            region_centers[region_id] = center
        
        # Connect regions that are close to each other
        for i, (region1, center1) in enumerate(region_centers.items()):
            for region2, center2 in list(region_centers.items())[i+1:]:
                if np.linalg.norm(center1 - center2) < self.region_threshold * 2:
                    # Find closest nodes between regions
                    closest_pair = self._find_closest_nodes(self.regions[region1], 
                                                          self.regions[region2])
                    if closest_pair:
                        node1, node2 = closest_pair
                        self.graph.add_edge(node1, node2,
                                          weight=np.linalg.norm(
                                              np.array(self.graph.nodes[node1]['pos']) - 
                                              np.array(self.graph.nodes[node2]['pos'])),
                                          type='inter-region')
    
    def _find_closest_nodes(self, nodes1, nodes2):
        """Find the closest pair of nodes between two sets of nodes."""
        min_dist = float('inf')
        closest_pair = None
        
        for n1 in nodes1:
            pos1 = self.graph.nodes[n1]['pos']
            for n2 in nodes2:
                pos2 = self.graph.nodes[n2]['pos']
                dist = np.linalg.norm(np.array(pos1) - np.array(pos2))
                if dist < min_dist:
                    min_dist = dist
                    closest_pair = (n1, n2)
        
        return closest_pair if min_dist < self.region_threshold * 2 else None

# Create figure with larger size
fig, ax = plt.subplots(figsize=(10, 10))
ax.set_title("Topological Map with Regions")
ax.set_xlabel("X (meters)")
ax.set_ylabel("Y (meters)")
ax.grid(True)
ax.set_aspect('equal', 'box')

# Initialize topological map
topo_map = TopologicalMap()

def update_graph(df):
    if df.empty:
        return
    
    # Convert to numpy arrays for easier manipulation
    x = df['x'].astype(float).values
    y = df['y'].astype(float).values
    theta = df['theta_deg'].astype(float).values
    
    # Update topological map
    for i in range(len(x)):
        node_id = f"Node_{i}"
        position = (x[i], y[i])
        orientation = theta[i]
        topo_map.add_node(node_id, position, orientation)
    
    # Update connections
    topo_map.update_connections()
    
    # Clear the plot
    ax.clear()
    ax.set_title("Topological Map with Regions")
    ax.set_xlabel("X (meters)")
    ax.set_ylabel("Y (meters)")
    ax.grid(True)
    ax.set_aspect('equal', 'box')
    
    if len(x) > 0:
        # Get positions for all nodes
        pos = nx.get_node_attributes(topo_map.graph, 'pos')
        
        # Draw the graph with different edge types
        # Draw all nodes
        nx.draw_networkx_nodes(topo_map.graph, pos, node_size=50, 
                             node_color='blue', alpha=0.3, ax=ax)
        
        # Draw intra-region edges in blue
        intra_edges = [(u, v) for u, v, d in topo_map.graph.edges(data=True) 
                      if d['type'] == 'intra-region']
        nx.draw_networkx_edges(topo_map.graph, pos, edgelist=intra_edges, 
                             edge_color='blue', width=2, alpha=0.7, ax=ax)
        
        # Draw inter-region edges in green
        inter_edges = [(u, v) for u, v, d in topo_map.graph.edges(data=True) 
                      if d['type'] == 'inter-region']
        nx.draw_networkx_edges(topo_map.graph, pos, edgelist=inter_edges, 
                             edge_color='green', width=1, alpha=0.3, ax=ax)
        
        # Highlight the current position (last node)
        if len(x) > 0:
            current_node = f"Node_{len(x)-1}"
            current_pos = pos[current_node]
            ax.scatter([current_pos[0]], [current_pos[1]], c='red', s=100)
            
            # Draw orientation arrow
            theta_rad = np.radians(theta[-1])
            arrow_length = 0.5
            dx = arrow_length * np.cos(theta_rad)
            dy = arrow_length * np.sin(theta_rad)
            ax.arrow(current_pos[0], current_pos[1], dx, dy, 
                    head_width=0.1, head_length=0.1, fc='red', ec='red')
        
        # Set plot limits with padding
        x_padding = 1.0
        y_padding = 1.0
        ax.set_xlim(min(x) - x_padding, max(x) + x_padding)
        ax.set_ylim(min(y) - y_padding, max(y) + y_padding)

def animate(i):
    df = read_csv()
    update_graph(df)

try:
    # Update more frequently (every 200ms)
    ani = animation.FuncAnimation(fig, animate, interval=200)
    plt.show()
except KeyboardInterrupt:
    print("\nPlotter terminated by user")
    sys.exit(0)
except Exception as e:
    print(f"Error running plotter: {e}")
    sys.exit(1)
