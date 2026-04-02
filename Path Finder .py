
import tkinter as tk
from tkinter import messagebox, ttk
import math
import random
import time
from collections import deque
from enum import Enum
from dataclasses import dataclass, field
from typing import List, Tuple, Set, Dict, Optional
import heapq

# ==================== ENUMS & DATA CLASSES ====================

class Algorithm(Enum):
    """Search algorithms"""
    ASTAR = "A* Search"
    GBFS = "Greedy Best-First Search"

class Heuristic(Enum):
    """Heuristic functions"""
    MANHATTAN = "Manhattan Distance"
    EUCLIDEAN = "Euclidean Distance"

class NodeState(Enum):
    """Node visualization states"""
    EMPTY = "empty"
    WALL = "wall"
    START = "start"
    GOAL = "goal"
    FRONTIER = "frontier"
    VISITED = "visited"
    PATH = "path"
    CURRENT = "current"

@dataclass
class Node:
    """Represents a node in the grid"""
    row: int
    col: int
    g: float = 0  # Cost from start
    h: float = 0  # Heuristic cost to goal
    f: float = 0  # Total cost
    parent: Optional['Node'] = None
    
    def __eq__(self, other):
        return self.row == other.row and self.col == other.col
    
    def __hash__(self):
        return hash((self.row, self.col))
    
    def __lt__(self, other):
        return self.f < other.f

@dataclass
class SearchMetrics:
    """Tracks search performance metrics"""
    nodes_visited: int = 0
    path_cost: float = 0
    execution_time: float = 0
    path_length: int = 0
    frontier_size: int = 0

# grid settings

class GridEnvironment:
    """Manages the grid world and obstacle dynamics"""
    
    def __init__(self, rows: int, cols: int, obstacle_density: float = 0.3):
        self.rows = rows
        self.cols = cols
        self.obstacle_density = obstacle_density
        self.grid = [[False for _ in range(cols)] for _ in range(rows)]
        self.start = None
        self.goal = None
        self.dynamic_obstacles: List[Tuple[int, int]] = []
        
    def generate_random_maze(self):
        """Generate maze with random obstacles"""
        for r in range(self.rows):
            for c in range(self.cols):
                if random.random() < self.obstacle_density:
                    self.grid[r][c] = True
    
    def is_walkable(self, row: int, col: int) -> bool:
        """Check if cell is walkable (not wall)"""
        if row < 0 or row >= self.rows or col < 0 or col >= self.cols:
            return False
        return not self.grid[row][col]
    
    def set_obstacle(self, row: int, col: int, is_wall: bool = True):
        """Manually place or remove obstacle"""
        if 0 <= row < self.rows and 0 <= col < self.cols:
            self.grid[row][col] = is_wall
    
    def set_start(self, row: int, col: int):
        """Set start position"""
        if self.is_walkable(row, col):
            self.start = (row, col)
    
    def set_goal(self, row: int, col: int):
        """Set goal position"""
        if self.is_walkable(row, col):
            self.goal = (row, col)
    
    def get_neighbors(self, row: int, col: int) -> List[Tuple[int, int]]:
        """Get valid neighboring cells (8-directional movement)"""
        neighbors = []
        directions = [
            (-1, -1), (-1, 0), (-1, 1),
            (0, -1),           (0, 1),
            (1, -1),  (1, 0),  (1, 1)
        ]
        
        for dr, dc in directions:
            nr, nc = row + dr, col + dc
            if self.is_walkable(nr, nc):
                neighbors.append((nr, nc))
        
        return neighbors
    
    def spawn_dynamic_obstacle(self, spawn_probability: float = 0.05):
        """Randomly spawn new obstacles during search"""
        if random.random() < spawn_probability:
            r = random.randint(0, self.rows - 1)
            c = random.randint(0, self.cols - 1)
            
            # Don't spawn on start or goal
            if (r, c) != self.start and (r, c) != self.goal:
                self.grid[r][c] = True
                self.dynamic_obstacles.append((r, c))
                return (r, c)
        return None
    
    def reset_grid(self):
        """Reset grid to initial state"""
        self.grid = [[False for _ in range(self.cols)] for _ in range(self.rows)]
        self.dynamic_obstacles = []

# using algorithms
    #This setion has two algorithms implemented one is A sterick other is Best First Search 

class PathfindingAgent:
    """Implements A* and Greedy Best-First Search"""
    
    def __init__(self, environment: GridEnvironment):
        self.env = environment
        self.metrics = SearchMetrics()
        self.frontier: List[Node] = []
        self.visited: Set[Tuple[int, int]] = set()
        self.frontier_set: Set[Tuple[int, int]] = set()
        self.all_frontier_nodes: List[Node] = []
        self.path: List[Tuple[int, int]] = []
        
    def heuristic(self, node: Node, goal: Tuple[int, int], 
                  heuristic_type: Heuristic) -> float:
        """Calculate heuristic value"""
        gr, gc = goal
        
        if heuristic_type == Heuristic.MANHATTAN:
            return abs(node.row - gr) + abs(node.col - gc)
        
        elif heuristic_type == Heuristic.EUCLIDEAN:
            return math.sqrt((node.row - gr)**2 + (node.col - gc)**2)
        
        return 0
    
    def movement_cost(self, from_pos: Tuple[int, int], 
                     to_pos: Tuple[int, int]) -> float:
        """Calculate movement cost (1 for orthogonal, sqrt(2) for diagonal)"""
        fr, fc = from_pos
        tr, tc = to_pos
        
        if abs(fr - tr) + abs(fc - tc) == 1:  # Orthogonal
            return 1.0
        else:  # Diagonal
            return math.sqrt(2)
    
    def reconstruct_path(self, node: Optional[Node]) -> List[Tuple[int, int]]:
        """Reconstruct path from start to goal"""
        path = []
        current = node
        
        while current is not None:
            path.append((current.row, current.col))
            current = current.parent
        
        path.reverse()
        return path
    
    def search(self, algorithm: Algorithm, heuristic_type: Heuristic) -> bool:
        """Execute search algorithm"""
        start_time = time.time()
        
        # Reset search state
        self.frontier = []
        self.visited = set()
        self.frontier_set = set()
        self.all_frontier_nodes = []
        self.path = []
        
        if not self.env.start or not self.env.goal:
            return False
        
        # Create start node
        start_node = Node(self.env.start[0], self.env.start[1])
        start_node.h = self.heuristic(start_node, self.env.goal, heuristic_type)
        start_node.f = start_node.g + start_node.h if algorithm == Algorithm.ASTAR else start_node.h
        
        heapq.heappush(self.frontier, start_node)
        self.frontier_set.add((start_node.row, start_node.col))
        self.all_frontier_nodes.append(start_node)
        
        while self.frontier:
            current = heapq.heappop(self.frontier)
            self.frontier_set.discard((current.row, current.col))
            
            if (current.row, current.col) == self.env.goal:
                self.path = self.reconstruct_path(current)
                self.metrics.path_cost = current.g
                self.metrics.path_length = len(self.path) - 1
                self.metrics.execution_time = (time.time() - start_time) * 1000
                self.metrics.frontier_size = len(self.frontier)
                return True
            
            if (current.row, current.col) in self.visited:
                continue
            
            self.visited.add((current.row, current.col))
            self.metrics.nodes_visited += 1
            
            # Explore neighbors
            for nr, nc in self.env.get_neighbors(current.row, current.col):
                if (nr, nc) in self.visited:
                    continue
                
                g = current.g + self.movement_cost((current.row, current.col), (nr, nc))
                
                neighbor = Node(nr, nc, g=g)
                neighbor.parent = current
                neighbor.h = self.heuristic(neighbor, self.env.goal, heuristic_type)
                neighbor.f = neighbor.g + neighbor.h if algorithm == Algorithm.ASTAR else neighbor.h
                
                # Check if neighbor already in frontier with better path
                existing = None
                for fn in self.all_frontier_nodes:
                    if fn.row == nr and fn.col == nc:
                        existing = fn
                        break
                
                if existing is None or g < existing.g:
                    heapq.heappush(self.frontier, neighbor)
                    if (nr, nc) not in self.frontier_set:
                        self.frontier_set.add((nr, nc))
                        self.all_frontier_nodes.append(neighbor)
        
        self.metrics.execution_time = (time.time() - start_time) * 1000
        self.metrics.frontier_size = len(self.frontier)
        return False
    
    def replan_if_needed(self, current_pos: Tuple[int, int], 
                        algorithm: Algorithm, heuristic_type: Heuristic) -> bool:
        """Check if current path is blocked and replan if necessary"""
        if not self.path:
            return False
        
        # Find next step in path
        try:
            next_idx = self.path.index(current_pos) + 1
            if next_idx < len(self.path):
                next_pos = self.path[next_idx]
                
                # Check if next position is blocked
                if not self.env.is_walkable(next_pos[0], next_pos[1]):
                    # Path is blocked, replan from current position
                    self.env.start = current_pos
                    return self.search(algorithm, heuristic_type)
        except (ValueError, IndexError):
            pass
        
        return True

# ==================== GUI APPLICATION ====================

class PathfindingGUI:
    """Tkinter-based GUI for the pathfinding agent"""
    
    def __init__(self, root):
        self.root = root
        self.root.title("Dynamic Pathfinding Agent")
        self.root.geometry("1400x900")
        
        # Configuration
        self.grid_rows = 20
        self.grid_cols = 20
        self.cell_size = 25
        self.obstacle_density = 0.2
        
        # State
        self.env = GridEnvironment(self.grid_rows, self.grid_cols, self.obstacle_density)
        self.agent = PathfindingAgent(self.env)
        self.current_pos = None
        self.is_searching = False
        self.is_dynamic_mode = False
        self.animation_step = 0
        self.path_execution = False
        
        # Selected tool and algorithm
        self.selected_tool = "start"  # 'start', 'goal', 'wall', 'erase'
        self.selected_algorithm = Algorithm.ASTAR
        self.selected_heuristic = Heuristic.MANHATTAN
        
        self.create_ui()
        
    def create_ui(self):
        """Create user interface"""
        # Top control panel
        control_frame = ttk.Frame(self.root)
        control_frame.pack(side=tk.TOP, fill=tk.X, padx=5, pady=5)
        
        # Grid configuration
        ttk.Label(control_frame, text="Grid Size:").pack(side=tk.LEFT, padx=5)
        ttk.Label(control_frame, text="Rows:").pack(side=tk.LEFT)
        rows_spin = ttk.Spinbox(control_frame, from_=5, to=50, width=5, 
                               command=self.update_grid_size)
        rows_spin.set(self.grid_rows)
        self.rows_spin = rows_spin
        rows_spin.pack(side=tk.LEFT, padx=2)
        
        ttk.Label(control_frame, text="Cols:").pack(side=tk.LEFT)
        cols_spin = ttk.Spinbox(control_frame, from_=5, to=50, width=5,
                               command=self.update_grid_size)
        cols_spin.set(self.grid_cols)
        self.cols_spin = cols_spin
        cols_spin.pack(side=tk.LEFT, padx=2)
        
        ttk.Label(control_frame, text="Obstacle Density:").pack(side=tk.LEFT, padx=5)
        density_spin = ttk.Spinbox(control_frame, from_=0, to=1, increment=0.1, width=5,
                                  command=self.update_obstacle_density)
        density_spin.set(self.obstacle_density)
        self.density_spin = density_spin
        density_spin.pack(side=tk.LEFT, padx=2)
        
        ttk.Button(control_frame, text="Generate Maze", 
                  command=self.generate_maze).pack(side=tk.LEFT, padx=5)
        ttk.Button(control_frame, text="Clear Grid", 
                  command=self.clear_grid).pack(side=tk.LEFT, padx=5)
        
        # Algorithm selection frame
        algo_frame = ttk.LabelFrame(self.root, text="Algorithm & Heuristic", padding=5)
        algo_frame.pack(side=tk.TOP, fill=tk.X, padx=5, pady=5)
        
        ttk.Label(algo_frame, text="Algorithm:").pack(side=tk.LEFT, padx=5)
        algo_var = tk.StringVar(value=self.selected_algorithm.value)
        self.algo_var = algo_var
        algo_combo = ttk.Combobox(algo_frame, textvariable=algo_var, state="readonly",
                                 values=[algo.value for algo in Algorithm])
        algo_combo.pack(side=tk.LEFT, padx=5)
        algo_combo.bind("<<ComboboxSelected>>", self.on_algorithm_changed)
        
        ttk.Label(algo_frame, text="Heuristic:").pack(side=tk.LEFT, padx=5)
        heur_var = tk.StringVar(value=self.selected_heuristic.value)
        self.heur_var = heur_var
        heur_combo = ttk.Combobox(algo_frame, textvariable=heur_var, state="readonly",
                                 values=[heur.value for heur in Heuristic])
        heur_combo.pack(side=tk.LEFT, padx=5)
        heur_combo.bind("<<ComboboxSelected>>", self.on_heuristic_changed)
        
        # Tool selection frame
        tool_frame = ttk.LabelFrame(self.root, text="Editing Tools", padding=5)
        tool_frame.pack(side=tk.TOP, fill=tk.X, padx=5, pady=5)
        
        ttk.Button(tool_frame, text="🟢 Set Start", 
                  command=lambda: self.select_tool("start")).pack(side=tk.LEFT, padx=5)
        ttk.Button(tool_frame, text="🔴 Set Goal", 
                  command=lambda: self.select_tool("goal")).pack(side=tk.LEFT, padx=5)
        ttk.Button(tool_frame, text="⬛ Place Wall", 
                  command=lambda: self.select_tool("wall")).pack(side=tk.LEFT, padx=5)
        ttk.Button(tool_frame, text="🗑️ Erase", 
                  command=lambda: self.select_tool("erase")).pack(side=tk.LEFT, padx=5)
        
        self.tool_label = ttk.Label(tool_frame, text=f"Selected: {self.selected_tool}")
        self.tool_label.pack(side=tk.LEFT, padx=20)
        
        # Search controls
        search_frame = ttk.Frame(self.root)
        search_frame.pack(side=tk.TOP, fill=tk.X, padx=5, pady=5)
        
        self.search_button = ttk.Button(search_frame, text="🔍 Start Search", 
                                       command=self.start_search)
        self.search_button.pack(side=tk.LEFT, padx=5)
        
        self.dynamic_button = ttk.Button(search_frame, text="⚡ Toggle Dynamic Mode", 
                                        command=self.toggle_dynamic_mode)
        self.dynamic_button.pack(side=tk.LEFT, padx=5)
        
        ttk.Button(search_frame, text="Reset Search", 
                  command=self.reset_search).pack(side=tk.LEFT, padx=5)
        
        self.dynamic_label = ttk.Label(search_frame, text="Dynamic Mode: OFF")
        self.dynamic_label.pack(side=tk.LEFT, padx=20)
        
        # Main grid canvas
        canvas_frame = ttk.Frame(self.root)
        canvas_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        self.canvas = tk.Canvas(canvas_frame, bg="white", 
                               width=self.grid_cols * self.cell_size,
                               height=self.grid_rows * self.cell_size)
        self.canvas.pack()
        self.canvas.bind("<Button-1>", self.on_canvas_click)
        self.canvas.bind("<B1-Motion>", self.on_canvas_drag)
        
        # Metrics panel
        metrics_frame = ttk.LabelFrame(self.root, text="Metrics", padding=10)
        metrics_frame.pack(side=tk.RIGHT, fill=tk.BOTH, padx=5, pady=5)
        
        self.metrics_text = tk.Text(metrics_frame, width=30, height=20, font=("Courier", 10))
        self.metrics_text.pack(fill=tk.BOTH, expand=True)
        self.metrics_text.config(state=tk.DISABLED)
        
        # Draw initial grid
        self.draw_grid()
    
    def select_tool(self, tool: str):
        """Select editing tool"""
        self.selected_tool = tool
        self.tool_label.config(text=f"Selected: {self.selected_tool}")
    
    def on_algorithm_changed(self, event=None):
        """Handle algorithm selection change"""
        for algo in Algorithm:
            if algo.value == self.algo_var.get():
                self.selected_algorithm = algo
                break
    
    def on_heuristic_changed(self, event=None):
        """Handle heuristic selection change"""
        for heur in Heuristic:
            if heur.value == self.heur_var.get():
                self.selected_heuristic = heur
                break
    
    def update_grid_size(self):
        """Update grid dimensions"""
        try:
            self.grid_rows = int(self.rows_spin.get())
            self.grid_cols = int(self.cols_spin.get())
            self.env = GridEnvironment(self.grid_rows, self.grid_cols, self.obstacle_density)
            self.agent = PathfindingAgent(self.env)
            self.canvas.config(width=self.grid_cols * self.cell_size,
                             height=self.grid_rows * self.cell_size)
            self.draw_grid()
        except ValueError:
            messagebox.showerror("Error", "Invalid grid size")
    
    def update_obstacle_density(self):
        """Update obstacle density"""
        try:
            self.obstacle_density = float(self.density_spin.get())
        except ValueError:
            messagebox.showerror("Error", "Invalid density value")
    
    def generate_maze(self):
        """Generate random maze"""
        self.env.reset_grid()
        self.env.generate_random_maze()
        self.draw_grid()
    
    def clear_grid(self):
        """Clear all obstacles"""
        self.env.reset_grid()
        self.agent = PathfindingAgent(self.env)
        self.draw_grid()
    
    def on_canvas_click(self, event):
        """Handle canvas click"""
        col = event.x // self.cell_size
        row = event.y // self.cell_size
        
        if 0 <= row < self.grid_rows and 0 <= col < self.grid_cols:
            self.apply_tool(row, col)
            self.draw_grid()
    
    def on_canvas_drag(self, event):
        """Handle canvas drag"""
        col = event.x // self.cell_size
        row = event.y // self.cell_size
        
        if 0 <= row < self.grid_rows and 0 <= col < self.grid_cols:
            self.apply_tool(row, col)
            self.draw_grid()
    
    def apply_tool(self, row: int, col: int):
        """Apply selected tool at grid position"""
        if self.selected_tool == "start":
            self.env.set_start(row, col)
        elif self.selected_tool == "goal":
            self.env.set_goal(row, col)
        elif self.selected_tool == "wall":
            self.env.set_obstacle(row, col, True)
        elif self.selected_tool == "erase":
            self.env.set_obstacle(row, col, False)
    
    def draw_grid(self):
        """Redraw grid"""
        self.canvas.delete("all")
        
        # Draw cells
        for r in range(self.grid_rows):
            for c in range(self.grid_cols):
                x1 = c * self.cell_size
                y1 = r * self.cell_size
                x2 = x1 + self.cell_size
                y2 = y1 + self.cell_size
                
                # Determine cell color
                if (r, c) == self.env.start:
                    color = "#00FF00"  # Green
                    self.canvas.create_rectangle(x1, y1, x2, y2, fill=color, outline="black")
                elif (r, c) == self.env.goal:
                    color = "#FF0000"  # Red
                    self.canvas.create_rectangle(x1, y1, x2, y2, fill=color, outline="black")
                elif self.env.grid[r][c]:
                    color = "#000000"  # Black
                    self.canvas.create_rectangle(x1, y1, x2, y2, fill=color, outline="gray")
                elif (r, c) in self.agent.visited:
                    color = "#87CEEB"  # Sky blue
                    self.canvas.create_rectangle(x1, y1, x2, y2, fill=color, outline="gray")
                elif (r, c) in self.agent.frontier_set:
                    color = "#FFFF00"  # Yellow
                    self.canvas.create_rectangle(x1, y1, x2, y2, fill=color, outline="gray")
                elif (r, c) in self.agent.path:
                    color = "#00FF00"  # Green
                    self.canvas.create_rectangle(x1, y1, x2, y2, fill=color, outline="gray")
                else:
                    color = "#FFFFFF"  # White
                    self.canvas.create_rectangle(x1, y1, x2, y2, fill=color, outline="gray")
        
        # Draw current position during execution
        if self.current_pos:
            r, c = self.current_pos
            x1 = c * self.cell_size
            y1 = r * self.cell_size
            x2 = x1 + self.cell_size
            y2 = y1 + self.cell_size
            self.canvas.create_oval(x1, y1, x2, y2, fill="#FF6600", outline="black", width=2)
    
    def start_search(self):
        """Start pathfinding search"""
        if not self.env.start or not self.env.goal:
            messagebox.showerror("Error", "Please set both start and goal positions")
            return
        
        self.agent = PathfindingAgent(self.env)
        
        if self.agent.search(self.selected_algorithm, self.selected_heuristic):
            messagebox.showinfo("Success", "Path found!")
            self.current_pos = self.env.start
            if self.is_dynamic_mode:
                self.animate_path()
            else:
                self.draw_grid()
            self.update_metrics()
        else:
            messagebox.showerror("Failed", "No path found!")
            self.draw_grid()
    
    def toggle_dynamic_mode(self):
        """Toggle dynamic obstacle mode"""
        self.is_dynamic_mode = not self.is_dynamic_mode
        mode_text = "ON" if self.is_dynamic_mode else "OFF"
        self.dynamic_label.config(text=f"Dynamic Mode: {mode_text}")
    
    def reset_search(self):
        """Reset search results"""
        self.agent = PathfindingAgent(self.env)
        self.current_pos = None
        self.draw_grid()
        self.clear_metrics()
    
    def animate_path(self):
        """Animate path execution with dynamic obstacles"""
        if not self.agent.path or self.animation_step >= len(self.agent.path) - 1:
            self.animation_step = 0
            messagebox.showinfo("Complete", "Path execution completed!")
            return
        
        self.current_pos = self.agent.path[self.animation_step]
        
        # Spawn dynamic obstacle
        new_obstacle = self.env.spawn_dynamic_obstacle(spawn_probability=0.1)
        
        # Check if path is blocked and replan if necessary
        if new_obstacle:
            if not self.agent.replan_if_needed(self.current_pos, 
                                              self.selected_algorithm, 
                                              self.selected_heuristic):
                messagebox.showwarning("Replanning", "Could not find alternative path!")
                self.animation_step = 0
                return
        
        self.animation_step += 1
        self.draw_grid()
        self.update_metrics()
        
        # Continue animation
        self.root.after(500, self.animate_path)
    
    def update_metrics(self):
        """Update metrics display"""
        metrics = self.agent.metrics
        text = f"""
SEARCH METRICS
{'='*25}
Nodes Visited: {metrics.nodes_visited}
Frontier Size: {metrics.frontier_size}
Path Length: {metrics.path_length}
Path Cost: {metrics.path_cost:.2f}
Exec Time: {metrics.execution_time:.2f}ms

CONFIGURATION
{'='*25}
Algorithm: {self.selected_algorithm.value}
Heuristic: {self.selected_heuristic.value}
Dynamic Mode: {'ON' if self.is_dynamic_mode else 'OFF'}
Obstacles: {sum(sum(row) for row in self.env.grid)}

GRID
{'='*25}
Dimensions: {self.grid_rows}x{self.grid_cols}
Density: {self.obstacle_density:.1%}
        """
        
        self.metrics_text.config(state=tk.NORMAL)
        self.metrics_text.delete(1.0, tk.END)
        self.metrics_text.insert(1.0, text)
        self.metrics_text.config(state=tk.DISABLED)
    
    def clear_metrics(self):
        """Clear metrics display"""
        self.metrics_text.config(state=tk.NORMAL)
        self.metrics_text.delete(1.0, tk.END)
        self.metrics_text.config(state=tk.DISABLED)

# ==================== MAIN PROGRAM ====================

if __name__ == "__main__":
    root = tk.Tk()
    gui = PathfindingGUI(root)

    root.mainloop()
