import sys
import random
import math
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Polygon

# Mixo Khoza, Palesa Rapolaki, Daniel Ngobe
# Uncomment the matplotlib stuff to visualize the RRT algorithm

"""
Why RRT instead of PRM (Probabilistic Roadmap):
- RRT is better at exploring narrow passages and constrained environments
- RRT can find a solution with fewer iterations in many cases
- RRT naturally produces a connected path, while PRM may require an additional 
  path-finding step after roadmap construction
- RRT is single-query, making it more efficient for one-time path planning scenarios
- RRT doesn't require pre-processing and can be used in dynamic environments
"""

# Fixed map boundaries
MAP_X_MIN = -12
MAP_X_MAX = 7
MAP_Y_MIN = -3
MAP_Y_MAX = 12

def parse_input():
    input_lines = []
    for line in sys.stdin:
        stripped_line = line.strip()
        if stripped_line == '-1':
            break
        input_lines.append(stripped_line)
    
    start_goal = input_lines[0].split(';')
    start = tuple(map(float, start_goal[0].split(',')))
    goal = tuple(map(float, start_goal[1].split(',')))
    obstacles = []
    
    for line in input_lines[1:]:
        points = line.split(';')
        p1 = tuple(map(float, points[0].split(',')))
        p2 = tuple(map(float, points[1].split(',')))
        x_min = min(p1[0], p2[0])
        x_max = max(p1[0], p2[0])
        y_min = min(p1[1], p2[1])
        y_max = max(p1[1], p2[1])
        obstacles.append((x_min, y_min, x_max, y_max))
    
    return start, goal, obstacles


def create_angled_bar(top_left_x, top_left_y, length, thickness, angle_degrees):
    """
    Create an angled bar obstacle as a list of 4 corner points.
    
    Args:
        top_left_x, top_left_y: Top-left corner coordinates
        length: Length of the bar
        thickness: Thickness of the bar
        angle_degrees: Rotation angle in degrees
    
    Returns:
        List of (x, y) tuples representing the 4 corners of the rotated rectangle
    """
    angle_rad = math.radians(angle_degrees)
    cos_a = math.cos(angle_rad)
    sin_a = math.sin(angle_rad)
    
    # Define the 4 corners of the rectangle before rotation (relative to top-left)
    corners = [
        (0, 0),  # top-left
        (length, 0),  # top-right
        (length, -thickness),  # bottom-right
        (0, -thickness)  # bottom-left
    ]
    
    # Rotate and translate each corner
    rotated_corners = []
    for x, y in corners:
        # Rotate around the top-left corner
        rotated_x = x * cos_a - y * sin_a + top_left_x
        rotated_y = x * sin_a + y * cos_a + top_left_y
        rotated_corners.append((rotated_x, rotated_y))
    
    return rotated_corners


def point_in_polygon(point, polygon):
    """Check if a point is inside a polygon using ray casting algorithm."""
    x, y = point
    n = len(polygon)
    inside = False
    
    p1x, p1y = polygon[0]
    for i in range(1, n + 1):
        p2x, p2y = polygon[i % n]
        if y > min(p1y, p2y):
            if y <= max(p1y, p2y):
                if x <= max(p1x, p2x):
                    if p1y != p2y:
                        xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                    if p1x == p2x or x <= xinters:
                        inside = not inside
        p1x, p1y = p2x, p2y
    
    return inside


def line_intersects_polygon(p1, p2, polygon):
    """Check if a line segment intersects with a polygon."""
    # Check if either endpoint is inside the polygon
    if point_in_polygon(p1, polygon) or point_in_polygon(p2, polygon):
        return True
    
    # Check if the line intersects any edge of the polygon
    for i in range(len(polygon)):
        edge_start = polygon[i]
        edge_end = polygon[(i + 1) % len(polygon)]
        if line_segments_intersect(p1, p2, edge_start, edge_end):
            return True
    
    return False


def line_segments_intersect(p1, q1, p2, q2):
    """Check if two line segments intersect."""
    def ccw(A, B, C):
        return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])
    
    return ccw(p1, p2, q2) != ccw(q1, p2, q2) and ccw(p1, q1, p2) != ccw(p1, q1, q2)


"""
    Liang-Barsky line-rectangle intersection algorithm (found online).
    
    Determines if a line segment from point a to point b intersects with
    a rectangle defined by (x_min, y_min, x_max, y_max).
    
    Args:
        a: Start point (x, y) of the line segment
        b: End point (x, y) of the line segment
        x_min, y_min, x_max, y_max: Rectangle bounds
        
    Returns:
        bool: True if the line segment intersects the rectangle, False otherwise
"""
def liang_barsky(a, b, x_min, y_min, x_max, y_max):
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    p = [-dx, dx, -dy, dy]
    q = [a[0] - x_min, x_max - a[0], a[1] - y_min, y_max - a[1]]
    t0, t1 = 0.0, 1.0
    for i in range(4):
        if p[i] == 0:
            if q[i] < 0:
                return False
        else:
            t = q[i] / p[i]
            if p[i] < 0:
                if t > t1:
                    return False
                t0 = max(t0, t)
            else:
                if t < t0:
                    return False
                t1 = min(t1, t)
    return t0 < t1


# Check if a line segment from point a to point b is collision-free.
def is_collision_free(a, b, obstacles, angled_obstacle=None):
    # Check collision with regular rectangular obstacles
    for (x_min, y_min, x_max, y_max) in obstacles:
        if liang_barsky(a, b, x_min, y_min, x_max, y_max):
            return False
    
    # Check collision with angled obstacle if it exists
    if angled_obstacle and line_intersects_polygon(a, b, angled_obstacle):
        return False
    
    return True


# Check if a point is within the map boundaries
def is_within_map(point):
    return (MAP_X_MIN <= point[0] <= MAP_X_MAX and 
            MAP_Y_MIN <= point[1] <= MAP_Y_MAX)


def distance(p1, p2):
    return math.hypot(p1[0]-p2[0], p1[1]-p2[1])


"""
    Steer from one point toward another, but limited by step_size.
    
    Args:
        from_point: Starting point (x, y)
        to_point: Target point (x, y)
        step_size: Maximum distance to move
        
    Returns:
        tuple: New point (x, y) that is at most step_size distance from from_point
"""
def steer(from_point, to_point, step_size):
    dx = to_point[0] - from_point[0]
    dy = to_point[1] - from_point[1]
    dist = math.hypot(dx, dy)
    if dist <= step_size:
        return (round(to_point[0], 2), round(to_point[1], 2))
    scale = step_size / dist
    new_x = from_point[0] + dx * scale
    new_y = from_point[1] + dy * scale
    return (round(new_x, 2), round(new_y, 2))


def rrt_path_planner(goal):
    """
    Main function to be called from other files.
    
    Args:
        goal: Goal position as tuple (x, y)
        
    Returns:
        List of (x, y) tuples representing the path from start to goal,
        or None if no path is found
    """
    # Fixed start position
    start = (0, 0)
    
    # Hardcoded obstacles from the input data
    obstacles = [
        (-11.5, 11, 2, 11.5),     # -11.5,11.5;2,11 -> min/max format
        (2, 8, 2.5, 11.5),       # 2,11.5;2.5,8 -> min/max format  
        (2.5, 8, 6, 8.5),        # 2.5,8.5;6,8 -> min/max format
        (6, -2.5, 6.5, 8.5),     # 6,8.5;6.5,-2.5 -> min/max format
        (-5.5, -2.5, 6, -2),     # -5.5,-2;6,-2.5 -> min/max format
        (-6, -2.5, -5.5, 7.5),   # -6,7.5;-5.5,-2.5 -> min/max format
        (-5.5, 7, -2.5, 7.5),    # -5.5,7.5;-2.5,7 -> min/max format
        (-5.5, -2, -2.5, 0),     # -5.5,0;-2.5,-2 -> min/max format
        (2, 5, 3, 6),            # 2,6;3,5 -> min/max format
        (-3.5, 2, 4, 2.5),       # -3.5,2.5;4,2 -> min/max format
        (5.75, 7.75, 6, 8),      # 5.75,8;6,7.75 -> min/max format
        (1.5, -0.5, 2.5, 0.5),   # 1.5,0.5;2.5,-0.5 -> min/max format
        (-5, 5, -4, 5.5),        # -5,5.5;-4,5 -> min/max format
        (0.25, -0.75, 0.75, -0.25) # 0.25,-0.25;0.75,-0.75 -> min/max format
    ]
    
    # Create the angled bar obstacle
    angled_bar = create_angled_bar(-4, 4, 4, 0.5, 30)
    
    # Run RRT algorithm
    return rrt(start, goal, obstacles, angled_bar)


def rrt(start, goal, obstacles, angled_obstacle=None, max_iter=5000, step_size=5, goal_threshold=5):
    """
    Rapidly-exploring Random Tree (RRT) algorithm implementation.
    
    Args:
        start: Start position (x, y)
        goal: Goal position (x, y)
        obstacles: List of (x_min, y_min, x_max, y_max) tuples defining obstacles
        angled_obstacle: List of (x, y) tuples defining angled obstacle corners
        max_iter: Maximum number of iterations
        step_size: Maximum step size for each tree extension
        goal_threshold: Distance threshold to consider goal reached
    """
    nodes = [{'point': start, 'parent': None}]
    goal_reached = False
    
    # Set up the plot with fixed map boundaries
    plt.figure(figsize=(12, 10))
    ax = plt.gca()
    ax.set_xlim(MAP_X_MIN - 2, MAP_X_MAX + 2)
    ax.set_ylim(MAP_Y_MIN - 2, MAP_Y_MAX + 2)
    
    # Draw map boundary
    map_width = MAP_X_MAX - MAP_X_MIN
    map_height = MAP_Y_MAX - MAP_Y_MIN
    ax.add_patch(Rectangle((MAP_X_MIN, MAP_Y_MIN), map_width, map_height, 
                          fill=False, edgecolor='black', linewidth=2))
    
    # Draw regular obstacles
    for (x_min, y_min, x_max, y_max) in obstacles:
        width = x_max - x_min
        height = y_max - y_min
        ax.add_patch(Rectangle((x_min, y_min), width, height, fill=True, color='gray', alpha=0.5))
    
    # Draw the angled bar obstacle (use the parameter, not create a new one)
    if angled_obstacle:
        ax.add_patch(Polygon(angled_obstacle, fill=True, color='gray', alpha=0.5))
    
    plt.plot(start[0], start[1], 'go', markersize=10, label='Start')
    plt.plot(goal[0], goal[1], 'ro', markersize=10, label='Goal')
    
    for _ in range(max_iter):
        # 5% chance to sample the goal, 95% chance to sample randomly within map
        if random.random() < 0.05:
            sample = goal
        else:
            sample = (
                round(random.uniform(MAP_X_MIN, MAP_X_MAX), 2),
                round(random.uniform(MAP_Y_MIN, MAP_Y_MAX), 2)
            )
        
        nearest = min(nodes, key=lambda node: distance(node['point'], sample))
        new_point = steer(nearest['point'], sample, step_size)
        
        # Check if new point is within map boundaries
        if not is_within_map(new_point):
            continue
            
        if is_collision_free(nearest['point'], new_point, obstacles, angled_obstacle):
            plt.plot([nearest['point'][0], new_point[0]],
                    [nearest['point'][1], new_point[1]],
                    color='blue', alpha=0.3, linewidth=0.5)
            
            new_node = {'point': new_point, 'parent': nodes.index(nearest)}
            nodes.append(new_node)
            
            if distance(new_point, goal) <= goal_threshold:
                if is_collision_free(new_point, goal, obstacles, angled_obstacle):
                    plt.plot([new_point[0], goal[0]],
                            [new_point[1], goal[1]],
                            color='blue', alpha=0.3, linewidth=0.5)
                    goal_node = {'point': goal, 'parent': len(nodes)-1}
                    nodes.append(goal_node)
                    goal_reached = True
                    break

    if goal_reached:
        path = []
        current = nodes[-1]
        while current['parent'] is not None:
            path.append(current['point'])
            parent_node = nodes[current['parent']]
            plt.plot([current['point'][0], parent_node['point'][0]],
                    [current['point'][1], parent_node['point'][1]],
                    color='red', linewidth=3, linestyle='-')
            current = parent_node
        path.append(start)
        path.reverse()
    else:
        path = None
    
    plt.title('RRT Path Planning')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    plt.show()
    
    return path


def main():
    start, goal, obstacles = parse_input()
    
    # Validate that start and goal are within map boundaries
    # if not is_within_map(start):
    #     print(f"Error: Start point {start} is outside map boundaries")
    #     return
    # if not is_within_map(goal):
    #     print(f"Error: Goal point {goal} is outside map boundaries")
    #     return
    
    # Create the angled bar obstacle
    angled_bar = create_angled_bar(-4, 4, 4, 0.5, 30)
    
    path = rrt(start, goal, obstacles, angled_bar)
    # if path:
    #     print("Path found:")
    #     for point in path:
    #         print(f"{point[0]},{point[1]}")
    # else:
    #     print("No path found")

if __name__ == "__main__":
    main()