#!/usr/bin/env python3

# Implementing Warehouse Inventory Management using A* Algorithm for Autonomous Robot Pathfinding

# A* Algorithm:
# f(n) = g(n) + h(n) where f(n) = total cost of path, g(n) = cost of path, h(n) = heuristic value of path

import time

import cv2
import numpy as np
from PIL import Image, ImageDraw


class Item:
    """Represents an item with name and quantity"""

    def __init__(self, name, quantity):
        """Initialize the item with name and quantity"""
        self.name = name
        self.quantity = quantity
        if name == "Robot":
            self.quantity = 0  # Robot is considered Obstacle

    def __repr__(self) -> str:
        """Return a string representation of the item"""
        return f"Item({self.name}, {self.quantity})"

    def add(self, quantity=1):
        """Add quantity to the current quantity"""
        self.quantity += quantity

    def remove(self, quantity=1):
        """Remove quantity from the current quantity"""
        self.quantity -= quantity
        if self.quantity == 0:
            self.name = "Obstacle"


class Grid:
    """Represents a grid of items"""

    def __init__(self, file_path):
        """Initialize the grid with items"""

        self.grid = []
        self.grid_from_png(file_path)  # 2D array of items [row][column]
        self.height = len(self.grid)
        self.width = len(self.grid[0])

    def __repr__(self) -> str:
        """Return a string representation of the grid"""

        repr_str = f"Grid: ( {self.width} x {self.height} )\n"
        repr_str += "-" * self.width * 3 + "\n"
        for row in self.grid:
            for column in row:  # Item object
                repr_str += f"{column.quantity:3}"  # Width of 3 characters, right-aligned
            repr_str += "\n"
        repr_str += "-" * self.width * 3 + "\n"
        return repr_str

    def grid_from_png(self, file_path) -> None:
        """Returns a 2D array of integers from a PNG image"""

        try:
            image = Image.open(file_path)
            pixels = image.load()
            self.grid = []
            # Loop through all rows and columns
            for row in range(image.height):
                row_items = []
                for column in range(image.width):
                    item_name, item_quantity = self.color_to_quantity(pixels[column, row])
                    # print(f"{item_name}, {item_quantity}")
                    row_items.append(Item(item_name, item_quantity))

                self.grid.append(row_items)

        except Exception as e:
            print(f"Error loading grid from PNG: {e}")
            exit()

    def color_to_quantity(self, pixel) -> tuple:
        """Returns the (item, quantity) corresponding to the pixel color"""

        r, g, b, a = pixel
        level_1 = 100  # Tolerance for color identification
        level_2 = 150
        level_3 = 200
        level_4 = 250
        name = ""
        quantity = 0
        intensity = -1

        if r > level_3 and g > level_3 and b > level_3:  # light shades
            name = "Open Space"
            intensity = -1
            return (name, intensity)
        elif r < level_3 and g > level_3 and b > level_3:  # Cyan shades
            name = "Robot"
            intensity = 0
        elif r >= level_1 and g < level_1 and b < level_1:  # Red shades
            name = "Red"
            intensity = r
        elif r < level_1 and g >= level_1 and b < level_1:  # Green shades
            name = "Green"
            intensity = g
        elif r < level_1 and g < level_1 and b >= level_1:  # Blue shades
            name = "Blue"
            intensity = b
        else:  # Considered Black
            name = "Obstacle"
            intensity = 0
            return (name, intensity)

        # Find the quantity based on the color intensity
        if intensity >= level_1:
            if intensity >= level_2:
                if intensity >= level_3:
                    if intensity >= level_4:
                        quantity = 4
                    else:
                        quantity = 3
                else:
                    quantity = 2
            else:
                quantity = 1
        else:
            quantity = 0

        return (name, quantity)

    def visualize(self, paths=None) -> Image:
        """Visualize the path on the grid and save it as an image file"""

        # Create a blank image
        image = Image.new("RGB", (self.width, self.height), color="white")
        draw = ImageDraw.Draw(image)

        # Draw grid cells
        for row in range(self.height):
            for column in range(self.width):
                item_color = self.item_to_color(self.grid[row][column])
                draw.point((column, row), fill=item_color)

        # Overlay a list of paths on the image if provided
        if paths is not None:
            for path in paths:
                if path is None:
                    continue
                for position in path:
                    column, row = position
                    if grid.grid[row][column].name == "Robot":
                        draw.point((column, row), fill="cyan")
                    else:
                        draw.point((column, row), fill="yellow")

        return image

    def item_to_color(self, item: Item) -> tuple:
        """Returns the (R, G, B) color corresponding to the quantity"""

        quantity = item.quantity
        name = item.name

        if quantity <= 0:  # If quantity is 0 or negative, consider it as Obstacle or Open Space
            intensity = 0
        elif quantity == 1:
            intensity = 100
        elif quantity == 2:
            intensity = 150
        elif quantity == 3:
            intensity = 200
        elif quantity > 3:
            intensity = 250

        if name == "Open Space":  # Considered White
            return (255, 255, 255)
        elif name == "Red":
            return (intensity, 0, 0)
        elif name == "Green":
            return (0, intensity, 0)
        elif name == "Blue":
            return (0, 0, intensity)
        elif name == "Robot":  # Cyan shades
            return (0, 255, 255)
        else:  # Considered Black or Obstacle
            return (0, 0, 0)

    def move_item(self, from_position: tuple, to_position: tuple) -> tuple:
        """Move an item from one position (column, row) to another"""

        item_to_move = self.grid[from_position[1]][from_position[0]]

        if self.grid[to_position[1]][to_position[0]].name == "Open Space":  # If space is open
            self.grid[from_position[1]][from_position[0]] = Item("Open Space", -1)  # Remove from the old position
            self.grid[to_position[1]][to_position[0]] = item_to_move  # Add to the new position
            return to_position

        else:
            print(f"Cannot move item from {from_position} to {to_position}. Space occupied.")
            return from_position


class Node:
    """A node in the graph with functions for A* pathfinding"""

    available_moves = [
        (0, -1),  # Up
        (0, 1),  # Down
        (-1, 0),  # Left
        (1, 0),  # Right
        (-1, -1),  # Diagonal left up
        (-1, 1),  # Diagonal left down
        (1, -1),  # Diagonal right up
        (1, 1),  # Diagonal right down
    ]
    adjacent_moves = [
        (0, -1),  # Up
        (0, 1),  # Down
        (-1, 0),  # Left
        (1, 0),  # Right
    ]

    def __init__(self, parent=None, position=None):
        """Initialize the node"""

        self.parent = parent  # Parent is a node
        self.position = position  # Position is a tuple (x, y)

        # Cost of path to this node (from start node)
        self.g = self.parent.g + 1 if self.parent is not None else 0
        # Heuristic value (Pythagorean distance from this node to the goal node)
        self.h = 0
        # Total cost
        self.f = self.g + self.h

    def __eq__(self, node):
        """Check if two nodes are equal"""

        return self.position == node.position

    def __repr__(self) -> str:
        """Return a string representation of the node"""

        return f"Node({self.position}, g={self.g}, h={self.h}, f={self.f})"

    def find_available_nodes(self, goal_node, grid, adjacent_only=False):
        """Returns a list of accessible adjacent nodes. Goal node is a Node object. Grid is a 2D array of items"""

        grid_height = len(grid)  # Get the height of the grid (rows)
        grid_width = len(grid[0])  # Get the width of the grid (columns)
        available_nodes = []

        moves = self.adjacent_moves if adjacent_only else self.available_moves
        for move in moves:  # Loop through all available actions
            new_position = (self.position[0] + move[0], self.position[1] + move[1])

            # Check if the new position is within the grid
            if (0 <= new_position[0] < grid_width) and (0 <= new_position[1] < grid_height):
                # Check if the new position is open space grid[row][column]
                if grid[new_position[1]][new_position[0]].quantity < 0:
                    adjacent_node = Node(self, new_position)  # Create a new node with the current node as parent
                    adjacent_node.find_cost_to(goal_node)  # Calculate the cost to the goal node
                    available_nodes.append(adjacent_node)  # Add the adjacent node to the list of available nodes

        return available_nodes

    def find_cost_to(self, goal_node):
        """Returns the cost of the path"""

        # g(n) = Number of steps taken to get to current node
        self.g = self.parent.g + 1 if self.parent is not None else 0

        # h(n) = Pythagorean distance from this node to the goal node
        x_diff = abs(goal_node.position[0] - self.position[0])
        y_diff = abs(goal_node.position[1] - self.position[1])
        distance = (x_diff**2 + y_diff**2) ** 0.5  # Square root of sum of squares
        self.h = round(distance, 5)  # Round the distance

        # f(n) = g(n) + h(n)
        self.f = self.g + self.h

        return self.f


def display_image(image, scale_factor=100, time_step=0.1):
    """Display an PIL image using OpenCV"""

    # Scale up the image and convert it to BGR for opencv to display
    image = image.resize((image.width * scale_factor, image.height * scale_factor), Image.NEAREST).convert("RGB")
    cv2.namedWindow("warehouse", cv2.WINDOW_NORMAL)
    cv2.imshow("warehouse", cv2.cvtColor(np.array(image), cv2.COLOR_RGB2BGR))
    cv2.waitKey(1)
    time.sleep(time_step)


class Robot:
    """The warehouse robot class for pathfinding"""

    global grid

    def __init__(self, start_position):
        """Initialize the robot with start position"""

        if start_position[0] >= grid.width or start_position[1] >= grid.height:
            if grid.grid[start_position[1]][start_position[0]].quantity >= 0:
                print("Start position is occupied")
            print("Invalid start position")
            exit()

        self.position = start_position
        # Add the robot to the grid
        grid.grid[self.position[1]][self.position[0]] = Item("Robot", 0)
        self.is_loaded = False  # Begin empty handed
        self.goal_position = None
        self.path = None
        self.mission_complete = False

    def __repr__(self) -> str:
        """Return a string representation of the robot"""

        return f"Robot({self.position} -> {self.goal_position})"

    def find_path_with_astar(self, start_node: Node, goal_node: Node, grid) -> list:
        """Returns a list of tuples as a path from the start node to the end node in the given grid"""

        open_nodes = []  # List of open nodes (unvisited nodes)
        closed_nodes = []  # List of closed nodes (visited nodes)
        open_nodes.append(start_node)  # Add the start node to the open nodes
        adjacent_goal_nodes = goal_node.find_available_nodes(goal_node, grid, adjacent_only=True)

        while open_nodes:  # Check all open nodes
            # Select the node with the lowest f (total cost)
            current_node = min(open_nodes, key=lambda node: node.f)

            # If goal node is occupied, see if adjacent goal nodes are available
            if grid[goal_node.position[1]][goal_node.position[0]].quantity >= 0:
                if current_node in adjacent_goal_nodes:
                    # print(f"Adjacent goal node found: {current_node.position}")
                    goal_node = current_node  # Update the goal node to the current node

            # If the goal node is found
            if current_node == goal_node:
                # Reconstruct the path from the goal node to the start node using parent nodes
                path = []
                while current_node is not None:
                    path.append(current_node.position)
                    current_node = current_node.parent
                return path[::-1]  # Return the reversed path (from start to goal)

            # Now that the current node is not the goal node, we mark it as visited
            # Transfer the current node from open to the closed nodes
            open_nodes.remove(current_node)
            closed_nodes.append(current_node)

            # Loop through all available nodes from the current node to update the open nodes list
            for available_node in current_node.find_available_nodes(goal_node, grid):
                if available_node in closed_nodes:  # Already visited, skip iteration
                    continue

                if available_node not in open_nodes:  # Add the available node to the open nodes
                    open_nodes.append(available_node)

        return None  # No path found

    def set_mission(self, goal_position, dropoff_position) -> None:
        """Set the goal position of the robot"""

        self.goal_reached = False

        if goal_position[0] >= grid.width or goal_position[1] >= grid.height:
            print("Invalid goal position")
            exit()
        self.goal_position = goal_position
        self.goal_node = Node(None, goal_position)

        if dropoff_position[0] >= grid.width or dropoff_position[1] >= grid.height:
            print("Invalid dropoff position")
            exit()
        self.dropoff_position = dropoff_position

    def move_towards_goal(self) -> None:
        """Move the robot toward the goal position"""

        if self.goal_reached:
            print("Mission complete: Sitting Idle")
            return

        # Find path using A* algorithm
        self.start_node = Node(None, self.position)
        self.start_node.find_cost_to(self.goal_node)  # Find initial cost for goal node
        self.path = self.find_path_with_astar(self.start_node, self.goal_node, grid.grid)
        if self.path is not None:
            # print(f"Path found: {self.path}")
            self.path.pop(0)  # Remove the start position from the path
            self.position = grid.move_item(self.position, self.path[0])  # Move robot ahead one step
            print(f"Robot moved to: {self.position}")
            if len(self.path) == 1:
                print(f"Reached goal position: {self.goal_position}")
                self.goal_reached = True
                if self.is_loaded:
                    print("Unloading item: 1 unit")
                    self.is_loaded = False
                    self.mission_complete = True  # Single use robot
                    print("Mission complete")
                else:
                    item = grid.grid[self.goal_position[1]][self.goal_position[0]]
                    print(f"Found Quantity: {item.quantity}")
                    print("Loading item: 1 unit")
                    item.remove()  # Remove one unit
                    print(f"Remaining Quantity: {item.quantity}")
                    self.is_loaded = True
                    self.goal_reached = False
                    self.goal_position = self.dropoff_position  # Change goal position to dropoff position
                    self.goal_node = Node(None, self.goal_position)
                    print(f"Dropoff position: {self.goal_position}")


class Job:
    """Class to store robot job information. Includes a singular mission for each robot"""

    def __init__(self, image_scale_factor=50, sim_time_step=0.01, max_steps=25):
        self.robots = []
        self.image_scale_factor = image_scale_factor
        self.sim_time_step = sim_time_step
        self.max_steps = max_steps

    def add_robot(self, robot_position, pickup_position, dropoff_position) -> None:
        """Add a robot to the simulation job. Position is (col, row) Tuple"""
        robot = Robot(robot_position)
        robot.set_mission(pickup_position, dropoff_position)
        self.robots.append(robot)

    def simulate_warehouse(self) -> None:
        """Main function to test the A* algorithm in warehouse environment"""

        try:
            for steps in range(self.max_steps):
                print(f"Time Step: {steps} " + "-" * 30)
                for i, robot in enumerate(self.robots):
                    print(f"Robot {i}:")
                    robot.move_towards_goal()
                    display_image(
                        grid.visualize([robot.path for robot in self.robots]),
                        scale_factor=self.image_scale_factor,
                        time_step=self.sim_time_step,
                    )
                    print("-" * 20)
                if all(robot.mission_complete for robot in self.robots):
                    break

            print("Simulation complete" + "-" * 50)

        except Exception as e:
            print(f"---------------- Exception in simulate_warehouse ----------------\n{e}")

        cv2.destroyAllWindows()


if __name__ == "__main__":
    grid = []
    grid = Grid("warehouse_1.png")  # Load grid from PNG image
    print(grid)

    ###########################################################################
    ###########################################################################

    # _____________Enter simulation jobs and parameters________________________

    # Parameters for visualization:
    image_scale_factor = 50  # Scale factor for image display
    # NOTE: (grid size * image_scale_factor = display image size)
    sim_time_step = 0.05  # seconds between each iteration of the simulation

    # Make jobs to run warehouse simulation:
    test_1 = Job(image_scale_factor, sim_time_step)
    # NOTE: Enter (column, row) for (robot, pickup, dropoff) positions
    test_1.add_robot((0, 0), (7, 2), (0, 0))  # Pickup Blue and dropoff at starting position
    test_1.add_robot((0, 1), (5, 4), (0, 1))  # Pickup Green and dropoff at starting position
    test_1.add_robot((0, 2), (4, 8), (0, 2))  # Pickup Red and dropoff at starting position
    test_1.simulate_warehouse()

    test_2 = Job(image_scale_factor, sim_time_step)  # Pickup Blue and dropoff at different positions
    test_2.add_robot((0, 0), (7, 2), (0, 9))
    test_2.add_robot((0, 1), (7, 2), (9, 0))
    test_2.add_robot((0, 2), (7, 2), (9, 9))
    test_2.simulate_warehouse()

    test_3 = Job(image_scale_factor, sim_time_step)  # Pickup Red and dropoff at different positions
    test_3.add_robot((0, 9), (4, 8), (9, 9))
    test_3.add_robot((9, 0), (4, 8), (9, 9))
    test_3.add_robot((9, 9), (4, 8), (9, 9))
    test_3.simulate_warehouse()

    ###########################################################################
    ###########################################################################
