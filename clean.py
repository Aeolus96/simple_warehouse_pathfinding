#!/usr/bin/env python3

# Implementing Vacuum Cleaning Simulation using And-Or Algorithm for Autonomous Robot Coverage and Pathfinding

import os
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
        self.size = self.height * self.width

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
        """Returns the (item, quantity) corresponding to the pixel color. Updated to work with dirty spaces"""

        r, g, b = pixel

        level_1 = 100  # Tolerance for color identification
        # level_2 = 150
        level_3 = 200
        level_4 = 250
        name = ""
        intensity = -1

        if r < level_1 and g < level_1 and b < level_1:  # Black
            name = "Obstacle"
            intensity = 0
            return (name, intensity)

        elif r > level_4 and g > level_4 and b > level_4:  # White
            name = "Open Space"
            intensity = -1
            return (name, intensity)

        elif r < level_3 and g > level_3 and b > level_3:  # Cyan shades
            name = "Robot"
            intensity = 0
            return (name, intensity)

        elif r > level_1 and g == b:  # Red shades
            name = "Dirty Space"
            intensity = 1
            return (name, intensity)

        else:  # Considered Invalid
            name = "Invalid"
            intensity = 0
            print(f"Invalid color: {pixel}")
            return (name, intensity)

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

        name = item.name

        if name == "Open Space":  # Considered White
            return (255, 255, 255)
        elif name == "Dirty Space":
            return (255, 150, 150)
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

        # Simulating cleaning dirty space by moving robot over the dirty space
        elif self.grid[to_position[1]][to_position[0]].name == "Dirty Space":  # If space is dirty
            self.grid[from_position[1]][from_position[0]] = Item("Open Space", -1)  # Remove from the old position
            self.grid[to_position[1]][to_position[0]] = item_to_move  # Add to the new position
            return to_position

        else:
            print(f"Cannot move item from {from_position} to {to_position}. Space occupied.")
            return from_position


class Node:
    """A node in the graph with functions for pathfinding"""

    diagonal_moves = [
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
        # Heuristic value
        self.h = 0
        # Total cost
        self.f = self.g + self.h

    def __eq__(self, node):
        """Check if two nodes are equal"""

        return self.position == node.position

    def __repr__(self) -> str:
        """Return a string representation of the node"""

        return f"Node({self.position}, g={self.g}, h={self.h}, f={self.f})"

    def find_available_nodes(self, goal_node, grid, adjacent_only=True) -> list:
        """Returns a list of accessible adjacent nodes. Goal node is a Node object. Grid is a 2D array of items"""

        grid_height = len(grid)  # Get the height of the grid (rows)
        grid_width = len(grid[0])  # Get the width of the grid (columns)
        available_nodes = []

        moves = self.adjacent_moves if adjacent_only else self.diagonal_moves
        for move in moves:  # Loop through all available actions
            new_position = (self.position[0] + move[0], self.position[1] + move[1])

            # Check if the new position is within the grid
            if (0 <= new_position[0] < grid_width) and (0 <= new_position[1] < grid_height):
                # Check if the new position is an obstacle
                obstacle = True if grid[new_position[1]][new_position[0]].quantity == 0 else False
                # Check if the new position is open space grid[row][column]
                if not obstacle:
                    adjacent_node = Node(self, new_position)  # Create a new node with the current node as parent
                    adjacent_node.find_cost_to(goal_node)  # Calculate the cost to the goal node
                    available_nodes.append(adjacent_node)  # Add the adjacent node to the list of available nodes

        return available_nodes

    def find_cost_to(self, goal_node, euclidean_distance=False) -> float:
        """Returns the cost of the path"""

        # g(n) = Number of steps taken to get to current node
        self.g = self.parent.g + 1 if self.parent is not None else 0

        if euclidean_distance:
            # h(n) = Pythagorean distance from this node to the goal node
            x_diff = abs(goal_node.position[0] - self.position[0])
            y_diff = abs(goal_node.position[1] - self.position[1])
            distance = (x_diff**2 + y_diff**2) ** 0.5  # Square root of sum of squares
            self.h = round(distance, 5)  # Round the distance
        else:
            # h(n) = Manhattan distance from this node to the goal node
            x_diff = abs(goal_node.position[0] - self.position[0])
            y_diff = abs(goal_node.position[1] - self.position[1])
            distance = x_diff + y_diff

        # f(n) = g(n) + h(n)
        self.f = self.g + self.h

        return self.f


def display_image(image, scale_factor=100, time_step=0.1):
    """Display an PIL image using OpenCV"""

    # Scale up the image and convert it to BGR for opencv to display
    image = image.resize((image.width * scale_factor, image.height * scale_factor), Image.NEAREST).convert("RGB")
    cv2.namedWindow("vacuum world", cv2.WINDOW_NORMAL)
    cv2.imshow("vacuum world", cv2.cvtColor(np.array(image), cv2.COLOR_RGB2BGR))
    cv2.waitKey(1)
    time.sleep(time_step)


class Robot:
    """The vacuum cleaning robot class for pathfinding"""

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

    def find_dirt(self) -> list:
        """Find all the dirt in the grid"""

        self.dirty_cells = []
        for row in range(grid.height):
            for column in range(grid.width):
                if grid.grid[row][column].name == "Dirty Space":
                    self.dirty_cells.append((column, row))  # Add the cell to the list of dirty cells
        # print(f"Dirty Cells: {self.dirty_cells}")
        return self.dirty_cells

    def find_best_route(self) -> list:
        """Find the best route for the robot\n
        Uses AND search strategy to select the route with most coverage."""

        self.find_dirt()  # Find the dirty cells every time
        if len(self.dirty_cells) == 0:
            return None

        routes = []  # List of paths from start to each dirty cell
        for dirt in self.dirty_cells:  # Loop through all dirty cells (position tuples)
            # print(f"Dirty Cell: {dirt}")
            self.start_node = Node(None, self.position)
            self.goal_node = Node(None, dirt)
            self.start_node.find_cost_to(self.goal_node)
            path = self.find_path_to_goal_node(self.start_node, self.goal_node, grid.grid)
            # print(f"Path: {path}")
            if path is not None:
                routes.append(path)  # Return the reversed path (from start to goal)

        # print(f"Routes: {routes}")
        longest_route = min(routes, key=len) if routes else []  # Find the longest shortest path for max coverage
        # print(f"Longest Route: {longest_route}")
        return longest_route

    def find_path_to_goal_node(self, start_node, goal_node, grid) -> list:
        """Returns a list of tuples as a path from the start node to the end node in the given grid\n
        Uses OR search strategy to find the first available path."""

        queue = [start_node]  # Create a queue of nodes
        visited = []  # Create a list of visited nodes

        # Loop until the queue is empty
        while queue:
            current_node = queue.pop(0)
            # print(f"Current Node: {current_node.position}")

            # If the goal node is found
            if current_node.position == goal_node.position:
                # print(f"Goal found: {current_node.position} -> {goal_node.position}")
                # Reconstruct the path from the goal node to the start node using parent nodes
                path = []
                while current_node is not None:
                    path.append(current_node.position)
                    current_node = current_node.parent
                return path[::-1]  # Return the reversed path (from start to goal)

            for adjacent_node in current_node.find_available_nodes(goal_node, grid):
                # print(f"Adjacent Node: {adjacent_node.position}")
                if adjacent_node in visited:  # Avoid cycles
                    continue
                if adjacent_node not in queue:
                    # print(f"Adding adjacent node to queue: {adjacent_node.position}")
                    queue.append(adjacent_node)

            visited.append(current_node)

        return None

    def move_towards_goal(self) -> None:
        """Move the robot toward the goal position"""

        if self.path is not None:  # If the robot has a mission, follow it until it reaches the goal
            print(f"Path: {self.path}")
            self.path.pop(0)  # Remove the start position from the path
            self.position = grid.move_item(self.position, self.path[0])  # Move robot ahead one step
            print(f"Robot moved to: {self.position}")
            if len(self.path) <= 1:  # If the robot has reached the goal, set the path to None
                self.path = None
        else:
            print("Mission complete: Sitting Idle")
            self.mission_complete = True

    def move_direction(self, direction) -> None:
        """Move the robot in the specified direction"""

        if direction == "up":
            self.new_position = (self.position[0], self.position[1] - 1)
        elif direction == "down":
            self.new_position = (self.position[0], self.position[1] + 1)
        elif direction == "left":
            self.new_position = (self.position[0] - 1, self.position[1])
        elif direction == "right":
            self.new_position = (self.position[0] + 1, self.position[1])

        if grid.grid[self.new_position[1]][self.new_position[0]].quantity != 0:
            self.position = grid.move_item(self.position, self.new_position)


class Job:
    """Class to store robot job information. Includes a singular mission for each robot"""

    def __init__(self, image_scale_factor=50, sim_time_step=0.01, max_steps=100, manual_control=False):
        self.robots = []
        self.image_scale_factor = image_scale_factor
        self.sim_time_step = sim_time_step
        self.max_steps = max_steps
        self.manual_control = manual_control

    def add_robot(self, robot_position) -> None:
        """Add a robot to the simulation job. Position is (col, row) Tuple"""
        robot = Robot(robot_position)
        self.robots.append(robot)

    def simulate_vacuum_world(self) -> None:
        """Main function to simulate vacuum world"""

        print("Starting new simulation " + "-" * 50)

        try:
            if not self.manual_control:
                for steps in range(self.max_steps):
                    print(f"Time Step: {steps} " + "-" * 30)
                    for i, robot in enumerate(self.robots):
                        print(f"Robot {i}:")
                        if robot.path is None:  # If the robot has no mission, find a new one
                            robot.path = robot.find_best_route()
                        robot.move_towards_goal()
                        display_image(
                            grid.visualize([robot.path for robot in self.robots]),
                            scale_factor=self.image_scale_factor,
                            time_step=self.sim_time_step,
                        )
                        print("-" * 20)
                    if all(robot.mission_complete for robot in self.robots):
                        break

            else:  # Manual Control
                display_image(
                    grid.visualize([robot.path for robot in self.robots]),
                    scale_factor=self.image_scale_factor,
                    time_step=self.sim_time_step,
                )

                robot = self.robots[0]
                exit_flag = False
                while not exit_flag:
                    direction = input("Manual Control Input (up, down, left, right | exit): ").lower()

                    if direction == "exit":
                        print("Exiting...")
                        exit_flag = True
                    elif direction in ["up", "down", "left", "right"]:
                        robot.move_direction(direction)
                    else:
                        print("Invalid direction")

                    display_image(
                        grid.visualize([robot.path for robot in self.robots]),
                        scale_factor=self.image_scale_factor,
                        time_step=self.sim_time_step,
                    )
                    print("-" * 50)

            print("Simulation complete " + "-" * 50)

        except Exception as e:
            print(f"---------------- Exception in simulate_vacuum_world ----------------\n{e}")

        cv2.destroyAllWindows()


if __name__ == "__main__":
    grid = []
    script_dir = os.path.dirname(os.path.abspath(__file__))  # Get the directory of the current script
    file_path = os.path.join(script_dir, "vacuum_world.png")  # Construct the absolute path to the input PNG file
    grid = Grid(file_path)  # Create the grid from the image
    print(grid)

    ###########################################################################
    ###########################################################################

    # _____________Enter simulation jobs and parameters________________________

    # Parameters for visualization:
    image_scale_factor = 50  # Scale factor for image display
    # NOTE: (grid size * image_scale_factor = display image size)
    sim_time_step = 0.05  # seconds between each iteration of the simulation

    # Make jobs to run the simulation:
    test_1 = Job(image_scale_factor, sim_time_step, max_steps=10 * grid.size)
    test_1.add_robot((0, 0))
    test_1.simulate_vacuum_world()

    # -------------------------------------------------------------------------
    time.sleep(1)  # Pause few seconds before running the next job
    # Since this vacuum world, the grid needs to reloaded with dirt from the image
    # OR manually add in dirt (supported but loading a new grid is easier and straightforward)
    grid = Grid(file_path)
    # -------------------------------------------------------------------------

    test_2 = Job(image_scale_factor, sim_time_step, max_steps=10 * grid.size)
    test_2.add_robot((0, 0))
    test_2.add_robot((grid.width - 1, grid.height - 1))
    test_2.add_robot((0, grid.height - 1))
    test_2.simulate_vacuum_world()

    # -------------------------------------------------------------------------
    # Manual mode - input text via terminal
    time.sleep(1)  # Pause few seconds before running the next job
    # Since this vacuum world, the grid needs to reloaded with dirt from the image
    # OR manually add in dirt (supported but loading a new grid is easier and straightforward)
    grid = Grid(file_path)
    # -------------------------------------------------------------------------
    test_3 = Job(image_scale_factor, sim_time_step, max_steps=10 * grid.size, manual_control=True)
    test_3.add_robot((0, 0))
    test_3.simulate_vacuum_world()

    ###########################################################################
    ###########################################################################

# End of File
