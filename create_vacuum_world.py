import random
import os
from PIL import Image

clean_space = 0
dirty_space = 1
obstacle = 2

###########################################################################
###########################################################################
# ____________Change parameters below to change grid______________________
dirt_probability = 0.7
default_height = 15
default_width = 15
###########################################################################
###########################################################################


def create_grid(width, height):
    """Create a new grid with empty cells on odd row/column combinations."""
    grid = []
    for row in range(height):
        grid.append([])
        for column in range(width):
            if column % 2 == 1 and row % 2 == 1:
                grid[row].append(clean_space)
            elif column == 0 or row == 0 or column == width - 1 or row == height - 1:
                grid[row].append(clean_space)
            else:
                grid[row].append(obstacle)

    # go over row and column to add dirt randomly
    for row in range(height):
        for column in range(width):
            if grid[row][column] == clean_space:
                if random.random() < dirt_probability:
                    grid[row][column] = dirty_space
    return grid


def make_depth_first_random_grid(grid_width, grid_height):
    """Create a grid with random depth-first search paths."""
    grid = create_grid(grid_width, grid_height)

    width = (len(grid[0]) - 1) // 2
    height = (len(grid) - 1) // 2
    visited = [[0] * width + [1] for _ in range(height)] + [[1] * (width + 1)]

    def walk(column: int, row: int):
        """Recursively walk the grid and mark the cells as visited."""
        visited[row][column] = 1

        directions = [(column - 1, row), (column, row + 1), (column + 1, row), (column, row - 1)]
        random.shuffle(directions)
        for new_column, new_row in directions:
            if visited[new_row][new_column]:
                continue
            if new_column == column:
                grid[max(row, new_row) * 2][column * 2 + 1] = clean_space
            if new_row == row:
                grid[row * 2 + 1][max(column, new_column) * 2] = clean_space

            walk(new_column, new_row)

    walk(random.randrange(width), random.randrange(height))

    return grid


def save_grid_to_png(grid, filepath):
    """Save a grid to a PNG file."""
    image = Image.new("RGB", (len(grid[0]), len(grid)), "white")  # Create a new image with white background
    pixels = image.load()
    for row in range(len(grid)):
        for column in range(len(grid[row])):
            if grid[row][column] == obstacle:
                pixels[column, row] = (0, 0, 0)  # Black color for crate
            if grid[row][column] == clean_space:
                pixels[column, row] = (255, 255, 255)  # White color for space
            if grid[row][column] == dirty_space:
                pixels[column, row] = (255, 150, 150)  # Light Red color for dirt
    pixels[0, 0] = (255, 255, 255)  # White color for robot start position
    pixels[len(grid[0]) - 1, len(grid) - 1] = (255, 255, 255)  # White color for robot goal position
    image.save(filepath)


def main():
    """Create a grid and save it to a PNG file."""
    randomized_grid = make_depth_first_random_grid(default_width, default_height)

    script_dir = os.path.dirname(os.path.abspath(__file__))  # Get the directory of the current script
    file_path = os.path.join(script_dir, "vacuum_world.png")  # Construct the absolute path to the input PNG file
    save_grid_to_png(randomized_grid, file_path)


if __name__ == "__main__":
    main()
