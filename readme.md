# Warehouse Inventory Management Simulation

This Python script simulates warehouse inventory management using the A* algorithm for autonomous robot pathfinding. It loads a grid representation of a warehouse from a PNG image and allows users to specify robot missions (pickup and dropoff positions). The robots navigate the warehouse to complete their missions efficiently.

## Description

Pathfinding using A* Algorithm: `f(n) = g(n) + h(n)`

### Simulation

- g(n) represents the number of steps taken to reach the current position (node) from the start position (node)
- h(n) represents the Euclidean distance from the current position (node) to the goal
- f(n) represents the total cost of the path
- Diagonal movement is ALLOWED therefore the heuristic is Euclidean distance not the Manhattan distance
- Robot is considered an Obstacle, and can move and interact with items diagonally as well

### Colors

- Black pixels are obstacles (or shelves)
- White pixels are empty spaces (can be traversed)
- Red, Green, Blue pixels are items (can be picked up)
- Cyan pixels are robots
- Yellow pixels are robots paths (these change every time a robot calculates its path)
- Inventory is managed using the individual Red, Green, and Blue channels representing respective item quantities

> For simplicity in current scope, count is limited to 4

### Pathfinding

- Path finding happens one step at a time, once a path is found, robot moves one step of the path, then the next robot calculates its path and so on. This prevents the robots from overlapping each other or colliding with each other.
- Movements and interactions with the robots for each time step are printed to the console.
- If there are any pathfinding errors, the simulation will time out after pre-set timesteps.

### Warehouse

- The simulation is limited to a single warehouse loaded from a PNG image `warehouse_1.png`.
- Warehouse layout and item inventory can be changed by modifying the `warehouse_1.png` file. An easy way to edit images is via VS Code Extension [Luna Paint](https://marketplace.visualstudio.com/items?itemName=Tyriar.luna-paint)
- Example image:
- <!-- markdownlint-disable MD033 -->
  <img src="warehouse_example.png" width="300" height="300" alt="Warehouse Example">
  <!-- markdownlint-disable MD033 -->

> Note: The warehouse image should be a PNG file with each pixel representing a grid space in the warehouse.
> Other warehouse-like features can be added in the future but not implemented due to scope of project.

## Dependencies

- Python >= 3.8
- numpy
- opencv-python
- Pillow

## Usage

1. Install the required dependencies using pip:

   ```bash
   pip3 install -r requirements.txt
   ```

2. Run the simulation script:

   ```bash
   python3 run.py
   ```

## Adding New Simulation Jobs

To add new simulation jobs, you can create instances of the `Job` class and configure them with the desired robot positions, pickup positions, and dropoff positions. Simply create a new `Job` instance and use the `add_robot` method to add robots to the simulation. Then, call the `simulate_warehouse` method to run the simulation.

Example:

```python
# Create a new simulation job
new_job = Job()

# Add robots to the simulation
new_job.add_robot((0, 0), (7, 2), (0, 0))  # Pickup Blue and dropoff at starting position
new_job.add_robot((0, 1), (5, 4), (0, 1))  # Pickup Green and dropoff at starting position
new_job.add_robot((0, 2), (4, 8), (0, 2))  # Pickup Red and dropoff at starting position

# Run the simulation
new_job.simulate_warehouse()
```

## Slow Down Simulation or change image scale

In `run.py`, you can change `image_scale_factor` variable to change the size of the displayed image related to the grid size.

```python
# Parameters for visualization:
image_scale_factor = 50  # Scale factor for image display
# NOTE: (grid size * image_scale_factor = display image size)
sim_time_step = 0.01  # seconds between each iteration of the simulation
```

To slow down the simulation, you can set the `sim_time_step` variable to a higher value. The default value is 0.01 seconds.
> Logs for each time step can be seen in the console.

## License

This project is licensed under the [MIT license](LICENSE)
