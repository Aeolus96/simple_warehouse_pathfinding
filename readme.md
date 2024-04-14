# Warehouse Inventory Management Simulation

This Python script simulates warehouse inventory management using the A* algorithm for autonomous robot pathfinding. It loads a grid representation of a warehouse from a PNG image and allows users to specify robot missions (pickup and dropoff positions). The robots navigate the warehouse to complete their missions efficiently.

## Dependencies

- Python >= 3.8
- numpy
- opencv-python
- Pillow

## How to Run

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

Warehouse layout and item inventory can be changed by modifying the `warehouse_1.png` file. An easy way to edit images is via VS Code Extension [Luna Paint](https://marketplace.visualstudio.com/items?itemName=Tyriar.luna-paint)

> Note: The warehouse image should be a PNG file with each pixel representing a grid space in the warehouse.

Example image:

<!-- markdownlint-disable MD033 -->
<img src="warehouse_example.png" width="300" height="300" alt="Warehouse Example">
<!-- markdownlint-disable MD033 -->

### Features / Limitations

- The simulation is limited to a single warehouse loaded from a PNG image `warehouse_1.png`.
- The inventory is limited to 3 items. `Red`, `Green`, and `Blue`.
- Item quantities are limited to 4. Color channel intensity is used to track inventory.
- Diagonal movement is supported. Imagine the robot can reach any adjacent grid space in any direction.

> Other warehouse-like features can be added in the future but not implemented due to scope of project.

## License

This project is licensed under the [MIT license](LICENSE)
