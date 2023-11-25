import time
import matplotlib.pyplot as plt

# Constants
acceleration = 9.81  # acceleration due to gravity in m/s^2
mass = 1.0           # mass of the object in kg
original_position = 0.0
original_velocity = 0.0

# Variables
position = original_position
velocity = original_velocity

# Time step (adjust as needed)
dt = 0.1

# Lists to store position and time values for plotting
positions = [original_position]
times = [0]

# Set up the plot
fig, ax = plt.subplots()
plt.ion()  # Enable interactive mode
plt.show()
ax.set_title("Gravity Simulation")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Position (m)")
line, = ax.plot(times, positions, label="Position")

# Simulation loop
try:
    while True:
        # Verlet integration method
        position_next = 2 * position - original_position + acceleration * dt**2
        velocity = (position_next - original_position) / (2 * dt)

        # Update position and reset original_position for the next iteration
        original_position, position = position, position_next

        # Append current position and time values for plotting
        positions.append(position)
        times.append(times[-1] + dt)

        # Update the plot data
        line.set_xdata(times)
        line.set_ydata(positions)

        # Adjust plot limits for better visualization
        ax.set_xlim(min(times), max(times))
        ax.set_ylim(min(positions), max(positions) + 1)

        # Redraw the plot
        fig.canvas.draw()
        fig.canvas.flush_events()

        print("Time: {:.1f} s, Position: {:.2f} m".format(times[-1], positions[-1]))

        # Sleep for a short interval to control simulation speed
        time.sleep(dt)

except KeyboardInterrupt:
    print("\nSimulation terminated by user.")
    plt.ioff()  # Disable interactive mode
    plt.show()  # Display the final plot
