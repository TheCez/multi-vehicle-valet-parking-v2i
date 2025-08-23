import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from PIL import Image
import io

# Function to create each frame of the plot
def create_plot(i):
    plt.clf()  # Clear the current figure
    x = np.linspace(0, 10, 1000)
    y = np.sin((i + 1) * x)
    plt.plot(x, y)
    plt.title(f"Sine Wave (Frequency: {i+1})")
    plt.xlim(0, 10)
    plt.ylim(-1.5, 1.5)

# Create a figure for plotting
fig = plt.figure(figsize=(10, 6))

# List to store frames
frames = []

# Generate frames for the animation
for i in range(10):
    create_plot(i)
    buf = io.BytesIO()  # Create an in-memory buffer
    plt.savefig(buf, format='png')  # Save the current figure to the buffer as a PNG image
    buf.seek(0)  # Rewind the buffer to the beginning
    frames.append(Image.open(buf))  # Append the image to the frames list

# Save frames as an animated GIF using Pillow
frames[0].save(
    'sine_wave_animation.gif',
    save_all=True,
    append_images=frames[1:],  # Add all frames except the first one
    duration=200,  # Duration of each frame in milliseconds
    loop=0  # Loop indefinitely (0 means infinite loop)
)

print("Animation saved as 'sine_wave_animation.gif'")