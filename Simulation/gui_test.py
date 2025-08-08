import tkinter as tk
from tkinter import filedialog
from PIL import Image, ImageTk

# Define the real-world dimensions of the floor plan in meters
REAL_WIDTH_M = 3.5
REAL_HEIGHT_M = 5

# Define the pixel size of the canvas to visualize the space
CANVAS_WIDTH = 300  # 500 pixels for 5 meters
CANVAS_HEIGHT = 500 # 300 pixels for 3 meters

# Calculate the scaling factor (pixels per meter)
PIXELS_PER_METER = CANVAS_WIDTH / REAL_WIDTH_M

# Global variables
background_image = None
wall_coordinates_m = []
start_point_m = None # This will store the first clicked point

def create_grid(canvas):
    """Draws a grid on the canvas to represent meters."""
    # Draw vertical lines every 1 meter
    for i in range(0, CANVAS_WIDTH + 1, int(PIXELS_PER_METER)):
        canvas.create_line(i, 0, i, CANVAS_HEIGHT, fill="gray")
    # Draw horizontal lines every 1 meter
    for i in range(0, CANVAS_HEIGHT + 1, int(PIXELS_PER_METER)):
        canvas.create_line(0, i, CANVAS_WIDTH, i, fill="gray")

def draw_wall(event):
    """Handles mouse clicks to draw a wall between two points."""
    global start_point_m
    
    # Convert pixel coordinates to meters
    x_meter = round(event.x / PIXELS_PER_METER, 2)
    y_meter = round(event.y / PIXELS_PER_METER, 2)
    
    if start_point_m is None:
        # First click: store the starting point and draw a visual indicator
        start_point_m = (x_meter, y_meter)
        
        # Draw a small circle to indicate the start of the wall
        start_x_px = event.x - 3
        start_y_px = event.y - 3
        end_x_px = event.x + 3
        end_y_px = event.y + 3
        canvas.create_oval(start_x_px, start_y_px, end_x_px, end_y_px, fill="red", outline="red", tags="start_marker")
        
    else:
        # Second click: store the end point, draw the line, and reset
        end_point_m = (x_meter, y_meter)
        
        # Draw the line on the canvas
        start_x_px = start_point_m[0] * PIXELS_PER_METER
        start_y_px = start_point_m[1] * PIXELS_PER_METER
        end_x_px = end_point_m[0] * PIXELS_PER_METER
        end_y_px = end_point_m[1] * PIXELS_PER_METER
        canvas.create_line(start_x_px, start_y_px, end_x_px, end_y_px, fill="black", width=2)
        
        # Store the wall as a pair of coordinates
        wall_coordinates_m.append((start_point_m, end_point_m))
        
        # Reset the start point and remove the marker for the next wall
        start_point_m = None
        canvas.delete("start_marker")

def output_coordinates():
    """Prints the stored paired coordinates to the console."""
    print("Wall Coordinates (meters):")
    for start, end in wall_coordinates_m:
        print(f"Start: ({start[0]} m, {start[1]} m), End: ({end[0]} m, {end[1]} m)")

def load_image():
    """Opens a file dialog to select and load an image."""
    global background_image
    file_path = filedialog.askopenfilename(
        title="Select an image file",
        filetypes=[("Image files", "*.png;*.jpg;*.jpeg;*.gif")]
    )
    if not file_path:
        return

    # Open and resize the image to fit the canvas
    original_image = Image.open(file_path)
    resized_image = original_image.resize((CANVAS_WIDTH, CANVAS_HEIGHT), Image.Resampling.LANCZOS)
    background_image = ImageTk.PhotoImage(resized_image)
    
    # Clear the canvas and display the image first
    canvas.delete("all")
    canvas.create_image(0, 0, anchor=tk.NW, image=background_image)
    
    # Redraw the grid on top of the image
    create_grid(canvas)

# Create the main application window
root = tk.Tk()
root.title("Maze Builder (Meters)")

# Create a canvas
canvas = tk.Canvas(root, width=CANVAS_WIDTH, height=CANVAS_HEIGHT, bg="white")
canvas.pack()

# Create buttons in a frame
button_frame = tk.Frame(root)
button_frame.pack()

load_button = tk.Button(button_frame, text="Load Image", command=load_image)
load_button.pack(side=tk.LEFT, padx=5, pady=5)

output_button = tk.Button(button_frame, text="Output Coordinates (Meters)", command=output_coordinates)
output_button.pack(side=tk.LEFT, padx=5, pady=5)

# Draw the initial grid
create_grid(canvas)

# Bind the left mouse button click event to the draw_wall function
canvas.bind("<Button-1>", draw_wall)

# Start the event loop
root.mainloop()