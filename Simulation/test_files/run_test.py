import mujoco
import mujoco.viewer

print("Attempting to load a minimal model...")

# Load the model
try:
    model = mujoco.MjModel.from_xml_path('test_model.xml')
    data = mujoco.MjData(model)
except Exception as e:
    print(f"Error loading model: {e}")
    exit()

print("Model loaded successfully.")
print("Launching viewer. You should see a red ball fall onto a plane.")
print("Close the window to exit.")

# Launch the viewer, which runs its own simulation loop
mujoco.viewer.launch(model, data)