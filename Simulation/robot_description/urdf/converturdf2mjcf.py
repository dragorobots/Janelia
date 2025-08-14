import mujoco

# Define the path to your main URDF file
input_filename = 'robot_description/urdf/car.urdf'

# Define the desired path for the output MJCF file
output_filename = 'robot_description/car.mjcf'

print(f"Loading model from: {input_filename}")

# Load the model from the URDF. MuJoCo performs the conversion in memory.
try:
    model = mujoco.MjModel.from_xml_path(input_filename)
except Exception as e:
    print(f"Error loading model: {e}")
    exit()

# Save the converted model as a native MJCF file
mujoco.mj_saveLastXML(output_filename, model)

print(f"Successfully converted model to: {output_filename}")