# Import the library using the alias "mi"
import mitsuba as mi
import os

os.chdir(os.path.dirname(os.path.abspath(__file__)))

# Set the variant of the renderer
mi.set_variant('cuda_ad_rgb')
# Load a scene

scene = mi.load_file("cbox.xml")

params = mi.traverse(scene)

albedo = [
    [0.0, 0.0, 0.0],
    [1.0, 1.0, 1.0],
    [0.75, 0.75, 0.75],
    [0.75, 0.75, 0.75],
    [0.5, 0.5, 0.5]
]

sigma_t = [0.5, 0.5, 0.5, 20.0, 1.0]

for i in range(len(albedo)):
    params["shape.interior_medium.albedo.value.value"] = albedo[i]
    params["shape.interior_medium.sigma_t.value.value"] = sigma_t[i]
    params.update()

    # Render the scene
    img = mi.render(scene)
    # Write the rendered image to an EXR file
    filename = f'cbox_{albedo[i][0]}_{sigma_t[i]}.exr'
    mi.Bitmap(img).write(filename)
    print(f"writing to file: {filename}")