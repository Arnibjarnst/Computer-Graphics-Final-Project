# Import the library using the alias "mi"
import mitsuba as mi
import argparse
import os
import subprocess
import pathlib
import xml.etree.ElementTree as ET
import numpy as np


BASE_DIR = pathlib.Path(__file__).absolute().parent.parent.parent
os.chdir(BASE_DIR / "validation/homogeneous")
mitsuba_file_name = "mitsuba_box"
mitsuba_dir = BASE_DIR / "validation/homogeneous" / f"{mitsuba_file_name}.xml"
nori_file_name = "nori_box_mis"
nori_dir = BASE_DIR / "validation/homogeneous" / f"{nori_file_name}.xml"


# Set the variant of the renderer
mi.set_variant('cuda_ad_rgb')
# Load a scene

albedo = np.array([
    [0.0, 0.0, 0.0],
    [1.0, 1.0, 1.0],
    [0.75, 0.75, 0.75],
    [0.75, 0.75, 0.75],
    [0.5, 0.5, 0.5]
])

sigma_t = np.array([1.0, 1.0, 1.0, 20.0, 1.0])


def main():
    DEFAULT_BUILD_DIR = BASE_DIR / "out/build/x64-RelWithDebInfo/RelWithDebInfo"
    parser = argparse.ArgumentParser(description="Run Nori tests")
    parser.add_argument(
        "--build_dir",
        help=f"Path to Nori build directory (containing `nori` and `warptest` executables). Default: {DEFAULT_BUILD_DIR}",
        default=DEFAULT_BUILD_DIR,
    )

    args = parser.parse_args()

    build_dir = pathlib.Path(args.build_dir)
    if not build_dir.exists():
        raise RuntimeError(f"Build directory {build_dir} does not exist")

    ext = ".exe" if isinstance(build_dir, pathlib.WindowsPath) else ""
    nori_exe = build_dir / f"nori{ext}"

    if not nori_exe.exists():
        raise RuntimeError(f"{nori_exe} does not exist")


    tree = ET.parse(str(nori_dir))
    root = tree.getroot()
    sigma_a = root[6][3][0].attrib
    sigma_s = root[6][3][1].attrib

    scene = mi.load_file(str(mitsuba_dir))
    mi_params = mi.traverse(scene)

    for i in range(len(albedo)):
        ss = albedo[i] * sigma_t[i]

        sigma_s["value"] = " ".join(np.char.mod('%f', ss))
        sigma_a["value"] = " ".join(np.char.mod('%f', -(ss - sigma_t[i])))

        fileending = f"albedo={albedo[i][0]}_t={sigma_t[i]}"
        nori_xml = f"cbox_mis_{fileending}.xml"
        with open(nori_xml, "wb") as f:
            f.write(ET.tostring(root))
        
        subprocess.run([str(nori_exe), "-b", str(nori_xml)], capture_output=False)

        os.remove(nori_xml)

        mi_params["shape.interior_medium.albedo.value.value"] = albedo[i]
        mi_params["shape.interior_medium.sigma_t.value.value"] = sigma_t[i]
        mi_params.update()

        # Render the scene
        img = mi.render(scene)
        # Write the rendered image to an EXR file
        filename = f'cbox_mitsuba_{fileending}.exr'
        mi.Bitmap(img).write(filename)
        print(f"writing to file: {filename}")


if __name__ == "__main__":
    main()
