# Import the library using the alias "mi"
import mitsuba as mi
import os
import argparse


os.chdir(os.path.dirname(os.path.abspath(__file__)))

# Set the variant of the renderer
mi.set_variant('cuda_ad_rgb')


def main():
    parser = argparse.ArgumentParser(description="generate mitsuba images")
    parser.add_argument(
        "-f",
        help=f"xml file to generate from",
        default="",
    )
    args = parser.parse_args()
    file = args.f
    
    file_prefix = file[:-4]

    scene = mi.load_file(file)

    img = mi.render(scene)
    # Write the rendered image to an EXR file
    filename_exr = f'{file_prefix}.exr'
    filename_png = f'{file_prefix}.png'
    bitmap = mi.Bitmap(img)
    bitmap.write(filename_exr)
    print(f"writing to file: {filename_exr}")
    bitmap_small = bitmap.convert(mi.Bitmap.PixelFormat.RGB, mi.Struct.Type.UInt8, True)
    bitmap_small.write(filename_png)
    print(f"writing to file: {filename_png}")


if __name__ == "__main__":
    main()
