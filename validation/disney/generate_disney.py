# Import the library using the alias "mi"
import mitsuba as mi
import os
import argparse
import pathlib
import numpy as np

BASE_DIR = pathlib.Path(__file__).absolute().parent.parent.parent
os.chdir(BASE_DIR / "validation/disney")
filename = "mitsuba_disney"

# Set the variant of the renderer
mi.set_variant('cuda_ad_rgb')

scene_dict = {
    "type" : "scene",
    "myintegrator" : {
        "type" : "path",
    },
    "mysensor" : {
        "type" : "perspective",
        "near_clip": 1.0,
        "far_clip": 1000.0,
        "to_world" : mi.ScalarTransform4f.look_at(origin=[0, 1, 13],
                                               target=[0, 0.893051, 4.41198],
                                               up=[0, 1, 0]),
        "myfilm" : {
            "type" : "hdrfilm",
            "rfilter" : { "type" : "gaussian"},
            "pixel_format" : "rgb",
            "component_format" : "float32",
            "width" : 2048,
            "height" : 512,
        },
        "mysampler" : {
            "type" : "independent",
            "sample_count" : 512,
        },
    },
    "walls" : {
        "type" : "obj",
        "filename" : "../meshes/walls.obj",
        "to_world" : mi.ScalarTransform4f.scale([100, 10, 20]),
        "bsdf" : {
            "type" : "diffuse",
            "reflectance" : {
                "type" : "rgb",
                "value" : [0.725, 0.71, 0.68],
            }
        }
    },
    "emitter" : {
        "type" : "sphere",
        "center" : [0, 50, 100],
        "radius" : 20,
        "emitter" : {
            "type" : "area",
            "radiance" : {
                "type" : "rgb",
                "value" : [30, 30, 30],
            }
        }
    }
}


def main():
    properties = ["roughness", "specular", "metallic", "anisotropic", "flatness"]
    for prop in properties:
        for i in range(11):
            scene_dict[f"sphere_{i}"] = {
                "type" : "sphere",
                "center" : [i - 5, 0.5, 0],
                "radius" : 0.25,
                "bsdf" : {
                    "type" : "principled",
                    "base_color" : {
                        "type" : "rgb",
                        "value" : [0.8, 0.65, 0.0]
                    },
                    "roughness": 0.5,
                    "specular" : 0.5,
                    "metallic" : 0.0,
                    "anisotropic" : 0.0,
                    "flatness" : 0.0
                }
            }

        for i in range(11):
            scene_dict[f"sphere_{i}"]["bsdf"][prop] = i / 10.0

        
        scene = mi.load_dict(scene_dict)

        img = mi.render(scene)
        # Write the rendered image to an EXR file
        filename_exr = f'{filename}_{prop}.exr'
        filename_png = f'{filename}_{prop}.png'
        bitmap = mi.Bitmap(img)
        bitmap.write(filename_exr)
        print(f"writing to file: {filename_exr}")
        bitmap_small = bitmap.convert(mi.Bitmap.PixelFormat.RGB, mi.Struct.Type.UInt8, True)
        bitmap_small.write(filename_png)
        print(f"writing to file: {filename_png}")


if __name__ == "__main__":
    main()
