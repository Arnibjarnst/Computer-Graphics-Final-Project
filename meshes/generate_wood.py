from random import random

instances = ""
emitters = ""
for i in range(-30, 30, 5):
    for j in range(-30, 30, 5):
        instances += f"""
<instance type="instance">
    <integer name="subscene" value="1"/>
    <transform name="toWorld">
        <translate value="{i + random()}, {j + random()}, 0"/>
        <rotate angle="{int(random() * 20)}" axis="0,0,1" />
    </transform>
</instance>"""
        
for i in range(-25, 25, 10):
    for j in range(-25, 25, 10):
        emitters += f"""
    <mesh type="sphere">
        <point name="center" value="{i + random()},{j + random()},1"/>
        <float name="radius" value="0.05"/>
        <bsdf type="diffuse">
            <color name="albedo" value="1,1,1"/>
        </bsdf>

        <emitter type="area">
            <color name="radiance" value="3000,2000,0"/>
        </emitter>
    </mesh>"""
        

    
with open('./wood_scene.xml', 'w') as f:
    f.write("""
<!-- Table scene, Copyright (c) 2012 by Olesya Jakob -->

<scene>
	<!-- Independent sample generator, 512 samples per pixel -->
	<sampler type="independent">
		<integer name="sampleCount" value="128"/>
	</sampler>

	<!-- Use the path tracer without multiple importance sampling -->
	<integrator type="direct_mis">
	</integrator>

	<!-- Render the scene as viewed by a perspective camera -->
	<camera type="perspective">
		<transform name="toWorld">
			<lookat target="31.6866, -67.2776, 36.1392" 
				origin="32.1259, -68.0505, 36.597" 
				up="-0.22886, 0.39656, 0.889024"/>
		</transform>

		<!-- Field of view: 35 degrees -->
		<float name="fov" value="35"/>

		<!-- 800x600 pixels -->
		<integer name="width" value="800"/>
		<integer name="height" value="600"/>
	</camera>

	<!-- Two light sources  -->
    <mesh type="obj">
		<string name="filename" value="scenes/pa4/table/meshes/mesh_1.obj"/>

		<emitter type="area">
			<color name="radiance" value="0, .5, .5"/>
		</emitter>

		<bsdf type="diffuse">
			<color name="albedo" value="0,0,0"/>
		</bsdf>


		<transform name="toWorld">
			<scale value="0.06,0.06,-1"/>
			<translate value="10,0,25"/>
		</transform>
	</mesh>
""" + emitters + """
	

	<subscene type="subscene">
		<integer name="id" value="1"/>
		<mesh type="obj">
			<string name="filename" value="../abjarnsteins_gfigini/meshes/meshes/Tree.038.obj"/>

			<bsdf type="microfacet">
				<color name="kd" value="0, 0.5, 0"/>
			</bsdf>
			<transform name="toWorld">
				<translate value="0,0,5"/>
			</transform>
		</mesh>
	</subscene>
""" + instances + """
    <!-- Diffuse floor -->
	<mesh type="obj">
		<string name="filename" value="./scenes/pa4/table/meshes/mesh_1.obj"/>

		<bsdf type="diffuse">
			<color name="albedo" value=".5,.5,.5"/>
		</bsdf>

		<transform name="toWorld">
			<scale value="0.2,0.35,0.5"/>
			<translate value="-35,25,0"/>
		</transform>

	</mesh>

	
</scene>
""")