// Standard matrices
float4x4 World;
float4x4 View;
float4x4 Projection;
// Inverse of the transpose matrix - computed by fixed pipeline
float4x4 ViewIT;
// Color for the diffuse component
float4 DiffColor;
// Ambient lightining parameters
float4 AmbColor;
float AmbIntensity;
// Specular component parameters
float4 SpecColor;
float SpecPower;
float SpecIntensity;
// Light direction and eye position from the fixed pipeline
float4 LightDir;
float4 EyePos;

// Texture containing the wood
texture wt;
// Normal map texture of the stone material
texture normalMapTexture;
// Stone material texture
texture stonet;

// Texture used by perlin noise -> they are actually buffers of data we will need stored as textures
texture permutationTexture;
texture gradientTexture;

sampler2D texSampler = sampler_state {
    Texture = (wt);
    MinFilter = Linear;
    MagFilter = Linear;
    AddressU = Clamp;
    AddressV = Clamp;
};

sampler2D normalMap = sampler_state
{
    Texture = <normalMapTexture>;
    MagFilter = Linear;
    MinFilter = Linear;
	AddressU = Clamp;
	AddressV = Clamp;
};

sampler2D stoneSampler = sampler_state {
    Texture = (stonet);
    MagFilter = Linear;
    MinFilter = Linear;
	AddressU = Clamp;
	AddressV = Clamp;
};

sampler permSampler = sampler_state
{
    texture = <permutationTexture>;
    AddressU  = Wrap;       
    AddressV  = Wrap;
    MAGFILTER = POINT;
    MINFILTER = POINT;
    MIPFILTER = NONE;  
};

sampler gradSampler = sampler_state
{
    texture = <gradientTexture>;
    AddressU  = Wrap;       
    AddressV  = Clamp;
    MAGFILTER = POINT;
    MINFILTER = POINT;
    MIPFILTER = NONE;
};

// USED by perlinnoise: partial blending function. Originally was the commented one, but the 5th grade one leads to improved smoothness
float3 fade(float3 t)
{
  return t * t * t * (t * (t * 6 - 15) + 10); // new curve
//  return t * t * (3 - 2 * t); // old curve
}

// USED by perlinnoise: lookup in the permutation - A 2D TEXTURE
float4 perm2d(float2 p)
{
    return tex2D(permSampler, p);
}

// USED by perlinnoise: lookup in the gradient texture - A 1D TEXTURE
float gradperm(float x, float3 p)
{
    return dot(tex1D(gradSampler, x), p);
}

/*
	Computes a pseudo random noise float value using a perlin noise algorithm
*/

float perlnoise3D(float3 inv)
{
	// BaseP is the "base" cube containing the point from which we will interpolate
	float3 baseP = fmod(floor(inv), 256.0);
	// p holds the coordinate of the point IN the cube -> the "displacement"
    float3 p = inv - floor(inv.xyz);
	// fade returns the interpolated curves. x,y, and z displacements are interpolated given their weight -> proximity to vertices of cubes
    float3 f = fade(p);

	// We have to divide because the permutation are indexed from 0-255
    baseP = baseP / 256.0;
    // shortcut. Perlin uses it
	float one = 1.0 / 256.0;
   
	// The grad texture already encodes the 2d displacements, we just need to define the 3d one
    float4 AA = perm2d(baseP.xy) + baseP.z;
 
    // Now, we do the lookup in the texture, choosing the right gradient vector given oure cube, and we interpolate
	// we interpolate XZ -> x,z - YW -> y,w AND XZ1 -> x+1,z+1 - YW1 -> y+1,w+1. Then, we interpolate S -> XZ, YW | U-> XZ1, YW1. And then, S,U
      return lerp( lerp( lerp( gradperm(AA.x, p ), 
                             gradperm(AA.z, p + float3(-1, 0, 0) ), f.x),
                       lerp( gradperm(AA.y, p + float3(0, -1, 0) ),
                             gradperm(AA.w, p + float3(-1, -1, 0) ), f.x), f.y),
                            
                 lerp( lerp( gradperm(AA.x+one, p + float3(0, 0, -1) ),
                             gradperm(AA.z+one, p + float3(-1, 0, -1) ), f.x),
                       lerp( gradperm(AA.y+one, p + float3(0, -1, -1) ),
                             gradperm(AA.w+one, p + float3(-1, -1, -1) ), f.x), f.y), f.z);

}

// Our input/output structures

struct VertexShaderInput
{
    float4 Position : POSITION0;
	float4 Normal: NORMAL0;

    // TODO: add input channels such as texture
    // coordinates and vertex colors here.
};

struct VertexShaderOutput
{
    float4 Position : POSITION0;
	float4 NormalFake: TEXCOORD0;
	float4 AdditionalData: TEXCOORD1;

    // TODO: add vertex shader outputs such as colors and texture
    // coordinates here. These values will automatically be interpolated
    // over the triangle, and provided as input to your pixel shader.
};

struct VertexShaderOutput2
{
    float4 Position : POSITION0;
	float4 NormalFake: TEXCOORD0;
	float4 LD: TEXCOORD1;
	float4 VertexPos: TEXCOORD2;

    // TODO: add vertex shader outputs such as colors and texture
    // coordinates here. These values will automatically be interpolated
    // over the triangle, and provided as input to your pixel shader.
};

struct TexVertexShaderInput
{
    float4 Position : POSITION0;
	float4 Normal: NORMAL;
	float4 Tex: TEXCOORD0;

    // TODO: add input channels such as texture
    // coordinates and vertex colors here.
};

struct TexVertexShaderOutput
{
    float4 Position : POSITION0;
	float4 Tex: TEXCOORD0;

    // TODO: add vertex shader outputs such as colors and texture
    // coordinates here. These values will automatically be interpolated
    // over the triangle, and provided as input to your pixel shader.
};

struct BMVertexShaderInput
{
	float4 Position: POSITION0;
	float3 Normal: NORMAL;
	float2 Texc: TEXCOORD0;
	float4 Tangent: TANGENT;
};

struct BMVertexShaderOutput
{
	float4 Position : POSITION0;
	float2 Texc: TEXCOORD0;
	float3 Norm: TEXCOORD1;
	float3 Tan: TEXCOORD2;
	float3 Binorm : TEXCOORD3;
	float4 Worldpos : TEXCOORD4;
};

/*
	The following vertex shader computes the correct position given the transformation matrices and outputs it
	together with the normal, which is stored in the output as a texturecoord
*/

VertexShaderOutput SimpleVertexShaderFunction(VertexShaderInput input)
{
    VertexShaderOutput output = (VertexShaderOutput) 0;

    float4 worldPosition = mul(input.Position, World);
    float4 viewPosition = mul(worldPosition, View);
    output.Position = mul(viewPosition, Projection);

	// Texturecoord can be declared as a float4.Perfect for storing a Normal
	// The output normal is multiplied by the inverse of the transpose of the ModelView Matrix for ensuring correctness in case of scaling
	output.NormalFake = mul(input.Normal, ViewIT);

    return output;
}

/*
	The following pixel shader outputs a color depending on the Normal, taken as input.
	Normals lie in the [-1, 1] range while colors in the [0,1]: to arrange a continuous map we first multiply by 1/2 and then we add 1/2
*/

float4 SimplePixelShaderFunction(VertexShaderOutput input) : COLOR0
{

    return float4( (input.NormalFake.x * 0.5) + 0.5, (input.NormalFake.y * 0.5) + 0.5, (input.NormalFake.z * 0.5) + 0.5, 1);
}

// Lambertian shaders functions

/*
	The following vertex shader does the preliminary vertex calculations for the lambertian shader.
	The used light is a DIRECTIONAL LIGHT.
	
*/

VertexShaderOutput LambVertexShaderFunction(VertexShaderInput input)
{
    VertexShaderOutput output = (VertexShaderOutput) 0;

	// Get correct vertexpos
    float4 wPosition = mul(input.Position, World);
	float4 wvPosition = mul(wPosition, View);
	output.Position = mul(wvPosition, Projection);
	
	// Get correct lightdir
	float4 l = (-1) * normalize(LightDir);

	// Correctly update the normal to pass
	float4 tnorm = mul(input.Normal, ViewIT);
	tnorm.w = 0;

	// NormalFake will hold the real normal
	output.NormalFake = tnorm;

	// AdditionalData will hold the right light direction
	output.AdditionalData = l;

    return output;
}

/*
	The following pixel shader computes the color referring to the lambertian lighting model. Thus, we will just have a diffuse component
	proportional to the dot product between the light direction and the normal direction.
	
*/

float4 LambPixelShaderFunction(VertexShaderOutput input) : COLOR0
{
	// Compute the dot product. Saturate clamps the value between 0 and 1, taking care of the negative normals (normals range from -1 to 1)
	// Normal is normalized because at this point we just care about the direction of the normal
	float4 diffcomp = saturate(dot( normalize(input.AdditionalData), normalize(input.NormalFake)));
	// Color is the diffcomp multiplied by the diffuse color itself
	return (DiffColor * diffcomp);
}

/*
	The following pixel shader compute the color referring to the lambertian lighting model and adding an ambient color component.
	The ambient color and color intensity are passed from the fixed pipeline as values.
*/

float4 LambAmbPixelShaderFunction(VertexShaderOutput input) : COLOR0
{
	// Compute the dot product
	float4 diffcomp = saturate(dot( normalize(input.AdditionalData), normalize(input.NormalFake))) * DiffColor;
	float4 ambc = AmbColor * AmbIntensity;

	return (diffcomp + ambc);

	//float4 col = float4(max(ambc.x, diffc.x), max(ambc.y, diffc.y), max(ambc.z, diffc.z), max(ambc.w, diffc.w) );
	//return max(ambc, diffc);
}

// Phong shader functions

/*
	The following vertex and pixel shaders perform Phong-Blinn shading.
	Phong-Blinn adds a specular component, thus we have to compute and pass the position of "eye" because it will influence the specular reflection.
	Blinn variation computes the vector somewhat half-way between L and V.
	Eye here will hold the POSITION of the vertex that will be used for calculating the view vector.
*/

VertexShaderOutput2 PhongBVertexShaderFunction(VertexShaderInput input)
{
    VertexShaderOutput2 output = (VertexShaderOutput2) 0;

	// Get correct vertexpos
    float4 wPosition = mul(input.Position, World);
	float4 wvPosition = mul(wPosition, View);
	output.Position = mul(wvPosition, Projection);
	
	// Get correct lightdir
	float4 l = (-1) * normalize(LightDir);

	output.LD = l;

	// Get the correct eye Position
	output.VertexPos = mul(input.Position, World);

	// Correctly update the normals
	float4 tnorm = mul(input.Normal, ViewIT);
	tnorm.w = 0;
	tnorm = normalize(tnorm);

	// NormalFake will hold the real normal
	output.NormalFake = tnorm;

    return output;
}

/*
	Phong-Blinn Pixel shader. We did not compute explicitely the light and view direction before: we will do it here, per vertex, otherwise they would have had interpolated
	after the vertex shader.
*/

float4 PhongBPixelShaderFunction(VertexShaderOutput2 input) : COLOR0
{

	float4 norm = (input.NormalFake);
	// We get the LightDirection
	float4 l = (-1) * normalize(LightDir);
	// We normalize eye vector
	float4 ep = normalize(EyePos);
	// And compute the view vector
	float4 v = normalize(input.VertexPos - ep);

	// The needed half-vector
	float4 halfvector = normalize(l + v);

	// We can calculate a specular and diffuse component
	float4 speccomp = SpecIntensity * pow(saturate(dot(halfvector, norm)), SpecPower);
	float4 diffcomp = saturate(dot(l, norm));

	return (AmbColor * AmbIntensity + DiffColor * diffcomp + SpecColor * speccomp);
}

/*
	The two following functions perform a simple UV 2D texture mapping on a surface
*/

TexVertexShaderOutput TexVertexShaderFunction(TexVertexShaderInput input)
{
    TexVertexShaderOutput output = (TexVertexShaderOutput) 0;
	
	// We calculate the right position 
    float4 worldPosition = mul(input.Position, World);
    float4 viewPosition = mul(worldPosition, View);
    output.Position = mul(viewPosition, Projection);
	// And we pass texturecoord to the pixel shader
	output.Tex = input.Tex;

    return output;
}

float4 TexPixelShaderFunction(TexVertexShaderOutput input) : COLOR0
{
    float4 output;
	// Here, we just need to query the sampler for a texture color
    output = tex2D(texSampler, input.Tex);
    output.a = 1.0;

	return output;
}

/*
	The following functions perform a procedural texture shading -> They don't read value from an external image but instead compute values of the fragments directly.
	In particular, the next functions apply a sort of checkerboard effect to the model: pixel are colored using the normals or the inverse normals alternatively.
*/

VertexShaderOutput ProcVertexShaderFunction(VertexShaderInput input)
{
	VertexShaderOutput output = (VertexShaderOutput) 0;

	// We compute the right position
	float4 worldPosition = mul(input.Position, World);
    float4 vpos = mul(worldPosition, View);
	output.Position = mul(vpos, Projection);
	// Moreover, we need to use this position in the pixel shader, but we normally can't use the standard output position.
	// Therefore, we save it in an additional variable and pass it
	output.AdditionalData = output.Position;
	
	output.NormalFake = mul(input.Normal, ViewIT);
	output.NormalFake.w = 0;

	return output;
}

float4 ProcPixelShaderFunction(VertexShaderOutput input) : COLOR0
{	
	// The test is simple: we use the absolute sum of the floored x/y to obtain the right "square"
	float tot;
	tot = abs(floor(input.AdditionalData.x)) + abs(floor(input.AdditionalData.y));
	tot = fmod(tot, 2.0);
	//tot = fmod(input.AdditionalData.x + input.AdditionalData.y, 1.0);

	// We color at the same way all the squares [pixels] at even positions
	if (tot < 1.0) return float4( (input.NormalFake.x * 0.5) + 0.5, (input.NormalFake.y * 0.5) + 0.5, (input.NormalFake.z * 0.5) + 0.5, 1.0);
	else return float4( (-input.NormalFake.x * 0.5) + 0.5, (-input.NormalFake.y * 0.5) + 0.5, (-input.NormalFake.z * 0.5) + 0.5, 1.0);

}

/*
	The following functions will be used to attempt a bump mapping.
	We perform bump-mapping by reading the values to use to offset the normals from a normal map.
	To keep everything correct, we need to transform everything in Tangent space, thus we need the transformation Matrix.
	If we have a correct Normal, Tangent and Bitangent, the transformation Matrix is a 3x3 Matrix T|B|N:
		| Tx Bx Nx |
		| Ty By Ny |
		| Tz Bz Nz |
	If we have the Normal and the Tangent encoded correctly, a Bitangent is N x T
*/

BMVertexShaderOutput BMVertexShaderFunction(BMVertexShaderInput input)
{
	BMVertexShaderOutput output = (BMVertexShaderOutput) 0;

	float4 worldPosition = mul(input.Position, World);
	output.Worldpos = worldPosition;

    float4 vpos = mul(worldPosition, View);
	output.Position = mul(vpos, Projection);

	// We pass the texture coordinate, used for both the color and the normal texture
	output.Texc = input.Texc;
	
	// Both normal and tangent must be corrected using the inverse transpose
	float3 t = mul(input.Tangent, (float3x3) ViewIT);
	float3 n = mul(input.Normal, (float3x3) ViewIT);
	// We compute the bitangent
	float3 b = cross(n, t) * input.Tangent.w;

	output.Norm = n;
	output.Tan = t;
	output.Binorm = b;

	return output;
}

float4 BMPixelShaderFunction(BMVertexShaderOutput input) : COLOR0
{
	float4 ep = mul(EyePos, World);

	// Get View Direction
	float4 vd = normalize(ep - input.Worldpos);

	// Get Light Direction
	float4 ld = (-1) * normalize(LightDir);

	// Calculate the Matrix for TBN space
	float3x3 TBN = {normalize(input.Tan), normalize(input.Binorm), normalize(input.Norm)}; 

	// Bump value extrapolation
	float3 tN = normalize(tex2D(normalMap, input.Texc) * 2 - 1);
	
	// Adjust the vectors to TBN space
	float3 ld3 = mul(float3(ld.x, ld.y, ld.z), TBN);
	float3 vd3 = mul(float3(vd.x, vd.y, vd.z), TBN);

	// Compute the halfvector
	float3 halfvector = normalize(vd3 + ld3);

	// Compute light components
	float3 speccomp = SpecIntensity * pow(max(dot(halfvector, tN), 0.00001f), SpecPower);
	float3 diffcomp = max(dot(tN, ld3), 0.0);
	float4 color = float4(AmbColor * AmbIntensity + DiffColor * diffcomp + SpecColor * speccomp, 1.0);
	// Then we add the texture color valute to our compute lighting color
	return (color * tex2D(stoneSampler, input.Texc) );

    //return float4(1, 0, 0, 1);
}

/*
	The following functions will peform Bump Mapping using perlin noise.
	Perlin Noise is based on 2 "hashtables", one used to permute the "cube" position of the remapped space, and one defining the
	gradient vectors used to add noise to the value belonging to one of the cubes.
	Here, the hashtables are passed as textures because we thought it was the best ways to pass buffers of data.
	Moreover, the permutation texture is a 2D texture encoding the X/Y lookup already, as suggested by the article on GPU Perlin noise
	published in GPU Gems 2. We didn't manage to have a 3D texture instad, which would have been even faster.

*/

VertexShaderOutput PNVertexShaderFunction(VertexShaderInput input)
{
    VertexShaderOutput output = (VertexShaderOutput) 0;

    float4 worldPosition = mul(input.Position, World);
    float4 viewPosition = mul(worldPosition, View);
    output.Position = mul(viewPosition, Projection);
	output.NormalFake = mul(input.Normal, ViewIT);

    // TODO: add your vertex shader code here.

    return output;
}

float4 PNPixelShaderFunction(VertexShaderOutput2 input) : COLOR0
{

	//float4 norm = (input.NormalFake);
	float4 l = (-1) * normalize(LightDir);

	float4 ep = normalize(EyePos);
	float4 v = normalize(input.VertexPos - ep);

	// Now, we perturbate the normal depending on position data using the perlin noise function.
	// The perturbation is performed using the gradient of the generated perlin noise.
	// We compute the gradient by taking the value in the position, then we offset the position in the X,Y and Z direction computing again
	// more noise values. In the end, we make 3 differences and divide by the "step"

	float f0 = perlnoise3D(input.VertexPos) * 0.5 + 0.5;
	
	float ds = 0.001f;
	float3 dx = input.VertexPos;
	dx.x += ds;
	float3 dy = input.VertexPos;
	dy.y += ds;
	float3 dz = input.VertexPos;
	dz.z += ds;

	float3 d = float3(perlnoise3D(dx) * 0.5 + 0.5, perlnoise3D(dy) * 0.5 + 0.5, perlnoise3D(dz) * 0.5 + 0.5);
	// distn is used to perturbate the normal
	float3 distn = float3( (d.x - f0) / ds, (d.y - f0) / ds, (d.z - f0) / ds); 
	// Then, we normalize it.
	float3 norm = normalize(input.NormalFake - distn);	

	// Our standerd specular and diffuse components
	float4 halfvector = normalize(l + v);
	float4 speccomp = SpecIntensity * pow(saturate(dot(halfvector, norm)), SpecPower);
	float4 diffcomp = saturate(dot(l, norm));

	return (AmbColor * AmbIntensity + DiffColor * diffcomp + SpecColor * speccomp);
}

/*
	The following are the real declarations of the shaders. Not much to say
*/

// The perlin noise shader has not Vertex shader, because it uses the standard Phong-Blinn VS

technique PerlinNoiseTechnique
{
	pass Pass1
	{
		// NOTE: Pixel/Verex shader model 2.0 was not enough to make the shader work because it exceeded the maximum allowed number of micro shader instructions
		// Hence, we are using 3.0

		VertexShader = compile vs_3_0 PhongBVertexShaderFunction();
		PixelShader = compile ps_3_0 PNPixelShaderFunction();	
	}
}

technique BumpMapTexturingTechnique
{
	pass Pass1
	{
        VertexShader = compile vs_2_0 BMVertexShaderFunction();
        PixelShader = compile ps_2_0 BMPixelShaderFunction();	
	}
}

technique ProcTexturingTechnique
{
    pass Pass1
    {
        VertexShader = compile vs_2_0 ProcVertexShaderFunction();
        PixelShader = compile ps_2_0 ProcPixelShaderFunction();
    }
}

technique TexturingTechnique
{
    pass Pass1
    {
        VertexShader = compile vs_2_0 TexVertexShaderFunction();
        PixelShader = compile ps_2_0 TexPixelShaderFunction();
    }
}

technique PhongBTechnique
{
    pass Pass1
    {
        // TODO: set renderstates here.

        VertexShader = compile vs_2_0 PhongBVertexShaderFunction();
        PixelShader = compile ps_2_0 PhongBPixelShaderFunction();
    }
}

technique LambTechnique
{
    pass Pass1
    {
        // TODO: set renderstates here.

        VertexShader = compile vs_2_0 LambVertexShaderFunction();
        //PixelShader = compile ps_2_0 LambPixelShaderFunction();
		PixelShader = compile ps_2_0 LambAmbPixelShaderFunction();
    }
}

technique SimpleTechnique
{
    pass Pass1
    {
        // TODO: set renderstates here.

        VertexShader = compile vs_2_0 SimpleVertexShaderFunction();
        PixelShader = compile ps_2_0 SimplePixelShaderFunction();
    }
}