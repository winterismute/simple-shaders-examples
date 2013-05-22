using System;
using System.Collections.Generic;
using System.Linq;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Audio;
using Microsoft.Xna.Framework.Content;
using Microsoft.Xna.Framework.GamerServices;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;
using Microsoft.Xna.Framework.Media;
using Microsoft.Xna.Framework.Graphics.PackedVector;

namespace GraphicsPractical2
{
    /// <summary>
    /// This is the main type for your game
    /// </summary>
    public class Game1 : Microsoft.Xna.Framework.Game
    {
        GraphicsDeviceManager graphics;
        SpriteBatch spriteBatch;

        // Defining a graphicsDevice
        GraphicsDevice device;

        Camera camera;

        // This keeps track of the framerate
        FrameRateCounter frameRateCounter;

        // Stores our model
        Model teapotModel;

        // Effect
        Effect effect;

        // Material and Light Position
        Material surfaceMat;
        Vector4 lightDir;

        // Mouse State to compute the difference of position
        MouseState previousState;

        // The surface on which the teapot will stand
        VertexPositionNormalTextureTangent[] tableVertices;

        // Array with the indices of the table model
        short[] tableIndices;

        // Table texture - wood
        Texture2D woodTexture;
        // Normal Map texture - stone
        Texture2D nmTexture;
        // Stone texture
        Texture2D stoneTexture;

        // Perlin Noise textures
        Texture2D gradTexture;
        Texture2D permTexture;

        static float[,] gradients =
        {
            {1,1,0},
            {-1,1,0},
            {1,-1,0},
            {-1,-1,0},
            {1,0,1},
            {-1,0,1},
            {1,0,-1},
            {-1,0,-1},
            {0,1,1},
            {0,-1,1},
            {0,1,-1},
            {0,-1,-1},
            {1,1,0},
            {0,-1,1},
            {-1,1,0},
            {0,-1,-1}
        };

        public Game1()
        {
            graphics = new GraphicsDeviceManager(this);
            Content.RootDirectory = "Content";

            // Construct the FrameRateCounter
            frameRateCounter = new FrameRateCounter(this);
            Components.Add(frameRateCounter);

        }

        /// <summary>
        /// Allows the game to perform any initialization it needs to before starting to run.
        /// This is where it can query for any required services and load any non-graphic
        /// related content.  Calling base.Initialize will enumerate through any components
        /// and initialize them as well.
        /// </summary>
        protected override void Initialize()
        {
            // Initialization logic here
            graphics.PreferredBackBufferWidth = 800;
            graphics.PreferredBackBufferHeight = 600;
            graphics.IsFullScreen = false;
            graphics.SynchronizeWithVerticalRetrace = false;
            graphics.ApplyChanges();
            this.IsFixedTimeStep = false;

            base.Initialize();
        }

        /// <summary>
        /// LoadContent will be called once per game and is the place to load
        /// all of your content.
        /// </summary>
        protected override void LoadContent()
        {
            // Create a new SpriteBatch, which can be used to draw textures.
            spriteBatch = new SpriteBatch(GraphicsDevice);

            device = graphics.GraphicsDevice;

            //camera = new Camera(new Vector3(0, 0, 0), new Vector3(0, 50, 100), new Vector3(0, 1, 0));
            camera = new Camera(new Vector3(0, 50, 100), new Vector3(0, 0, 0), new Vector3(0, 1, 0));

            effect = Content.Load<Effect>("Effect1");
            //effect = Content.Load<Effect>("EffectTex");

            // Load the model file
            teapotModel = Content.Load<Model>("teapot");
            teapotModel.Meshes[0].MeshParts[0].Effect = effect;

            // Init Material
            surfaceMat = new Material();
            // Set the diffuse color of the material
            surfaceMat.DiffuseColor = Color.Red;
            surfaceMat.AmbientColor = Color.Red;
            surfaceMat.AmbientIntensity = 0.2f;

            surfaceMat.SpecularColor = Color.White;
            surfaceMat.SpecularPower = 25.0f;
            surfaceMat.SpecularIntensity = 2.0f;

            // Set the lighting position
            lightDir = new Vector4(-1, -1, -1, 0);

            // Add vertex data
            // This is the data for the table, hence we will also add tangents
            // TANGENTS: they should be computed automatically and not "hardcoded". But we had problems in doing so and rendering using
            // indexes hence in the end wechose to keep it simple
            tableVertices = new VertexPositionNormalTextureTangent[8];

            tableVertices[0].Position = new Vector3(-50, -3, -50);
            tableVertices[0].Normal = new Vector3(0, 1, 0);
            tableVertices[0].TextureCoordinate = new Vector2(0, 1);
            tableVertices[0].Tangent = new Vector4(1.0f, 0.0f, 0.0f, 0.0f);

            tableVertices[1].Position = new Vector3(50, -3, -50);
            tableVertices[1].Normal = new Vector3(0, 1, 0);
            tableVertices[1].TextureCoordinate = new Vector2(1, 1);
            tableVertices[1].Tangent = new Vector4(1.0f, 0.0f, 0.0f, 0.0f);

            tableVertices[2].Position = new Vector3(-50, -3, 50);
            tableVertices[2].Normal = new Vector3(0, 1, 0);
            tableVertices[2].TextureCoordinate = new Vector2(0, 0);
            tableVertices[2].Tangent = new Vector4(1.0f, 0.0f, 0.0f, 0.0f);

            tableVertices[3].Position = new Vector3(50, -3, 50);
            tableVertices[3].Normal = new Vector3(0, 1, 0);
            tableVertices[3].TextureCoordinate = new Vector2(1, 0);
            tableVertices[3].Tangent = new Vector4(1.0f, 0.0f, 0.0f, 0.0f);

            // -Y face
            tableVertices[4].Position = new Vector3(-50, -103, -50);
            tableVertices[4].Normal = new Vector3(0, -1, 0);
            tableVertices[4].TextureCoordinate = new Vector2(0, 1);
            tableVertices[4].Tangent = new Vector4(-1.0f, 0.0f, 0.0f, 0.0f);

            tableVertices[5].Position = new Vector3(50, -103, -50);
            tableVertices[5].Normal = new Vector3(0, -1, 0);
            tableVertices[5].TextureCoordinate = new Vector2(1, 1);
            tableVertices[5].Tangent = new Vector4(-1.0f, 0.0f, 0.0f, 0.0f);

            tableVertices[6].Position = new Vector3(-50, -103, 50);
            tableVertices[6].Normal = new Vector3(0, -1, 0);
            tableVertices[6].TextureCoordinate = new Vector2(0, 0);
            tableVertices[6].Tangent = new Vector4(-1.0f, 0.0f, 0.0f, 0.0f);

            tableVertices[7].Position = new Vector3(50, -103, 50);
            tableVertices[7].Normal = new Vector3(0, -1, 0);
            tableVertices[7].TextureCoordinate = new Vector2(1, 0);
            tableVertices[7].Tangent = new Vector4(-1.0f, 0.0f, 0.0f, 0.0f);

            // Vertex indices
            tableIndices = new short[12];
            tableIndices[0] = 0;
            tableIndices[1] = 1;
            tableIndices[2] = 2;
            tableIndices[3] = 1;
            tableIndices[4] = 2;
            tableIndices[5] = 3;

            tableIndices[6] = 4;
            tableIndices[7] = 5;
            tableIndices[8] = 6;
            tableIndices[9] = 5;
            tableIndices[10] = 6;
            tableIndices[11] = 7;

            // Load the textures
            woodTexture = Content.Load<Texture2D>("wood");
            nmTexture = Content.Load<Texture2D>("nrm_dx");
            stoneTexture = Content.Load<Texture2D>("diffuse");

            // The following function allocates the Textures for perlin noise generation 
            fillPerlinNoiseData();

            // TODO: use this.Content to load your game content here
        }

        protected void fillPerlinNoiseData()
        {
            int[] permu = new int[512];
            Random rand = new Random();

            // Set empty
            for (int i = 0; i < 256; i++)
            {
                permu[i] = -1;
            }

            // Generate random numbers
            for (int i = 0; i < 256; i++)
            {
                bool found = false;
                int r;
                while (!found)
                {
                    r = rand.Next() % permu.Length;
                    if (permu[r] == -1)
                    {
                        permu[r] = i;
                        found = true;
                    }
                }
            }

            for (int i = 0; i < 256; i++)
            {
                permu[255 + i] = permu[i];
            }

            // Create the permutation texture
            permTexture = new Texture2D(device, 256, 256, true, SurfaceFormat.Color);
            Color[] data = new Color[256 * 256];

            // HERE: AA is just partial, we will add Z in the shader
            for (int x = 0; x < 256; x++)
            {
                for (int y = 0; y < 256; y++)
                {
                    int A = permu[x % 256] + y;
                    int AA = permu[A % 256];
                    int AB = permu[(A + 1) % 256];
                    int B = permu[(x + 1) % 256] + y;
                    int BA = permu[B % 256];
                    int BB = permu[(B + 1) % 256 ];
                    data[x + (y * 256)] = new Color((byte)(AA), (byte)(AB),(byte)(BA), (byte)(BB));
                }
            }
            permTexture.SetData<Color>(data);

            // Allocate the gradient texture

            gradTexture = new Texture2D(device, 256, 1, true, SurfaceFormat.NormalizedByte4);
            NormalizedByte4[] gdata = new NormalizedByte4[256 * 1];
            for (int x = 0; x < 256; x++)
            {
                for (int y = 0; y < 1; y++)
                {
                    gdata[x + (y * 256)] = new NormalizedByte4(gradients[permu[x] % 16, 0],
                                                               gradients[permu[x] % 16, 1],
                                                               gradients[permu[x] % 16, 2], 1);
                }
            }
            gradTexture.SetData<NormalizedByte4>(gdata);
        }

        /// <summary>
        /// UnloadContent will be called once per game and is the place to unload
        /// all content.
        /// </summary>
        protected override void UnloadContent()
        {
            // TODO: Unload any non ContentManager content here
        }

        /// <summary>
        /// Allows the game to run logic such as updating the world,
        /// checking for collisions, gathering input, and playing audio.
        /// </summary>
        /// <param name="gameTime">Provides a snapshot of timing values.</param>
        protected override void Update(GameTime gameTime)
        {

            float timeStep = (float)gameTime.ElapsedGameTime.TotalSeconds * 60.0f;

            // Update the title of the window depending on the framRateCounter state
            Window.Title = "Graphics Tutorial | FPS: " + frameRateCounter.FrameRate;

            float x = 0.0f;
            float y = 0.0f;
            float z = 0.0f;

            float anglex = 0.0f;
            float angley = 0.0f;

            KeyboardState keyState = Keyboard.GetState();
            MouseState currentState = Mouse.GetState();

            // Moving the camera depending on keyboard input
            // W or S move UP OR DOWN
            if (keyState.IsKeyDown(Keys.W))
                y = 1.0f;
            else if (keyState.IsKeyDown(Keys.S))
                y = -1.0f;

            // A and D move FORWARD or BACKWARD
            if (keyState.IsKeyDown(Keys.A))
                z = -1.0f;
            else if (keyState.IsKeyDown(Keys.D))
                z = 1.0f;

            // Q and E move LEFT OR RIGHT
            if (keyState.IsKeyDown(Keys.Q))
                x = -1.0f;
            else if (keyState.IsKeyDown(Keys.E))
                x = 1.0f;

            /*
            if (keyState.IsKeyDown(Keys.Right))
                //x = 3.0f * timeStep;
                anglex = -0.01f;
            else if (keyState.IsKeyDown(Keys.Left))
                anglex = 0.01f;
                //x = -3.0f * timeStep;
            
            if (keyState.IsKeyDown(Keys.Up))
                //y = 3.0f * timeStep;
                angley = 0.01f;
            else if (keyState.IsKeyDown(Keys.Down))
                //y = -3.0f * timeStep;
                angley = -0.01f;
            */

            // Rotating the camera depending on MOUSE input
            // Only if left button is PRESSED
            if (currentState.LeftButton == ButtonState.Pressed)
            {
                // Calculate offset
                anglex = (currentState.X - previousState.X) * 0.05f * timeStep;
                angley = (currentState.Y - previousState.Y) * 0.05f * timeStep;
            }

            // Needed to calculate offsets
            previousState = currentState;

            // Perform the rotations
            if (anglex != 0.0f)
                //camera.TransCamera(x, y, z);
                camera.RotX(anglex * timeStep);
            if (angley != 0.0f)
                camera.RotY(angley * timeStep);
            //if ( (x + y + z) != 0.0f)
            //    camera.TransCamera(x, y, z);

            // Perform the translation
            if (y != 0.0f)
                camera.MoveUpOrDown(y);
            if (z != 0.0f)
                camera.MoveForwardOrBackward(z);
            if (x != 0.0f)
                camera.MoveLeftOrRight(x);

            // Allows the game to exit
            if (GamePad.GetState(PlayerIndex.One).Buttons.Back == ButtonState.Pressed)
                this.Exit();


            base.Update(gameTime);
        }

        /// <summary>
        /// This is called when the game should draw itself.
        /// </summary>
        /// <param name="gameTime">Provides a snapshot of timing values.</param>
        protected override void Draw(GameTime gameTime)
        {
            // The following line turns off backface culling
            device.RasterizerState = new RasterizerState() { CullMode = CullMode.None, FillMode = FillMode.Solid };

            GraphicsDevice.Clear(Color.CornflowerBlue);

            /*
                Drawing method: two objects can be drawn
             * - A table surface
             * - The teapot
             This first part will influence the effect for drawing the TABLE.
            */
                // Uncomment the following to have table rendered with a wood texture
            //effect.CurrentTechnique = effect.Techniques["TexturingTechnique"];
                // Uncomment the following to have a table rendered with a stone material and a small bump effect
            effect.CurrentTechnique = effect.Techniques["BumpMapTexturingTechnique"];

            // Parameters that will be passed to the shader programs
            //Matrix worldMat = Matrix.Multiply(Matrix.Identity, 10.0f);
            Matrix worldMat = Matrix.Identity;
            //effect.Parameters["World"].SetValue(Matrix.Multiply(Matrix.Multiply(Matrix.Identity, 10.0f), Matrix.CreateTranslation(30.0f, 6.5f, 2.5f) ));
            effect.Parameters["World"].SetValue(worldMat);
            effect.Parameters["View"].SetValue(camera.ViewMatrix);
            effect.Parameters["ViewIT"].SetValue(Matrix.Invert(Matrix.Transpose(Matrix.Multiply(camera.ViewMatrix, worldMat))));
            effect.Parameters["Projection"].SetValue(camera.ProjectionMatrix);
           
            effect.Parameters["Projection"].SetValue(Matrix.CreatePerspectiveFieldOfView(MathHelper.PiOver4, 1.0f, 0.1f, 5000.0f));
            effect.Parameters["DiffColor"].SetValue(surfaceMat.DiffuseColor.ToVector4());
            effect.Parameters["AmbColor"].SetValue(surfaceMat.AmbientColor.ToVector4());
            effect.Parameters["AmbIntensity"].SetValue(surfaceMat.AmbientIntensity);
            effect.Parameters["LightDir"].SetValue(lightDir);

            effect.Parameters["SpecColor"].SetValue(surfaceMat.SpecularColor.ToVector4());
            effect.Parameters["SpecPower"].SetValue(surfaceMat.SpecularPower);
            effect.Parameters["SpecIntensity"].SetValue(surfaceMat.SpecularIntensity);

            effect.Parameters["wt"].SetValue(woodTexture);
            effect.Parameters["normalMapTexture"].SetValue(nmTexture);
            effect.Parameters["stonet"].SetValue(stoneTexture);

            effect.Parameters["permutationTexture"].SetValue(permTexture);
            effect.Parameters["gradientTexture"].SetValue(gradTexture);

            // Actually draws the table
            
            foreach (EffectPass pass in effect.CurrentTechnique.Passes)
            {
                pass.Apply();
            }

            graphics.GraphicsDevice.DrawUserIndexedPrimitives(PrimitiveType.TriangleList, tableVertices, 0, tableVertices.Length, tableIndices, 0, tableIndices.Length / 3);
            

            /*
             *  This part will instead draw the TEAPOT. Hence, the effect uncommented here will influence that rendering
             */

                // Uncomment the following to render a simple normal-based coloring effect
            //effect.CurrentTechnique = effect.Techniques["SimpleTechnique"];
                // Uncomment the following to apply the Lambertian shading model
            //effect.CurrentTechnique = effect.Techniques["LambTechnique"];
                // Uncomment the following to apply the Phong-Blinn shading model
            effect.CurrentTechnique = effect.Techniques["PhongBTechnique"];
                // Uncomment the following to color the teapot in a checkerboard-like fashion, with tiles colored after normals
            //effect.CurrentTechnique = effect.Techniques["ProcTexturingTechnique"];
                // Uncomment the following to render the teapot using normal Phong-Linn shading but with a bump effect generated using pelin noise
            //effect.CurrentTechnique = effect.Techniques["PerlinNoiseTechnique"];

            teapotModel.Meshes[0].Draw();

            base.Draw(gameTime);
        }
    }
}