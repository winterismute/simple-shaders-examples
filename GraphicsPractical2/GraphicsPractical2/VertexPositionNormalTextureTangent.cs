using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;

/*
 * This class represent a vertex structure holding:
 - A position
 - A normal
 - A 2D texture coordinate
 - A 4D tangent
 * XNA didn't have this data structure by default, hence we had to declare it
 * All the code is pretty straightforward: the "inspiration" was the "VertexColorNormal" class of the previous tutorial
*/

namespace GraphicsPractical2
{
    /// <summary>
    /// This is a structure that will hold our vertex data. Inherits from IVertexType of XNA
    /// </summary>
    public struct VertexPositionNormalTextureTangent : IVertexType
    {
        public Vector3 Position;
        public Vector3 Normal;
        public Vector2 TextureCoordinate;
        public Vector4 Tangent;

        // Each element is just a triple of Position, Color and Normal for every vertex

        public VertexPositionNormalTextureTangent(Vector3 position, Vector3 normal, Vector2 texture, Vector4 tangent)
        {
            Position = position;
            Normal = normal;
            TextureCoordinate = texture;
            Tangent = tangent;
        }

        public static VertexElement[] VertexElements =
        {
            new VertexElement(0, VertexElementFormat.Vector3, VertexElementUsage.Position, 0),
            new VertexElement(sizeof(float) * 3, VertexElementFormat.Vector3, VertexElementUsage.Normal, 0),
            new VertexElement(sizeof(float) * 6, VertexElementFormat.Vector2, VertexElementUsage.TextureCoordinate, 0),
            new VertexElement(sizeof(float) * 8, VertexElementFormat.Vector4, VertexElementUsage.Tangent, 0),

        };

        public readonly static VertexDeclaration VertexDeclaration = new VertexDeclaration(VertexElements);

        VertexDeclaration IVertexType.VertexDeclaration
        {
            get { return VertexDeclaration; }
        }
    }
}