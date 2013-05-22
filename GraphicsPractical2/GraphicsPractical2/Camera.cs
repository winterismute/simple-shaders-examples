using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

using Microsoft.Xna.Framework;

namespace GraphicsPractical2
{
    /// <summary>
    /// The camera class allows us to display the 3D world correctly. It manages the projection and view matrices
    /// </summary>
    class Camera
    {
        // 3D API Matrices
        Matrix viewMatrix;
        Matrix projectionMatrix;

        // Vectors
        Vector3 up;
        Vector3 eye;
        Vector3 focus;

        // Keep track of the rotations!
        float yaw;
        float pitch;
        float roll;
        Matrix cameraRotation;

        // The speed of translation depends on this internal value
        float moveSpeed;

        public Camera(Vector3 camEye, Vector3 camFocus, Vector3 camUp, float aspectRatio = 4.0f / 3.0f)
        {
            eye = camEye;
            focus = camFocus;
            up = camUp;

            yaw = 0.0f;
            pitch = 0.0f;
            roll = 0.0f;

            cameraRotation = Matrix.Identity;
            // We set the input up vector as the up vector of the rotation
            cameraRotation.Up = up;

            moveSpeed = 0.2f;

            // We build the matrices from the input vectors
            viewMatrix = Matrix.CreateLookAt(eye, focus, up);
            projectionMatrix = Matrix.CreatePerspectiveFieldOfView(MathHelper.PiOver4, aspectRatio, 1.0f, 300.0f);

            UpdateViewMatrix();
        }

        /*
         * Method used to update the viewMatrix, typically after one of the view-related vectors has been changed 
        */
        private void UpdateViewMatrix()
        {
            // First, normalize the 3 rotation vectors of the matrix
            cameraRotation.Forward.Normalize();
            cameraRotation.Up.Normalize();
            //cameraRotation.Right.Normalize();

            // Then we apply the rotation to the matrix
            if (Math.Abs(pitch) > 0.001f)
                cameraRotation *= Matrix.CreateFromAxisAngle(cameraRotation.Right, pitch);
            if (Math.Abs(yaw) > 0.001f)
                cameraRotation *= Matrix.CreateFromAxisAngle(cameraRotation.Up, yaw);
            //if (Math.Abs(roll) > 0.001f)
            //cameraRotation *=
            Matrix.CreateFromAxisAngle(cameraRotation.Forward, roll);

            // The quantity of rotation is reseted
            yaw = 0.0f;
            pitch = 0.0f;
            //roll = 0.0f;

            // If we moved, we have to update the position. We did this after having taken care of the rotations
            focus = eye + cameraRotation.Forward;

            // Finally, we create our new viewMatrix
            viewMatrix = Matrix.CreateLookAt(eye, focus, cameraRotation.Up);
        }

        // Rotates around X axis
        public void RotX(float angle)
        {
            yaw += angle;
            UpdateViewMatrix();
        }

        // Rotates around Y axis
        public void RotY(float angle)
        {
            pitch += angle;
            UpdateViewMatrix();
        }

        // Rotates around Z axis
        public void RotZ(float angle)
        {
            roll += angle;
            UpdateViewMatrix();
        }

        // Updates the position according to the right move vector and the speed
        private void MoveCamera(Vector3 toadd)
        {
            eye += toadd * moveSpeed;
        }

        // input MUST be -1 or +1: move forward or backward
        public void MoveForwardOrBackward(float zz)
        {
            MoveCamera(zz * cameraRotation.Forward);
            UpdateViewMatrix();
        }

        // input MUST be -1 or +1: move up or down
        public void MoveUpOrDown(float yy)
        {
            MoveCamera(yy * cameraRotation.Up);
            UpdateViewMatrix();
        }

        // input MUST be -1 or +1: move left or right
        public void MoveLeftOrRight(float xx)
        {
            MoveCamera(xx * cameraRotation.Right);
            UpdateViewMatrix();
        }

        // Getters / Setters
        public Vector3 Eye
        {
            get { return eye; }
            set { eye = value; UpdateViewMatrix(); }
        }
        public Vector3 Focus
        {
            get { return focus; }
            set { focus = value; UpdateViewMatrix(); }
        }

        public Matrix ViewMatrix
        {
            get { return viewMatrix; }
        }
        public Matrix ProjectionMatrix
        {
            get { return projectionMatrix; }
        }

    }
}