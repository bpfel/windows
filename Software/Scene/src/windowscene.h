#ifndef WINDOWSCENE_H
#define WINDOWSCENE_H

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <stb_image.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <learnopengl/filesystem.h>
#include <learnopengl/shader_m.h>
#include <learnopengl/camera.h>
#include <learnopengl/model.h>


#include <iostream>
#include <vector>

class WindowScene{
private:

  // ---------- Internal variables ------------------

  GLFWwindow* window;

  // screen settings
  const unsigned int SCR_WIDTH = 1680;
  const unsigned int SCR_HEIGHT = 1050;
  // view frustum settings
  float near_f;
  float far_f;
  // rendered texture dimensions
  unsigned int rtex_w = 1280*4/3;
  unsigned int rtex_h = 800*4/3;

  // camera
  Camera camera;
  float lastX;
  float lastY;
  bool firstMouse;

  // timing
  float deltaTime;
  float lastFrame;

  // Data from main function
  glm::vec3 viewer_pos;
  glm::vec3 projector_pos;
  float window_right;
  float window_top;
  glm::mat4 projector_K;
  glm::mat4 projector_H;
  bool render_frame;

  // set up vertex data and configure vertex attributes
  // ------------------------------------------------------------------
  float cubeVertices[180] = {//Cube of size 1 with midpoint 0,-1,-2
      // positions          // texture Coords
      -0.5f, -1.5f, -2.5f,  0.0f, 0.0f,
       0.5f, -1.5f, -2.5f,  1.0f, 0.0f,
       0.5f, -0.5f, -2.5f,  1.0f, 1.0f,
       0.5f, -0.5f, -2.5f,  1.0f, 1.0f,
      -0.5f, -0.5f, -2.5f,  0.0f, 1.0f,
      -0.5f, -1.5f, -2.5f,  0.0f, 0.0f,

      -0.5f, -1.5f, -1.5f,  0.0f, 0.0f,
       0.5f, -1.5f, -1.5f,  1.0f, 0.0f,
       0.5f, -0.5f, -1.5f,  1.0f, 1.0f,
       0.5f, -0.5f, -1.5f,  1.0f, 1.0f,
      -0.5f, -0.5f, -1.5f,  0.0f, 1.0f,
      -0.5f, -1.5f, -1.5f,  0.0f, 0.0f,

      -0.5f, -0.5f, -1.5f,  1.0f, 0.0f,
      -0.5f, -0.5f, -2.5f,  1.0f, 1.0f,
      -0.5f, -1.5f, -2.5f,  0.0f, 1.0f,
      -0.5f, -1.5f, -2.5f,  0.0f, 1.0f,
      -0.5f, -1.5f, -1.5f,  0.0f, 0.0f,
      -0.5f, -0.5f, -1.5f,  1.0f, 0.0f,

       0.5f, -0.5f, -1.5f,  1.0f, 0.0f,
       0.5f, -0.5f, -2.5f,  1.0f, 1.0f,
       0.5f, -1.5f, -2.5f,  0.0f, 1.0f,
       0.5f, -1.5f, -2.5f,  0.0f, 1.0f,
       0.5f, -1.5f, -1.5f,  0.0f, 0.0f,
       0.5f, -0.5f, -1.5f,  1.0f, 0.0f,

      -0.5f, -1.5f, -2.5f,  0.0f, 1.0f,
       0.5f, -1.5f, -2.5f,  1.0f, 1.0f,
       0.5f, -1.5f, -1.5f,  1.0f, 0.0f,
       0.5f, -1.5f, -1.5f,  1.0f, 0.0f,
      -0.5f, -1.5f, -1.5f,  0.0f, 0.0f,
      -0.5f, -1.5f, -2.5f,  0.0f, 1.0f,

      -0.5f, -0.5f, -2.5f,  0.0f, 1.0f,
       0.5f, -0.5f, -2.5f,  1.0f, 1.0f,
       0.5f, -0.5f, -1.5f,  1.0f, 0.0f,
       0.5f, -0.5f, -1.5f,  1.0f, 0.0f,
      -0.5f, -0.5f, -1.5f,  0.0f, 0.0f,
      -0.5f, -0.5f, -2.5f,  0.0f, 1.0f
  };
  float signVertices[105] = {//Some sort of triangular sign on the cube
      // positions          // texture Coords
       0.1f,  0.5f, -2.0f,  0.0f, 0.0f,
       0.2f,  0.5f, -2.0f,  0.2f, 0.0f,
       0.2f, -0.5f, -2.0f,  0.2f, 1.0f,
       0.2f, -0.5f, -2.0f,  0.2f, 1.0f,
       0.1f, -0.5f, -2.0f,  0.0f, 1.0f,
       0.1f,  0.5f, -2.0f,  0.0f, 0.0f,

      0.15f,  0.5f, -2.1f,  0.0f, 0.0f,
       0.1f,  0.5f, -2.0f,  0.2f, 0.0f,
       0.1f, -0.5f, -2.0f,  0.2f, 1.0f,
       0.1f, -0.5f, -2.0f,  0.2f, 1.0f,
      0.15f, -0.5f, -2.1f,  0.0f, 1.0f,
      0.15f,  0.5f, -2.1f,  0.0f, 0.0f,

       0.2f,  0.5f, -2.0f,  0.0f, 0.0f,
      0.15f,  0.5f, -2.1f,  0.2f, 0.0f,
      0.15f, -0.5f, -2.1f,  0.2f, 1.0f,
      0.15f, -0.5f, -2.1f,  0.2f, 1.0f,
       0.2f, -0.5f, -2.0f,  0.0f, 1.0f,
       0.2f,  0.5f, -2.0f,  0.0f, 0.0f,

       0.4f, 0.45f, -1.99f,  0.94f, 0.54f,
      -0.1f, 0.45f, -1.99f,  0.46f, 0.54f,
      0.15f,  0.7f, -2.01f, 0.65f, 0.07f

  };
  float skyboxVertices[108] = {
      // positions
      -1.0f,  1.0f, -1.0f,
      -1.0f, -1.0f, -1.0f,
       1.0f, -1.0f, -1.0f,
       1.0f, -1.0f, -1.0f,
       1.0f,  1.0f, -1.0f,
      -1.0f,  1.0f, -1.0f,

      -1.0f, -1.0f,  1.0f,
      -1.0f, -1.0f, -1.0f,
      -1.0f,  1.0f, -1.0f,
      -1.0f,  1.0f, -1.0f,
      -1.0f,  1.0f,  1.0f,
      -1.0f, -1.0f,  1.0f,

       1.0f, -1.0f, -1.0f,
       1.0f, -1.0f,  1.0f,
       1.0f,  1.0f,  1.0f,
       1.0f,  1.0f,  1.0f,
       1.0f,  1.0f, -1.0f,
       1.0f, -1.0f, -1.0f,

      -1.0f, -1.0f,  1.0f,
      -1.0f,  1.0f,  1.0f,
       1.0f,  1.0f,  1.0f,
       1.0f,  1.0f,  1.0f,
       1.0f, -1.0f,  1.0f,
      -1.0f, -1.0f,  1.0f,

      -1.0f,  1.0f, -1.0f,
       1.0f,  1.0f, -1.0f,
       1.0f,  1.0f,  1.0f,
       1.0f,  1.0f,  1.0f,
      -1.0f,  1.0f,  1.0f,
      -1.0f,  1.0f, -1.0f,

      -1.0f, -1.0f, -1.0f,
      -1.0f, -1.0f,  1.0f,
       1.0f, -1.0f, -1.0f,
       1.0f, -1.0f, -1.0f,
      -1.0f, -1.0f,  1.0f,
       1.0f, -1.0f,  1.0f
  };
  float g_quad_vertex_buffer_data[30] = {
    -1.0f, -1.0f, 0.0f, 0.0f, 0.0f,
     1.0f, -1.0f, 0.0f, 1.0f, 0.0f,
    -1.0f,  1.0f, 0.0f, 0.0f, 1.0f,
    -1.0f,  1.0f, 0.0f, 0.0f, 1.0f,
     1.0f, -1.0f, 0.0f, 1.0f, 0.0f,
     1.0f,  1.0f, 0.0f, 1.0f, 1.0f
  };

  // Textures
  unsigned int cubeTexture, signTexture, mobileTexture, cubemapTexture;
  GLuint renderedTexture;
  unsigned int frameTexture;

  // Shaders
  Shader* shader;
  Shader* skyboxShader;

  // Stuff for rendering to texture
  GLuint myFramebuffer = 0;
  GLuint depthrenderbuffer;
  // and for using the rendered texture
  GLuint quad_vertexArrayID;
  GLuint quad_vertexbuffer;

  // Vertex Array/Buffer Objects
  unsigned int cubeVAO, cubeVBO;
  unsigned int signVAO, signVBO;
  unsigned int mobileVAO, mobileVBO;
  unsigned int skyboxVAO, skyboxVBO;
  unsigned int frameVAO, frameVBO;

  // utility function for loading a 2D texture from file
  // ---------------------------------------------------
  unsigned int loadTexture(char const * path);

  // loads a cubemap texture from 6 individual texture faces
  // order:
  // +X (right)
  // -X (left)
  // +Y (top)
  // -Y (bottom)
  // +Z (front)
  // -Z (back)
  // -------------------------------------------------------
  unsigned int loadCubemap(std::vector<std::string> faces);

public:

  WindowScene();

  // Set up glfw, the window, load shaders, etc.
  int initialize(bool fullscreen);

  // Pass data from main function to scene
  // Eye position: self-explanatory
  void passEyePosition(float x, float y, float z);

  // Projector position: Position where the light comes from
  void passProjectorPosition(float x, float y, float z);

  // Projected window, given by half its width and height
  void passProjectedWindow(float right, float top);

  // Projector intrinsics
  void passProjectorIntrinsics(float k11, float k12, float k13,
                               float k21, float k22, float k23,
                               float k31, float k32, float k33);

  // Toggle frame
  void toggleFrame();

  // Render next frame
  void render();

  // Cleanup
  void terminate();

  // Loop while this holds
  bool isActive() {return !glfwWindowShouldClose(window); }

};
#endif
