#include "windowscene.h"
#include "callbacks.h"

//uncomment to enable casting of glm matrices to strings
//#define GLM_ENABLE_EXPERIMENTAL
//#include <glm/gtx/string_cast.hpp>

// ################## texture loading ###################

unsigned int WindowScene::loadTexture(char const * path) {
  unsigned int textureID;
  glGenTextures(1, &textureID);

  int width, height, nrComponents;
  unsigned char *data = stbi_load(path, &width, &height, &nrComponents, 0);
  if (data) {
    GLenum format;
    if (nrComponents == 1)
      format = GL_RED;
    else if (nrComponents == 3)
      format = GL_RGB;
    else if (nrComponents == 4)
      format = GL_RGBA;

    glBindTexture(GL_TEXTURE_2D, textureID);
    glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, format, GL_UNSIGNED_BYTE, data);
    glGenerateMipmap(GL_TEXTURE_2D);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    stbi_image_free(data);
  } else {
    std::cout << "Texture failed to load at path: " << path << std::endl;
    stbi_image_free(data);
  }

  return textureID;
}

unsigned int WindowScene::loadCubemap(vector<std::string> faces) {
  unsigned int textureID;
  glGenTextures(1, &textureID);
  glBindTexture(GL_TEXTURE_CUBE_MAP, textureID);

  int width, height, nrChannels;
  for (unsigned int i = 0; i < faces.size(); i++) {
    unsigned char *data = stbi_load(faces[i].c_str(), &width, &height, &nrChannels, 0);
    if (data) {
      glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
      stbi_image_free(data);
    } else {
      std::cout << "Cubemap texture failed to load at path: " << faces[i] << std::endl;
      stbi_image_free(data);
    }
  }
  glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);

  return textureID;
}

// ######################## initialization #############################

WindowScene::WindowScene(){

  window = NULL;

  near_f = 0.1f;
  far_f = 100.0f;

  projector_K = glm::mat4(1);
  projector_pos = glm::vec3(0,0,1);
  window_right = 0.3f;
  window_top = 0.2f;
  render_frame = false;

  camera = Camera();
  lastX = (float)SCR_WIDTH / 2.0;
  lastY = (float)SCR_HEIGHT / 2.0;
  firstMouse = true;

  deltaTime = 0.0f;
  lastFrame = 0.0f;
}

int WindowScene::initialize(bool fullscreen) {
  // glfw: initialize and configure
  glfwInit();
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#ifdef __APPLE__
  // Not that this was actually tested on Apple
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

  // glfw window creation
  if(fullscreen){
    window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "Projected window scene", glfwGetPrimaryMonitor(), NULL);
  } else {
    window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "Projected window scene", NULL, NULL);
  }
  if (window == NULL) {
    std::cout << "Failed to create GLFW window" << std::endl;
    glfwTerminate();
    return -1;
  }
  glfwMakeContextCurrent(window);
  glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
  //glfwSetCursorPosCallback(window, mouse_callback);
  //glfwSetScrollCallback(window, scroll_callback);

  // tell GLFW to capture our mouse
  glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

  // glad: load all OpenGL function pointers
  if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
    std::cout << "Failed to initialize GLAD" << std::endl;
    return -1;
  }

  // configure global opengl state
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  // cube VAO
  glGenVertexArrays(1, &cubeVAO);
  glGenBuffers(1, &cubeVBO);
  glBindVertexArray(cubeVAO);
  glBindBuffer(GL_ARRAY_BUFFER, cubeVBO);
  glBufferData(GL_ARRAY_BUFFER, sizeof(cubeVertices), &cubeVertices, GL_STATIC_DRAW);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));

  // sign VAO
  glGenVertexArrays(1, &signVAO);
  glGenBuffers(1, &signVBO);
  glBindVertexArray(signVAO);
  glBindBuffer(GL_ARRAY_BUFFER, signVBO);
  glBufferData(GL_ARRAY_BUFFER, sizeof(signVertices), &signVertices, GL_STATIC_DRAW);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));

  // window frame VAO
  glGenVertexArrays(1, &frameVAO);
  glGenBuffers(1, &frameVBO);
  glBindVertexArray(frameVAO);
  glBindBuffer(GL_ARRAY_BUFFER, frameVBO);
  glBufferData(GL_ARRAY_BUFFER, sizeof(g_quad_vertex_buffer_data), &g_quad_vertex_buffer_data, GL_STATIC_DRAW);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));

  // skybox VAO
  glGenVertexArrays(1, &skyboxVAO);
  glGenBuffers(1, &skyboxVBO);
  glBindVertexArray(skyboxVAO);
  glBindBuffer(GL_ARRAY_BUFFER, skyboxVBO);
  glBufferData(GL_ARRAY_BUFFER, sizeof(skyboxVertices), &skyboxVertices, GL_STATIC_DRAW);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);

  // mobile object VAO
  glGenVertexArrays(1, &mobileVAO);
  glGenBuffers(1, &mobileVBO);

  // quad displayed on the actual wall
  glGenVertexArrays(1, &quad_vertexArrayID);
  glBindVertexArray(quad_vertexArrayID);
  glGenBuffers(1, &quad_vertexbuffer);
  glBindBuffer(GL_ARRAY_BUFFER, quad_vertexbuffer);
  glBufferData(GL_ARRAY_BUFFER, sizeof(g_quad_vertex_buffer_data), g_quad_vertex_buffer_data, GL_DYNAMIC_DRAW);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));

  // load textures
  cubeTexture = loadTexture(FileSystem::getPath("Scene/resources/textures/marble.jpg").c_str());
  signTexture = loadTexture(FileSystem::getPath("Scene/resources/textures/somesign.png").c_str());
  mobileTexture = loadTexture(FileSystem::getPath("Scene/resources/textures/butterfly_transparent.png").c_str());
  frameTexture = loadTexture(FileSystem::getPath("Scene/resources/textures/NiceWindow.png").c_str());

  vector<std::string> faces {
    FileSystem::getPath("Scene/resources/textures/skybox/right.jpg"),
    FileSystem::getPath("Scene/resources/textures/skybox/left.jpg"),
    FileSystem::getPath("Scene/resources/textures/skybox/top.jpg"),
    FileSystem::getPath("Scene/resources/textures/skybox/bottom.jpg"),
    FileSystem::getPath("Scene/resources/textures/skybox/front.jpg"),
    FileSystem::getPath("Scene/resources/textures/skybox/back.jpg")
  };
  cubemapTexture = loadCubemap(faces);

  // shader configuration
  shader = new Shader("cubemaps.vs", "cubemaps.fs");
  skyboxShader = new Shader("skybox.vs", "skybox.fs");

  shader->use();
  shader->setInt("texture1", 0);

  skyboxShader->use();
  skyboxShader->setInt("skybox", 0);

  // For rendering to texture
  glGenFramebuffers(1, &myFramebuffer);
  glBindFramebuffer(GL_FRAMEBUFFER, myFramebuffer);

  glGenTextures(1, &renderedTexture);
  glBindTexture(GL_TEXTURE_2D, renderedTexture);

  glTexImage2D(GL_TEXTURE_2D, 0,GL_RGB, rtex_w, rtex_h, 0, GL_RGB, GL_UNSIGNED_BYTE, 0);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

  glGenRenderbuffers(1, &depthrenderbuffer);
  glBindRenderbuffer(GL_RENDERBUFFER, depthrenderbuffer);
  glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, rtex_w, rtex_h);
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depthrenderbuffer);

  glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, renderedTexture, 0);
  GLenum DrawBuffers[1] = {GL_COLOR_ATTACHMENT0};
  glDrawBuffers(1, DrawBuffers);

  if(glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE){
    cout << "Error with framebuffer :(\n";
    return -1;
  }

  return 0;
}

void WindowScene::passEyePosition(float x, float y, float z){
  viewer_pos = glm::vec3(x, y, z);
}

void WindowScene::passProjectorPosition(float x, float y, float z)
{
  projector_pos = glm::vec3(x, y, z);
  projector_H = glm::translate(glm::mat4(1.0f), projector_pos);
  projector_H = projector_K * projector_H;
}

void WindowScene::passProjectedWindow(float right, float top)
{
  window_right = right;
  window_top = top;
}

void WindowScene::passProjectorIntrinsics(float k11, float k12, float k13,
                                          float k21, float k22, float k23,
                                          float k31, float k32, float k33)
{
  projector_K = glm::mat4(k11, k12, 0, k13,
                          k21, k22, 0, k23,
                          0, 0, 1, 0,
                          k31, k32, 0, k33);
  projector_H = glm::translate(glm::mat4(1.0f), projector_pos);
  projector_H = projector_K * projector_H;
  //TODO ... actually use this?
}

void WindowScene::toggleFrame(){
  render_frame = !render_frame;
}

void WindowScene::render() {
  // per-frame time logic
  float currentFrame = glfwGetTime();
  deltaTime = currentFrame - lastFrame;
  lastFrame = currentFrame;

  // input from keyboard
  processInput(window);

  camera.Position = viewer_pos;
  camera.Front = -glm::normalize(camera.Position);

  // Render scene as texture
  glBindFramebuffer(GL_FRAMEBUFFER, myFramebuffer);
  glViewport(0,0,rtex_w,rtex_h);

  glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glm::mat4 model = glm::mat4(1.0f);
  glm::mat4 view = camera.GetViewMatrix();
  glm::mat4 projection = glm::perspective(glm::radians(camera.Zoom), (float)SCR_WIDTH / (float)SCR_HEIGHT, near_f, far_f);

  // draw skybox
  //    this used to be the last object drawn, but the transparent
  //    butterfly required a reordering
  glDepthFunc(GL_LEQUAL);  // change depth function so depth test passes when values are equal to depth buffer's content
  skyboxShader->use();
  // remove translation from the view matrix
  skyboxShader->setMat4("view", glm::mat4(glm::mat3(camera.GetViewMatrix())));
  skyboxShader->setMat4("projection", projection);
  // skybox cube
  glBindVertexArray(skyboxVAO);
  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_CUBE_MAP, cubemapTexture);
  glDrawArrays(GL_TRIANGLES, 0, 36);
  glBindVertexArray(0);
  glDepthFunc(GL_LESS); // set depth function back to default

  // draw scene
  shader->use();
  shader->setMat4("model", model);
  shader->setMat4("view", view);
  shader->setMat4("projection", projection);

  // cube
  glBindVertexArray(cubeVAO);
  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, cubeTexture);
  glDrawArrays(GL_TRIANGLES, 0, 36);
  glBindVertexArray(0);

  // sign
  glBindVertexArray(signVAO);
  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, signTexture);
  glDrawArrays(GL_TRIANGLES, 0, 21);
  glBindVertexArray(0);

  // mobile object (butterfly)
  const float b_x = viewer_pos.x;
  const float b_y = viewer_pos.y;
  const float b_z = -1.0f;
  const float cos_t = cos(16*currentFrame);
  const float sin_t = sin(16*currentFrame);
  const float d_x = abs(sin_t * 0.2f);
  const float d_y = 0.135f;
  const float d_z = cos_t * 0.2f;
  float mobileVertices[60] = {
      // positions                      // texture Coords
      b_x - d_x, b_y + d_y, b_z + d_z,  0.0f, 0.0f,
      b_x,       b_y + d_y, b_z,        0.5f, 0.0f,
      b_x,       b_y - d_y, b_z,        0.5f, 1.0f,
      b_x - d_x, b_y + d_y, b_z + d_z,  0.0f, 0.0f,
      b_x,       b_y - d_y, b_z,        0.5f, 1.0f,
      b_x - d_x, b_y - d_y, b_z + d_z,  0.0f, 1.0f,

      b_x + d_x, b_y + d_y, b_z + d_z,  1.0f, 0.0f,
      b_x + d_x, b_y - d_y, b_z + d_z,  1.0f, 1.0f,
      b_x,       b_y - d_y, b_z,        0.5f, 1.0f,
      b_x + d_x, b_y + d_y, b_z + d_z,  1.0f, 0.0f,
      b_x,       b_y - d_y, b_z,        0.5f, 1.0f,
      b_x,       b_y + d_y, b_z,        0.5f, 0.0f
  };
  glBindVertexArray(mobileVAO);
  glBindBuffer(GL_ARRAY_BUFFER, mobileVBO);
  glBufferData(GL_ARRAY_BUFFER, sizeof(mobileVertices), &mobileVertices, GL_DYNAMIC_DRAW);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));
  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, mobileTexture);
  glDrawArrays(GL_TRIANGLES, 0, 12);
  glBindVertexArray(0);

  // Compute texture coordinates for the quad
  //  or: Where in the texture do the corners of the window land?
  glm::mat4 MVP = projection * view * model;
  glm::vec4 corner_1 = MVP * glm::vec4( window_right,  window_top, 0.0f, 1.0f);
  glm::vec4 corner_2 = MVP * glm::vec4(-window_right,  window_top, 0.0f, 1.0f);
  glm::vec4 corner_3 = MVP * glm::vec4( window_right, -window_top, 0.0f, 1.0f);
  glm::vec4 corner_4 = MVP * glm::vec4(-window_right, -window_top, 0.0f, 1.0f);

  //Render textured wall from projector
  glBindFramebuffer(GL_FRAMEBUFFER,0);
  glViewport(0,0,SCR_WIDTH,SCR_HEIGHT);
  glClearColor(0.5f, 0.5f, 0.5f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  model = glm::mat4(1.0f);
  camera.Position = projector_pos;
  camera.Front = -glm::normalize(camera.Position);
  view = camera.GetViewMatrix();
  //projection = glm::mat4(1.0f);
  projection = glm::perspective(glm::radians(camera.Zoom), (float)rtex_w / (float)rtex_h, near_f, far_f);

  shader->use();
  shader->setMat4("model", model);
  shader->setMat4("view", view);
  shader->setMat4("projection", projection);

  //render the quad using the rendererd texture
  const float quadVertices[30] = {
      // positions          // texture Coords
      -1.0f, -1.0f, 0.0f,   corner_4.x+0.5f, corner_4.y+0.5f,
       1.0f, -1.0f, 0.0f,   corner_3.x+0.5f, corner_3.y+0.5f,
      -1.0f,  1.0f, 0.0f,   corner_2.x+0.5f, corner_2.y+0.5f,
      -1.0f,  1.0f, 0.0f,   corner_2.x+0.5f, corner_2.y+0.5f,
       1.0f, -1.0f, 0.0f,   corner_3.x+0.5f, corner_3.y+0.5f,
       1.0f,  1.0f, 0.0f,   corner_1.x+0.5f, corner_1.y+0.5f
  };

  glBindVertexArray(quad_vertexArrayID);
  glBindBuffer(GL_ARRAY_BUFFER, quad_vertexArrayID);
  glBufferData(GL_ARRAY_BUFFER, sizeof(quadVertices), &quadVertices, GL_DYNAMIC_DRAW);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));
  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, renderedTexture);
  glDrawArrays(GL_TRIANGLES, 0, 6);
  glBindVertexArray(0);

  //Render a window frame if desired
  if(render_frame){
    shader->setMat4("model", glm::mat4(1.0f));
    shader->setMat4("view", glm::mat4(1.0f));
    shader->setMat4("projection", glm::mat4(1.0f));

    glBindVertexArray(frameVAO);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, frameTexture);
    glDrawArrays(GL_TRIANGLES, 0, 6);
    glBindVertexArray(0);
  }

  // glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
  glfwSwapBuffers(window);
  glfwPollEvents();
}

void WindowScene::terminate() {
  // de-allocate all resources once they've outlived their purpose:
  glDeleteVertexArrays(1, &cubeVAO);
  glDeleteVertexArrays(1, &signVAO);
  glDeleteVertexArrays(1, &skyboxVAO);
  glDeleteVertexArrays(1, &mobileVAO);
  glDeleteVertexArrays(1, &quad_vertexArrayID);
  glDeleteVertexArrays(1, &frameVAO);
  glDeleteBuffers(1, &cubeVBO);
  glDeleteBuffers(1, &signVBO);
  glDeleteBuffers(1, &skyboxVBO);
  glDeleteBuffers(1, &mobileVBO);
  glDeleteBuffers(1, &quad_vertexArrayID);
  glDeleteBuffers(1, &frameVBO);

  glfwTerminate();

  delete shader;
  delete skyboxShader;
}
