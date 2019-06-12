#ifndef SCENE_CALLBACKS_H
#define SCENE_CALLBACKS_H

#include "windowscene.h"


//##################### Callback functions ############################

void processInput(GLFWwindow *window) {
  if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
    glfwSetWindowShouldClose(window, true);
}

void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
  // make sure the viewport matches the new window dimensions; note that width and
  // height will be significantly larger than specified on retina displays.
  glViewport(0, 0, width, height);
}

void mouse_callback(GLFWwindow* window, double xpos, double ypos) {
   //Originally rotated camera, does nothing now that the camera should change from other inputs
}

void scroll_callback(GLFWwindow* window, double xoffset, double yoffset) {
}


#endif
