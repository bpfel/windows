#ifndef SCENE_CALLBACKS_H
#define SCENE_CALLBACKS_H

#include "windowscene.h"

  // Callback functions


  // process all input: query GLFW whether relevant keys are pressed/released this frame and react accordingly
  // ---------------------------------------------------------------------------------------------------------
void processInput(GLFWwindow *window);

  // glfw: whenever the window size changed (by OS or user resize) this callback function executes
  // ---------------------------------------------------------------------------------------------
void framebuffer_size_callback(GLFWwindow* window, int width, int height);

  // glfw: whenever the mouse moves, this callback is called (though nothing is actually done)
  // -------------------------------------------------------
void mouse_callback(GLFWwindow* window, double xpos, double ypos);

  // glfw: whenever the mouse scroll wheel scrolls, this callback is called
  // ----------------------------------------------------------------------
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);

#endif
