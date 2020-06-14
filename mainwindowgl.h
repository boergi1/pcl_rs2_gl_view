#ifndef MAINWINDOWGL_H
#define MAINWINDOWGL_H
#include "format.h"

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <mutex>
#include <sstream>
#include <fstream>
#include <chrono>
// Include GLEW. Always include it before gl.h and glfw3.h, since it's a bit magic.
#include <GL/glew.h>
// Include GLFW
#include <GLFW/glfw3.h>
// Include GLM
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
using namespace glm;

#define USE_VBO 1

class MainWindowGL
{
private:
    GLFWwindow* _window;
    bool _active = false;
    GLuint _programID;
    // Initial position : on +Z
    glm::vec3 position = glm::vec3( 0, 0, 5 );
    // Initial horizontal angle : toward -Z
    float horizontalAngle = 3.14f;
    // Initial vertical angle : none
    float verticalAngle = 0.0f;
    // Initial Field of View
    float initialFoV = 45.0f;
    float speed = 3.0f; // 3 units / second
    float mouseSpeed = 0.005f;

    glm::mat4 _ViewMatrix;
    glm::mat4 _ProjectionMatrix;
    glm::mat4 _ModelMatrix = glm::mat4(1.0);

    GLuint _MVPMatrixID;
    GLuint _ViewMatrixID;
    GLuint _ModelMatrixID;

    // static float g_vertex_buffer_data[]
    static const size_t _data_size_total = FRAME_DATA_SIZE;
    //    float* _gl_data_vertices = new float[_data_size_total];
    //    float* _gl_data_colors = new float[_data_size_total];
    float _gl_data_vertices[_data_size_total];
    float _gl_data_colors[_data_size_total];
    std::vector<unsigned int>* _indices = new std::vector<unsigned int>(_data_size_total);
    std::mutex _mtx_vertices;
    std::mutex _mtx_colors;

    GLuint _VertexArrayObjectID;
    GLuint _VerticesID;
    GLuint _ColorsID;
    GLuint _VertexBufferObjectID;

    GLuint LoadShaders(const char * vertex_file_path,const char * fragment_file_path)
    {
        // Create the shaders
        GLuint VertexShaderID = glCreateShader(GL_VERTEX_SHADER);
        GLuint FragmentShaderID = glCreateShader(GL_FRAGMENT_SHADER);
        // Read the Vertex Shader code from the file
        std::string VertexShaderCode;
        std::ifstream VertexShaderStream(vertex_file_path, std::ios::in);
        if(VertexShaderStream.is_open())
        {
            std::stringstream sstr;
            sstr << VertexShaderStream.rdbuf();
            VertexShaderCode = sstr.str();
            VertexShaderStream.close();
        }
        else
        {
            std::cerr << "Impossible to open " << vertex_file_path << ". Are you in the right directory ?" << std::endl;
            return 0;
        }
        // Read the Fragment Shader code from the file
        std::string FragmentShaderCode;
        std::ifstream FragmentShaderStream(fragment_file_path, std::ios::in);
        if(FragmentShaderStream.is_open()){
            std::stringstream sstr;
            sstr << FragmentShaderStream.rdbuf();
            FragmentShaderCode = sstr.str();
            FragmentShaderStream.close();
        }
        GLint Result = GL_FALSE;
        int InfoLogLength;
        // Compile Vertex Shader
        std::cout << "Compiling shader: " << vertex_file_path << std::endl;
        char const * VertexSourcePointer = VertexShaderCode.c_str();
        glShaderSource(VertexShaderID, 1, &VertexSourcePointer , NULL);
        glCompileShader(VertexShaderID);
        // Check Vertex Shader
        glGetShaderiv(VertexShaderID, GL_COMPILE_STATUS, &Result);
        glGetShaderiv(VertexShaderID, GL_INFO_LOG_LENGTH, &InfoLogLength);
        if ( InfoLogLength > 0 ){
            std::vector<char> VertexShaderErrorMessage(InfoLogLength+1);
            glGetShaderInfoLog(VertexShaderID, InfoLogLength, NULL, &VertexShaderErrorMessage[0]);
            std::cerr << &VertexShaderErrorMessage[0] << std::endl;
            //    printf("%s\n", &VertexShaderErrorMessage[0]);
        }
        // Compile Fragment Shader
        std::cout << "Compiling shader: " << fragment_file_path << std::endl;
        char const * FragmentSourcePointer = FragmentShaderCode.c_str();
        glShaderSource(FragmentShaderID, 1, &FragmentSourcePointer , NULL);
        glCompileShader(FragmentShaderID);
        // Check Fragment Shader
        glGetShaderiv(FragmentShaderID, GL_COMPILE_STATUS, &Result);
        glGetShaderiv(FragmentShaderID, GL_INFO_LOG_LENGTH, &InfoLogLength);
        if ( InfoLogLength > 0 ){
            std::vector<char> FragmentShaderErrorMessage(InfoLogLength+1);
            glGetShaderInfoLog(FragmentShaderID, InfoLogLength, NULL, &FragmentShaderErrorMessage[0]);
            std::cerr << &FragmentShaderErrorMessage[0] << std::endl;
        }
        // Link the program
        GLuint ProgramID = glCreateProgram();
        glAttachShader(ProgramID, VertexShaderID);
        glAttachShader(ProgramID, FragmentShaderID);
        glLinkProgram(ProgramID);
        // Check the program
        glGetProgramiv(ProgramID, GL_LINK_STATUS, &Result);
        glGetProgramiv(ProgramID, GL_INFO_LOG_LENGTH, &InfoLogLength);
        if ( InfoLogLength > 0 ){
            std::vector<char> ProgramErrorMessage(InfoLogLength+1);
            glGetProgramInfoLog(ProgramID, InfoLogLength, NULL, &ProgramErrorMessage[0]);
            std::cerr << &ProgramErrorMessage[0] << std::endl;
        }
        glDetachShader(ProgramID, VertexShaderID);
        glDetachShader(ProgramID, FragmentShaderID);
        glDeleteShader(VertexShaderID);
        glDeleteShader(FragmentShaderID);
        return ProgramID;
    }

    void computeMatricesFromInputs()
    {

        // glfwGetTime is called only once, the first time this function is called
        static double lastTime = glfwGetTime();

        // Compute time difference between current and last frame
        double currentTime = glfwGetTime();
        float deltaTime = float(currentTime - lastTime);

        // Get mouse position
        double xpos, ypos;
        glfwGetCursorPos(_window, &xpos, &ypos);

        // Reset mouse position for next frame
        glfwSetCursorPos(_window, 1024/2, 768/2);

        // Compute new orientation
        horizontalAngle += mouseSpeed * float(1024/2 - xpos );
        verticalAngle   += mouseSpeed * float( 768/2 - ypos );

        // Direction : Spherical coordinates to Cartesian coordinates conversion
        glm::vec3 direction(
                    cos(verticalAngle) * sin(horizontalAngle),
                    sin(verticalAngle),
                    cos(verticalAngle) * cos(horizontalAngle)
                    );

        // Right vector
        glm::vec3 right = glm::vec3(
                    sin(horizontalAngle - 3.14f/2.0f),
                    0,
                    cos(horizontalAngle - 3.14f/2.0f)
                    );

        // Up vector
        glm::vec3 up = glm::cross( right, direction );

        // Move forward
        if (glfwGetKey( _window, GLFW_KEY_UP ) == GLFW_PRESS){
            position += direction * deltaTime * speed;
        }
        // Move backward
        if (glfwGetKey( _window, GLFW_KEY_DOWN ) == GLFW_PRESS){
            position -= direction * deltaTime * speed;
        }
        // Strafe right
        if (glfwGetKey( _window, GLFW_KEY_RIGHT ) == GLFW_PRESS){
            position += right * deltaTime * speed;
        }
        // Strafe left
        if (glfwGetKey( _window, GLFW_KEY_LEFT ) == GLFW_PRESS){
            position -= right * deltaTime * speed;
        }

        float FoV = initialFoV;// - 5 * glfwGetMouseWheel(); // Now GLFW 3 requires setting up a callback for this. It's a bit too complicated for this beginner's tutorial, so it's disabled instead.

        // Projection matrix : 45° Field of View, 4:3 ratio, display range : 0.1 unit <-> 100 units
        _ProjectionMatrix = glm::perspective(glm::radians(FoV), 4.0f / 3.0f, 0.1f, 100.0f);
        // Camera matrix
        _ViewMatrix       = glm::lookAt(
                    position,           // Camera is here
                    position+direction, // and looks here : at the same position, plus "direction"
                    up                  // Head is up (set to 0,-1,0 to look upside-down)
                    );

        // For the next frame, the "last time" will be "now"
        lastTime = currentTime;
    }


public:
    MainWindowGL();

    void setVerticesBuffer(size_t bufferOffset, std::vector <float>* verticesData)
    {
        auto vert_start = std::chrono::high_resolution_clock::now();
        size_t end_size = bufferOffset + verticesData->size();
        if (end_size > _data_size_total)
        {
            std::cerr << "Error: Invalid vertices buffer access" << std::endl;
            return;
        }
        size_t in_idx = 0;

        _mtx_vertices.lock();
        for (size_t i = bufferOffset; i < end_size; i++)
            _gl_data_vertices[i] = verticesData->at(in_idx++);
        _mtx_vertices.unlock();
        auto vert_end = std::chrono::duration_cast <std::chrono::milliseconds>(std::chrono::high_resolution_clock::now()-vert_start).count();
        std::cout << "setVerticesBuffer took " << vert_end << " ms" << std::endl;
    }
    void setColorsBuffer(size_t bufferOffset, size_t dataSize, std::vector <float>* colorData)
    {
        std::cout << "DEBUG setColorsBuffer " << bufferOffset << " " << dataSize;
        size_t end_size = bufferOffset + dataSize;
        if (end_size > _data_size_total)
        {
            std::cerr << "Error: Invalid colors buffer access" << std::endl;
            return;
        }
        size_t in_idx = 0;
        _mtx_colors.lock();
        for (size_t i = bufferOffset; i < end_size; i++)
            _gl_data_colors[i] = colorData->at(in_idx++);
        _mtx_colors.unlock();
    }

    void exit()
    {
        // Cleanup VBO and shader
        glDeleteBuffers(1, &_VerticesID);
        glDeleteBuffers(1, &_ColorsID);
#if USE_VBO
        glDeleteBuffers(1, &_VertexBufferObjectID);
#endif
        glDeleteProgram(_programID);
        glDeleteVertexArrays(1, &_VertexArrayObjectID);
        // Close OpenGL window and terminate GLFW
        glfwTerminate();
    }

    bool initialize()
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        // Initialise GLFW
        glewExperimental = true; // Needed for core profile
        if( !glfwInit() )
        {
            std::cerr << "Failed to initialize GLFW" << std::endl;
            return false;
        }
        // Create window
        glfwWindowHint(GLFW_SAMPLES, 4); // 4x antialiasing
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3); // We want OpenGL 3.3
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
        glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // To make MacOS happy; should not be needed
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE); // We don't want the old OpenGL

        // Open a window and create its OpenGL context

        int window_width = 1024;
        int window_height = 768;
        _window = glfwCreateWindow( window_width, window_height, "Pointcloud Viewer", NULL, NULL);
        if( _window == NULL )
        {
            std::cerr << "Failed to open GLFW window." << std::endl;
            glfwTerminate();
            return false;
        }
        glfwMakeContextCurrent(_window); // Initialize GLEW
        glewExperimental = true; // Needed in core profile
        if (glewInit() != GLEW_OK)
        {
            std::cerr << "Failed to initialize GLEW." << std::endl;
            return false;
        }
        // Ensure we can capture the escape key being pressed below
        glfwSetInputMode(_window, GLFW_STICKY_KEYS, GL_TRUE);
        // Hide the mouse and enable unlimited mouvement
        // glfwSetInputMode(_window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

        // Set the mouse at the center of the screen
        glfwPollEvents();
        glfwSetCursorPos(_window, 1024/2, 768/2);

        // background color
        glClearColor(0.0f, 0.0f, 0.4f, 0.0f);

        // Enable depth test - Correct near and far objects by Z value
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_PROGRAM_POINT_SIZE);
        // Accept fragment if it closer to the camera than the former one
        glDepthFunc(GL_LESS);
        // Cull triangles which normal is not towards the camera
        // glEnable(GL_CULL_FACE);

        // VAO - create a Vertex Array Object and set it as the current one
        glGenVertexArrays(1, &_VertexArrayObjectID);
        glBindVertexArray(_VertexArrayObjectID);

        // GLSL - Load shaders and get corresponding ID
        std::string file_location = "/home/boergi/Projects/pcl_rs2_gl_view/shaders/";
        _programID = LoadShaders( (file_location+"vertexshader.glsl").c_str(), (file_location+"fragmentshader.glsl").c_str() );
        glUseProgram(_programID);

        // Pass MVP matrix to GLSL
        // Get a handle for our "MVP" uniform
        // Only during the initialisation
        _MVPMatrixID = glGetUniformLocation(_programID, "MVP");
        _ViewMatrixID = glGetUniformLocation(_programID, "V");
        _ModelMatrixID = glGetUniformLocation(_programID, "M");

        // Vertex buffer
        glGenBuffers(1, &_VerticesID);     // Generate 1 buffer, put the resulting identifier in vertexbuffer
        glBindBuffer(GL_ARRAY_BUFFER, _VerticesID);    // The following commands will talk about our 'vertexbuffer' buffer
        glBufferData(GL_ARRAY_BUFFER, sizeof(_gl_data_vertices), NULL, GL_STREAM_DRAW);    // Give our vertices to OpenGL.
        // Color buffer
        glGenBuffers(1, &_ColorsID);
        glBindBuffer(GL_ARRAY_BUFFER, _ColorsID);
        glBufferData(GL_ARRAY_BUFFER, sizeof(_gl_data_colors), NULL, GL_STREAM_DRAW);
#if USE_VBO
        // Index buffer / Vertex Buffer Object
        for (unsigned int i=0; i<(unsigned int)_data_size_total; i++)
            _indices->at(i) = i;
        glGenBuffers(1, &_VertexBufferObjectID);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _VertexBufferObjectID);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, _indices->size() * sizeof(unsigned int), &_indices->at(0), GL_STATIC_DRAW);
#endif
        // Fill default colors
        for (size_t i=0; i<_data_size_total; i++)
        {
            float val = ((float)std::rand()/(float)RAND_MAX);
            _gl_data_colors[i] = val;
        }

        return true;
    }

    bool isActive() { return _active; }

    void drawPointClouds()
    {
        _active = true;
        // Check if the ESC key was pressed or the window was closed
        while( _active && glfwGetKey(_window, GLFW_KEY_ESCAPE ) != GLFW_PRESS &&
               glfwWindowShouldClose(_window) == 0 )
        {
            auto draw_start = std::chrono::high_resolution_clock::now();
            // Clear the screen. can cause flickering
            glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
            // Compute the MVP matrix from keyboard and mouse input
            computeMatricesFromInputs();
            glm::mat4 MVP = _ProjectionMatrix * _ViewMatrix * _ModelMatrix;

            // Process data start

            // Process data end

            glBindBuffer(GL_ARRAY_BUFFER, _VerticesID);
            _mtx_vertices.lock();
            glBufferData(GL_ARRAY_BUFFER, sizeof(_gl_data_vertices), NULL, GL_STREAM_DRAW); // Buffer orphaning, a common way to improve streaming perf.
            glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(_gl_data_vertices), _gl_data_vertices);
            _mtx_vertices.unlock();
            glBindBuffer(GL_ARRAY_BUFFER, _ColorsID);
            _mtx_colors.lock();
            glBufferData(GL_ARRAY_BUFFER, sizeof(_gl_data_colors), NULL, GL_STREAM_DRAW); // Buffer orphaning, a common way to improve streaming perf.
            glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(_gl_data_colors), _gl_data_colors);
            _mtx_colors.unlock();

            // Use our shader
            glUseProgram(_programID);

            // Send our transformation to the currently bound shader,
            // in the "MVP" uniform
            glUniformMatrix4fv(_MVPMatrixID, 1, GL_FALSE, &MVP[0][0]);

            // 1st attribute buffer : vertices
            glEnableVertexAttribArray(0);
            glBindBuffer(GL_ARRAY_BUFFER, _VerticesID);
            glVertexAttribPointer(
                        0,                  // attribute 0. No particular reason for 0, but must match the layout in the shader.
                        3,                  // size
                        GL_FLOAT,           // type
                        GL_FALSE,           // normalized?
                        0,                  // stride
                        (void*)0            // array buffer offset
                        );
            // 2nd attribute buffer : colors
            // glVertexAttrib3f(colorbuffer, 1.0f, 0.5f, 0.2f);
            glEnableVertexAttribArray(1);
            glBindBuffer(GL_ARRAY_BUFFER, _ColorsID);
            glVertexAttribPointer(
                        1,                                // attribute. No particular reason for 1, but must match the layout in the shader.
                        3,                                // size
                        GL_FLOAT,                         // type
                        GL_FALSE,                         // normalized?
                        0,                                // stride
                        (void*)0                          // array buffer offset
                        );

            // Draw
#if USE_VBO
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _VertexBufferObjectID);
            glDrawElements(
                        GL_POINTS,      // mode
                        _indices->size(),    // count
                        GL_UNSIGNED_INT,   // type
                        (void*)0           // element array buffer offset
                        );
#else
            glDrawArrays(GL_POINTS, 0, 12*3); // 12*3 indices starting at 0 -> 12 triangles -> 6 squares

#endif
            glDisableVertexAttribArray(0);
            glDisableVertexAttribArray(1);

            // Swap buffers
            glfwSwapBuffers(_window);
            glfwPollEvents();

            auto draw_end = std::chrono::duration_cast <std::chrono::milliseconds>(std::chrono::high_resolution_clock::now()-draw_start).count();
            std::cout << "Draw took " << draw_end << " ms" << std::endl;

        }
        _active = false;
        this->exit();
    }

};

#endif // MAINWINDOWGL_H
