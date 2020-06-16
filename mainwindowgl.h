#ifndef MAINWINDOWGL_H
#define MAINWINDOWGL_H
#include "format.h"
#include "filters.h"

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

#define USE_VBO 0
#define CLIP_PLANE 0

class MainWindowGL
{
private:
    GLFWwindow* _Window;

    int _windowWidth = 1024;
    int _windowHeight = 768;
    bool _active = false;
    GLuint _programID;
    // Initial position : on +Z
    glm::vec3 _viewPosition = glm::vec3( 0, 1, -2 );
    // Initial horizontal angle : toward -Z
    float _horizontalAngle = 0.0f;//3.14f;
    // Initial vertical angle
    float _verticalAngle = degreesToRadians(-20.0);
    // Initial Field of View
    float _initialFoV = 45.0f;
    float _moveSpeed = 2.0f; // units / second
    float _mouseSpeed = 0.005f;
    double _cursorPosX = _windowWidth/2;
    double _cursorPosY = _windowHeight/2;

    glm::mat4 _ViewMatrix;
    glm::mat4 _ProjectionMatrix;
    glm::mat4 _ModelMatrix = glm::mat4(1.0);


    // Memory order
    /* #1 */ static const size_t _data_size_points = FRAME_DATA_SIZE*3;
    /* #2 */ static const size_t _data_size_cs_world = 6*3;
    /* #3 */ static const size_t _data_size_region_pts = 6*3;//8*3;

    static const size_t _data_size_total = _data_size_points + _data_size_cs_world + _data_size_region_pts;
    // Data arrays
    float _gl_data_vertices[_data_size_total];
    float _gl_data_colors[_data_size_total];
    std::vector<unsigned int>* _indices = new std::vector<unsigned int>(_data_size_total);
    std::mutex _mtx_vertices;
    std::mutex _mtx_colors;
    // GLSL access ids
    GLuint _VertexArrayObjectID;
    GLuint _VerticesID;
    GLuint _ColorsID;
    GLuint _VertexBufferObjectID;
    GLuint _ViewMatrixID, _ModelMatrixID, _ProjectionMatrixID;

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
        static double lastTime = glfwGetTime();
        double currentTime = glfwGetTime();
        float deltaTime = float(currentTime - lastTime);

        //        if (glfwGetMouseButton(_Window, GLFW_KEY_SPACE) == GLFW_PRESS)
        //        {
        //            std::cerr << "SPACE ";
        //            _viewPosition = glm::vec3( 0, 1, -2 );
        //            _horizontalAngle = 0.0f;
        //            _verticalAngle = degreesToRadians(-20.0);
        //        }

        // Get mouse position
        if (glfwGetMouseButton(_Window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS)
        {

            glfwGetCursorPos(_Window, &_cursorPosX, &_cursorPosY);
            // Compute new orientation
            _horizontalAngle += _mouseSpeed * float(1024/2 - _cursorPosX );
            _verticalAngle   += _mouseSpeed * float( 768/2 - _cursorPosY );
        }
        // Reset mouse position for next frame
        glfwSetCursorPos(_Window, _windowWidth/2, _windowHeight/2);

        // Direction : Spherical coordinates to Cartesian coordinates conversion
        glm::vec3 direction(
                    cos(_verticalAngle) * sin(_horizontalAngle),
                    sin(_verticalAngle),
                    cos(_verticalAngle) * cos(_horizontalAngle)
                    );

        // Right vector
        glm::vec3 right = glm::vec3(
                    sin(_horizontalAngle - 3.14f/2.0f),
                    0,
                    cos(_horizontalAngle - 3.14f/2.0f)
                    );

        // Up vector
        glm::vec3 up = glm::cross( right, direction );

        // Move forward
        if (glfwGetKey( _Window, GLFW_KEY_UP ) == GLFW_PRESS){
            _viewPosition += direction * deltaTime * _moveSpeed;
        }
        // Move backward
        if (glfwGetKey( _Window, GLFW_KEY_DOWN ) == GLFW_PRESS){
            _viewPosition -= direction * deltaTime * _moveSpeed;
        }
        // Strafe right
        if (glfwGetKey( _Window, GLFW_KEY_RIGHT ) == GLFW_PRESS){
            _viewPosition += right * deltaTime * _moveSpeed;
        }
        // Strafe left
        if (glfwGetKey( _Window, GLFW_KEY_LEFT ) == GLFW_PRESS){
            _viewPosition -= right * deltaTime * _moveSpeed;
        }

        float FoV = _initialFoV;// - 5 * glfwGetMouseWheel(); // Now GLFW 3 requires setting up a callback for this. It's a bit too complicated for this beginner's tutorial, so it's disabled instead.

        // Projection matrix : 45° Field of View, 4:3 ratio, display range : 0.1 unit <-> 100 units
        _ProjectionMatrix = glm::perspective(glm::radians(FoV), 4.0f / 3.0f, 0.1f, 100.0f);
        // Camera matrix
        _ViewMatrix       = glm::lookAt(
                    _viewPosition,           // Camera is here
                    _viewPosition+direction, // and looks here : at the same position, plus "direction"
                    up                  // Head is up (set to 0,-1,0 to look upside-down)
                    );

        lastTime = currentTime;
    }


public:
    MainWindowGL();


    void setVerticesBuffer(size_t bufferOffset, std::vector <float>* verticesData) // 5-10ms each
    {
        auto vert_start = std::chrono::high_resolution_clock::now();
        size_t end_size = bufferOffset + verticesData->size();
        if (end_size > _data_size_points)
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
        // Window settings
        glfwWindowHint(GLFW_SAMPLES, 0); // antialiasing
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3); // We want OpenGL 3.3
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
        glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // To make MacOS happy; should not be needed
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE); // We don't want the old OpenGL
        // Open a window and create its OpenGL context
        _Window = glfwCreateWindow( _windowWidth, _windowHeight, "Pointcloud Viewer", NULL, NULL);
        if( _Window == NULL )
        {
            std::cerr << "Failed to open GLFW window." << std::endl;
            glfwTerminate();
            return false;
        }
        glfwMakeContextCurrent(_Window); // Initialize GLEW
        glewExperimental = true; // Needed in core profile
        if (glewInit() != GLEW_OK)
        {
            std::cerr << "Failed to initialize GLEW." << std::endl;
            return false;
        }
        // Ensure we can capture the escape key being pressed below
        glfwSetInputMode(_Window, GLFW_STICKY_KEYS, GL_TRUE);
        // Hide the mouse and enable unlimited mouvement
        glfwSetInputMode(_Window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
        // Disable vsync
        glfwSwapInterval(0);

        // Set the mouse at the center of the screen
        glfwPollEvents();
        glfwSetCursorPos(_Window, 1024/2, 768/2);

        // background color
        glClearColor(0.0f, 0.0f, 0.4f, 0.0f);

        glEnable(GL_DEPTH_TEST); // depth test - Correct near and far objects by Z value
        //  glEnable(GL_DEPTH_CLAMP);
        glEnable(GL_PROGRAM_POINT_SIZE); // point size can be specified in shader
        glDepthFunc(GL_LESS); // cull fragments hidden from camera
      //  glEnable(GL_CULL_FACE); // cull triangles which normal is not towards the camera

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
        _ModelMatrixID = glGetUniformLocation(_programID, "M");
        _ViewMatrixID = glGetUniformLocation(_programID, "V");
        _ProjectionMatrixID = glGetUniformLocation(_programID, "P");


        //   glEnable(GL_CLIP_PLANE0); // clipping
#if CLIP_PLANE
        glEnable(GL_CLIP_DISTANCE0);
        // Clip plane
        float PlaneEquation[] = {0, 0, -1, GLOBAL_REGION_Z_MAX_M};
        GLuint _ClipPlaneID_Z = glGetUniformLocation(_programID, "ClipPlane_Z");
        glUniform4fv(_ClipPlaneID_Z, 1, &PlaneEquation[0]);
#endif


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
        fillPointModelColors();
        fillCoordSystemWorld();
        fillPointModelRegion();

        return true;
    }

    void fillPointModelColors()
    {
        // Fill default colors
        _mtx_colors.lock();
        for (size_t i=0; i<_data_size_points; i++)
        {
            float val = 0.f;
            if (i < FRAME_DATA_SIZE) // R
                switch (i % 3) {
                case 0:
                { // x
                    val = 1.f;
                    break;
                }
                case 1:
                { // y
                    val = 0.f;
                    break;
                }
                case 2:
                { // z
                    val = 0.f;
                    break;
                }
                }
            else if (i < FRAME_DATA_SIZE*2) // G
                switch (i % 3) {
                case 0:
                {
                    val = 0.f;
                    break;
                }
                case 1:
                {
                    val = 1.f;
                    break;
                }
                case 2:
                {
                    val = 0.f;
                    break;
                }
                }
            else // B
                switch (i % 3) {
                case 0:
                {
                    val = 0.f;
                    break;
                }
                case 1:
                {
                    val = 0.f;
                    break;
                }
                case 2:
                {
                    val = 1.f;
                    break;
                }
                }
            _gl_data_colors[i] = val;
        }
        _mtx_colors.unlock();
    }

    void fillCoordSystemWorld()
    {
        const float coordsys_world_vertices[_data_size_cs_world] = {
            0.f,0.f,0.f,
            0.5f,0.f,0.f,
            0.f,0.f,0.f,
            0.f,0.5f,0.f,
            0.f,0.f,0.f,
            0.f,0.f,0.5f
        };
        const float coordsys_world_colors[_data_size_cs_world] = {
            1.f,0.f,0.f,
            1.f,0.f,0.f,
            0.f,1.f,0.f,
            0.f,1.f,0.f,
            0.f,0.f,1.f,
            0.f,0.f,1.f
        };
        // Fill world coordinate system
        size_t idx = 0;
        _mtx_vertices.lock();
        _mtx_colors.lock();
        for (size_t i=_data_size_points; i<_data_size_points+_data_size_cs_world; i++)
        {
            _gl_data_vertices[i] = coordsys_world_vertices[idx];
            _gl_data_colors[i] =coordsys_world_colors[idx];
            idx++;
        }
        _mtx_vertices.unlock();
        _mtx_colors.unlock();
    }

    void fillPointModelRegion()
    {

        //        const float region_points_vertices[_data_size_region_pts] = {
        //            GLOBAL_REGION_X_MIN_M,GLOBAL_REGION_Y_MIN_M,GLOBAL_REGION_Z_MIN_M,
        //            GLOBAL_REGION_X_MAX_M,GLOBAL_REGION_Y_MIN_M,GLOBAL_REGION_Z_MIN_M,
        //            GLOBAL_REGION_X_MAX_M,GLOBAL_REGION_Y_MAX_M,GLOBAL_REGION_Z_MIN_M,
        //            GLOBAL_REGION_X_MIN_M,GLOBAL_REGION_Y_MAX_M,GLOBAL_REGION_Z_MIN_M,
        //            GLOBAL_REGION_X_MIN_M,GLOBAL_REGION_Y_MAX_M,GLOBAL_REGION_Z_MAX_M,
        //            GLOBAL_REGION_X_MIN_M,GLOBAL_REGION_Y_MIN_M,GLOBAL_REGION_Z_MAX_M,
        //            GLOBAL_REGION_X_MAX_M,GLOBAL_REGION_Y_MIN_M,GLOBAL_REGION_Z_MAX_M,
        //            GLOBAL_REGION_X_MAX_M,GLOBAL_REGION_Y_MAX_M,GLOBAL_REGION_Z_MAX_M
        //        };
        const float region_points_vertices[_data_size_region_pts] = { // Triangles
                                                                      GLOBAL_REGION_X_MAX_M,GLOBAL_REGION_Y_MAX_M,GLOBAL_REGION_Z_MAX_M,
                                                                      GLOBAL_REGION_X_MAX_M,GLOBAL_REGION_Y_MIN_M,GLOBAL_REGION_Z_MAX_M,
                                                                      GLOBAL_REGION_X_MIN_M,GLOBAL_REGION_Y_MIN_M,GLOBAL_REGION_Z_MAX_M,
                                                                      GLOBAL_REGION_X_MIN_M,GLOBAL_REGION_Y_MIN_M,GLOBAL_REGION_Z_MAX_M,
                                                                      GLOBAL_REGION_X_MIN_M,GLOBAL_REGION_Y_MAX_M,GLOBAL_REGION_Z_MAX_M,
                                                                      GLOBAL_REGION_X_MAX_M,GLOBAL_REGION_Y_MAX_M,GLOBAL_REGION_Z_MAX_M
                                                                    };
        float gray = 0.6f;
        const float region_points_colors[_data_size_region_pts] = {
            gray,gray,gray,
            gray,gray,gray,
            gray,gray,gray,
            gray,gray,gray,
            gray,gray,gray,
            gray,gray,gray/*,
            gray,gray,gray,
            gray,gray,gray*/
        };
        // Fill region data
        size_t idx = 0;
        _mtx_vertices.lock();
        _mtx_colors.lock();
        for (size_t i=_data_size_points+_data_size_cs_world; i<_data_size_points+_data_size_cs_world+_data_size_region_pts; i++)
        {
            _gl_data_vertices[i] = region_points_vertices[idx];
            _gl_data_colors[i] = region_points_colors[idx];
            idx++;
        }
        _mtx_vertices.unlock();
        _mtx_colors.unlock();
    }


    bool isActive() { return _active; }

    void drawDataPoints()
    {
        _active = true;
        // Check if the ESC key was pressed or the window was closed
        while( _active && glfwGetKey(_Window, GLFW_KEY_ESCAPE ) != GLFW_PRESS &&
               glfwWindowShouldClose(_Window) == 0 )
        {
#if (VERBOSE > 2)
            auto draw_start = std::chrono::high_resolution_clock::now();
#endif
            // Clear the screen. can cause flickering
            glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
            // Compute the MVP matrix from keyboard and mouse input
            computeMatricesFromInputs();

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
            glUniformMatrix4fv(_ModelMatrixID, 1, GL_FALSE, &_ModelMatrix[0][0]);
            glUniformMatrix4fv(_ViewMatrixID, 1, GL_FALSE, &_ViewMatrix[0][0]);
            glUniformMatrix4fv(_ProjectionMatrixID, 1, GL_FALSE, &_ProjectionMatrix[0][0]);

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
                        //  _indices->size(),    // count
                        _data_size_points,    // count
                        GL_UNSIGNED_INT,   // type
                        (void*)0           // element array buffer offset
                        );
            glDrawElements(
                        GL_LINES,      // mode
                        _data_size_cs,    // count
                        GL_UNSIGNED_INT,   // type
                        (void*)_data_size_points           // element array buffer offset
                        );
#else
            glDrawArrays(GL_POINTS, 0, FRAME_DATA_LENGTH*3);
            glDrawArrays(GL_LINES, FRAME_DATA_LENGTH*3, 6);
          //  glDrawArrays(GL_LINE_STRIP, FRAME_DATA_LENGTH*3+6, 8);
#if CLIP_PLANE
            glDrawArrays(GL_TRIANGLES, FRAME_DATA_LENGTH*3+6, 6);
#endif
#endif
            glDisableVertexAttribArray(0);
            glDisableVertexAttribArray(1);

            // Swap buffers
            glfwSwapBuffers(_Window);
            glfwPollEvents();
#if (VERBOSE > 2)
            auto draw_end = std::chrono::duration_cast <std::chrono::milliseconds>(std::chrono::high_resolution_clock::now()-draw_start).count();
            std::cout << "Draw took " << draw_end << " ms" << std::endl;
#endif

        }
        _active = false;
        this->exit();
    }

};

#endif // MAINWINDOWGL_H
