#version 330 core

//layout(location = 0) in vec3 vertexPosition_modelspace;

//void main(){
//    gl_Position.xyz = vertexPosition_modelspace;
//    gl_Position.w = 1.0;
//}

// Input vertex data, different for all executions of this shader.
layout(location = 0) in vec3 vertexPosition_modelspace;
// Notice that the "1" here equals the "1" in glVertexAttribPointer
layout(location = 1) in vec3 vertexColor;

// Output data ; will be interpolated for each fragment.
out vec3 fragmentColor;

// Values that stay constant for the whole mesh.
uniform mat4 MVP;


void main(){
    gl_PointSize = 0.1;
    // Output position of the vertex, in clip space : MVP * position
    gl_Position =  MVP * vec4(vertexPosition_modelspace,1);
    // The color of each vertex will be interpolated
    // to produce the color of each fragment
    fragmentColor = vertexColor;
}


//// Input vertex data, different for all executions of this shader.
//layout(location = 0) in vec3 vertexPosition_modelspace;
//layout(location = 1) in vec2 vertexUV;

//// Output data ; will be interpolated for each fragment.
//out vec2 UV;

//// Values that stay constant for the whole mesh.
//uniform mat4 MVP;

//void main(){

//    // Output position of the vertex, in clip space : MVP * position
//    gl_Position =  MVP * vec4(vertexPosition_modelspace,1);

//    // UV of the vertex. No special space for this one.
//    UV = vertexUV;
//}
