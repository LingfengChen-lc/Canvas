// This example is heavily based on the tutorial at https://open.gl

////////////////////////////////////////////////////////////////////////////////
// OpenGL Helpers to reduce the clutter
#include "helpers.h"
// GLFW is necessary to handle the OpenGL context
#include <GLFW/glfw3.h>
// Linear Algebra Library
#include <Eigen/Dense>
#include <Eigen/Geometry>
// STL headers
#include <chrono>
#include <iostream>
#include <fstream>
////////////////////////////////////////////////////////////////////////////////
using namespace Eigen;



bool draw, erase, erase_hold;
std::vector<bool> mode_list;

int points = 3, k=3;
float rubber_diameter = 0.1;
Matrix4f view;
Matrix4f rubber_box;
float x_max = 1.0;
float x_min = -1.0;
float y_max = 1.0;
float y_min = -1.0;
VertexBufferObject VBO, click_VBO;
VertexBufferObject VBO_color, click_VBO_color;

Vector3f RED(1.0f, 0.0f, 0.0f);
Vector3f WHITE(1.0f, 1.0f, 1.0f);

MatrixXf Full(4,3);
MatrixXf Color(3,3);

MatrixXf click(4,3);
MatrixXf click_color(3,3);

// Mesh object, with both CPU data (Eigen::Matrix) and GPU data (the VBOs)
struct Mesh {
	Eigen::MatrixXf V; // mesh vertices [3 x n]
	Eigen::MatrixXi F; // mesh triangles [3 x m]

	// VBO storing vertex position attributes
	VertexBufferObject V_vbo;

	// VBO storing vertex indices (element buffer)
	VertexBufferObject F_vbo;

	// VAO storing the layout of the shader program for the object 'bunny'
	VertexArrayObject vao;
};

Mesh bunny;

////////////////////////////////////////////////////////////////////////////////

// Read a triangle mesh from an off file
void load_off(const std::string &filename, Eigen::MatrixXf &V, Eigen::MatrixXi &F) {
	std::ifstream in(filename);
	std::string token;
	in >> token;
	int nv, nf, ne;
	in >> nv >> nf >> ne;
	V.resize(3, nv);
	F.resize(3, nf);
	for (int i = 0; i < nv; ++i) {
		in >> V(0, i) >> V(1, i) >> V(2, i);
	}
	for (int i = 0; i < nf; ++i) {
		int s;
		in >> s >> F(0, i) >> F(1, i) >> F(2, i);
		assert(s == 3);
	}
}



////////////////////////////////////////////////////////////////////////////////
//return the lower left corner coordinate and upper right coordinate
Matrix4f creat_rubber(const float &x, const float &y) {
	Matrix4f rst;
	Vector4f U_L(x-rubber_diameter, y+rubber_diameter, 0.0f, 1.0f);
	Vector4f U_R(x+rubber_diameter, y+rubber_diameter, 0.0f, 1.0f);
	Vector4f B_L(x-rubber_diameter, y-rubber_diameter, 0.0f, 1.0f);
	Vector4f B_R(x+rubber_diameter, y-rubber_diameter, 0.0f, 1.0f);
	rst.col(0) = U_L;
	rst.col(1) = U_R;
	rst.col(2) = B_R;
	rst.col(3) = B_L;
	return rst;
}	


//detect if the given vertex inside the rubber
std::vector<int> inside_rubber(const Matrix4f &rubber){
	std::vector<int> rst;
	Vector4f B_L = rubber.col(3);
	Vector4f U_R = rubber.col(1);
	for (int i = 0; i < Full.cols(); ++i){
		float x = Full.col(i)[0];
		float y = Full.col(i)[1];
		if (x > B_L[0] && x < U_R[0]){
			if (y > B_L[1] && y < U_R[1]){
				rst.push_back(i);
			}
		}
	}
	return rst;
}

void remove_column(MatrixXf &matrix, int &ColToRemove){
	int numrows = matrix.rows();
	int numcols = matrix.cols()-1;

	if (ColToRemove < numcols){
		matrix.block(0, ColToRemove, numrows, numcols-ColToRemove) = matrix.block(0, ColToRemove+1, numrows, numcols-ColToRemove);
	}
	matrix.conservativeResize(numrows, numcols);
}

void mouse_move_callback(GLFWwindow* window, double xpos, double ypos) {
	// Get viewport size (canvas in number of pixels)
	int width, height;
	glfwGetFramebufferSize(window, &width, &height);

	// Get the size of the window (may be different than the canvas size on retina displays)
	int width_window, height_window;
	glfwGetWindowSize(window, &width_window, &height_window);

	// Deduce position of the mouse in the viewport
	double highdpi = (double) width / (double) width_window;    //number of pixel / size of screen
	xpos *= highdpi;
	ypos *= highdpi;

	// Convert screen position to world coordinates      
	double xcanonical = ((xpos/double(width))*2)-1;
	double ycanonical = (((height-1-ypos)/double(height))*2)-1; // NOTE: y axis is flipped in glfw

	// TODO: Ray-casting for object selection (Ex.3)
	Vector4f canonical;
	canonical << float(xcanonical), float(ycanonical), 0.0f, 1.0f;

	Matrix4f view_inverse = view.inverse();

	Vector4f world = view_inverse * canonical;

	if (erase){
		//draw a rectangle around the world position
		rubber_box = creat_rubber(world[0], world[1]);
		std::cerr << "rubber_box: " << rubber_box << std::endl;
		// std::cerr << "erase: " << erase << std::endl;
		// std::cerr << "draw: " << draw << std::endl;		
		Full.col(points) =  rubber_box.col(0);
		Full.col(points+1) =  rubber_box.col(1);
		Full.col(points+2) =  rubber_box.col(2);
		Full.col(points+3) =  rubber_box.col(3);
		Color.col(points) << WHITE;
		Color.col(points+1) << WHITE;
		Color.col(points+2) << WHITE;
		Color.col(points+3) << WHITE;
		if (erase_hold) {
			std::vector<int> list_pt_inside = inside_rubber(rubber_box);
			std::cerr << "points inside: " << list_pt_inside.size() << std::endl;
			int num_points_inside = list_pt_inside.size();

			for (int i = 0; i < num_points_inside; ++i){
				remove_column(Full, list_pt_inside[i]);
				remove_column(Color, list_pt_inside[i]);
			}
			points -= num_points_inside;
		}
		std::cerr << "erase_hold: " << erase_hold << std::endl;

	}

	else if (draw){
		Full.conservativeResize(4,points+1);
		// std::cerr << "columns num: "  << Full.cols() << std::endl;
		Full.col(points) = world;
		Color.conservativeResize(3,points+1);
		Color.col(points) << 1.0, 0.0, 0.0;
		points++;
		// std::cerr << points << std::endl;
	}
	VBO.update(Full);
	VBO_color.update(Color);
	return;
}
////////////////////////////////////////////////////////////////////////////////

void mouse_button_callback(GLFWwindow* window, int button, int action, int mods) {

	// Get viewport size (canvas in number of pixels)
	int width, height;
	glfwGetFramebufferSize(window, &width, &height);

	// Get the size of the window (may be different than the canvas size on retina displays)
	int width_window, height_window;
	glfwGetWindowSize(window, &width_window, &height_window);

	// Get the position of the mouse in the window
	double xpos, ypos;
	glfwGetCursorPos(window, &xpos, &ypos);

	// Deduce position of the mouse in the viewport
	double highdpi = (double) width / (double) width_window;
	xpos *= highdpi;
	ypos *= highdpi;

	// Convert screen position to the canonical viewing volume
	double xcanonical = ((xpos/double(width))*2)-1;
	double ycanonical = (((height-1-ypos)/double(height))*2)-1; // NOTE: y axis is flipped in glfw

	// TODO: Ray-casting for object selection (Ex.3)
	Vector4f canonical;
	canonical << float(xcanonical), float(ycanonical), 0.0f, 1.0f;

	Matrix4f view_inverse = view.inverse();

	Vector4f world = view_inverse * canonical;

	if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS){
		glfwSetCursorPosCallback(window, mouse_move_callback);
		Full.conservativeResize(4,points+1);
		Full.col(points) << world;
		Color.conservativeResize(3,points+1);
		Color.col(points) << RED;
		points++;
		//if erase mode is on, then enable erase
		std::cerr << "mouse press" << std::endl;
		if (erase){
			erase_hold = true;
		}
	}
	if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_RELEASE){
		if (erase){
			erase_hold = false;
		}
		else {
			glfwSetCursorPosCallback(window, NULL);
		}

		std::cerr << "mouse release" << std::endl;
	}

	// if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS){
	// 	click.conservativeResize(4,k+1);
	// 	click.col(k) << world;
	// 	click_color.conservativeResize(3,k+1);
	// 	click_color.col(k) << 0.0f, 0.0f, 1.0f;
	// 	k++;
	// }
	VBO.update(Full);
	VBO_color.update(Color);
	return;

}

//enable the specified mode and disable all the rest
// void enable (bool &mode){
// 	for (auto i = mode_list.begin(); i != mode_list.end(); ++i) {
// 		*i = false;
// 	}
// 	mode = false;
// 	return;
// }

void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {
	// Update the position of the first vertex if the keys 1,2, or 3 are pressed
	switch (key) {
		case GLFW_KEY_1:
			Full.col(0) << -.5, .5, 0.0, 1.0;
			break;
		case GLFW_KEY_2:
			Full.col(0) << .5, .5, 0.0, 1.0;
			break;
		case GLFW_KEY_3:
			load_off(DATA_DIR "dragon.off", bunny.V, bunny.F);
			bunny.V_vbo.update(bunny.V);
			bunny.F_vbo.update(bunny.F);
			break;
		case GLFW_KEY_4:
			// enable(draw);
			draw = true;
			erase = false;
			break;
		case GLFW_KEY_5:
			// enable(erase);
			erase = true;
			draw = false;
			//register move callback
			Full.conservativeResize(4,points+4);
			Color.conservativeResize(3, points+4);
			glfwSetCursorPosCallback(window, mouse_move_callback);
			break;
		default:
			break;
	}
	VBO.update(Full);
}

////////////////////////////////////////////////////////////////////////////////

int main(void) {
	// Initialize the GLFW library
	if (!glfwInit()) {
		return -1;
	}

	// Activate supersampling
	glfwWindowHint(GLFW_SAMPLES, 8);

	// Ensure that we get at least a 3.2 context
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);

	// On apple we have to load a core profile with forward compatibility
#ifdef __APPLE__
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

	// Create a windowed mode window and its OpenGL context
	GLFWwindow * window = glfwCreateWindow(640, 640, "[Float] Canvas", NULL, NULL);
	if (!window) {
		glfwTerminate();
		return -1;
	}

	// Make the window's context current
	glfwMakeContextCurrent(window);

	// Load OpenGL and its extensions
	if (!gladLoadGL()) {
		printf("Failed to load OpenGL and its extensions");
		return(-1);
	}
	printf("OpenGL Version %d.%d loaded", GLVersion.major, GLVersion.minor);

	int major, minor, rev;
	major = glfwGetWindowAttrib(window, GLFW_CONTEXT_VERSION_MAJOR);
	minor = glfwGetWindowAttrib(window, GLFW_CONTEXT_VERSION_MINOR);
	rev = glfwGetWindowAttrib(window, GLFW_CONTEXT_REVISION);
	printf("OpenGL version recieved: %d.%d.%d\n", major, minor, rev);
	printf("Supported OpenGL is %s\n", (const char*)glGetString(GL_VERSION));
	printf("Supported GLSL is %s\n", (const char*)glGetString(GL_SHADING_LANGUAGE_VERSION));

	// Initialize the OpenGL Program
	// A program controls the OpenGL pipeline and it must contains
	// at least a vertex shader and a fragment shader to be valid
	Program program;
	const GLchar* vertex_shader = R"(
		#version 150 core
		
		uniform mat4 model;
		uniform mat4 view;
		uniform mat4 proj;

		in vec3 position;
		in vec3 color;

		out vec3 o_color;

		void main() {
			gl_Position = proj * view * model * vec4(position, 1.0);
			o_color = color;
		}
	)";

	const GLchar* fragment_shader = R"(
		#version 150 core

		uniform vec3 triangleColor;
		out vec4 outColor;

		in vec3 o_color;

		void main() {
			outColor = vec4(o_color, 1.0);
		}
	)";

	// Compile the two shaders and upload the binary to the GPU
	// Note that we have to explicitly specify that the output "slot" called outColor
	// is the one that we want in the fragment buffer (and thus on screen)
	program.init(vertex_shader, fragment_shader, "outColor");

	// Prepare a dummy bunny object
	// We need to initialize and fill the two VBO (vertex positions + indices),
	// and use a VAO to store their layout when we use our shader program later.
	{
		// Initialize the VBOs
		bunny.V_vbo.init(GL_FLOAT, GL_ARRAY_BUFFER);
		bunny.F_vbo.init(GL_UNSIGNED_INT, GL_ELEMENT_ARRAY_BUFFER);

		// Vertex positions
		bunny.V.resize(3, 3);
		bunny.V <<
			0, 0.5, -0.5,
			0.5, -0.5, -0.5,
			0, 0, 0;
		bunny.V_vbo.update(bunny.V);

		// Triangle indices
		bunny.F.resize(3, 1);
		bunny.F << 0, 1, 2;
		bunny.F_vbo.update(bunny.F);

		// Create a new VAO for the bunny. and bind it
		bunny.vao.init();
		bunny.vao.bind();

		// Bind the element buffer, this information will be stored in the current VAO
		bunny.F_vbo.bind();

		// The vertex shader wants the position of the vertices as an input.
		// The following line connects the VBO we defined above with the position "slot"
		// in the vertex shader
		program.bindVertexAttribArray("position", bunny.V_vbo);

		// Unbind the VAO when I am done
		bunny.vao.unbind();
	}

	// For the first exercises, 'view' and 'proj' will be the identity matrices
	// However, the 'model' matrix must change for each model in the scene
	Eigen::Matrix4f I = Eigen::Matrix4f::Identity();
	program.bind();
	glUniformMatrix4fv(program.uniform("view"), 1, GL_FALSE, I.data());
	glUniformMatrix4fv(program.uniform("proj"), 1, GL_FALSE, I.data());

	// Save the current time --- it will be used to dynamically change the triangle color
	auto t_start = std::chrono::high_resolution_clock::now();

	// Register the keyboard callback
	glfwSetKeyCallback(window, key_callback);

	// Register the mouse callback
	glfwSetMouseButtonCallback(window, mouse_button_callback);

	VBO.init(GL_FLOAT, GL_ARRAY_BUFFER);
	click_VBO.init(GL_FLOAT, GL_ARRAY_BUFFER);

	VBO_color.init(GL_FLOAT, GL_ARRAY_BUFFER);
	click_VBO_color.init(GL_FLOAT, GL_ARRAY_BUFFER);


	click.resize(4,3);
	click << 0,  0.5, -0.5, 
		0.5, -0.5, -0.5,
		0.0, 0.0, 0.0,
		1.0, 1.0, 1.0;
	click_VBO.update(click);

	click_color.resize(3,3);
	click_color << 1.0f,  1.0f, 1.0f, 
		0.0f, 0.0f, 0.0f,
		0.0f, 0.0f, 0.0f;
	click_VBO_color.update(click_color);

	//initiate Full and Color
	Full.resize(4,3);
	Full << 0,  0.5, -0.5, 
		0.5, -0.5, -0.5,
		0.0, 0.0, 0.0,
		1.0, 1.0, 1.0;
	VBO.update(Full);

	Color.resize(3,3);
	Color << 1.0f,  1.0f, 1.0f, 
		0.0f, 0.0f, 0.0f,
		0.0f, 0.0f, 0.0f;
	VBO_color.update(Color);


	VertexArrayObject VAO;
	VAO.init();
	VAO.bind();
	program.bindVertexAttribArray("position", VBO);
	program.bindVertexAttribArray("color", VBO_color);
	VAO.unbind();

	VertexArrayObject click_VAO;
	click_VAO.init();
	click_VAO.bind();
	program.bindVertexAttribArray("position", click_VBO);
	program.bindVertexAttribArray("color", click_VBO_color);
	click_VAO.unbind();
	//enable point size
	// glEnable(GL_PROGRAM_POINT_SIZE);
	glPointSize(5.0f);

	mode_list.push_back(draw);
	mode_list.push_back(erase);
	
	// Loop until the user closes the window
	while (!glfwWindowShouldClose(window)) {
		// Set the size of the viewport (canvas) to the size of the application window (framebuffer)
		int width, height;
		glfwGetFramebufferSize(window, &width, &height);
		glViewport(0, 0, width, height);

		// Clear the framebuffer
		glClearColor(0.5f, 0.5f, 0.5f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT);

		view << 2/(x_max - x_min),0.0f,0.0f,(x_max+x_min)/(x_min-x_max),
		0.0f,2/(y_max - y_min),0.0f,(y_max+y_min)/(y_min-y_max),
		0.0f,0.0f,1.0f,0.0f,
		0.0f,0.0f,0.0f,1.0f;

		glUniformMatrix4fv(program.uniform("view"), 1, GL_FALSE, view.data());

		// Bind your program
		program.bind();

		if (!draw && !erase){
			// Bind the VAO for the bunny
			bunny.vao.bind();

			// Model matrix for the bunny
			glUniformMatrix4fv(program.uniform("model"), 1, GL_FALSE, I.data());

			// Set the uniform value depending on the time difference
			auto t_now = std::chrono::high_resolution_clock::now();
			float time = std::chrono::duration_cast<std::chrono::duration<float>>(t_now - t_start).count();
			glUniform3f(program.uniform("triangleColor"), (float)(sin(time * 4.0f) + 1.0f) / 2.0f, 0.0f, 0.0f);

			// Draw the triangles
			glDrawElements(GL_TRIANGLES, 3 * bunny.F.cols(), bunny.F_vbo.scalar_type, 0);

			bunny.vao.unbind();
		}


		else if (draw){

			VAO.bind();
			// program.bindVertexAttribArray("position", click_VBO);
			// program.bindVertexAttribArray("color", click_VBO_color);
			glDrawArrays(GL_POINTS, 3, Full.cols() - 3);
			// std::cerr<<"click.cols: " << click.cols() << std::endl;

			// for (int i = 0; i < click.cols(); i++){
			// }
	
			VAO.unbind();
		}

		else if (erase) {
			VAO.bind();
			glDrawArrays(GL_POINTS, 3, Full.cols() - 7);
			glDrawArrays(GL_LINE_LOOP, Full.cols() - 4, 4);
			//cant reach
			VAO.unbind();

		}

		// num_tri = 1;

		// for (int i = 0; i < num_tri*3; i+=3){
		// 	glDrawArrays(GL_TRIANGLES, i, 3);
		// }

		// Swap front and back buffers
		glfwSwapBuffers(window);

		// Poll for and process events
		glfwPollEvents();
	}

	// Deallocate opengl memory
	program.free();
	bunny.vao.free();
	bunny.V_vbo.free();
	bunny.F_vbo.free();
	VAO.free();
	VBO.free();
	VBO_color.free();
	click_VAO.free();
	click_VBO.free();
	click_VBO_color.free();

	// Deallocate glfw internals
	glfwTerminate();
	return 0;
}
