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



bool draw, erase, erase_hold, move_mode, delete_mode, chang_color, selected, move_hold, move_second_click;

std::vector<Matrix4f> transforms;
Matrix4f transform;
std::vector<int> inside_points_select;
Matrix4f select_box;
std::vector<bool> mode_list;

float angle_10 = 10*M_PI/180;
float angle = 0.0;

Vector3f centroid;
int points=3, k=3;
float rubber_diameter = 0.1, point_size = 5.0;
Matrix4f view;
Matrix4f rubber_box;
float x_max = 1.0;
float x_min = -1.0;
float y_max = 1.0;
float y_min = -1.0;
VertexBufferObject VBO, click_VBO;
VertexBufferObject VBO_color, click_VBO_color;

//transform matrix
Matrix4f rotate_clock;
Matrix4f rotate_normal;		
Matrix4f scale_up;
Matrix4f scale_down;

Vector3f RED(1.0f, 0.0f, 0.0f);
Vector3f WHITE(1.0f, 1.0f, 1.0f);
Vector3f GRAY(0.5f, 0.5f, 0.5f);

Vector3f C1(0.2f, 0.3f, 0.6f);
Vector3f C2(0.4f, 0.1f, 0.2f);
Vector3f C3(0.2f, 0.9f, 0.6f);

Vector3f Current_Color = RED;

Vector4f move_click;
Vector4f move_click2;

MatrixXf Full(4,3);
MatrixXf Color(3,3);




// Mesh object, with both CPU data (Eigen::Matrix) and GPU data (the VBOs)
struct Mesh {
	Eigen::MatrixXf V; // mesh vertices [3 x n]
	Eigen::MatrixXi F; // mesh triangles [3 x m]

	Matrix4f trans;
	// VBO storing vertex position attributes
	VertexBufferObject V_vbo;

	// VBO storing vertex indices (element buffer)
	VertexBufferObject F_vbo;

	// VAO storing the layout of the shader program for the object 'bunny'
	VertexArrayObject vao;
};

// std::vector<Mesh> meshs;
Mesh bunny1;
Mesh bunny2;
Mesh bunny3;
Mesh current_mesh;


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
		// list.push_back(Matrix4f::Identity());
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


//detect if the given vertex inside the rubber   (after transform)
void inside_box(std::vector<int> &rst, const Matrix4f &box){
	Vector4f B_L = box.col(3);
	Vector4f U_R = box.col(1);
	for (int i = 0; i < Full.cols(); ++i){
		float x = (transforms[i] * Full.col(i))(0);
		float y = (transforms[i] * Full.col(i))(1);
		// float x = Full.col(i)(0);
		// float y = Full.col(i)(1);
		if (x > B_L(0) && x < U_R(0)){
			if (y > B_L(1) && y < U_R(1)){
				rst.push_back(i);
			}
		}
	}
	return;
}

void remove_column(MatrixXf &matrix, int &ColToRemove){
	int numrows = matrix.rows();
	int numcols = matrix.cols()-1;

	if (ColToRemove < numcols){
		matrix.block(0, ColToRemove, numrows, (numcols-ColToRemove)) = matrix.block(0, ColToRemove+1, numrows, (numcols-ColToRemove));
	}
	matrix.conservativeResize(numrows, numcols);
}

void erase_by_change_color(MatrixXf &matrix, int &ColTochange){
	matrix.col(ColTochange) = GRAY;
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
		
		Full.col(points) =  rubber_box.col(0);
		Full.col(points+1) =  rubber_box.col(1);
		Full.col(points+2) =  rubber_box.col(2);
		Full.col(points+3) =  rubber_box.col(3);
		Color.col(points) << WHITE;
		Color.col(points+1) << WHITE;
		Color.col(points+2) << WHITE;
		Color.col(points+3) << WHITE;
		if (erase_hold) {
			std::vector<int> list_pt_inside;
			list_pt_inside.clear();
			inside_box(list_pt_inside, rubber_box);
			int num_points_inside = list_pt_inside.size();

			for (int i = 0; i < num_points_inside; ++i){
				erase_by_change_color(Color, list_pt_inside[i]);

			}
		}
	}

	else if (draw){
		//add points and add matrix
		Full.conservativeResize(4,points+1);
		Full.col(points) = world;
		Color.conservativeResize(3,points+1);
		Color.col(points) = Current_Color;
		points++;
		transforms.push_back(Matrix4f::Identity());
	}
	else if (move_hold){
		if (!move_second_click){
			Vector4f a(move_click(0), world(1), 0.0f, 1.0f);   // same x differnt y
			Vector4f b(world(0), move_click(1), 0.0f, 1.0f);   // same y differnt x
			Vector4f c(world(0), world(1), 0.0f, 1.0f);   // different x differnt y
			Full.col(points) = move_click;
			Full.col(points+1) = a;
			Full.col(points+2) = c;
			Full.col(points+3) = b;
			Color.col(points) << WHITE;
			Color.col(points+1) << WHITE;
			Color.col(points+2) << WHITE;
			Color.col(points+3) << WHITE;



			if (a(1) > move_click(1) && b(0) > move_click(0)){
				select_box.col(0) = a;
				select_box.col(1) = c;
				select_box.col(2) = b;
				select_box.col(3) = move_click;
			}
			else if (a(1) > move_click(1) && b(0) < move_click(0)){
				select_box.col(0) = c;
				select_box.col(1) = a;
				select_box.col(2) = move_click;
				select_box.col(3) = b;
			}
			else if (a(1) < move_click(1) && b(0) > move_click(0)){
				select_box.col(0) = move_click;
				select_box.col(1) = b;
				select_box.col(2) = c;
				select_box.col(3) = a;
			}
			else if (a(1) < move_click(1) && b(0) < move_click(0)){
				select_box.col(0) = b;
				select_box.col(1) = move_click;
				select_box.col(2) = a;
				select_box.col(3) = c;
			}
			
		}


		//second click happens
		else {
			
			float x_dis = world(0) - move_click2(0);
			float y_dis = world(1) - move_click2(1);

			transform.resize(4,4);
			transform << 1.0f, 0.0f, 0.0f, x_dis,
					0.0f, 1.0f, 0.0f, y_dis,
					0.0f, 0.0f, 1.0f, 0.0f,
					0.0f, 0.0f, 0.0f, 1.0f;

			for (int i = 0; i < inside_points_select.size(); ++i){
				transforms[inside_points_select[i]] = transform * transforms[inside_points_select[i]];
			}
			transforms[points] = transform * transforms[points];
			transforms[points+1] = transform * transforms[points+1];
			transforms[points+2] = transform * transforms[points+2];
			transforms[points+3] = transform * transforms[points+3];
			move_click2 << world(0), world(1), 0.0, 1.0;
		}
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

	if (button == GLFW_MOUSE_BUTTON_LEFT && action != GLFW_RELEASE){
		glfwSetCursorPosCallback(window, mouse_move_callback);
		Full.conservativeResize(4,points+4);
		Color.conservativeResize(3, points+4);
		if (erase){
			erase_hold = true;
		}
		if (move_mode){
			if (!move_second_click){
				move_hold = true;
				move_click = world;
				transforms[points] = Matrix4f::Identity();
				transforms[points+1] = Matrix4f::Identity();
				transforms[points+2] = Matrix4f::Identity();
				transforms[points+3] = Matrix4f::Identity();
			}
			else{
				move_click2 = world;
				move_hold = true;
			}
		}

	}
	if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_RELEASE){
		if (erase){
			erase_hold = false;
		}
		else if (move_mode){
			if (!move_second_click) {
				move_second_click = true;
				inside_box(inside_points_select, select_box);
			}
			else {
				move_second_click = false;
				Color.col(points) = GRAY;
				Color.col(points+1) = GRAY;
				Color.col(points+2) = GRAY;
				Color.col(points+3) = GRAY;
				move_click = world;
				inside_points_select.clear();
			}
			move_hold =false;
		}
		else {
			glfwSetCursorPosCallback(window, NULL);
		}

	}


	VBO.update(Full);
	VBO_color.update(Color);
	return;

}


Vector2f get_triangle_centroid(const Vector4f &a, const Vector4f &b, const Vector4f &c) {
	Vector2f centroid;
	Vector2f D ((a(0) + b(0)) / 2, (a(1) + b(1)) / 2);
	centroid(0) = (2 * D(0) + 1 * c(0)) / 3;
	centroid(1) = (2 * D(1) + 1 * c(1)) / 3;
	return centroid;
}

void Translate(Matrix4f &matrix, const float p_x, const float p_y, const float p_z) {
	Matrix4f trans;
	trans << 1.0, 0.0, 0.0, p_x,
			0.0, 1.0, 0.0f, p_y,
			0.0, 0.0, 1.0, p_z,
			0.0, 0.0, 0.0, 1.0f;
	matrix= trans * matrix;
	return;
}

void Rotate_clock(Matrix4f &matrix) {
	rotate_clock.resize(4,4);
	rotate_clock << float(cos(angle_10)), -float(sin(angle_10)), 0.0f, 0.0f,
		float(sin(angle_10)), float(cos(angle_10)), 0.0f, 0.0f,
		0.0f, 0.0f, 1.0f, 0.0f,
		0.0f, 0.0f, 0.0f, 1.0f;
	matrix = rotate_clock * matrix;
	return;
}

void Rotate_con_clock(Matrix4f &matrix) {
	rotate_clock.resize(4,4);
	rotate_clock << float(cos(angle_10)), float(sin(angle_10)), 0.0f, 0.0f,
		-float(sin(angle_10)), float(cos(angle_10)), 0.0f, 0.0f,
		0.0f, 0.0f, 1.0f, 0.0f,
		0.0f, 0.0f, 0.0f, 1.0f;
	matrix = rotate_clock * matrix;
	return;
}

void Bigger(Matrix4f &matrix) {
	scale_up.resize(4,4);
	scale_up << 1.25f, 0.0f, 0.0f, 0.0f,
			0.0f, 1.25f, 0.0f, 0.0f,
			0.0f, 0.0f, 1.25f, 0.0f,
			0.0f, 0.0f, 0.0f, 1.0f;
	matrix = scale_up * matrix;
	return;
	
}

void Smaller(Matrix4f &matrix) {
	scale_down.resize(4,4);
	scale_down << 0.85f, 0.0f, 0.0f, 0.0f,
			0.0f, 0.85f, 0.0f, 0.0f,
			0.0f, 0.0f, 0.85f, 0.0f,
			0.0f, 0.0f, 0.0f, 1.0f;
	matrix = scale_down * matrix;
	return;
}

Vector3f find_mesh_centroid(const Matrix4f &trans, const MatrixXf &V, const MatrixXi &F) {   // V:<0.5, 0.5, 0.2>  F:<3,6,2>
	Vector3f rst;
	int Vsiz = V.cols();
	int Fsiz = F.cols();
	for (int i = 0; i < Fsiz; ++i){
		Vector3i tri_pos = F.col(i);
		int a = tri_pos(0);
		int b = tri_pos(1);
		int c = tri_pos(2);
		Vector3f A = V.col(a);
		Vector3f B = V.col(b);
		Vector3f C = V.col(c);
		Vector4f A1(A(0), A(1), A(2), 0.0f);
		Vector4f B1(B(0), B(1), B(2), 0.0f);
		Vector4f C1(C(0), C(1), C(2), 0.0f);
		A1 = trans * A1;
		B1 = trans * B1;
		C1 = trans * C1;
		Vector4f cent;
		cent = (A1+B1+C1) / 3;
		rst(0) += cent(0);
		rst(1) += cent(1);
		rst(2) += cent(2);
	}
	rst /= Fsiz;
	return rst;
}

void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {
	// Update the position of the first vertex if the keys 1,2, or 3 are pressed
	if (action == GLFW_PRESS) {
		switch (key) {
			case GLFW_KEY_I:
				Full.conservativeResize(4,points);
				Color.conservativeResize(3, points);
				erase = false;
				move_mode = false;
				draw = true;
				glfwSetCursorPosCallback(window, NULL);
				break;

			case GLFW_KEY_O:
				draw = false;
				move_mode = true;
				erase = false;
				Full.conservativeResize(4,points);
				Color.conservativeResize(3, points);
				transforms.push_back(Matrix4f::Identity());
				transforms.push_back(Matrix4f::Identity());
				transforms.push_back(Matrix4f::Identity());
				transforms.push_back(Matrix4f::Identity());
				break;
			case GLFW_KEY_P:
				erase = true;
				draw = false;
				move_mode = false;
				Full.conservativeResize(4,points);
				Color.conservativeResize(3, points);
				Full.conservativeResize(4,points+4);
				Color.conservativeResize(3, points+4);
				glfwSetCursorPosCallback(window, mouse_move_callback);
				transforms[points] = Matrix4f::Identity();
				transforms[points+1] = Matrix4f::Identity();
				transforms[points+2] = Matrix4f::Identity();
				transforms[points+3] = Matrix4f::Identity();
				
				break;
			case GLFW_KEY_1:
				load_off(DATA_DIR "bunny.off", bunny1.V, bunny1.F);
				bunny1.V_vbo.update(bunny1.V);
				bunny1.F_vbo.update(bunny1.F);
				current_mesh = bunny1;
				break;
			case GLFW_KEY_2:
				load_off(DATA_DIR "bunny.off", bunny2.V, bunny2.F);
				bunny2.V_vbo.update(bunny2.V);
				bunny2.F_vbo.update(bunny2.F);	
				current_mesh = bunny2;			
				break;
			case GLFW_KEY_3:
				load_off(DATA_DIR "bunny.off", bunny3.V, bunny3.F);
				bunny3.V_vbo.update(bunny3.V);
				bunny3.F_vbo.update(bunny3.F);
				current_mesh = bunny3;
				break;
			case GLFW_KEY_4:      
				if (draw){
					Current_Color = C1;
				}
				break;
			case GLFW_KEY_5:
				if (draw){
					Current_Color = C2;
				}
				break;
			case GLFW_KEY_6:
				if (draw){
					Current_Color = C3;
				}
				break;

			case GLFW_KEY_7:
				point_size-=1;
				break;
			case GLFW_KEY_8:
				point_size+=1;
				break;


			case GLFW_KEY_W:
				Translate(current_mesh.trans, 0.0, 0.1, 0.0);
				break;

			case GLFW_KEY_H:

				centroid = find_mesh_centroid(current_mesh.trans, current_mesh.V, current_mesh.F);
				Translate(current_mesh.trans, -centroid(0), -centroid(1), -centroid(2));
				Rotate_clock(current_mesh.trans);
				Translate(current_mesh.trans, centroid(0), centroid(1), centroid(2));
				break;

			case GLFW_KEY_J:

				centroid = find_mesh_centroid(current_mesh.trans, current_mesh.V, current_mesh.F);
				Translate(current_mesh.trans, -centroid(0), -centroid(1), -centroid(2));
				Rotate_con_clock(current_mesh.trans);
				Translate(current_mesh.trans, centroid(0), centroid(1), centroid(2));	
				break;
			case GLFW_KEY_K:

				centroid = find_mesh_centroid(current_mesh.trans, current_mesh.V, current_mesh.F);
				Translate(current_mesh.trans, -centroid(0), -centroid(1), -centroid(2));
				Bigger(current_mesh.trans);
				Translate(current_mesh.trans, centroid(0), centroid(1), centroid(2));	
				break;
			case GLFW_KEY_L:

				centroid = find_mesh_centroid(current_mesh.trans, current_mesh.V, current_mesh.F);
				Translate(current_mesh.trans, -centroid(0), -centroid(1), -centroid(2));
				Smaller(current_mesh.trans);
				Translate(current_mesh.trans, centroid(0), centroid(1), centroid(2));
				break;
			default:
				break;
		}	
	}
	VBO.update(Full);
	VBO_color.update(Color);
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

	bunny1.trans = Matrix4f::Identity();
	bunny2.trans = Matrix4f::Identity();
	bunny3.trans = Matrix4f::Identity();

	// Prepare a dummy bunny object
	// We need to initialize and fill the two VBO (vertex positions + indices),
	// and use a VAO to store their layout when we use our shader program later.
	{
		// Initialize the VBOs
		bunny1.V_vbo.init(GL_FLOAT, GL_ARRAY_BUFFER);
		bunny1.F_vbo.init(GL_UNSIGNED_INT, GL_ELEMENT_ARRAY_BUFFER);


		// Vertex positions
		bunny1.V.resize(3, 3);
		bunny1.V <<
			0, 0.5, -0.5,
			0.5, -0.5, -0.5,
			0, 0, 0;
		bunny1.V_vbo.update(bunny1.V);

		// Triangle indices
		bunny1.F.resize(3, 1);
		bunny1.F << 0, 1, 2;
		bunny1.F_vbo.update(bunny1.F);


		// Create a new VAO for the bunny. and bind it
		bunny1.vao.init();
		bunny1.vao.bind();

		// Bind the element buffer, this information will be stored in the current VAO
		bunny1.F_vbo.bind();

		// The vertex shader wants the position of the vertices as an input.
		// The following line connects the VBO we defined above with the position "slot"
		// in the vertex shader
		program.bindVertexAttribArray("position", bunny1.V_vbo);
		
		// Unbind the VAO when I am done
		bunny1.vao.unbind();
		
	}

	{
		// Initialize the VBOs
		bunny2.V_vbo.init(GL_FLOAT, GL_ARRAY_BUFFER);
		bunny2.F_vbo.init(GL_UNSIGNED_INT, GL_ELEMENT_ARRAY_BUFFER);


		// Vertex positions
		bunny2.V.resize(3, 3);
		bunny2.V <<
			0, 0.5, -0.5,
			0.5, -0.5, -0.5,
			0, 0, 0;
		bunny2.V_vbo.update(bunny2.V);

		// Triangle indices
		bunny2.F.resize(3, 1);
		bunny2.F << 0, 1, 2;
		bunny2.F_vbo.update(bunny2.F);


		// Create a new VAO for the bunny. and bind it
		bunny2.vao.init();
		bunny2.vao.bind();

		// Bind the element buffer, this information will be stored in the current VAO
		bunny2.F_vbo.bind();

		// The vertex shader wants the position of the vertices as an input.
		// The following line connects the VBO we defined above with the position "slot"
		// in the vertex shader
		program.bindVertexAttribArray("position", bunny2.V_vbo);
		
		// Unbind the VAO when I am done
		bunny2.vao.unbind();
		
	}

	{
		// Initialize the VBOs
		bunny3.V_vbo.init(GL_FLOAT, GL_ARRAY_BUFFER);
		bunny3.F_vbo.init(GL_UNSIGNED_INT, GL_ELEMENT_ARRAY_BUFFER);


		// Vertex positions
		bunny3.V.resize(3, 3);
		bunny3.V <<
			0, 0.5, -0.5,
			0.5, -0.5, -0.5,
			0, 0, 0;
		bunny3.V_vbo.update(bunny3.V);

		// Triangle indices
		bunny3.F.resize(3, 1);
		bunny3.F << 0, 1, 2;
		bunny3.F_vbo.update(bunny3.F);


		// Create a new VAO for the bunny. and bind it
		bunny3.vao.init();
		bunny3.vao.bind();

		// Bind the element buffer, this information will be stored in the current VAO
		bunny3.F_vbo.bind();

		// The vertex shader wants the position of the vertices as an input.
		// The following line connects the VBO we defined above with the position "slot"
		// in the vertex shader
		program.bindVertexAttribArray("position", bunny3.V_vbo);
		
		// Unbind the VAO when I am done
		bunny3.vao.unbind();
		
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

	VBO_color.init(GL_FLOAT, GL_ARRAY_BUFFER);

	//initiate Full and Color
	Full.resize(4,3);
	Full << 0,  0.5, -0.5, 
		0.5, -0.5, -0.5,
		0.0, 0.0, 0.0,
		1.0, 1.0, 1.0;
	
	transforms.push_back(Matrix4f::Identity());
	transforms.push_back(Matrix4f::Identity());
	transforms.push_back(Matrix4f::Identity());
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



	glPointSize(point_size);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);
	glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
	glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);
	
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


		if (!draw && !erase && !move_mode){
			// Bind the VAO for the bunny
			bunny1.vao.bind();

			glUniformMatrix4fv(program.uniform("model"), 1, GL_FALSE, bunny1.trans.data());

			// Draw the triangles
			// glDrawArrays(GL_LINE_LOOP, 0, 1000);
			glDrawElements(GL_TRIANGLES, 3 * bunny1.F.cols(), bunny1.F_vbo.scalar_type, 0);

			bunny1.vao.unbind();
		}

		if (!draw && !erase && !move_mode){
			// Bind the VAO for the bunny
			bunny2.vao.bind();

			// Model matrix for the bunny
			// program.bindVertexAttribArray("position", bunny1.V_vbo);

			glUniformMatrix4fv(program.uniform("model"), 1, GL_FALSE, bunny2.trans.data());

			// Draw the triangles
			// glDrawArrays(GL_LINE_LOOP, 0, 1000);
			glDrawElements(GL_TRIANGLES, 3 * bunny2.F.cols(), bunny2.F_vbo.scalar_type, 0);

			bunny2.vao.unbind();
		}

		if (!draw && !erase && !move_mode){
			// Bind the VAO for the bunny
			bunny3.vao.bind();

			// Model matrix for the bunny
			// program.bindVertexAttribArray("position", bunny1.V_vbo);

			glUniformMatrix4fv(program.uniform("model"), 1, GL_FALSE, bunny3.trans.data());

			// Set the uniform value depending on the time difference
			// auto t_now = std::chrono::high_resolution_clock::now();
			// float time = std::chrono::duration_cast<std::chrono::duration<float>>(t_now - t_start).count();
			// glUniform3f(program.uniform("triangleColor"), (float)(sin(time * 4.0f) + 1.0f) / 2.0f, 0.0f, 0.0f);

			// Draw the triangles
			// glDrawArrays(GL_LINE_LOOP, 0, 1000);
			glDrawElements(GL_TRIANGLES, 3 * bunny3.F.cols(), bunny3.F_vbo.scalar_type, 0);

			bunny3.vao.unbind();
		}

		VAO.bind();
		if (erase || move_mode) {
			for (int i = 0; i < Full.cols() - 7; ++i){
			
			glUniformMatrix4fv(program.uniform("model"), 1, GL_FALSE, transforms[i].data());

			glPointSize(point_size);

			glDrawArrays(GL_POINTS, 3+i, 1);

		}
			glUniformMatrix4fv(program.uniform("model"), 1, GL_FALSE, transforms[points].data());
			glDrawArrays(GL_LINE_LOOP, Full.cols() - 4, 4);
		}
		else {
			for (int i = 0; i < Full.cols() - 3; ++i){

			glPointSize(point_size);
			
			glUniformMatrix4fv(program.uniform("model"), 1, GL_FALSE, transforms[i].data());
			glDrawArrays(GL_POINTS, 3+i, 1);

			}
		}
		

		VAO.unbind();
		// Swap front and back buffers
		glfwSwapBuffers(window);

		// Poll for and process events
		glfwPollEvents();
	}

	// Deallocate opengl memory
	program.free();
	bunny1.vao.free(); 
	bunny1.V_vbo.free();
	bunny1.F_vbo.free();
	bunny2.vao.free(); 
	bunny2.V_vbo.free();
	bunny2.F_vbo.free();
	bunny3.vao.free(); 
	bunny3.V_vbo.free();
	bunny3.F_vbo.free();
	VAO.free();
	VBO.free();
	VBO_color.free();

	// Deallocate glfw internals
	glfwTerminate();
	return 0;
}
