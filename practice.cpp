
// #include <GLFW/glfw3.h>
// Linear Algebra Library
#include <Eigen/Dense>
#include <Eigen/Geometry>
// STL headers
#include <chrono>
#include <iostream>
#include <fstream>

using namespace Eigen;


void remove_column(MatrixXf &matrix, int &ColToRemove){
	int numrows = matrix.rows();
	int numcols = matrix.cols()-1;

	if (ColToRemove < numcols){
		matrix.block(0, ColToRemove, numrows, numcols-ColToRemove) = matrix.block(0, ColToRemove+1, numrows, numcols-ColToRemove);
	}
	matrix.conservativeResize(numrows, numcols);
}

Vector2f find_mesh_centroid(MatrixXf &mesh_matrix) {
	Vector2f rst;
	int siz = mesh_matrix.size();
	for (int i = 0; i < mesh_matrix.cols(); ++i) {
		rst(0) += mesh_matrix.col(i)[0];
		rst(1) += mesh_matrix.col(i)[1];
	}
	rst /= siz;
	return rst;
}

int main(){
    MatrixXf matrix;
    matrix.resize(4,3);
	matrix << 0,  0.5, -0.5, 
		0.5, -0.5, -0.5,
		0.0, 0.0, 0.0,
		1.0, 1.0, 1.0;
    std::cerr << "before remove: " << matrix << std::endl;
    int x = 2;
    int y = 1;
    remove_column(matrix, x);
    remove_column(matrix, y);
    std::cerr << "after remove: " << matrix << std::endl;
    std::cerr << "after remove size: " << matrix.cols() << std::endl;
}