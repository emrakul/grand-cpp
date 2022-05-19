#include <igl/readOFF.h>
#include <GLES2/gl2.h>
#include <EGL/egl.h>
#include <igl/opengl/glfw/Viewer.h>
#include <gudhi/distance_functions.h>
#include <gudhi/Simplex_tree.h>
#include <gudhi/Euclidean_strong_witness_complex.h>
#include <gudhi/Persistent_cohomology.h>
#include <gudhi/Points_off_io.h>
#include <gudhi/pick_n_random_points.h>
#include <gudhi/choose_n_farthest_points.h>
#include <gudhi/Rips_complex.h>
#include <iostream>

 
#include <gudhi/graph_simplicial_complex.h>
#include <gudhi/Persistent_cohomology.h>


Eigen::MatrixXd V;
Eigen::MatrixXi F;

#include <CGAL/Epick_d.h>
 
using K = CGAL::Epick_d<CGAL::Dynamic_dimension_tag>;
using Simplex_tree = Gudhi::Simplex_tree<Gudhi::Simplex_tree_options_fast_persistence>;
using Proximity_graph = Gudhi::Proximity_graph<Simplex_tree>;
using Point_d = K::Point_d;
using Filtration_value = Simplex_tree::Filtration_value;
using Rips_complex = Gudhi::rips_complex::Rips_complex<Filtration_value>;


int main(int argc, char *argv[])
{
  // Load a mesh in OFF format
  //igl::readOFF("./human.off", V, F);
  auto point_cloud = V;
  Filtration_value threshold = 0.4;

  Gudhi::Points_off_reader<Point_d> off_reader("./mono_torus.off");
  // Proximity_graph prox_graph = Gudhi::compute_proximity_graph<Simplex_tree>(off_reader.get_point_cloud(),
  //                                                                           threshold,
  //                                                                           Gudhi::Euclidean_distance());


  Rips_complex rips_complex_from_file(off_reader.get_point_cloud(), threshold, Gudhi::Euclidean_distance());
  Simplex_tree simplex_tree;
 
  rips_complex_from_file.create_complex(simplex_tree, 2);
  std::clog << "The complex contains " << simplex_tree.num_simplices() << " simplices \n";
  std::clog << "   and has dimension " << simplex_tree.dimension() << " \n";
  //std::clog << simplex_tree.assign_filtration()
  // Plot the mesh
  igl::opengl::glfw::Viewer viewer;
  //viewer.data().set_mesh(V, F);
  viewer.launch();

}
