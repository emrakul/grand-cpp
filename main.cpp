// #include <igl/readOFF.h>
// #include <GLES2/gl2.h>
// #include <EGL/egl.h>
// #include <igl/opengl/glfw/Viewer.h>

#include <vector>
#include <cmath>
#include <ranges>

#include "ripser.h"
#include "cnpy.h"


#include <gudhi/graph_simplicial_complex.h>
#include <gudhi/Persistent_cohomology.h>
#include <gudhi/distance_functions.h>
#include <gudhi/Simplex_tree.h>
#include <gudhi/Euclidean_strong_witness_complex.h>
#include <gudhi/Persistent_cohomology.h>
#include <gudhi/Points_off_io.h>
#include <gudhi/pick_n_random_points.h>
#include <gudhi/choose_n_farthest_points.h>
#include <gudhi/Rips_complex.h>
#include <gudhi/Tangential_complex.h>

// Eigen::MatrixXd V;
// Eigen::MatrixXi F;

#include <CGAL/Epick_d.h>

using K = CGAL::Epick_d<CGAL::Dynamic_dimension_tag>;
using Simplex_tree = Gudhi::Simplex_tree<Gudhi::Simplex_tree_options_fast_persistence>;
using Proximity_graph = Gudhi::Proximity_graph<Simplex_tree>;
using Point_d = K::Point_d;
using Filtration_value = Simplex_tree::Filtration_value;
using Rips_complex = Gudhi::rips_complex::Rips_complex<Filtration_value>;
using volume = std::vector<std::vector<std::vector<uint8_t>>>;

// using views = ;

template <typename T>
void print_container(std::ostream &os, const T &container, const std::string &delimiter)
{
  std::copy(std::begin(container),
            std::end(container),
            std::ostream_iterator<typename T::value_type>(os, delimiter.c_str()));
}

int main(int argc, char *argv[])
{
  // Load a mesh in OFF format
  // igl::readOFF("./human.off", V, F);
  // auto point_cloud = V;
  

  // Gudhi::Points_off_reader<Point_d> off_reader("./mono_torus.off");

  cnpy::NpyArray arr = cnpy::npy_load("volumetric_data_uint8_400.npy");

  auto vec = arr.data<uint8_t>();

  // print_container<uint8_t> (std::ostream(), vec);
  std::clog << arr.num_vals << " : " << arr.word_size <<  std::endl;
  std::vector<Eigen::Vector3<short>> points;
  points.reserve(arr.num_vals);
  size_t i = 0;
  
  while (i < arr.num_vals)
  {

    if (std::fabs<int>((unsigned int)arr.data<uint8_t>()[i] - 56) < 5)
    {
      auto coord = Eigen::Vector3 <short> ({i % 400, (i / 400) % 400, i / 160000});
      points.push_back(coord);
    }
    i = i + 1;
  };
  std::clog << points.size() << std::endl;
  std::vector <Eigen::Vector3<short> > out;
  std::copy_if(points.begin(), 
    points.end(), 
    std::back_inserter(out),  
    [](Eigen::Vector3<short> a){ 
        return (a[0] < 100) & (a[1] < 100) & (a[2] < 100) ; 
    } 
  );
  //auto filtered = points | std::ranges::views::filter([](Eigen::Vector3<short> a){ return (a[0] < 100) & (a[1] < 100) & (a[2] < 100) ; });
  std::clog << out.size() << std::endl;
  
  

  //Proximity_graph prox_graph = Gudhi::compute_proximity_graph<Simplex_tree>(points,
  //                                                                          threshold,
  //                                                                          Gudhi::Euclidean_distance());
  // Rips_complex Rips_complex(prox_graph);
  
  ripser::read_point_cloud;

  Filtration_value threshold = 2.0;

  Rips_complex rips_complex_from_points(points, threshold, Gudhi::Euclidean_distance());
  
  //Rips_complex::create_complex();
  Simplex_tree simplex_tree;
  rips_complex_from_points.create_complex(simplex_tree, 2);
  std::clog << "Rips complex is of dimension " << simplex_tree.dimension() <<
              " - " << simplex_tree.num_simplices() << " simplices - " <<
              simplex_tree.num_vertices() << " vertices." << std::endl;
  
  //Rips_complex.create_complex(simplex_tree, 2);
  // std::clog << "The complex contains " << simplex_tree.num_simplices() << " simplices \n";
  // std::clog << "   and has dimension " << simplex_tree.dimension() << " \n";
  // std::clog << arr.shape[0] << " : " << arr.shape[1] << std::endl;
  // std::clog << simplex_tree.assign_filtration()
  // Plot the mesh
  // igl::opengl::glfw::Viewer viewer;
  // viewer.data().set_mesh(V, F);
  // viewer.launch();
}
