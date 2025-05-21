// g++ 01_boolean.cpp -o 01_boolean.out

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Iso_cuboid_3.h>
#include <CGAL/Nef_polyhedron_3.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/IO/Polyhedron_iostream.h>
#include <iostream>

// typedef CGAL::Simple_cartesian<double>  Kernel_simple;
typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;
typedef CGAL::Iso_cuboid_3<Kernel> /*------------------*/ Iso_cuboid_3;
typedef CGAL::Nef_polyhedron_3<Kernel> /*--------------*/ Nef_polyhedron_3;
typedef CGAL::Polyhedron_3<Kernel> /*------------------*/ Polyhedron_3;
typedef Kernel::Point_3 /*-----------------------------*/ Point_3;
typedef Kernel::Iso_cuboid_3 /*------------------------*/ Iso_cuboid_3;

int main()
{
    // Define the diagonal opposite points of the cuboid
    Point_3 p(0.0, 0.0, 0.0);
    Point_3 q(1.0, 2.0, 3.0);

    Point_3 r(0.75, 0.75, 0.75);
    Point_3 s(4.00, 4.00, 4.00);

    // Create an Iso_cuboid_3 object using the two points
    Iso_cuboid_3 cuboid1(p, q);
    Iso_cuboid_3 cuboid2(r, s);

    // Construct Nef polyhedra from the cuboids
    Nef_polyhedron_3 nef1(cuboid1);
    Nef_polyhedron_3 nef2(cuboid2);

    // Perform boolean operations
    Nef_polyhedron_3 difference_nef = nef1 - nef2; // Difference

    Polyhedron_3 difference_polyhedron;
    difference_nef.convert_to_polyhedron( difference_polyhedron );

    std::ofstream output_file_difference( "difference.off" );
    output_file_difference << difference_polyhedron;
    output_file_difference.close();
    return 0;
}
/*
In file included from /usr/include/boost/iterator/iterator_categories.hpp:14,
                 from /usr/include/boost/iterator/iterator_facade.hpp:13,
                 from /usr/include/boost/range/iterator_range_core.hpp:27,
                 from /usr/include/boost/lexical_cast.hpp:30,
                 from /usr/include/CGAL/exceptions.h:28,
                 from /usr/include/CGAL/assertions_impl.h:29,
                 from /usr/include/CGAL/assertions.h:334,
                 from /usr/include/CGAL/basic.h:26,
                 from /usr/include/CGAL/Cartesian/Cartesian_base.h:20,
                 from /usr/include/CGAL/Simple_cartesian.h:20,
                 from 01_boolean.cpp:3:
/usr/include/boost/mpl/eval_if.hpp: In instantiation of ‘struct boost::mpl::eval_if<boost::detail::has_vertex_property_type<CGAL::Iso_cuboid_3<CGAL::Epeck>, mpl_::bool_<false> >, boost::detail::get_vertex_property_type<CGAL::Iso_cuboid_3<CGAL::Epeck> >, boost::no_property>’:
/usr/include/boost/graph/graph_traits.hpp:315:8:   required from ‘struct boost::vertex_property_type<CGAL::Iso_cuboid_3<CGAL::Epeck> >’
/usr/include/boost/graph/properties.hpp:225:12:   required from ‘struct boost::detail::vertex_property_map<CGAL::Iso_cuboid_3<CGAL::Epeck>, boost::face_index_t>’
/usr/include/boost/graph/properties.hpp:234:8:   required from ‘struct boost::property_map<CGAL::Iso_cuboid_3<CGAL::Epeck>, boost::face_index_t, void>’
/usr/include/CGAL/Nef_3/polygon_mesh_to_nef_3.h:332:64:   required from ‘void CGAL::polygon_mesh_to_nef_3(SM&, SNC_structure&) [with SM = CGAL::Iso_cuboid_3<CGAL::Epeck>; SNC_structure = CGAL::SNC_structure<CGAL::Epeck, CGAL::SNC_indexed_items, bool>]’
/usr/include/CGAL/Nef_polyhedron_3.h:627:54:   required from ‘CGAL::Nef_polyhedron_3<K, I, Mk>::Nef_polyhedron_3(const PolygonMesh&) [with PolygonMesh = CGAL::Iso_cuboid_3<CGAL::Epeck>; Kernel_ = CGAL::Epeck; Items_ = CGAL::SNC_indexed_items; Mark_ = bool]’
01_boolean.cpp:33:34:   required from here
/usr/include/boost/mpl/eval_if.hpp:38:31: error: no type named ‘type’ in ‘boost::mpl::eval_if<boost::detail::has_vertex_property_type<CGAL::Iso_cuboid_3<CGAL::Epeck>, mpl_::bool_<false> >, boost::detail::get_vertex_property_type<CGAL::Iso_cuboid_3<CGAL::Epeck> >, boost::no_property>::f_’ {aka ‘struct boost::no_property’}
   38 |     typedef typename f_::type type;
      |                               ^~~~
In file included from /usr/include/CGAL/Nef_polyhedron_3.h:48,
                 from 01_boolean.cpp:6:
/usr/include/CGAL/Nef_3/polygon_mesh_to_nef_3.h: In instantiation of ‘void CGAL::polygon_mesh_to_nef_3(SM&, SNC_structure&) [with SM = CGAL::Iso_cuboid_3<CGAL::Epeck>; SNC_structure = CGAL::SNC_structure<CGAL::Epeck, CGAL::SNC_indexed_items, bool>]’:
/usr/include/CGAL/Nef_polyhedron_3.h:627:54:   required from ‘CGAL::Nef_polyhedron_3<K, I, Mk>::Nef_polyhedron_3(const PolygonMesh&) [with PolygonMesh = CGAL::Iso_cuboid_3<CGAL::Epeck>; Kernel_ = CGAL::Epeck; Items_ = CGAL::SNC_indexed_items; Mark_ = bool]’
01_boolean.cpp:33:34:   required from here
/usr/include/CGAL/Nef_3/polygon_mesh_to_nef_3.h:332:64: error: no type named ‘type’ in ‘struct boost::property_map<CGAL::Iso_cuboid_3<CGAL::Epeck>, boost::face_index_t, void>’
  332 |   typedef typename boost::property_map<SM, face_index_t>::type FIMap;
      |                                                                ^~~~~
/usr/include/CGAL/Nef_3/polygon_mesh_to_nef_3.h:334:75: error: no type named ‘type’ in ‘struct boost::property_map<CGAL::Iso_cuboid_3<CGAL::Epeck>, boost::halfedge_index_t, void>’
  334 |   typedef typename boost::property_map<SM, boost::halfedge_index_t>::type HIMap;
      |                                                                           ^~~~~
*/