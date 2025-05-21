// g++ 00_cuboid.cpp -o 00_cuboid.out

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Iso_cuboid_3.h>
#include <iostream>

typedef CGAL::Simple_cartesian<double>  Kernel;
typedef Kernel::Point_3                 Point_3;
typedef Kernel::Iso_cuboid_3            Iso_cuboid_3;

int main()
{
    // Define the diagonal opposite points of the cuboid
    Point_3 p(0.0, 0.0, 0.0);
    Point_3 q(1.0, 2.0, 3.0);

    // Create an Iso_cuboid_3 object using the two points
    Iso_cuboid_3 cuboid(p, q);

    // Access the coordinates of the cuboid's vertices (example)
    std::cout << "Minimum x: " << cuboid.xmin() << std::endl;
    std::cout << "Maximum y: " << cuboid.ymax() << std::endl;

    return 0;
}