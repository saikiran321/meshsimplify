#include <iostream>
#include <pcl/io/obj_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include "simplify.h"



using namespace std;

int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PolygonMesh mesh;

    pcl::io::loadPolygonFilePLY("../data/example.ply",mesh);
    pcl::fromPCLPointCloud2(mesh.cloud,*cloud);

    std::cout <<"[Vertices]: "<< cloud->points.size() <<std::endl;
    std::cout <<"[Triangles]: "<<mesh.polygons.size() << std::endl;

    Simplify::vertices.clear();
    Simplify::triangles.clear();


    for(int i=0;i<cloud->points.size();i++)
    {
        Simplify::Vertex V;
        V.p.x=cloud->points[i]._PointXYZ::x;
        V.p.y=cloud->points[i]._PointXYZ::y;
        V.p.z=cloud->points[i]._PointXYZ::z;
        Simplify::vertices.push_back(V);
    }

    for(int i=0;i<mesh.polygons.size();i++)
    {
        Simplify::Triangle t;
        t.v[0] = mesh.polygons[i].vertices[0];
        t.v[1] = mesh.polygons[i].vertices[1];
        t.v[2] = mesh.polygons[i].vertices[2];
        t.attr = 0;
        t.material = -1;
        Simplify::triangles.push_back(t);
    }

    int target_count =  Simplify::triangles.size() >> 1;


    float reduceFraction = 0.5;
    target_count = round((float)Simplify::triangles.size() * 0.5);
    double agressiveness = 7.0;

    clock_t start = clock();
    std::cout << "Input:  vertices, triangles (target )"<< Simplify::vertices.size()<<","<< Simplify::triangles.size()<<","<< target_count<<std::endl;

    int startSize = Simplify::triangles.size();

    Simplify::simplify_mesh(target_count, agressiveness, true);


    if ( Simplify::triangles.size() >= startSize) {
            printf("Unable to reduce mesh.\n");
            return EXIT_FAILURE;
        }
    Simplify::write_obj("test.obj");


    return 0;

}
