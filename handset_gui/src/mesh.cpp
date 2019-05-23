#include "../include/handset_gui/mesh.hpp"

namespace handset_gui{
///////////////////////////////////////////////////////////////////////////////////////////
Mesh::Mesh(int argc, char **argv){
    // Inicia ponteiro da nuvem de pontos
    nuvem_inicial = (PointCloud<PointT>::Ptr) new (PointCloud<PointT>());
}
///////////////////////////////////////////////////////////////////////////////////////////
void Mesh::triangulate(){
    if(nuvem_inicial->size() > 0){
        PointCloud<PointTN>::Ptr cloud_normals (new PointCloud<PointTN>());
        calculateNormalsAndConcatenate(nuvem_inicial, cloud_normals);

        pcl::search::KdTree<PointTN>::Ptr tree2 (new pcl::search::KdTree<PointTN>);

        GreedyProjectionTriangulation<PointTN> gp3;
        gp3.setSearchRadius (0.025);
        gp3.setMu (2.5);
        gp3.setMaximumNearestNeighbors (100);
        gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
        gp3.setMinimumAngle(M_PI/18); // 10 degrees
        gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
        gp3.setNormalConsistency(false);

        gp3.setInputCloud (cloud_normals);
        gp3.setSearchMethod (tree2);
        gp3.reconstruct (triangulos);

        geometry::MeshIO::write("/home/grin/Desktop/mesh.ply", triangulos);

    }
}
///////////////////////////////////////////////////////////////////////////////////////////
void Mesh::calculateNormalsAndConcatenate(PointCloud<PointT>::Ptr cloud, PointCloud<PointTN>::Ptr cloud2){
    NormalEstimation<PointT, Normal> ne;
    ne.setInputCloud(cloud);
    search::KdTree<PointT>::Ptr tree (new search::KdTree<PointT>());
    ne.setSearchMethod(tree);
    PointCloud<Normal>::Ptr cloud_normals (new PointCloud<Normal>());
    ne.setKSearch(20);

    ne.compute(*cloud_normals);

    concatenateFields(*cloud, *cloud_normals, *cloud2);

    vector<int> indicesnan;
    removeNaNNormalsFromPointCloud(*cloud2, *cloud2, indicesnan);
}
///////////////////////////////////////////////////////////////////////////////////////////
void Mesh::saveMesh(std::string nome){
    io::savePolygonFilePLY(nome, *triangulos);
}
///////////////////////////////////////////////////////////////////////////////////////////
void Mesh::setPointCloud(PointCloud::Ptr cloud_in){
    *nuvem_inicial = *cloud_in;
}
///////////////////////////////////////////////////////////////////////////////////////////
} // Fim do namespace handset_gui
