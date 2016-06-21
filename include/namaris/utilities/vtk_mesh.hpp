#ifndef VTK_MESH_UTILITIES_HPP
#define VTK_MESH_UTILITIES_HPP

// STD includes
#include <iostream>

// VTK includes
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkPointData.h>
#include <vtkIdList.h>
#include <vtkCell.h>
#include <vtkCleanPolyData.h>
#include <vtkPLYReader.h>
#include <vtkPLYWriter.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>

// Eigen includes
#include <eigen3/Eigen/Dense>

// My includes
#include <namaris/utilities/graph.hpp>

// PCL includes
#include <pcl/io/vtk_lib_io.h>

namespace utl
{
  namespace vtk
  {
    /** \brief Read a PLY mesh file
      * \param[in] filename mesh file name
      * \param[out] mesh  vtk polydata object
      */
    inline
    bool loadMeshPly(const std::string &filename, vtkSmartPointer<vtkPolyData> &mesh)
    {
      mesh = vtkSmartPointer<vtkPolyData>::New();
      vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();
      reader->SetFileName(filename.c_str());
      reader->Update();
      mesh = reader->GetOutput();   
      
      return true;
    }
    
    /** \brief Save a PLY mesh file. Point RGB data will be saved provided it is unisgned char.
      * \param[in] filename mesh file name
      * \param[in] mesh  vtk polydata object
      */
    inline
    bool saveMeshPly(const std::string &filename, vtkSmartPointer<vtkPolyData> &mesh)
    {
      // Find if mesh has RGB data or not
      bool hasRGB = false;
      std::string RGBdataArrayName = "";
      
      for (size_t i = 0; i < static_cast<size_t>(mesh->GetPointData()->GetNumberOfArrays()); i++)
      {
        if (  mesh->GetPointData()->GetArray(i)->GetDataType() == VTK_UNSIGNED_CHAR &&  
              mesh->GetPointData()->GetArray(i)->GetNumberOfComponents() == 3 &&  
              mesh->GetPointData()->GetArray(i)->GetNumberOfTuples() == mesh->GetNumberOfPoints()
            )
        {
          hasRGB = true;
          RGBdataArrayName = mesh->GetPointData()->GetArrayName(i);
        }
      }
      
      // Write data
      vtkSmartPointer<vtkPLYWriter> plyWriter = vtkSmartPointer<vtkPLYWriter>::New();
      plyWriter->SetFileName(filename.c_str());
      plyWriter->SetInput(mesh);
      if (hasRGB)
        plyWriter->SetArrayName(RGBdataArrayName.c_str());
      return (plyWriter->Write() == 1);
    }

    /** \brief Transform mesh
      * \param[in]  mesh_in input mesh
      * \param[out] mesh_out output mesh
      * \param[in]  T affine transform
      */
    inline
    void transformMesh(const vtkSmartPointer<vtkPolyData> &mesh_in, vtkSmartPointer<vtkPolyData> &mesh_out, const Eigen::Affine3f &T)
    {
      // Prepare output variable
      if (mesh_in != mesh_out)
        mesh_out = vtkSmartPointer<vtkPolyData>::New();
      
      // Get transformation matrix
      vtkSmartPointer<vtkMatrix4x4> T_matrix_vtk = vtkMatrix4x4::New();
      for (size_t i = 0; i < 4; i++)
        for (size_t j = 0; j < 4; j++)
          T_matrix_vtk->SetElement(i, j, static_cast<double>(T(i,j)));
      
      // Create VTK transformation
      vtkSmartPointer<vtkTransform> T_vtk = vtkSmartPointer<vtkTransform>::New();
      T_vtk->SetMatrix(T_matrix_vtk);
          
      // Transform mesh
      vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter =
        vtkSmartPointer<vtkTransformPolyDataFilter>::New();
      transformFilter->SetInput(mesh_in);
      transformFilter->SetTransform(T_vtk);
      transformFilter->Update();    
      mesh_out = transformFilter->GetOutput();
    }
    
    /** \brief Find connctivity graph of mesh vertices
      * \param[in] mesh  input mesh
      * \param[in] CCs connected components of the mesh
      */
    inline
    void getMeshVertexConnectivity (const vtkSmartPointer<vtkPolyData> &mesh, graph::Graph &vertex_connectivity)
    {
      // Construct vertex connectivity graph  
      vertex_connectivity = graph::Graph(mesh->GetNumberOfPoints());
      
      for (size_t cellId = 0; cellId < static_cast<size_t>(mesh->GetNumberOfCells()); cellId++)
      {
        // Get all cell vertices
        int numVtx = mesh->GetCell(cellId)->GetNumberOfPoints();
        std::vector<int> vtxIds(numVtx);
        for (size_t vtxIdIt = 0; vtxIdIt < static_cast<size_t>(numVtx); vtxIdIt++)
          vtxIds[vtxIdIt] = static_cast<int>(mesh->GetCell(cellId)->GetPointIds()->GetId(vtxIdIt));
        
        // Add corresponding edges
        for (size_t srcVtxId = 0; srcVtxId < static_cast<size_t>(numVtx); srcVtxId++)
          for (size_t tgtVtxId = srcVtxId+1; tgtVtxId < static_cast<size_t>(numVtx); tgtVtxId++)
            graph::addEdge(vtxIds[srcVtxId], vtxIds[tgtVtxId], vertex_connectivity);
      }    
    }  
    
    /** \brief Find connected components of the mesh
      * \param[in] mesh  input mesh
      * \param[in] CCs point incices of connected components of the mesh
      */
    inline
    void getMeshConnectedComponents (const vtkSmartPointer<vtkPolyData> &mesh, std::vector<std::vector<int> > &CCs)
    {
      graph::Graph vertexConnectivity;
      getMeshVertexConnectivity(mesh, vertexConnectivity);
      CCs = graph::getConnectedComponents(vertexConnectivity);
    }
    
    /** \brief Remove a set of vertices from mesh
      * \param[in] mesh_in  input mesh
      * \param[in] vertex_ids indeces of vertices to be removed
      * \param[out] mesh_out  output mesh
      */
    inline
    void meshRemoveVertices(const vtkSmartPointer<vtkPolyData> &mesh_in, const std::vector<int> &vertex_ids, vtkSmartPointer<vtkPolyData> &mesh_out)
    {
      // First prepare output mesh
      if (mesh_in != mesh_out)
      {
        mesh_out = vtkSmartPointer<vtkPolyData>::New();
        mesh_out->DeepCopy(mesh_in);
      }
      mesh_out->BuildLinks();
      
      // Get a list of polygons that need to be removed
      std::vector<int> poly_ids;
      
      for (auto vtxIt = vertex_ids.begin(); vtxIt != vertex_ids.end(); vtxIt++)
      {     
        vtkSmartPointer<vtkIdList> cellIds = vtkSmartPointer<vtkIdList>::New();
        mesh_out->GetPointCells(*vtxIt, cellIds);
        
        for (size_t cellIdIt = 0; cellIdIt < static_cast<size_t>(cellIds->GetNumberOfIds()); cellIdIt++)
          poly_ids.push_back(cellIds->GetId(cellIdIt));
      }
      
      std::sort(poly_ids.begin(), poly_ids.end());
      poly_ids.erase(std::unique(poly_ids.begin(), poly_ids.end()), poly_ids.end());

      
      // Remove vertices and polygons
      for (auto vtxIt = vertex_ids.begin(); vtxIt != vertex_ids.end(); vtxIt++)
        mesh_out->DeletePoint(*vtxIt);  
      for (auto polyIt = poly_ids.begin(); polyIt != poly_ids.end(); polyIt++)
        mesh_out->DeleteCell(*polyIt);
      
      mesh_out->RemoveDeletedCells();    
      
      vtkSmartPointer<vtkCleanPolyData> cleaner = vtkSmartPointer<vtkCleanPolyData>::New ();
      cleaner->SetTolerance(0.0);
  //     cleaner->ConvertLinesToPointsOff();
  //     cleaner->ConvertPolysToLinesOff();
  //     cleaner->ConvertStripsToPolysOff();
      cleaner->SetInput(mesh_out);
      cleaner->Update();
      mesh_out = cleaner->GetOutput();
    }
    
    /** \brief Remove all vertices that are not specified
      * \param[in] mesh_in  input mesh
      * \param[in] vertex_ids indeces of vertices to be kept
      * \param[out] mesh_out  output mesh
      */
    inline
    void meshKeepVertices(const vtkSmartPointer<vtkPolyData> &mesh_in, const std::vector<int> &vertex_ids, vtkSmartPointer<vtkPolyData> &mesh_out)
    {
      // Find all vertices that must be removed
      std::vector<int> vertex_ids_all (mesh_in->GetNumberOfPoints());
      for (size_t fii = 0; fii < vertex_ids_all.size(); fii++)  // fii = full indices iterator
        vertex_ids_all[fii] = fii;    
      
      std::vector<int> vertex_ids_copy = vertex_ids;
      std::sort(vertex_ids_copy.begin(), vertex_ids_copy.end());
      
      std::vector<int> vertex_ids_inverted;
      std::set_difference(vertex_ids_all.begin(), vertex_ids_all.end(), vertex_ids_copy.begin(), vertex_ids_copy.end(), std::back_inserter(vertex_ids_inverted));
      
      // Remove points
      meshRemoveVertices(mesh_in, vertex_ids_inverted, mesh_out);
    }

    /** \brief Cleanup mesh by removing all points that are not part of any polygon
      * \param[in] mesh_in  input mesh
      * \param[out] mesh_out  output mesh
      */
    inline
    void meshRemoveIsolatedPoints(const vtkSmartPointer<vtkPolyData> &mesh_in, vtkSmartPointer<vtkPolyData> &mesh_out)
    {
      // Find all poins that belong to at least one polygon
      std::vector<int> polyPointIds (0);
      for (size_t cellId = 0; cellId < static_cast<size_t>(mesh_in->GetNumberOfCells()); cellId++)
      {
        int numVtx = mesh_in->GetCell(cellId)->GetNumberOfPoints();   
        for (size_t vtxIt = 0; vtxIt < static_cast<size_t>(numVtx); vtxIt++)
          polyPointIds.push_back(mesh_in->GetCell(cellId)->GetPointIds()->GetId(vtxIt));      
      }
      
      // Only leave unique indices
      std::sort(polyPointIds.begin(), polyPointIds.end());
      polyPointIds.erase(std::unique(polyPointIds.begin(), polyPointIds.end()), polyPointIds.end());

      // Only keep good points
      meshKeepVertices(mesh_in, polyPointIds, mesh_out);
    }
    
    /** \brief Cleanup mesh by merging points that are too close to each other
      * \param[in] mesh_in  input mesh
      * \param[out] mesh_out  output mesh
      * \param[in] distance_thresh minimum distance between two points that are too close
      */
    inline
    void meshMergeClosePoints(const vtkSmartPointer<vtkPolyData> &mesh_in, vtkSmartPointer<vtkPolyData> &mesh_out, float distance_thresh = 1e-6)
    {
      vtkSmartPointer<vtkCleanPolyData> cleaner = vtkSmartPointer<vtkCleanPolyData>::New ();
      cleaner->SetTolerance (distance_thresh);
      cleaner->SetInput (mesh_in);
      cleaner->PointMergingOn ();
      cleaner->Update ();
      mesh_out = cleaner->GetOutput();    
    }
    
    /** \brief Cleanup mesh by removing connected components smaller than a specified size
      * \param[in] mesh_in  input mesh
      * \param[out] mesh_out  output mesh
      * \param[in] min_cc_size minimum distance between two points that are too close
      */
    inline
    void meshRemoveSmallCCs(const vtkSmartPointer<vtkPolyData> &mesh_in, vtkSmartPointer<vtkPolyData> &mesh_out, size_t min_cc_size)
    {
      // Find connected components
      std::vector<std::vector<int> > CCs;
      getMeshConnectedComponents(mesh_in, CCs);
      
      // Find indices of poitns in small connected components
      std::vector<int> good_vertices;
      for (size_t ccId = 0; ccId < CCs.size(); ccId++)
      {     
        // Check size
        if (CCs[ccId].size() < min_cc_size)
          continue;
        
        good_vertices.insert(good_vertices.end(), CCs[ccId].begin(), CCs[ccId].end());
      }
      
      // Remove bad vertices
      meshKeepVertices(mesh_in, good_vertices, mesh_out);
    }
    
    /** \brief Cleanup mesh by merging points that are too close to each other and
    * removing connected components that are too close to each other.
    * \param[in] mesh_in input mesh
    * \param[in] mesh_out cleaned mesh
    * \param[in] distance_thresh minimum distance between two points that are too close
    * \param[in] min_cc_size minimum number of points in a connected component
    */
    bool cleanupMesh(const vtkSmartPointer<vtkPolyData> &mesh_in, vtkSmartPointer<vtkPolyData> &mesh_out, float distance_thresh = 1e-6, int min_cc_size = 100)
    {
      std::cout << std::endl;  
      std::cout << "Original mesh" << std::endl;
      std::cout << "  points:   " << mesh_in->GetNumberOfPoints() << std::endl;
      std::cout << "  polygons: " << mesh_in->GetNumberOfPolys() << std::endl;
      
      //----------------------------------------------------------------------------
      // Merge points that are too close to each other
      //----------------------------------------------------------------------------

      std::cout << std::endl;
      std::cout << "Merging points that are too close to each other" << std::endl;
        
      meshMergeClosePoints(mesh_in, mesh_out);

      std::cout << "  points:   " << mesh_out->GetNumberOfPoints() << std::endl;
      std::cout << "  polygons: " << mesh_out->GetNumberOfPolys() << std::endl;
      
      //----------------------------------------------------------------------------
      // Remove connected components that are too small
      //----------------------------------------------------------------------------
        
      std::cout << std::endl;
      std::cout << "Removing connected components that are too small" << std::endl;;

      meshRemoveSmallCCs(mesh_out, mesh_out, min_cc_size);

      std::cout << "  points:   " << mesh_out->GetNumberOfPoints() << std::endl;
      std::cout << "  polygons: " << mesh_out->GetNumberOfPolys() << std::endl;
      
      return true;
    }
    
    /** \brief Convert vtk mesh to pointcloud
      * \param[in] mesh input mesh
      * \param[out] cloud output cloud
      * \return number of points in the cloud
      * \NOTE: adapted from pcl::io::vtk2mesh
      * \NOTE: all point data available in vtk mesh is copied. That includes points that do not belong to any polygons.
      */
    template <typename PointT>
    inline
    int vtkMesh2pclCloud (const vtkSmartPointer<vtkPolyData> &mesh, typename pcl::PointCloud<PointT> &cloud)
    { 
      // Iniitalize empty cloud
      cloud.points.clear();
      cloud.width = cloud.height = 0;
      cloud.is_dense = true;

      // First check that cloud has point data field
      if (pcl::getFieldsList(cloud).find("x y z") == std::string::npos)
      {
        std::cout << "[utl::vtkMesh2pclCloud] cloud does not has xyz field, cannot convert" << std::endl;
        return -1;
      }

      // Check that mesh has points
      vtkSmartPointer<vtkPoints> mesh_points = mesh->GetPoints ();
      vtkIdType nr_points = mesh_points->GetNumberOfPoints ();

      if (nr_points == 0)
        return (0);
      
      // Get XYZ information
      cloud.points.resize (nr_points);
      cloud.width   = nr_points;
      cloud.height  = 1;
      double point_xyz[3];
      for (vtkIdType i = 0; i < mesh_points->GetNumberOfPoints (); i++)
      {
        mesh_points->GetPoint (i, &point_xyz[0]);
        cloud.points[i].x = static_cast<float> (point_xyz[0]);
        cloud.points[i].y = static_cast<float> (point_xyz[1]);
        cloud.points[i].z = static_cast<float> (point_xyz[2]);
      }
      
      // Then color information, if any    
      if (pcl::getFieldsList(cloud).find("rgb") != std::string::npos)
      {
        vtkUnsignedCharArray* poly_colors = NULL;
        if (mesh->GetPointData() != NULL)
          poly_colors = vtkUnsignedCharArray::SafeDownCast (mesh->GetPointData ()->GetScalars ("Colors"));

        // some applications do not save the name of scalars (including PCL's native vtk_io)
        if (!poly_colors && mesh->GetPointData () != NULL)
          poly_colors = vtkUnsignedCharArray::SafeDownCast (mesh->GetPointData ()->GetScalars ("scalars"));

        if (!poly_colors && mesh->GetPointData () != NULL)
          poly_colors = vtkUnsignedCharArray::SafeDownCast (mesh->GetPointData ()->GetScalars ("RGB"));

        // TODO: currently only handles rgb values with 3 components
        if (poly_colors && (poly_colors->GetNumberOfComponents () == 3))
        {
          unsigned char point_color[3];
          for (vtkIdType i = 0; i < mesh_points->GetNumberOfPoints (); i++)
          {
            poly_colors->GetTupleValue (i, &point_color[0]);
            cloud.points[i].r = point_color[0];
            cloud.points[i].g = point_color[1];
            cloud.points[i].b = point_color[2];
          }
        }
      }
      
      // Finally normal information, if any
      if (pcl::getFieldsList(cloud).find("normal_x normal_y normal_z") != std::string::npos)
      {
        vtkFloatArray* normals = NULL;
        if (mesh->GetPointData () != NULL)
          normals = vtkFloatArray::SafeDownCast (mesh->GetPointData ()->GetNormals ());
        if (normals != NULL)
        {
          for (vtkIdType i = 0; i < mesh_points->GetNumberOfPoints (); i++)
          {
            float normal[3];
            normals->GetTupleValue (i, normal);
            cloud.points[i].normal_x = normal[0];
            cloud.points[i].normal_y = normal[1];
            cloud.points[i].normal_z = normal[2];
          }
        }
      }
      
      return 0;
    }
    
    /** \brief Convert vtk mesh to pointcloud and estimate vertex normals from face normals
      * \param[in] mesh input mesh
      * \param[out] cloud output cloud
      * \return number of points in the cloud
      */
    inline
    int getVertexNormalsFromMeshFaceNormals (const vtkSmartPointer<vtkPolyData> &mesh, pcl::PointCloud<pcl::Normal> &normals)
    {
      
      // Initialize empty normal cloud
      normals.points.clear();
      normals.height = normals.width = 0;
      normals.is_dense = true;
      
      // Check that mesh is not empty
      int nr_points = mesh->GetNumberOfPoints();
      int nr_polys  = mesh->GetNumberOfPolys();
      
      if (nr_points == 0)
      {
        std::cout << "[utl::getVertexNormalsFromMeshFaceNormals] mesh has no points" << std::endl;
        return 0;
      }
      
      if (nr_polys == 0)
      {
        std::cout << "[utl::getVertexNormalsFromMeshFaceNormals] mesh has no polygons, cannot extract normals" << std::endl;
        return 0;
      }
      
      // Extract normals
      normals.resize(nr_points);
      normals.width   = nr_points;
      normals.height  = 1;

      double point_xyz[3];
      std::vector<int> vertexFaceNeighbourCount(normals.size(), 0);     
          
      for (size_t cellId = 0; cellId < static_cast<size_t>(mesh->GetNumberOfCells()); cellId++)
      {
        // Only use polygons with 3 vertices
        int numVtx = mesh->GetCell(cellId)->GetNumberOfPoints();
        
        if (numVtx != 3)
          continue;
        
        // Get face points
        std::vector<int> vtxIds(numVtx);
        std::vector<Eigen::Vector3f> polyPoints(3);
        for (size_t vtxIdIt = 0; vtxIdIt < static_cast<size_t>(numVtx); vtxIdIt++)
        {
          vtxIds[vtxIdIt] = static_cast<int>(mesh->GetCell(cellId)->GetPointIds()->GetId(vtxIdIt));
          mesh->GetPoint (vtxIds[vtxIdIt], &point_xyz[0]);
          polyPoints[vtxIdIt] = Eigen::Vector3d(point_xyz[0], point_xyz[1], point_xyz[2]).cast<float>();
        }
        
        // Compute face normal
        Eigen::Vector3f line1 = polyPoints[1] - polyPoints[0];
        Eigen::Vector3f line2 = polyPoints[2] - polyPoints[0];
        Eigen::Vector3f normal = - line2.cross(line1);
        normal.normalize();
        
        if (!std::isfinite(normal[0]) || !std::isfinite(normal[1]) || !std::isfinite(normal[2]))
        {
          std::cout << "[utl::getVertexNormalsFromMeshFaceNormals] face normal is NaN! Ignoring it."  << std::endl;
          continue;
        }
        
        // Assign normals
        for (size_t vtxIdIt = 0; vtxIdIt < static_cast<size_t>(numVtx); vtxIdIt++)
        {
          normals.points[vtxIds[vtxIdIt]].getNormalVector3fMap() += normal;
          vertexFaceNeighbourCount[vtxIds[vtxIdIt]]++;
        }
      }
      
      // Normalize normals by the number of faces that were used to estimate them
      for (size_t i = 0; i < normals.size(); i++)
        normals.points[i].getNormalVector3fMap() /= vertexFaceNeighbourCount[i];
      
      // Check if there are divisions by zero
      if (std::find(vertexFaceNeighbourCount.begin(), vertexFaceNeighbourCount.end(), 0) != vertexFaceNeighbourCount.end())
        std::cout << "[utl::getVertexNormalsFromMeshFaceNormals] there are points in the mesh that do not belong to any polygon. These points have their normals set to NaN" << std::endl;
      
      return nr_points;
    }
  }
}

#endif  // VTK_MESH_UTILITIES_HPP