#include "ros/ros.h"
#include <household_objects_database_msgs/GetModelMesh.h>
#include <shape_msgs/Mesh.h>
#include <geometry_msgs/Point.h>
#include <fstream>
#include <cstdlib>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "export_mesh");
	if (argc != 2)
	{
		// TODO Fetch mesh by entering a description like 'can'
		ROS_INFO("usage: export_mesh <ScaledModelMeshId>");
		return 1;
	}

	  ros::NodeHandle n;
	  ros::ServiceClient client = n.serviceClient
			  <household_objects_database_msgs::GetModelMesh>("objects_database_node/get_model_mesh");
	  household_objects_database_msgs::GetModelMesh srv;

	  srv.request.model_id = atoi(argv[1]);

	  if (client.call(srv)) // if service call successful?
	  {
		  switch (srv.response.return_code.code) {
			case 1:
			{
				ROS_ERROR("Database: UNKNOWN ERROR");
				return EXIT_FAILURE;
				break;
			}
			case 2:
			{
				ROS_ERROR("Database: DATABASE_NOT_CONNECTED (maybe login error)");
				return EXIT_FAILURE;
				break;
			}
			case 3:
			{
				ROS_ERROR("Database: DATABASE_QUERY_ERROR");
				return EXIT_FAILURE;
				break;
			}
			case -1:
			{
				// Query to db was successful
				ROS_INFO("Database: SUCCESS");

				// Open file to start exporting mesh
				std::ofstream file;
				std::string fileName(argv[1]);
				fileName.append(".obj");
				file.open(fileName.c_str());

				// material lib, needed to open .obj file
				// mtllib <string PathTo.mtlFile>
				file << "mtllib\n";

				// name the object, here name = entered id
				// TODO: replace object name by description
				file << "o " << argv[1];

				// fetch the mesh, starting with the vertices
				std::vector<geometry_msgs::Point>* vertices = &srv.response.mesh.vertices;

				// dump all the vertices into a file
				for (std::vector<geometry_msgs::Point>::iterator it = vertices->begin();
						it != vertices->end(); it++)
				{
					file << "v " << it->x << " " << it->y << " " << it->z << "\n";
				}

				// dump all the faces into the file
				std::vector<shape_msgs::MeshTriangle>* triangles = &srv.response.mesh.triangles;
				for (std::vector<shape_msgs::MeshTriangle>::iterator it = triangles->begin();
						it != triangles->end(); it++)
				{
					file << "f " << it->vertex_indices[0] << " " << it->vertex_indices[1] << " "
					<< it->vertex_indices[2] << "\n";
				}

				//close file
				file.close();
				ROS_INFO("Export completed");
				break;
			}

			default:
			{
				ROS_ERROR("Database: undefined error code!");
				return EXIT_FAILURE;
				break;
			}
		  }
	  } else {
		  ROS_ERROR("Failed to call service get_model_mesh");
		  return EXIT_FAILURE;
	  }

	  return EXIT_SUCCESS;

}
