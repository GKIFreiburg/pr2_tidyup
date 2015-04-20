#include <ros/ros.h>
#include <octomap/OcTree.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "octree_size_to_collision_space");

    if(argc != 2) {
        ROS_FATAL("Usage: %s octree.bt", argv[0]);
        return 1;
    }

    octomap::OcTree tree(argv[1]);

    double ox, oy, oz;
    double sx, sy, sz;
    tree.getMetricMin(ox, oy, oz);
    tree.getMetricSize(sx, sy, sz);

    printf("    origin_x: %f\n", ox);
    printf("    origin_y: %f\n", oy);
    printf("    origin_z: %f\n", 0.0);
    printf("    size_x: %f\n", sx);
    printf("    size_y: %f\n", sy);
    printf("    size_z: %f\n", 2.0);    //FIXME: Hack, set collision space size hard to 0-2m
    // When taking octomap params the values might be too big for the 
    // collision space to fit into memory

    return 0;
}

