string provider_name        # package or similar that this entry refers to, e.g. moveit_simple_grasps

# Custom Shape types
int32 BOWL=-1

# All values > 0 are shape types from shape_msgs/SolidPrimitive
int32 shape_type


# Grasp types to generate
uint8 ENCLOSURE_GRASPS=1    # From the outside around the body
uint8 EDGE_GRASPS=2         # On an edge for open containers
uint8 ALL_GRASPS=3

uint8 grasp_types

