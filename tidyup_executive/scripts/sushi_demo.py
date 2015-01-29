#!/usr/bin/env python
import roslib; roslib.load_manifest('sushi_executive')
from pr2_tasks.table_tasks import TableTasks, Setting
from pr2_python.torso import Torso
import rospy
import pr2_python.visualization_tools as vt
from visualization_msgs.msg import MarkerArray
import pymongo as pm
from sushi_turntable.grasp_turntable import GraspTurntable
from pr2_tasks.exceptions import DetectionError
from geometry_msgs.msg import PoseStamped

class SushiParameters:
    def __init__(self, clean_table, eating_table, dirty_table, setting):
        self.clean_table = clean_table
        self.dirty_table = dirty_table
        self.eating_table = eating_table
        self.setting = setting


def get_tables_from_db():
    """
    Get table info from db
    Note that we have a slight difference in naming.  What the db calls the
    'serving table', this script refers to as the 'eating table', and what the
    db calls the 'shelves', this script refers to the as the 'clean table'
    """
    conn = pm.Connection(port=rospy.get_param('warehouse_port', 27017))
    coll = conn.semantic_world_model.sushi_planes
    l = list(coll.find({}))
    if len(l) != 1:
        raise RuntimeError("DB collection {0} was empty".format(coll))

    def get_table(name):
        if name in l[0]:
            return dict(zip(('x', 'y', 'z'), l[0][name]))
        else:
            raise RuntimeError("Stored db hypothesis {0} doesn't include {1}".\
                               format(l[0], name))

    try:
        locs = l[0]['place-locs']
    except KeyError:
        rospy.logwarn('No setting centers found')
        locs = None
    return (get_table('shelves'),
            get_table('dirty-table'),
            get_table('serving-table'),
            locs)

def get_params(tables_from_db=False):
    if tables_from_db:
        clean_table, dirty_table, eating_table, setting_centers = get_tables_from_db()
    else:
        clean_table = rospy.get_param('sushi_groups/clean_table')
        dirty_table = rospy.get_param('sushi_groups/dirty_table')
        eating_table = rospy.get_param('sushi_groups/eating_table')
        setting_centers = None
    setting_places_dict = rospy.get_param('sushi_groups/setting/places')
    setting_places = {}
    for label in setting_places_dict:
        setting_places[label] = (setting_places_dict[label]['x'], setting_places_dict[label]['y'])
    setting_dimensions = rospy.get_param('sushi_groups/setting/dimensions')
    return SushiParameters((clean_table['x'], clean_table['y'], clean_table['z']),
                           (eating_table['x'], eating_table['y'], eating_table['z']),
                           (dirty_table['x'], dirty_table['y'], dirty_table['z']),
                           Setting(setting_places, (setting_dimensions['x'], setting_dimensions['y']),
                                   setting_centers))   

def set_table(tt, setting, clean_table, eating_table):
    try:
        tt.set_setting(setting, clean_table, eating_table)
        return True
    except DetectionError:
        return False

def clear_table(tt, eating_table, dirty_table):
    tt.clear_table(eating_table, dirty_table)
    
def show_setting():
    params = get_params()
    tt = TableTasks()
    places = tt.go_get_setting_places(params.setting, params.eating_table)
    vpub = rospy.Publisher('setting_markers', MarkerArray)
    marray = MarkerArray()
    for p in places:
        rospy.loginfo(p +': (' + str(places[p].pose.position.x)+', '+str(places[p].pose.position.y)+
                      str(places[p].pose.position.z)+')')
        marray.markers.append(vt.marker_at(places[p], ns=p))
    while not rospy.is_shutdown():
        vpub.publish(marray)
        rospy.sleep(0.1)
        
def main():
    #read in the parameters
    params = get_params()
    rospy.loginfo('Shelves are '+str(params.clean_table))
    rospy.loginfo('Centers are '+str(params.setting.centers))
    torso = Torso()
    rospy.loginfo('Raising torso')
    torso.up()
    
    tt = TableTasks()
    gg = GraspTurntable(arm_name='right_arm')
    gg.reset()
#    rospy.loginfo('Clearing table')
#    clear_table(tt, params.eating_table, params.dirty_table)
    rospy.loginfo('Setting table')
    if not set_table(tt, params.setting, params.clean_table, params.eating_table):
        rospy.loginfo('Could not set table')
    rospy.loginfo('Returning to home position')
    tt._tasks.move_arms_to_side()

    rospy.loginfo('Trying to pick up bowl from turntable')
    tt._base.move_to_look(2.2375, -0.5375, 0.5875)
    tt._tasks._head.look_at_map_point(2.2375,-0.5375,0.5875)


    if not rospy.is_shutdown():
        # torso height for Willow = 0.25
        # torso height for Turtlebot = 0.10
        gg.torso.move(0.25)
        gg.pickup_object()
        
        pose_stamped = PoseStamped()
        pose_stamped.pose.position.x = 0.462
        pose_stamped.pose.position.y = 1.589
        pose_stamped.pose.position.z = 0.752
        pose_stamped.pose.orientation.w = 1.0
        pose_stamped.header.frame_id = '/map'

        tt._tasks.go_and_place('right_arm',pose_stamped)


rospy.init_node('sushi_demo_node')
main()
#show_setting()
