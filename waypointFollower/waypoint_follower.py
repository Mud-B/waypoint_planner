import math
import rospy   
import tf    
from tf_conversions import transformations  
import csv
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

class WayPointFollower:
    def __init__(self):
        self._seq = 0   # use to figure out visiualized waypoint
        self._wpIdx = 0
        self._goalIdx = 0
        self._wps = []
        self._cur_pose=[]
        self._filename = 'waypoints.csv'
        self._stop_flag = False
        self.v = Twist()
        self.tf_listener = tf.TransformListener()
        self.markerArray = MarkerArray()
        self.cmd_pub = rospy.Publisher('cmd_vel',Twist,queue_size=10)
        self.rviz_pub = rospy.Publisher('waypoints', MarkerArray, queue_size=10)

    def _load_waypoints(self):
        ''' Load waypoints to a list '''
        with open(self._filename,'r') as f:
            rdr = csv.reader(f)
            for i,row in enumerate(rdr):
                if i>0:     # remove header
                    # convert str to float
                    wp = []
                    for p in row:
                        wp.append(float(p))
                    self._wps.append(wp)
        # record final waypoint index
        self._goalIdx = len(self._wps)-1

    # def _preprocess_wps(self):
    #     ''' not used yet '''
    #     offset = 7
    #     for i in self._wps:
    #         if offset >= self._goalIdx:
    #             break
    #         max = 0
    #         idx = 0
    #         print(len(self._wps[i:offset]))
    #         for wp in self._wps[:offset]:
    #             if wp[-1] > max:
    #                 max = wp[-1]
    #                 idx = wp[0]
    #         print(max,idx)
    #         offset+=1

    def _visualize_wps(self):
        ''' Visualize waypoints in rviz '''
        ourmarkerArray = []
        for wp in self._wps:
            marker = Marker()
            marker.header.frame_id = "/map"
            marker.header.stamp = rospy.Time.now()
            marker.header.seq = self._seq
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.id = wp[0]
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = wp[1]
            marker.pose.position.y = wp[2]
            marker.pose.position.z = 0
            
            ourmarkerArray.append(marker)
        self._seq += 1
        self.markerArray = ourmarkerArray
        self.rviz_pub.publish(self.markerArray)

    def _get_current_pos(self):
        ''' Get current position '''
        try:
            trans, rot = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return False
        roll,pitch,yaw = transformations.euler_from_quaternion(rot)
        x = trans[0]
        y = trans[1]
        self._cur_pose = [x,y,yaw]
        if len(self._cur_pose):
            return True
        else:
            return False

    def _get_closet_wpIdx(self): 
        ''' get closest waypoint from remaining waypoints '''  
        cur_wpIdx = self._wpIdx
        if self._get_current_pos(): 
            min = math.sqrt(math.pow(self._cur_pose[0]-self._wps[0][1],2)+math.pow(self._cur_pose[1]-self._wps[0][2],2))
            for wp in self._wps[cur_wpIdx:-1]:
                dst = math.sqrt(math.pow(self._cur_pose[0]-wp[1],2)+math.pow(self._cur_pose[1]-wp[2],2))
                if dst < min:
                    min = dst
                    self._wpIdx = int(wp[0])

    def _reach_the_goal(self):
        ''' Check if reach the goal '''
        if self._wpIdx+1 == self._goalIdx:
            self._stop_flag = True
            return True
        return False
        
    def _send_vel_cmd(self):
        ''' Compute and send velocity command '''
        self.v.linear.x = 0.1
        prev_wp_angle = self._wps[self._wpIdx + 1][-1] - self._cur_pose[-1]
        back_wp_angle = self._wps[self._wpIdx][-1] - self._cur_pose[-1]
        self.v.angular.z = (prev_wp_angle * 4 + back_wp_angle * 6)/10
        if self.v.angular.z > 0.5:
            self.v.angular.z = 0.8 * self.v.angular.z
        self.cmd_pub.publish(self.v)

    def _stop(self):
        ''' Send stop command '''
        self.v.linear.x = 0
        self.v.angular.z = 0
        while self._stop_flag:
            self.cmd_pub.publish(self.v)

    def run(self):
        self._load_waypoints()
        # self._preprocess_wps()
        r = rospy.Rate(2)
        while not rospy.is_shutdown():
            self._visualize_wps()
            if self._reach_the_goal():
                self._stop()
            if self._get_current_pos():
                self._get_closet_wpIdx()
                self._send_vel_cmd()
            r.sleep()

if __name__ == "__main__":
    rospy.init_node('wayPoint_follower')   
    follower = WayPointFollower()  
    follower.run()
