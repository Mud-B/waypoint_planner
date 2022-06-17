import rospy   
from tf_conversions import transformations     
import tf    
import csv

class WayPointSaver:
    def __init__(self):
        self._pIdx = 0
        self._wp = []
        self._header = ['Idx','x','y','yaw']
        self._filename = 'waypoints.csv'
        self.tf_listener = tf.TransformListener()
        with open(self._filename,'w') as f:
                wtr = csv.writer(f)
                wtr.writerow(self._header)
 
    def _get_wp(self):
        ''' get current pose '''
        try:
            trans, rot = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return
        roll,pitch,yaw = transformations.euler_from_quaternion(rot)  
        x = trans[0]
        y = trans[1]
        if x and y:
            self._wp = [self._pIdx, x, y, yaw]
            return True
        else:
            return False

    def _process_wp(self):
        ''' process points '''
        # your can put your own code here
        self._pIdx += 1
        
    def _write_file(self):
        ''' write point to file '''
        with open(self._filename,'a') as f:
            wtr = csv.writer(f)
            wtr.writerow(self._wp)

    def run(self):
        ''' main loop '''
        if self._get_wp():
            self._process_wp()
            self._write_file()

if __name__ == "__main__":
    rospy.init_node('wayPoint_saver')   
    saver = WayPointSaver()
    r = rospy.Rate(2)
    while not rospy.is_shutdown():  
        saver.run()
        r.sleep()
