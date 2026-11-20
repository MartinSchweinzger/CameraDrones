#!/usr/bin/env python3
import time
import errno
from socket import error as socket_error
#import traceback
import rospy
import logging
import sys

# setup logging logger
file_handler = logging.FileHandler(filename='/tmp/tmp.wait_for_roscore.log')
stdout_handler = logging.StreamHandler(sys.stdout)
handlers = [file_handler, stdout_handler]

logging.basicConfig(
    level=logging.DEBUG, 
    format='[%(asctime)s] {%(filename)s:%(lineno)d} %(levelname)s - %(message)s',
    handlers=handlers
)

logger = logging.getLogger('LOGGER_NAME')

def wait_for_roscore():# wait_for_ros_master()
  """
  Solution taken from here: https://answers.ros.org/question/282587/dont-block-on-rospyinit_node/?answer=282595#post-id-282595
  """
  ros_master_available = False
  while not rospy.core.is_shutdown() and not ros_master_available:
    #print("Waiting for roscore...")
    logging.info("Waiting for roscore...")
    try:  
      # query ROS Master for published topics   
      rospy.get_published_topics()  
      ros_master_available = True
    except socket_error as serr: 
      # logging.warn("[wait_for_roscore] Unexpected error:" + str(serr) + "; traceback: " + str(traceback.format_exc()) ) 
      if serr.errno != errno.ECONNREFUSED:
        pass
        #raise serr  # re-raise error if its not the one we want     
    time.sleep(1.0) # seconds
  return ros_master_available

if __name__ == '__main__':
  print("** BEGIN: wait_for_roscore.py script **")
  ros_master_available = wait_for_roscore()
  print('  ros_master_available: "{0}".'.format(ros_master_available))
  print("--   END: wait_for_roscore.py script --")
