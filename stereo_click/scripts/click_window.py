#!/usr/bin/env python

##	@package click_window
#	This module uses OpenCV's HighGUI platform to import a camera stream and allow a
#	person to click on an arbitrary pixel at arbitrary levels of zoom. It outputs
#	a message containing the pixel value (at zoom 100%) and camera_info, to the
#	topic specified by "outputName"

import roslib
import sys
roslib.load_manifest("stereo_click")
import rospy
import math
import tf
from tf.msg import tfMessage
import cv
from std_msgs.msg import String
from std_msgs.msg import Empty as EmptyMsg
from std_srvs.srv import Empty as EmptySrv
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import thread
from stereo_click.msg import *

##	ClickWindow documentation
#
#	A class which, when instantiated, creates a clickable window from a camera stream
#	Note that this class does not automatically persist: it must have its .listen() function
#	called periodically. This may change in future releases.
class ClickWindow:

	##	The constructor
	#	@param self The object pointer
	#	@param cameraName The name of the camera in the stereo pair. Ex: /wide_stereo_left
	#	@param outputName The name of the output node
	def __init__(self,cameraName,outputName):
		self.name = "%s Viewer"%cameraName
		self.cp = False
		self.ch_x = 0
		self.ch_y = 0
		self.zoom = 1
		self.offset = (0.0,0.0)
		self.outputName = outputName
		self.bridge = CvBridge()
		self.create_window()
		self.cameraTopic = "%s/image_rect_color"%cameraName
		self.cameraInfoTopic = "%s/camera_info"%cameraName
		self.camera_sub = rospy.Subscriber(self.cameraTopic,Image,self.update_background)
		self.camera_info_sub = rospy.Subscriber(self.cameraInfoTopic,CameraInfo,self.update_camera_info)
		
		self.clear_serv = rospy.Service("%s/received"%self.outputName,EmptySrv,self.clear_request)
		self.point_pub = rospy.Publisher(self.outputName,ClickPoint)
		self.set_background(cv.CreateImage((500,500),8,3))
		self.set_listeners()
	
	##	Creates a window and updates it for the first time
	def create_window(self):
		cv.NamedWindow(self.name)
		cv.WaitKey(25)
		print "Window created"	
		
	##	Sets the background (used for updating the camera stream)
	#	@param background A pointer to the cvImage which will be the background of the window
	def set_background(self,background):
		self.background = background
	
	##	Updates the background, given a new packet of camera data
	#	@param data The camera data (in Image format)
	def update_background(self,data):
		try:
			cv_image = self.bridge.imgmsg_to_cv(data, "bgr8")
		except CvBridgeError, e:
			print e
		self.set_background(cv_image)
	
	def update_camera_info(self,data):
		self.set_camera_info(data)
		
	def set_camera_info(self,info):
		self.camera_info = info
		
	def set_listeners(self):
		cv.SetMouseCallback(self.name,self.onMouse,0)
		print "Set Listeners"
	
	##	Called every time a mouse event is registered. Used to draw crosshairs, zoom, and register clicks.
	#	@param event The cv mouse event
	#	@param (zoom_x,zoom_y) The location, in pixels, of the click on the window. Not necessarily the same as on the camera.
	#	@param flags CV flags
	#	@param param Unused argument, required by OpenCV
	def onMouse(self,event,zoom_x,zoom_y,flags,param):
		self.setCrosshairs(zoom_x,zoom_y)
		(x,y) = self.unZoomPt(zoom_x,zoom_y)
		
		if event == cv.CV_EVENT_LBUTTONUP:
			print "Clicked on point (%d,%d)"%(x,y)
			self.output_point(x,y)
		if event == cv.CV_EVENT_MBUTTONUP:
			print "Zooming to point (%d,%d)"%(x,y)
			self.zoom *= 2
			self.offset = (x - self.background.width/(2*self.zoom),y - self.background.height/(2*self.zoom))
		if event == cv.CV_EVENT_RBUTTONUP:
			print "Unzooming"
			self.zoom = 1
			self.offset = (0,0)
		
	def setCrosshairs(self,x,y):
		self.ch_x = x
		self.ch_y = y
	
	##	Given a pixel on a possibly zoomed window, outputs the proper camera pixel value
	#	@param (zoom_x,zoom_y) The (x,y) coordinates of the click
	def unZoomPt(self,zoom_x,zoom_y):
		scaled_x = zoom_x / float(self.zoom)
		scaled_y = zoom_y / float(self.zoom)
		centered_x = scaled_x + self.offset[0]
		centered_y = scaled_y + self.offset[1]
		return (centered_x,centered_y)
	
	##	Given a pixel on the camera, outputs the location of that point in terms of the current, possibly zoomed window.
	#	@param (x,y) The (x,y) coordinates of the pixel, in the camera's view 
	def zoomPt(self,x,y):
		uncentered_x = x - self.offset[0]
		uncentered_y = y - self.offset[1]
		x = uncentered_x * self.zoom
		y = uncentered_y * self.zoom
		return (x,y)
			
	## Publishes the proper point and camera information to the given topic
	#	@param (x,y) The (x,y) coordinates of the pixel, in the camera's view
	def output_point(self,x,y):
		cp = ClickPoint()
		cp.x = x
		cp.y = y
		cp.camera_info = self.camera_info
		self.cp = cp
		self.point_pub.publish(cp)
	
	##	The listener, which updates the camera feed and registers onMouse events	
	def listen(self):
		bgimg = cv.CreateImage((self.background.width,self.background.height),8,3)
		img = cv.CreateImage((self.background.width,self.background.height),8,3)
		cv.Copy(self.background,bgimg)
		smallimg = cv.CreateImage((self.background.width/self.zoom,self.background.height/self.zoom),8,3)
		cv.GetRectSubPix(bgimg,smallimg,(self.background.width/(2*self.zoom)+self.offset[0],self.background.height/(2*self.zoom)+self.offset[1]))
		cv.Resize(smallimg,img)
		if(self.cp != False):
			cv.Circle(img,self.zoomPt(self.cp.x,self.cp.y),3,cv.RGB(0,255,0),-1)

		cv.Line(img,(self.ch_x-25,self.ch_y),(self.ch_x+25,self.ch_y),cv.RGB(255,255,0))
		cv.Line(img,(self.ch_x,self.ch_y-25),(self.ch_x,self.ch_y+25),cv.RGB(255,255,0))
		cv.ShowImage(self.name,img)
		cv.WaitKey(25)
		
	## Clears the current click point
	def clear_request(self,args):
		self.cp = False
		return []
		
def usage():
	print "clickwindow.py [name] [cameraName] [outputName]"

## Instantiate a new click_window node
def main(args):
#	if len(args) != 3:
#		return usage()
#	[name, cameraName, outputName] = args
	name = "ClickWindowName"
	rospy.init_node(name)
	cameraName = rospy.get_param("~cam","defaultClickPointCamera")
	outputName = rospy.get_param("~output","defaultClickPointOutput")
	gui = ClickWindow(cameraName=cameraName,outputName=outputName)
	while not rospy.is_shutdown():
		gui.listen()
	cv.DestroyAllWindows()

if __name__ == '__main__':
	args = sys.argv[1:]
	try:
		main(args)
	except rospy.ROSInterruptException: pass
