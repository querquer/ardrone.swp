#!/usr/bin/python
import roslib
roslib.load_manifest('follow_line')
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import pdb

import cv
import math
import time
import sys
import numpy
import time
from follow_line.msg import LinePos

def get_canny_init():
  return canny_init

def get_canny_link():
  return canny_link

def set_canny_init(x):
  global canny_init
  canny_init = x

def set_canny_link(x):
  global canny_link
  canny_link = x

def on_init_change(x):
	set_canny_init(x)

def on_link_change(x):
	set_canny_link(x)


def los():
    global frame
    global frame_size
    global gray_frame
    global canny_result
    global forrun
    global old_target
    global canny_init
    global canny_link
    global frame_on
    global canny_on
    global init_on
    global border_on
    global points_on
    global color_on
    global corridor
    global epsilon
    global angle
    global font
    global font_dist
    global subject
    global pub
    global mmat
    global c1
    global c2
    color_dst = cv.CreateImage( frame_size, cv.IPL_DEPTH_8U, 3 )

    ## 1. capture the current image

    #frame2 = cv.QueryFrame(capture)
    #if frame is None:
      #break
      #continue

    #for i in range(1,5):
    # um die doffe Zeitverzoegerung des Kamerabildes zu minimieren, schmeissen
    # wir erstmal 4 Bilder aus dem Framebuffer weg...
    #cv.GrabFrame(capture)
    #cv.GrabFrame(capture)
    #cv.GrabFrame(capture)
    #cv.GrabFrame(capture)
    #frame2 = cv.QueryFrame(capture)
    if frame is None:
      #break
      #continue
      return
    #frame = cv.QueryFrame(capture)
    #frame = cv.CloneImage(frame2)
    #cv.WarpPerspective(frame2, frame, mmat)

    #frame = cv.CloneImage(frame2)

    ## 2. copy current frame to a gray scale frame -> dst
    cv.CvtColor( frame, gray_frame, cv.CV_BGR2GRAY );

    ## 3. line detection in dst
    ##        canny_init - initial edge detction
    ##        canny_link - responsible for edge links
    cv.Canny(gray_frame, canny_result, get_canny_init(), get_canny_link(), 3);
    ## Look for all points detected
    # die linke und rechte Kante, die durch die Perspektivenkorrektur entstehen
    # interessieren uns nicht, deshalb links und rechts Pixel weglassen
    left_border = 20
    right_border = 620

    if init_on==True:
      points = numpy.nonzero(canny_result[(forrun-corridor/2):(forrun+corridor/2),left_border:right_border])
      if points:
	    for i in range(0,len(points[1])):
	      points[1][i] = points[1][i] + left_border
    else:
      left = old_target[0]-epsilon
      if left<0:
	left=0
      right = old_target[0]+epsilon
      if right>=right_border:
	  #if right>=frame_size[0]:
	#right=frame_size[0]-1
	right=right_border


      points = numpy.nonzero(canny_result[(forrun-corridor/2):(forrun+corridor/2),left:right])
      if points:
	for i in range(0,len(points[1])):
	  points[1][i] = points[1][i] + left

    cv.CvtColor(canny_result, color_dst, cv.CV_GRAY2BGR );

    if frame_on==True:
      frame = color_dst
    else:
      #cv.CvtColor( gray_frame, frame, cv.CV_GRAY2BGR );
      if color_on==False:
	cv.CvtColor( gray_frame, frame, cv.CV_GRAY2BGR );

    if border_on==True:
      start=(0,frame_size[1]/2)
      end=(frame_size[0],frame_size[1]/2)
      cv.Line( frame, start, end, cv.CV_RGB(0,255,0), 3, 8 )
      start=(0,frame_size[1]/2+corridor/2)
      end=(frame_size[0],frame_size[1]/2+corridor/2)
      cv.Line( frame, start, end, cv.CV_RGB(0,255,0), 1, 8 )
      start=(0,frame_size[1]/2-corridor/2)
      end=(frame_size[0],frame_size[1]/2-corridor/2)
      cv.Line( frame, start, end, cv.CV_RGB(0,255,0), 1, 8 )

      start=(old_target[0]-epsilon,frame_size[1]/2-corridor/2)
      end=(old_target[0]-epsilon,frame_size[1]/2+corridor/2)
      cv.Line( frame, start, end, cv.CV_RGB(0,255,0), 1, 8 )

      start=(old_target[0]+epsilon,frame_size[1]/2-corridor/2)
      end=(old_target[0]+epsilon,frame_size[1]/2+corridor/2)
      cv.Line( frame, start, end, cv.CV_RGB(0,255,0), 1, 8 )

    # "Fadenkreuz" zum Ausrichten der Kamera
    start=(frame_size[0]/2,(frame_size[1]/2)+30)
    end=(frame_size[0]/2,(frame_size[1]/2)-30)
    cv.Line( frame, start, end, cv.CV_RGB(0,255,0), 1, 8 )
    start=(frame_size[0]/2+30,(frame_size[1]/2))
    end=(frame_size[0]/2-30,(frame_size[1]/2))
    cv.Line( frame, start, end, cv.CV_RGB(0,255,0), 1, 8 )
    if border_on==False:
      cv.Circle(frame, (frame_size[0]/2, frame_size[1]/2), 10, cv.CV_RGB(0,255,0), 1 )

    if  len(points[1])>0:
      if points_on==True:
      	for i in range(0,len(points[1])):
	   cv.Circle( frame, (points[1][i], points[0][i]+(forrun-corridor/2)), 5, cv.CV_RGB(255,255,0), 2 );
      left=numpy.max(points[1])
      right=numpy.min(points[1])
      center=int((left+right)/2)
      old_target[0]=center

      if border_on==True:
	start=(center,(frame_size[1]/2)+30)
	end=(center,(frame_size[1]/2)-30)
	cv.Line( frame, start, end, cv.CV_RGB(0,255,0), 1, 8 )

      left_points=[]
      right_points=[]
      left_points2=[]
      right_x_upper=-1
      right_x_lower=-1
      left_x_upper=-1
      left_x_lower=-1

      for i in range(0,len(points[1])):
	if points[1][i]<center:
	  left_points.append(points[1][i])
	  if left_x_upper == -1:
	    left_x_upper = points[1][i]
	  left_x_lower = points[1][i]
	else:
	  right_points.append(points[1][i])
	  if right_x_upper == -1:
	    right_x_upper = points[1][i]
	  right_x_lower = points[1][i]
      if len(left_points)>0:
	diff_left=numpy.max(left_points)-numpy.min(left_points)
      if len(right_points)>0:
	diff_right=numpy.max(right_points)-numpy.min(right_points)
      if len(left_points)> 0 and len(right_points)>0:
	diff_mean=(diff_left+diff_right)/2
      if len(left_points)> 0 and len(right_points)==0:
	diff_mean=diff_left
      if len(left_points)== 0 and len(right_points)>0:
	diff_mean=diff_right
      if diff_mean<>0:
	angle=math.atan2(corridor, diff_mean)
	if left_x_upper!=-1 & right_x_upper!=-1:
	  if (left_x_upper < left_x_lower) & (right_x_upper < right_x_lower):
	    angle=math.pi-angle
      else:
	angle=math.pi/2
    cv.Circle( frame, (old_target[0], old_target[1]), 5,  cv.CV_RGB(255,0,0), 2 );

    if len(points[1])>0:
      start=(old_target[0],old_target[1])
      end=(int(old_target[0]+math.cos(angle)*30),int(old_target[1]-math.sin(angle)*30))
      cv.Line( frame, start, end, cv.CV_RGB(255,0,0), 3, 8 )

    output="Next positon x=%3.1f y=%3.1f angle=%3.1f"%(old_target[0],old_target[1],int(((angle*180/math.pi))))
    cv.PutText( frame,output,(20, 20), font, cv.CV_RGB(0,255,0));
    lp = LinePos()
    lp.x = old_target[0]
    lp.y = old_target[1]
    lp.angle = int(((angle*18000/math.pi)-9000)*(-1))
    pub.publish(lp)
    if frame_on==False:
      output="Video (no contour)  visible   (f)"
    else:
      output="(no video) Contoure visible    (f)"
    cv.PutText(frame , output, (350,300), font, cv.CV_RGB(0,255,0));
    if points_on==False:
      output="Points              visible   (p)"
    else:
      output="Points              invisible (p)"
    cv.PutText(frame , output, (350,300+font_dist), font, cv.CV_RGB(0,255,0));
    if init_on==True:
      output="Initiation phase              (i)"
    else:
      output="Running mode                  (i)"
    cv.PutText(frame , output, (350,300+2*font_dist), font, cv.CV_RGB(0,255,0));
    if color_on==True:
      output="show gray                     (c)"
    else:
      output="show color                    (c)"
#    output="switch between gray and color (c)"
    cv.PutText(frame , output, (350,300+3*font_dist), font, cv.CV_RGB(0,255,0));

    cv.ShowImage("camera", frame)

    #lp = LinePos()
    """if (init_on==False) & (len(points[1])>0):
      if angle==0:
	message=struct.pack('>hhh',old_target[0], old_target[1], 0)
        lp.x = old_target[0]
        lp.y = old_target[1]
        lp.angle = 0
      else:
	message=struct.pack('>hhh',old_target[0], old_target[1], int(((angle*18000/math.pi)-9000)*(-1)))
        lp.x = old_target[0]
        lp.y = old_target[1]
        lp.angle = int(((angle*18000/math.pi)-9000)*(-1))"""
      #e=event.Event(subject,message)
    #lp.x = old_target[0]
    #lp.y = old_target[1]
    #lp.angle = int(((angle*18000/math.pi)-9000)*(-1))
    #pub.publish(lp)
      #pub.publish(e)
    #time.sleep(0.25)
    """k = cv.WaitKey (250)
    if k>0:
      k = chr(k%256)
      if k == 'q':
	#break
        return
      if k == 'E':
	canny_init=canny_init+5
	print "canny_init = %d"%canny_init
	cv.SetTrackbarPos('canny_init', 'camera', get_canny_init())
      if k == 'e':
	canny_init=canny_init-5
	cv.SetTrackbarPos('canny_init', 'camera', get_canny_init())
	print "canny_init = %d"%canny_init
      if k == 'L':
	canny_link=canny_link+5
	cv.SetTrackbarPos('canny_link', 'camera', get_canny_link())
	print "canny_link = %d"%canny_link
      if k == 'l':
	canny_link=canny_link-5
	cv.SetTrackbarPos('canny_link', 'camera', get_canny_link())
	print "canny_link = %d"%canny_link
      if k == 'f':
	if frame_on == False:
	  frame_on = True
	else:
	  frame_on = False
      if k == 'b':
	if border_on == False:
	  border_on = True
	else:
	  border_on = False
      if k == 'p':
	if points_on == False:
	  points_on = True
	else:
	  points_on = False
      if k == 'i':
	if init_on==False:
	  init_on = True
	else:
	  init_on = False
      if k == 'c':
	if color_on==False:
	  color_on=True
	else:
	  color_on=False"""

def callback(data):
    global frame
    global frame_size
    global gray_frame
    global canny_result
    global forrun
    global old_target
    global count
    bridge = CvBridge()
    try:
      cv_image = bridge.imgmsg_to_cv(data, "bgr8")
    except CvBridgeError, e:
      print e
    frame = cv_image
    frame_size=cv.GetSize (frame)
    #frame_size[0] = 320
    #frame_size[1] = 240
    #frame_size = [320, 240]
    gray_frame = cv.CreateImage( frame_size, cv.IPL_DEPTH_8U, 1 )
    canny_result = cv.CreateImage( frame_size, cv.IPL_DEPTH_8U, 1 );
    cv.CreateTrackbar("canny_init", 'camera', get_canny_init(), 1000, on_init_change)
    cv.CreateTrackbar("canny_link", 'camera', get_canny_link(), 1000, on_link_change)
    forrun=frame_size[1]/2
    old_target=[frame_size[0]/2,forrun]
    los()
    #rospy.loginfo("count: %s", count)
    """if count != 0:
       count += 1
       if count == 2:
          count = 0
       #return
    else:
	    bridge = CvBridge()
	    try:
	      cv_image = bridge.imgmsg_to_cv(data, "bgr8")
	    except CvBridgeError, e:
	      print e
	    frame = cv_image
	    frame_size=cv.GetSize (frame)
            #frame_size[0] = 320
            #frame_size[1] = 240
            #frame_size = [320, 240]
	    gray_frame = cv.CreateImage( frame_size, cv.IPL_DEPTH_8U, 1 )
	    canny_result = cv.CreateImage( frame_size, cv.IPL_DEPTH_8U, 1 );
	    cv.CreateTrackbar("canny_init", 'camera', get_canny_init(), 1000, on_init_change)
	    cv.CreateTrackbar("canny_link", 'camera', get_canny_link(), 1000, on_link_change)
	    forrun=frame_size[1]/2
	    old_target=[frame_size[0]/2,forrun]
	    los()
            count += 1"""

def main(args):
	global canny_init
	global canny_link
	global frame_on
	global canny_on
	global init_on
	global border_on
	global points_on
	global color_on
	global corridor
	global epsilon
	global angle
	global font
	global font_dist
        global subject
        global pub
        global mmat
        global c1
        global c2
        global count

	cv.NamedWindow("camera", 1)
	#cv.NamedWindow("camera", cv.CV_WINDOW_NORMAL)
	#cv.ResizeWindow('Camera', 920, 840)
	cv.MoveWindow ('Camera', 10, 10)

	#cv.NamedWindow("camera2", 1)

	# load the Famouso Python Bindings
	sys.path.append('/home/ulrich/famouso/Bindings/Python')
	from pyFAMOUSO.famouso import *


	# try to get the device number from the command line
	#if len(sys.argv) < 2:
	#  device= 0
	#else:
	#  device = int (sys.argv [1])



	#capture = cv.CaptureFromCAM(device)
	#cv.SetCaptureProperty(capture, cv.CV_CAP_PROP_FRAME_HEIGHT, 480)
	#cv.SetCaptureProperty(capture, cv.CV_CAP_PROP_FRAME_WIDTH, 640)
	#frame = cv.QueryFrame(capture)

	#print device

        count = 0

	subject = "WayPoint"
	#pub = publisher.PublisherEventChannel(subject)
        pub = rospy.Publisher('LinePos', LinePos)
	# pub.announce(subject)

	#canny_init
	canny_init = 300
	canny_link=100
	frame_on=False
	canny_on=False
	init_on=True
	border_on=True
	points_on=False
	color_on=True
	corridor=20
	epsilon=50
	angle=0
	font=cv.InitFont(cv.CV_FONT_VECTOR0,0.5,0.5,0,1);
	font_dist=20

	# Perspektive korrigieren
	mmat = cv.CreateMat(3,3,cv.CV_32FC1)
	c1 = ((173,240), (467,240), (126,330), (514,330))
	c2 = ((173,240), (467,240), (173,330), (467,330));
	#cv.GetPerspectiveTransform(c1, c2, mmat);
	#Punkte: oben rechts: 467, 240 -> bleibt
	# oben links: 173, 240 -> bleibt
	#unten rechts: 514, 330 -> 467, 330
	#unten links: 126, 330 > 173, 330
	rospy.init_node('TrackLine', anonymous=True)
	image_sub = rospy.Subscriber("ardrone/image_raw",Image,callback)
	try:
	  rospy.spin()
	except KeyboardInterrupt:
	  print "Shutting down"
	cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
