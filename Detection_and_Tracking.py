import pyrealsense2 as rs
import numpy as np
import cv2
from cv2 import aruco
from statistics import mean
import dagger
import math
import memcache   ##for common sharing memory


##initialising memcache client
memc=memcache.Client(['127.0.0.1:11211'],debug=1)

############# csrt and depth 
# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))
bad = False


# Setting up marker size
MARKER_SIZE = 5.00  #cm

# Setting up marker dictionary with aruco marker of ID=0 
marker_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
param_markers = aruco.DetectorParameters_create()


found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

if device_product_line == 'L500':
    config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
else:
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)
(major_ver, minor_ver, subminor_ver) = (cv2.__version__).split('.')

def com_region(depth_frame,left_corner_pt,bbox_dimension):
    # This will calculate the depth of the center of mass of the region where aruco is detected. i.e it will
    # calculate the distance of aruco marker from camera. 
    
    distance_list=[]
    if bbox_dimension[0]>=7 and bbox_dimension[1]>=7:
        for i in range(int(3*bbox_dimension[0]/7), int(4*bbox_dimension[0]/7)):         ##traversing through height
            for j in range(int(3*bbox_dimension[1]/7), int(4*bbox_dimension[1]/7)):
                distance=depth_frame[int(left_corner_pt[1]+j),int(left_corner_pt[0]+i)]
                distance_list.append(distance)
    else:
        for i in range(int(bbox_dimension[0])):         ##traversing through width
            for j in range(int(bbox_dimension[1])):
                distance=depth_frame[int(left_corner_pt[1]+j),int(left_corner_pt[0]+i)]
                distance_list.append(distance)
                
    # taking mean of all depth values calculated in different pixels

    pixel = mean(distance_list)
    
    return pixel*scale



previous_distance_z=0

def arucodetect(color_frame):
    
    # This will detect the aruco marker fixed on the drone, and compute the bounding box dimensions that will be used for CSRT tracking.

        # Getting the color frame as input
        frame = np.array(color_frame.get_data())
    
        kernel =np.array([[-1,-1,-1],[-1,9,-1],[-1,-1,-1]])
        
        # sharpening the image
        image_sharp = cv2.filter2D(src=frame, ddepth=-1, kernel=kernel)

        # aruco detection        
        gray_frame = cv2.cvtColor(image_sharp, cv2.COLOR_BGR2GRAY)
        marker_corners, marker_IDs, reject = aruco.detectMarkers(
            gray_frame, marker_dict, parameters=param_markers)
    
        if marker_corners:
            
            total_markers = range(0, marker_IDs.size)
            for ids, corners, i in zip(marker_IDs, marker_corners, total_markers):
                cv2.polylines(frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv2.LINE_AA)
                corners = corners.reshape(4, 2)
                corners = corners.astype(int)
                top_right = corners[0].ravel()
                top_left = corners[1].ravel()
                bottom_right = corners[3].ravel()
                bottom_left = corners[2].ravel()
                print(top_right, top_left, bottom_right, bottom_left)
                bbox = (bottom_right[0], top_left[1], abs(int(bottom_right[0]-bottom_left[0])), abs(int(bottom_right[1]-top_right[1])))
                
                #returns the bounding box
                return bbox
            

if __name__ == '__main__' :

    ## Initializing the tracker type as CSRT
    tracker = cv2.TrackerCSRT_create()

    # Initiating the image pipeline
    frames = pipeline.wait_for_frames()

    # Getting depth image feed from the pipeline
    depth_frame = frames.get_depth_frame()

    # Getting color image feed from the pipeline
    color_frame = frames.get_color_frame()
    depth_image = np.array(depth_frame.get_data())
    frame = np.array(color_frame.get_data())
   
    # condition to continue aruco detection until bounding box is found
    while True:
        bbox= arucodetect(color_frame)
        if len(bbox)!=0:
            break
        
    # Initializing CSRT tracker for the first frame
    ok = tracker.init(frame, bbox)
    a = frame.shape[1]
    b = frame.shape[0]
    x=[]
    y=[]
    j=0

    # Start camera loop
    while True:
            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            
            # Convert images to numpy arrays
            depth_sensor = device.first_depth_sensor()
            scale = depth_sensor.get_depth_scale()

            depth_image = np.array(depth_frame.get_data(),dtype="uint16")
            pixel = depth_image[0]
            
            frame = np.array(color_frame.get_data())

            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
            depth_colormap_dim = depth_colormap.shape
            color_colormap_dim = frame.shape

            # If depth and color resolutions are different, resize color image to match depth image for display
            if depth_colormap_dim != color_colormap_dim:
                resized_color_image = cv2.resize(frame, dsize=(1*depth_colormap_dim[1]/2, 1*depth_colormap_dim[0]/2), interpolation=cv2.INTER_AREA)
                images = np.hstack((resized_color_image, depth_colormap))
            else:
                images = np.hstack((frame, depth_colormap))

            # Start timer
            timer = cv2.getTickCount()

            # Update tracker
            ok, bbox = tracker.update(frame)

            # Calculate Frames per second (FPS)
            fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer)

            # Draw bounding box
            if ok:
                # Tracking success
                p1 = (int(bbox[0]), int(bbox[1]))
                p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
                cv2.rectangle(frame, p1, p2, (255,0,0), 2, 1)
                xcm_bbox=int((2*bbox[0]+bbox[2])/2)
                ycm_bbox=int((2*bbox[1]+bbox[3])/2)
                xcm=xcm_bbox-(a/2)
                ycm=-ycm_bbox+(b/2)
                x.append(int(xcm_bbox))
                y.append(int(ycm_bbox))
            else :
                # Tracking failure
                cv2.putText(frame, "Tracking failure detected", (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)
                print("Tracking Failure")
                t = dagger.PlutoConnection()
                t.connect(("192.168.4.1",23))

                # setting up Raw RC from dagger (python wrapper)
                cmd = dagger.SetRawRC(t)
                tk = dagger.SetCommand(t)
                tk.command(dagger.CmdType.LAND)

            ##determining depth distance of com
            left_corner=[bbox[0],bbox[1]]
            bbox_dimension=[bbox[2],bbox[3]]
            distance_com=com_region(depth_image,left_corner,bbox_dimension)
            zcm = distance_com  # z distance is in meter
            if(zcm==0.0):
                zcm=previous_distance_z
            previous_distance_z=zcm

            #### transformation of coordinates
            xcm = xcm*zcm/2.510
            xcm=(66/102)*xcm   ## conversion in cm
            ycm = ycm*zcm/2.510
            ycm = (77/125)*ycm  ##conversion in cm

            if(j==0):
                ##updating in shared memory
                initial_coordinates={'x_initial':xcm,'y_initial':ycm,'z_initial':2.0}
                memc.set_multi(initial_coordinates)   ##updating initial coordinates in cache memory
                j=1   ##only getting initial values

            ##updating current coordinates in cache memory
            current_coordinates={'x':xcm,'y':ycm,'z':zcm}
            memc.set_multi(current_coordinates)    
                        
            # Display tracker type on frame
            cv2.putText(frame, "CSRT Tracker", (100,20), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50),2)    

            # Display FPS on frame
            cv2.putText(frame, "FPS : " + str(int(fps)), (100,50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50), 2)
            
            #showing real time x-y coordinates
            cv2.putText(frame,"x,y:" + str(round(xcm,0)) + ":" + str(round(ycm,0)),(100,70),cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255,0,0), 2)

            #drawing the bounding box
            cv2.line(frame,(0,int(b/2)) ,(int(a),int(b/2)), (50,170,50), 1)
            cv2.line(frame,(int(a/2),0) ,(int(a/2),int(b)), (50,170,50), 1)
                        
            # Display result
            cv2.imshow("Tracking", frame)
            cv2.waitKey(1)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        


