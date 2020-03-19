import sys
from time import time
from time import sleep
import argparse
from argparse import ArgumentParser
import json
import numpy as np
import cv2

import depthai

import consts.resource_paths
from depthai_helpers import utils




########################## ADDED FOR COLOR DETECTION CWM #####################
from __future__ import print_function

#import cv2
#import numpy as np
from difflib import SequenceMatcher
#import sys


def similar(a, b):
	return SequenceMatcher(None, a, b).ratio()

def nothing(*arg):
	pass

def limit(inputVal,limits):
	output=inputVal
	for i,val in enumerate(inputVal,0):
		if val>limits[i][1]:
			val=limits[i][1]
		elif val<limits[i][0]:
			val=limits[i][0]
		output[i]=val

	return output
##############################################################################



def parse_args():
    epilog_text = '''
    Displays video streams captured by DepthAI.

    Example usage:

    # Pass thru pipeline config options

    ## USB3 w/onboard cameras board config:
    python3 test.py -co '{"board_config": {"left_to_right_distance_cm": 7.5}}'

    ## Show the depth stream:
    python3 test.py -co '{"streams": [{"name": "depth_sipp", "max_fps": 12.0}]}'
    '''
    parser = ArgumentParser(epilog=epilog_text,formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("-co", "--config_overwrite", default=None,
                        type=str, required=False,
                        help="JSON-formatted pipeline config object. This will be override defaults used in this script.")
    parser.add_argument("-debug", "--dev_debug", default=None, action='store_true', help="Used by board developers for debugging.")
    options = parser.parse_args()

    return options

args = vars(parse_args())

if args['config_overwrite']:
    args['config_overwrite'] = json.loads(args['config_overwrite'])

print("Using Arguments=",args)


cmd_file = consts.resource_paths.device_cmd_fpath
if args['dev_debug']:
    cmd_file = ''
    print('depthai will not load cmd file into device.')

labels = []
with open(consts.resource_paths.blob_labels_fpath) as fp:
    labels = fp.readlines()
    labels = [i.strip() for i in labels]



print('depthai.__version__ == %s' % depthai.__version__)
print('depthai.__dev_version__ == %s' % depthai.__dev_version__)



if not depthai.init_device(cmd_file):
    print("Error initializing device. Try to reset it.")
    exit(1)


print('Available streams: ' + str(depthai.get_available_steams()))

# Do not modify the default values in the config Dict below directly. Instead, use the `-co` argument when running this script.
config = {
    # Possible streams:
    # ['left', 'right','previewout', 'metaout', 'depth_sipp', 'disparity', 'depth_color_h']
    # If "left" is used, it must be in the first position.
    # To test depth use:
    #'streams': [{'name': 'depth_sipp', "max_fps": 12.0}, {'name': 'previewout', "max_fps": 12.0}, ],
    'streams': [{'name': 'previewout', "max_fps": 3.0}, {'name': 'metaout', "max_fps": 3.0}],
    #'streams': ['metaout', 'previewout'],
    'depth':
    {
        'calibration_file': consts.resource_paths.calib_fpath,
        # 'type': 'median',
        'padding_factor': 0.3
    },
    'ai':
    {
        'blob_file': consts.resource_paths.blob_fpath,
        'blob_file_config': consts.resource_paths.blob_config_fpath,
        'calc_dist_to_bb': True
    },
    'board_config':
    {
        'swap_left_and_right_cameras': True, # True for 1097 (RPi Compute) and 1098OBC (USB w/onboard cameras)
        'left_fov_deg': 69.0, # Same on 1097 and 1098OBC
        #'left_to_right_distance_cm': 9.0, # Distance between stereo cameras
        'left_to_right_distance_cm': 7.5, # Distance between 1098OBC cameras
        'left_to_rgb_distance_cm': 2.0 # Currently unused
    }
}

if args['config_overwrite'] is not None:
    config = utils.merge(args['config_overwrite'],config)
    print("Merged Pipeline config with overwrite",config)

if 'depth_sipp' in config['streams'] and ('depth_color_h' in config['streams'] or 'depth_mm_h' in config['streams']):
    print('ERROR: depth_sipp is mutually exclusive with depth_color_h')
    exit(2)
    # del config["streams"][config['streams'].index('depth_sipp')]

stream_names = [stream if isinstance(stream, str) else stream['name'] for stream in config['streams']]

# create the pipeline, here is the first connection with the device
p = depthai.create_pipeline(config=config)

if p is None:
    print('Pipeline is not created.')
    exit(2)


t_start = time()
frame_count = {}
frame_count_prev = {}
for s in stream_names:
    frame_count[s] = 0
    frame_count_prev[s] = 0

entries_prev = []


################## ADDED FOR COLOR DETECTION CWM ####################
cv2.namedWindow('image')
thrs=50
cv2.createTrackbar('Hue', 'image', 61, 179, nothing)
cv2.createTrackbar('Sat', 'image', 235, 255, nothing)
cv2.createTrackbar('Val', 'image', 255, 255, nothing)
cv2.createTrackbar('filterThresh', 'image', 0, 100, nothing) 

 #sets how much to blur
filt=39
exitNow=0
pause=0
####################################################################



while True:
    # retreive data from the device
    # data is stored in packets, there are nnet (Neural NETwork) packets which have additional functions for NNet result interpretation
    nnet_packets, data_packets = p.get_available_nnet_and_data_packets()

    for i, nnet_packet in enumerate(nnet_packets):
        # the result of the MobileSSD has detection rectangles (here: entries), and we can iterate threw them
        for i, e in enumerate(nnet_packet.entries()):
            # for MobileSSD entries are sorted by confidence
            # {id == -1} or {confidence == 0} is the stopper (special for OpenVINO models and MobileSSD architecture)
            if e[0]['id'] == -1.0 or e[0]['confidence'] == 0.0:
                break

            if i == 0:
                entries_prev.clear()

            # save entry for further usage (as image package may arrive not the same time as nnet package)
            entries_prev.append(e)

    for packet in data_packets:
        if packet.stream_name not in stream_names:
            continue # skip streams that were automatically added
        elif packet.stream_name == 'previewout':
            data = packet.getData()
            # the format of previewout image is CHW (Chanel, Height, Width), but OpenCV needs HWC, so we
            # change shape (3, 300, 300) -> (300, 300, 3)
            data0 = data[0,:,:]
            data1 = data[1,:,:]
            data2 = data[2,:,:]
            frame = cv2.merge([data0, data1, data2])
            
            
            
            
            
            
####################### ADDED FOR COLOR DETECTION CWM #######################
                ##### dump professor's file in here ... ? ######    
            
#            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
#            lower_green = np.array([40, 10, 200])
#            upper_green = np.array([120, 245, 255])
#
#            mask = cv2.inRange(hsv, lower_green, upper_green)
#            result = cv2.bitwise_and(frame, frame, mask=mask)   
#            cv2.imshow('mask', mask)
#            cv2.imshow('result', result)
            

            
    		imgInit = frame
    
    		
    		imgBGR = cv2.resize(imgInit,(300, 300),cv2.INTER_AREA)
    		img=cv2.cvtColor(imgBGR, cv2.COLOR_BGR2HSV) 
    		

			hue = cv2.getTrackbarPos('Hue', 'image')
			sat = cv2.getTrackbarPos('Sat', 'image')
			val = cv2.getTrackbarPos('Val', 'image')
			
			

			lower=[hue,sat,val]
			lower=np.array(lower, dtype="uint8")
			lower2=[[[hue,sat,val]]]
			lower2=np.array(lower2, dtype="uint8")
			chosenColor = cv2.cvtColor(lower2, cv2.COLOR_HSV2BGR)##Tr
		

			upperBound=limit(lower+thrs/2,[[0,179],[0,255],[0,255]])
			lowerBound=limit(lower-thrs/2,[[0,179],[0,255],[0,255]])
			mask=np.uint8(cv2.inRange(img,lowerBound,upperBound))


			vis = np.uint8(img.copy())
			vis[mask==0]=(0,0,0)
			
			
			gray2 = img[:,:,2] #only want black and white image
			gray = vis[:,:,2]

			blurred = cv2.GaussianBlur(gray, (filt, filt), 0)

			thresholdValue = cv2.getTrackbarPos('filterThresh', 'image')
			thresh = cv2.threshold(blurred, thresholdValue, 255, cv2.THRESH_BINARY)[1]
			testArray=[(lower-thrs/2).tolist(),(lower+thrs/2).tolist(),lowerBound.tolist(),upperBound.tolist(),thresholdValue]


			cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[1]
		

			areas=int(len(cnts))
			splotch = np.zeros((1,areas),dtype=np.uint8)
			
			# loop over the contours
			try:	
				for i,c in enumerate(cnts,0):
				
					M = cv2.moments(c)
					splotch[0][i] = int(M["m00"])
				try:
					max1=np.argmax(splotch)
				except:
					max1=-1
				
				original=vis.copy()
				if max1>-1:
					M = cv2.moments(cnts[max1])
					cX = int(M["m10"] / M["m00"])
					cY = int(M["m01"] / M["m00"])


					
					cv2.drawContours(vis, [cnts[max1]], -1, (0, 255, 0), 2)
					cv2.circle(vis, (cX, cY), 7, (255, 255, 255), -1)
					cv2.putText(vis, "Green Light", (cX - 20, cY - 20),
						cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
			except:
				pass
			
			cc=(int(chosenColor[0][0][0]),int(chosenColor[0][0][1]),int(chosenColor[0][0][2]))
			cv2.circle(imgBGR, (50, 50), 50, cc, -1)

			visBGR=cv2.cvtColor(vis, cv2.COLOR_HSV2BGR) 
			thresh = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)
			
			cv2.imshow('image',np.hstack([imgBGR,thresh, visBGR])) #np.hstack([original, vis]))#np.hstack([thresh, gray2]))
    			
###########################################################################
            
            
            

            
            
            

            img_h = frame.shape[0]
            img_w = frame.shape[1]
            
            
            confidences = []
            #cwm: Takes highest confidence value and selects only that object
            for e in entries_prev:
                confidences.append(e[0]['confidence'])
                
            print(confidences)
            
            new_entries_prev = entries_prev
            
            if confidences:
                index_of_max_confidence = np.argmax(confidences)
            
                new_entries_prev = []
                new_entries_prev.append(entries_prev[index_of_max_confidence])
            

            # iterate threw pre-saved entries & draw rectangle & text on image:
            for e in new_entries_prev:
                # the lower confidence threshold - the more we get false positives
                if e[0]['confidence'] > 0.5:
                    x1 = int(e[0]['left'] * img_w)
                    y1 = int(e[0]['top'] * img_h)

                    pt1 = x1, y1
                    pt2 = int(e[0]['right'] * img_w), int(e[0]['bottom'] * img_h)

                    cv2.rectangle(frame, pt1, pt2, (0, 0, 255))

                    # Handles case where TensorEntry object label = 7552.
                    if e[0]['label'] > len(labels):
                        print("Label index=",e[0]['label'], "is out of range. Not applying text to rectangle.")
                    else:
                        pt_t1 = x1, y1 + 20
                        cv2.putText(frame, labels[int(e[0]['label'])], pt_t1, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                        print('\n')
                        print(labels[int(e[0]['label'])])

                        pt_t2 = x1, y1 + 40
                        cv2.putText(frame, '{:.2f}'.format(100*e[0]['confidence']) + ' %', pt_t2, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255))
                        print('{:.2f}'.format(100*e[0]['confidence']) + ' %')
                        if config['ai']['calc_dist_to_bb']:
                            pt_t3 = x1, y1 + 60
                            cv2.putText(frame, 'x:' '{:7.3f}'.format(e[0]['distance_x']) + ' m', pt_t3, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255))
                            print('x:' '{:7.3f}'.format(e[0]['distance_x']) + ' m')

                            pt_t4 = x1, y1 + 80
                            cv2.putText(frame, 'y:' '{:7.3f}'.format(e[0]['distance_y']) + ' m', pt_t4, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255))
                            print('y:' '{:7.3f}'.format(e[0]['distance_y']))

                            pt_t5 = x1, y1 + 100
                            cv2.putText(frame, 'z:' '{:7.3f}'.format(e[0]['distance_z']) + ' m', pt_t5, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255))
                            print('z:' '{:7.3f}'.format(e[0]['distance_z']))
            print('----------frame over ----------')





            cv2.putText(frame, "fps: " + str(frame_count_prev[packet.stream_name]), (25, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0))
            cv2.imshow('previewout', frame)
        elif packet.stream_name == 'left' or packet.stream_name == 'right' or packet.stream_name == 'disparity':
            frame_bgr = packet.getData()
            cv2.putText(frame_bgr, packet.stream_name, (25, 25), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0))
            cv2.putText(frame_bgr, "fps: " + str(frame_count_prev[packet.stream_name]), (25, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0))
            cv2.imshow(packet.stream_name, frame_bgr)
        elif packet.stream_name.startswith('depth'):
            frame = packet.getData()

            if len(frame.shape) == 2:
                if frame.dtype == np.uint8: # grayscale
                    cv2.putText(frame, packet.stream_name, (25, 25), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255))
                    cv2.putText(frame, "fps: " + str(frame_count_prev[packet.stream_name]), (25, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255))
                    cv2.imshow(packet.stream_name, frame)
                else: # uint16
                    frame = (65535 // frame).astype(np.uint8)
                    #colorize depth map, comment out code below to obtain grayscale
                    frame = cv2.applyColorMap(frame, cv2.COLORMAP_HOT)
                    # frame = cv2.applyColorMap(frame, cv2.COLORMAP_JET)
                    cv2.putText(frame, packet.stream_name, (25, 25), cv2.FONT_HERSHEY_SIMPLEX, 1.0, 255)
                    cv2.putText(frame, "fps: " + str(frame_count_prev[packet.stream_name]), (25, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, 255)
                    cv2.imshow(packet.stream_name, frame)
            else: # bgr
                cv2.putText(frame, packet.stream_name, (25, 25), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255))
                cv2.putText(frame, "fps: " + str(frame_count_prev[packet.stream_name]), (25, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, 255)
                cv2.imshow(packet.stream_name, frame)
        
        frame_count[packet.stream_name] += 1

    t_curr = time()
    if t_start + 1.0 < t_curr:
        t_start = t_curr

        for s in stream_names:
            frame_count_prev[s] = frame_count[s]
            frame_count[s] = 0

    if cv2.waitKey(1) == ord('q'):
        break

del p  # in order to stop the pipeline object should be deleted, otherwise device will continue working. This is required if you are going to add code after the main loop, otherwise you can ommit it.

cv2.destroyAllWindows()

print('py: DONE.')
                                                                                      
