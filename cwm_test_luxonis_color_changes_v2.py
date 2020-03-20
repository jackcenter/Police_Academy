import sys
from time import time
from time import sleep
import argparse
from argparse import ArgumentParser
import json
import numpy as np
import cv2
# from __future__ import print_function
from difflib import SequenceMatcher
import depthai

import consts.resource_paths
from depthai_helpers import utils



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
    ###############################################################################



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
    #'streams': [{'name': 'previewout', "max_fps": 3.0}, {'name': 'depth_mm_h', "max_fps": 3.0}],
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
cv2.namedWindow('g_image')
cv2.namedWindow('r_image')
cv2.namedWindow('r1_sliders')
cv2.namedWindow('r2_sliders')
cv2.namedWindow('g_sliders')
# white blank image
blank_image = 255 * np.ones(shape=[10, 256, 3], dtype=np.uint8)
thrs=50

# set default slider values
thresholdValue = 15
r1LowHue = 0
r1LowSat = 160
r1LowVal = 120
r1UpHue = 10
r1UpSat = 255
r1UpVal = 255
r2LowHue = 170
r2LowSat = 160
r2LowVal = 135
r2UpHue = 179
r2UpSat = 255
r2UpVal = 255
gLowHue = 30
gLowSat = 50
gLowVal = 60
# gLowSat = 120
# gLowVal = 70
gUpHue = 70
gUpSat = 245
gUpVal = 255


#cv2.createTrackbar('Hue', 'image', 80, 179, nothing)
#cv2.createTrackbar('Sat', 'image', 127, 255, nothing)
#cv2.createTrackbar('Val', 'image', 222, 255, nothing)

'''
cv2.createTrackbar('filterThresh', 'r1_sliders', thresholdValue, 100, nothing)
cv2.createTrackbar('r1LowHue', 'r1_sliders', r1LowHue, 179, nothing)
cv2.createTrackbar('r1LowSat', 'r1_sliders', r1LowSat, 255, nothing)
cv2.createTrackbar('r1LowVal', 'r1_sliders', r1LowVal, 255, nothing)
cv2.createTrackbar('r1UpHue', 'r1_sliders', r1UpHue, 179, nothing)
cv2.createTrackbar('r1UpSat', 'r1_sliders', r1UpSat, 255, nothing)
cv2.createTrackbar('r1UpVal', 'r1_sliders', r1UpVal, 255, nothing)
cv2.createTrackbar('r2LowHue', 'r2_sliders', r2LowHue, 179, nothing)
cv2.createTrackbar('r2LowSat', 'r2_sliders', r2LowSat, 255, nothing)
cv2.createTrackbar('r2LowVal', 'r2_sliders', r2LowVal, 255, nothing)
cv2.createTrackbar('r2UpHue', 'r2_sliders', r2UpHue, 179, nothing)
cv2.createTrackbar('r2UpSat', 'r2_sliders', r2UpSat, 255, nothing)
cv2.createTrackbar('r2UpVal', 'r2_sliders', r2UpVal, 255, nothing)
cv2.createTrackbar('gLowHue', 'g_sliders', gLowHue, 179, nothing)
cv2.createTrackbar('gLowSat', 'g_sliders', gLowSat, 255, nothing)
cv2.createTrackbar('gLowVal', 'g_sliders', gLowVal, 255, nothing)
cv2.createTrackbar('gUpHue', 'g_sliders', gUpHue, 179, nothing)
cv2.createTrackbar('gUpSat', 'g_sliders', gUpSat, 255, nothing)
cv2.createTrackbar('gUpVal', 'g_sliders', gUpVal, 255, nothing)
'''

## red ball mask areas
#red_mask_1 = cv2.inRange(im_hsv, (0, 120, 70), (10, 255, 255))
#red_mask_2 = cv2.inRange(im_hsv, (170, 120, 70), (179, 255, 255)) 

#lower_red1 = np.array([0, 120, 100])
#upper_red1 = np.array([10, 255, 255])
#lower_red2 = np.array([170, 120, 100])
#upper_red2 = np.array([179, 255, 255])

#green mask area centered around 
# (80, 127, 222)
#green_mask = cv2.inRange(im_hsv, (55, 120, 70), (105, 255, 255)) 
#    lower_green = np.array([40, 10, 200])
#    upper_green = np.array([120, 245, 255])

#lower_green = np.array([55, 120, 70])
#upper_green = np.array([105, 255, 255])


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
            
            # our file for lab 3:        
            #            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            #            lower_green = np.array([40, 10, 200])
            #            upper_green = np.array([120, 245, 255])
            #
            #            mask = cv2.inRange(hsv, lower_green, upper_green)
            #            result = cv2.bitwise_and(frame, frame, mask=mask)   
            #            cv2.imshow('mask', mask)
            #            cv2.imshow('result', result)
            
            # Prof Reamon's file posted to canvas with start and end removed ... 
            
            
            
            try:

                imgInit = frame
                
                imgBGR = cv2.resize(imgInit,(300, 300),cv2.INTER_AREA)
                r_imgBGR = imgBGR.copy()
                g_imgBGR = imgBGR.copy()
                img=cv2.cvtColor(imgBGR, cv2.COLOR_BGR2HSV)
                #print("img")
                #print(img)
    
#                hue = cv2.getTrackbarPos('Hue', 'image')
#                sat = cv2.getTrackbarPos('Sat', 'image')
#                val = cv2.getTrackbarPos('Val', 'image')

                '''
                thresholdValue = cv2.getTrackbarPos('filterThresh', 'r1_sliders')
                r1LowHue = cv2.getTrackbarPos('r1LowHue', 'r1_sliders')
                r1LowSat = cv2.getTrackbarPos('r1LowSat', 'r1_sliders')
                r1LowVal = cv2.getTrackbarPos('r1LowVal', 'r1_sliders')
                r1UpHue = cv2.getTrackbarPos('r1UpHue', 'r1_sliders')
                r1UpSat = cv2.getTrackbarPos('r1UpSat', 'r1_sliders')
                r1UpVal = cv2.getTrackbarPos('r1UpVal', 'r1_sliders')
                r2LowHue = cv2.getTrackbarPos('r2LowHue', 'r2_sliders')
                r2LowSat = cv2.getTrackbarPos('r2LowSat', 'r2_sliders')
                r2LowVal = cv2.getTrackbarPos('r2LowVal', 'r2_sliders')
                r2UpHue = cv2.getTrackbarPos('r2UpHue', 'r2_sliders')
                r2UpSat = cv2.getTrackbarPos('r2UpSat', 'r2_sliders')
                r2UpVal = cv2.getTrackbarPos('r2UpVal', 'r2_sliders')
                gLowHue = cv2.getTrackbarPos('gLowHue', 'g_sliders')
                gLowSat = cv2.getTrackbarPos('gLowSat', 'g_sliders')
                gLowVal = cv2.getTrackbarPos('gLowVal', 'g_sliders')
                gUpHue = cv2.getTrackbarPos('gUpHue', 'g_sliders')
                gUpSat = cv2.getTrackbarPos('gUpSat', 'g_sliders')
                gUpVal = cv2.getTrackbarPos('gUpVal', 'g_sliders')
                '''

                lower_red1 = np.array([r1LowHue, r1LowSat, r1LowVal])
                upper_red1 = np.array([r1UpHue, r1UpSat, r1UpVal])

                lower_red2 = np.array([r2LowHue, r2LowSat, r2LowVal])
                upper_red2 = np.array([r2UpHue, r2UpSat, r2UpVal])
                #green mask area centered around 
                # (80, 127, 222)
                lower_green = np.array([gLowHue, gLowSat, gLowVal])
                upper_green = np.array([gUpHue, gUpSat, gUpVal])














    
#                lower=[hue,sat,val]
#                lower=np.array(lower, dtype="uint8")
#                lower2=[[[hue,sat,val]]]
#                lower2=np.array(lower2, dtype="uint8")
                red1 = [[[upper_red1[0]-((upper_red1[0]-lower_red1[0])/2),upper_red1[1]-((upper_red1[1]-lower_red1[1])/2),upper_red1[2]-((upper_red1[2]-lower_red1[2])/2)]]]
                red2 = [[[upper_red2[0]-((upper_red2[0]-lower_red2[0])/2),upper_red2[1]-((upper_red2[1]-lower_red2[1])/2),upper_red2[2]-((upper_red2[2]-lower_red2[2])/2)]]]
                green = [[[upper_green[0]-((upper_green[0]-lower_green[0])/2),upper_green[1]-((upper_green[1]-lower_green[1])/2),upper_green[2]-((upper_green[2]-lower_green[2])/2)]]]
                red1=np.array(red1, dtype="uint8")
                red2=np.array(red2, dtype="uint8")
                green=np.array(green, dtype="uint8")
                redColor1 = cv2.cvtColor(red1, cv2.COLOR_HSV2BGR)##Tr
                redColor2 = cv2.cvtColor(red2, cv2.COLOR_HSV2BGR)##Tr
                greenColor = cv2.cvtColor(green, cv2.COLOR_HSV2BGR)##Tr
    
#                upperBound=limit(lower+thrs/2,[[0,179],[0,255],[0,255]])
#                lowerBound=limit(lower-thrs/2,[[0,179],[0,255],[0,255]])
                
                
                gmask=np.uint8(cv2.inRange(img,lower_green,upper_green))
                rmask1=np.uint8(cv2.inRange(img,lower_red1,upper_red1))
                rmask2=np.uint8(cv2.inRange(img,lower_red2,upper_red2))
                rmask = rmask1+rmask2
                #print("rmask")
                #print(rmask)
                #print("gmask")
                #print(gmask)
    
                rvis = np.uint8(img.copy())
                gvis = np.uint8(img.copy())
                rvis[rmask==0]=(0,0,0)
                gvis[gmask==0]=(0,0,0)
                #print("vis")
                #print(vis)
                
#                gray2 = img[:,:,2] #only want black and white image
                rgray = rvis[:,:,2]
                ggray = gvis[:,:,2]
                #print("gray")
                #print(gray)
    
                rblurred = cv2.GaussianBlur(rgray, (filt, filt), 0)
                gblurred = cv2.GaussianBlur(ggray, (filt, filt), 0)
                #print("blurred")
                #print(blurred)
    
                
#                 thresholdValue = cv2.getTrackbarPos('filterThresh', 'r1_sliders')
                rthresh = cv2.threshold(rblurred, thresholdValue, 255, cv2.THRESH_BINARY)[1]
                gthresh = cv2.threshold(gblurred, thresholdValue, 255, cv2.THRESH_BINARY)[1]
                
                red_locations = np.where(rthresh == [255])
                green_locations = np.where(gthresh == [255])
                
#                 x_red_avg = np.mean(red_locations[0])
#                 y_red_avg = np.mean(red_locations[1])
#                 x_green_avg = np.mean(green_locations[0])
#                 y_green_avg = np.mean(green_locations[1])
                

                
                
                #print("thresh")
                #print(thresh)
                
#                testArray=[(lower-thrs/2).tolist(),(lower+thrs/2).tolist(),lowerBound.tolist(),upperBound.tolist(),thresholdValue]
#    
                
                
#                cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[1]
#                
#                if cnts is None:
#                    # print("cnts is NoneType!")
#                    pass
#                else:
#            
#                    areas=int(len(cnts))
#                    splotch = np.zeros((1,areas),dtype=np.uint8)
#                    
#                    # loop over the contours
#                    try:    
#                        for i,c in enumerate(cnts,0):
#                        
#                            M = cv2.moments(c)
#                            splotch[0][i] = int(M["m00"])
#                        try:
#                            max1=np.argmax(splotch)
#                        except:
#                            max1=-1
#                        
#                        original=vis.copy()
#                        if max1>-1:
#                            M = cv2.moments(cnts[max1])
#                            cX = int(M["m10"] / M["m00"])
#                            cY = int(M["m01"] / M["m00"])
#        
#        
#                            
#                            cv2.drawContours(vis, [cnts[max1]], -1, (0, 255, 0), 2)
#                            cv2.circle(vis, (cX, cY), 7, (255, 255, 255), -1)
#                            cv2.putText(vis, "Green Light", (cX - 20, cY - 20),
#                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
#                    except:
#                        pass
                    
                
                ccr1=(int(redColor1[0][0][0]),int(redColor1[0][0][1]),int(redColor1[0][0][2]))
                ccr2=(int(redColor2[0][0][0]),int(redColor2[0][0][1]),int(redColor2[0][0][2]))
                ccg=(int(greenColor[0][0][0]),int(greenColor[0][0][1]),int(greenColor[0][0][2]))
                
                cv2.circle(r_imgBGR, (25, 25), 20, ccr1, -1)
                cv2.circle(r_imgBGR, (70, 25), 20, ccr2, -1)
                cv2.circle(g_imgBGR, (25, 25), 20, ccg, -1)
    
                rvisBGR=cv2.cvtColor(rvis, cv2.COLOR_HSV2BGR)
                gvisBGR=cv2.cvtColor(gvis, cv2.COLOR_HSV2BGR)
                
                rthresh = cv2.cvtColor(rthresh, cv2.COLOR_GRAY2BGR)
                gthresh = cv2.cvtColor(gthresh, cv2.COLOR_GRAY2BGR)
                
                
                if green_locations[0].size != 0:
                    green_left   = np.amin(green_locations[1])
                    green_right  = np.amax(green_locations[1])
                    green_top    = np.amin(green_locations[0])
                    green_bottom = np.amax(green_locations[0])
                    gvisBGR = cv2.rectangle(gvisBGR, (green_left, green_top), (green_right, green_bottom), (0, 255, 0), 2)
                    gvisBGR = cv2.putText(gvisBGR, 'GOOD GUY', (green_left, green_top-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

                    
                if red_locations[0].size != 0:
                    red_left   = np.amin(red_locations[1])
                    red_right  = np.amax(red_locations[1])
                    red_top    = np.amin(red_locations[0])
                    red_bottom = np.amax(red_locations[0])
                    rvisBGR = cv2.rectangle(rvisBGR, (red_left, red_top), (red_right, red_bottom), (0, 0, 255), 2)
                    rvisBGR = cv2.putText(rvisBGR, 'BAD GUY', (red_left, red_top-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
                    
                
                cv2.imshow('r_image',np.hstack([r_imgBGR, rthresh, rvisBGR])) #np.hstack([original, vis]))#np.hstack([thresh, gray2]))
                cv2.imshow('g_image',np.hstack([g_imgBGR, gthresh, gvisBGR])) #np.hstack([original, vis]))#np.hstack([thresh, gray2]))
                cv2.imshow('g_sliders', blank_image)
                cv2.imshow('r1_sliders', blank_image)
                cv2.imshow('r2_sliders', blank_image)
                
#                    cv2.imshow('image',np.hstack([imgBGR, visBGR])) #np.hstack([original, vis]))#np.hstack([thresh, gray2]))
                
            except KeyboardInterrupt:
                raise
            except cv2.error as e:
        
                print("Here it is \n",str(e), "\n")
                if similar(str(e), " /home/pi/opencv-3.3.0/modules/imgproc/src/imgwarp.cpp:3483: error: (-215) ssize.width > 0 && ssize.height > 0 in function resize")>.8:
                    print("\n\n\n\n Your video appears to have ended\n\n\n")
                break
                    
            
            
###########################################################################
            
            

            img_h = frame.shape[0]
            img_w = frame.shape[1]
            
            
            confidences = []
            #cwm: Takes highest confidence value and selects only that object
            for e in entries_prev:
                confidences.append(e[0]['confidence'])
                
            # print(confidences)
            
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
            #print(frame)

            if len(frame.shape) == 2:
                if frame.dtype == np.uint8: # grayscale
                    cv2.putText(frame, packet.stream_name, (25, 25), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255))
                    cv2.putText(frame, "fps: " + str(frame_count_prev[packet.stream_name]), (25, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255))
                    cv2.imshow(packet.stream_name, frame)
                else: # uint16
                    #print("frame before integer division")
                    #print(frame)
                    frame = (65535 // frame).astype(np.uint8)
                    #print("frame after integer division")
                    #print(frame)
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
