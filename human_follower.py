"""
Project: WheeHelper
Author: A. A.
"""

import common as cm
import cv2
import numpy as np
from PIL import Image
import time
from threading import Thread
from move import move
from teleop import teleop
import sys

import pyrealsense2 as rs
import stretch_body.robot

MAX_FRAMES=-1

class Human_Follower:
    def __init__(self,robot=None,pipeline=None, max_frames=-1):
        if robot is None:
            robot = get_robot()
        if pipeline is None:
            pipeline = get_camera()
        robot.head.move_to('head_pan', 0)
        robot.head.move_to('head_tilt', -np.pi/20)
        self.robot = robot
        self.pipeline = pipeline
        self.max_frames  =max_frames

        self.lbl = 'coco_labels.txt'
        self.model_dir = 'all_models'
        self.model_edgetpu = 'mobilenet_ssd_v2_coco_quant_postprocess.tflite'
        self.object_to_track='person'
        self.top_k =5
        self.threshold = 0.2
        self.track_method='depth' #['depth','area']

        self.dev_tolerance = 0.1
        self.diff_tolerance = [0.35, 0.4]
        self.dist_tolerance = [600, 850]
        self.delay_to_translate=2

        self.CURRENT_POS = 'back'
        self.NEXT_POS = 'back'
        self.LAST_POSE = 'back'
        self.V_M = 1
        self.V_R = 10

        self.save_demo = False
        self._init_track_data()
        self._set_video_write()
        self._load_model()

    def _init_track_data(self):
        self.arr_track_data = {'x_center': 0, 'y_center': 0, 'z_center': 0, 'x_deviation': 0, 'y_deviation': 0,
                               'x_diff': 0, 'y_diff': 0, 'cmd': 'stop', 'delay': 0}

    def _set_video_write(self):
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        resolution_color = [640, 480]
        # fourcc = -1
        # ret, frame = cap.read()
        ret, frame, depthframe = self.get_video()
        if ret:
            vw = frame.shape[1]
            vh = frame.shape[0]
            print('rgb: ', frame.shape,', depth: ', depthframe.shape)
        else:
            vw = resolution_color[1]
            vh = resolution_color[0]
        self.vw = vw
        self.vh = vh
        outvideo_path = 'camera-det.mp4'
        # outvideo = cv2.VideoWriter(outvideo_path, fourcc, 0.2, (vw, vh))
        self.outvideo = cv2.VideoWriter(outvideo_path, fourcc, 1, (vw, vh))

    def _load_model(self):
        self.interpreter, self.labels = cm.load_model(self.model_dir, self.model_edgetpu, self.lbl, 0)

    def finish_run(self):
        print('Finished')
        # cap.release()
        self.robot.stop()
        self.pipeline.stop()
        cv2.destroyAllWindows()
        self.outvideo.release()
        print('done')

    def set_pos(self,pos):
        self.NEXT_POS = pos

    def stop_run(self):
        self.max_frames = 1

    def start_run(self):
        self.max_frames = -1
        self.run_track()

    def get_video(self):
        try:
            for i_frame in range(100):
                # Get the latest frames from the camera.
                frames = self.pipeline.wait_for_frames()
                depth_frame = frames.get_depth_frame()
                color_frame = frames.get_color_frame()
                if not color_frame:
                    continue

                # Convert images to numpy arrays.
                depth_image = np.asanyarray(depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())

                return True, color_image, depth_image
        except:
            print('Failed to get video')
            return False, None, None

    def track_object(self, objs, labels,depth_image):

        if (len(objs) == 0):
            print("no objects to track")
            self._init_track_data()
            return None
        flag = 0
        cand_list=[]
        for obj in objs:
            lbl = labels.get(obj.id, obj.id)
            if (lbl==self.object_to_track):
                x_min, y_min, x_max, y_max = list(obj.bbox)
                flag += 1
                d = abs(y_max-y_min)+abs(x_max-x_min)
                w = int((x_max - x_min)/2 * self.vw)
                h = int((y_max - y_min)/2 * self.vh)
                d = -depth_image[h,w]
                cand_list.append([x_min, y_min, x_max, y_max,d ])

        if (flag == 0):
            print("selected object no present")
            self._init_track_data()
            return None

        cand_list = sorted(cand_list, key= lambda x: x[4],reverse=True)
        x_min, y_min, x_max, y_max, s = cand_list[0]

        x_diff = x_max - x_min
        y_diff = y_max - y_min


        obj_x_center = x_min + (x_diff / 2)
        obj_x_center = round(obj_x_center, 3)

        obj_y_center = y_min + (y_diff / 2)
        obj_y_center = round(obj_y_center, 3)

        depth_list=[]
        w = int(obj_x_center* self.vw)
        h = int(obj_y_center* self.vh)
        for ii in range(h-5,h+5):
            for jj in range(w-5,w+5):
                if ii < 0 or ii >= len(depth_image) or jj <0 or jj >= len(depth_image[0]):
                    continue
                if depth_image[ii,jj]>0:
                    #print('depth',ii,jj,depth_image[ii,jj])
                    depth_list.append(depth_image[ii,jj])
        if len(depth_list)==0:
            depth_list=[0]
        obj_z_center = int(max(depth_list))

        #print("selected object  present", self.object_to_track, "x_diff: ", round(x_diff, 5),"y_diff: ", round(y_diff, 5), 'center:', "[",obj_x_center, obj_y_center,obj_z_center,"]")

        x_deviation = round(0.5 - obj_x_center, 3)
        y_deviation = round(0.5 - obj_y_center, 3)

        #print("Deviation, {", x_deviation, y_deviation, "}")

        self.arr_track_data['x_center'] = obj_x_center
        self.arr_track_data['y_center'] = obj_y_center
        self.arr_track_data['z_center'] = obj_z_center
        self.arr_track_data['x_deviation'] = x_deviation
        self.arr_track_data['y_deviation'] = y_deviation
        self.arr_track_data['x_diff'] = x_diff
        self.arr_track_data['y_diff'] = y_diff

        if self.CURRENT_POS =='left' or self.CURRENT_POS =='right':
            thread = Thread(target=self.move_robot_alongside)
        else:
            thread = Thread(target=self.move_robot_back)
        thread.start()
        thread.join()
        return self.arr_track_data

    def move_robot_back(self):

        cmd=''
        delay1=0
        direction=''
        x_deviation = self.arr_track_data['x_deviation']
        x_diff = self.arr_track_data['x_diff']
        obj_z_center =  self.arr_track_data['z_center']

        cmd_fb, cmd_lr=[],[]
        if (x_deviation >= self.dev_tolerance):
            delay1 = self.get_delay(x_deviation, 'l')
            direction ='l'
            cmd = "Move Left"
            cmd_lr = [delay1, direction, cmd]

        elif (x_deviation <= -1 * self.dev_tolerance):
            delay1 = self.get_delay(x_deviation, 'r')
            direction = 'r'
            delay1 = -delay1
            cmd = "Move Right"
            cmd_lr = [delay1, direction, cmd]

        if self.track_method=='depth' :
            if (obj_z_center > self.dist_tolerance[1]):
                delay1 = (obj_z_center - self.dist_tolerance[1])/1000
                delay1 = min(delay1/2, 0.4)
                direction = 'f'
                cmd = "Move Forward"
                cmd_fb = [delay1, direction, cmd]

            elif (obj_z_center <= self.dist_tolerance[0]):
                delay1 = (self.dist_tolerance[0]-obj_z_center)/1000
                delay1 = min(delay1/2, 0.4)
                direction = 'b'
                delay1 = -delay1
                cmd = "Move Backward"
                cmd_fb = [delay1, direction, cmd]
        else:
            if (x_diff <= self.diff_tolerance[0]):
                delay1 = self.get_delay(x_diff - self.diff_tolerance[0], 'f')
                direction = 'f'
                cmd = "Move Forward"
                cmd_fb = [delay1, direction, cmd]

            elif (x_diff >= self.diff_tolerance[1]):
                delay1 = self.get_delay(x_diff - self.diff_tolerance[1], 'b')
                direction = 'b'
                delay1 = -delay1
                cmd = "Move Backward"
                cmd_fb = [delay1, direction, cmd]

        if len(cmd)==0:
            cmd = 'Stop'
            self.arr_track_data['cmd'] = cmd
            print('Stop robot back !!!', self.arr_track_data)
        else:
            if len(cmd_lr)>0:
                delay1, direction, cmd = cmd_lr
            else:
                delay1, direction, cmd = cmd_fb
            self.arr_track_data['cmd'] = cmd
            self.arr_track_data['delay'] = delay1
            print('Move robot back !!!', self.arr_track_data)
            self.robot_base_move(delay1, direction)

    def move_robot_alongside(self):

        cmd = ''
        delay1 = 0
        direction = ''

        x_deviation = self.arr_track_data['x_deviation']
        x_diff = self.arr_track_data['x_diff']
        obj_z_center =  self.arr_track_data['z_center']

        cmd_fb, cmd_lr=[],[]
        dist_tolerance = [self.dist_tolerance[0]-200,self.dist_tolerance[1]+500]
        diff_tolerance = [self.diff_tolerance[0] - 0.5, self.diff_tolerance[1] + 0.5]
        tolerance = self.dev_tolerance*0.7
        dev_to_translate=0.7

        if self.track_method == 'depth':
            if (obj_z_center > dist_tolerance[1]):
                delay1 = (obj_z_center - dist_tolerance[1]) / 1000
                delay1 =  self.get_delay(delay1, 'r')
                if self.CURRENT_POS == 'left':
                    delay1 = -delay1
                    direction = 'r'
                    cmd = "Move Right"
                else:
                    direction = 'l'
                    cmd = "Move Left"
                cmd_lr = [delay1, direction, cmd]

            elif (obj_z_center <= dist_tolerance[0]) and obj_z_center>10:
                delay1 = (dist_tolerance[0] - obj_z_center) / 1000
                delay1 =  self.get_delay(delay1, 'l')
                if self.CURRENT_POS == 'left':
                    direction = 'l'
                    cmd = "Move Left"
                else:
                    delay1 = -delay1
                    direction = 'r'
                    cmd = "Move Right"
                cmd_lr = [delay1, direction, cmd]
        else:
            if (x_diff <= diff_tolerance[0]):
                delay1 = self.get_delay(x_diff - diff_tolerance[0], 'r')
                if self.CURRENT_POS == 'left':
                    delay1 = -delay1
                    direction = 'r'
                    cmd = "Move Right"
                else:
                    direction = 'l'
                    cmd = "Move Left"
                cmd_lr = [delay1, direction, cmd]

            elif (x_diff >= diff_tolerance[1]):
                delay1 = self.get_delay(x_diff - diff_tolerance[1], 'f')
                if self.CURRENT_POS == 'left':
                    direction = 'l'
                    cmd = "Move Left"
                else:
                    delay1 = -delay1
                    direction = 'r'
                    cmd = "Move Right"
                cmd_lr = [delay1, direction, cmd]

        if (x_deviation >= tolerance):
            delay1 = dev_to_translate*self.get_delay(x_deviation, 'f')
            if self.CURRENT_POS == 'left':
                direction = 'f'
                cmd = "Move Forward"
            else:
                delay1 = -delay1
                direction = 'b'
                cmd = "Move Backward"
            cmd_fb = [delay1, direction, cmd]

        elif (x_deviation <= -1 * tolerance):
            delay1 = dev_to_translate*self.get_delay(x_deviation, 'b')
            if self.CURRENT_POS == 'left':
                direction = 'b'
                delay1 = -delay1
                cmd = "Move Backward"
            else:
                direction = 'f'
                cmd = "Move Forward"
            cmd_fb = [delay1, direction, cmd]


        if len(cmd)==0:
            cmd = 'Stop'
            self.arr_track_data['cmd'] = cmd
            print('Stop robot alongside !!!', self.arr_track_data)
        else:
            if len(cmd_fb)>0:
                delay1, direction, cmd = cmd_fb
                self.arr_track_data['cmd'] = cmd
                self.arr_track_data['delay'] = delay1
                print('Move robot alongside !!!', self.arr_track_data)
                self.robot_base_move(delay1, direction)
            else:
                cmd = 'Stop'
                self.arr_track_data['cmd'] = cmd
                print('Stop robot alongside !!!', self.arr_track_data)

    def robot_base_move(self, delay, direction):
        if direction in ['f','b']:
            self.robot.base.translate_by(delay*self.delay_to_translate, v_m=self.V_M)
            self.robot.push_command()
            print('robot.base.translate_by',delay*self.delay_to_translate,direction)
        elif direction in ['l','r']:
            self.robot.base.rotate_by(delay*self.delay_to_translate, v_r=self.V_R)
            self.robot.push_command()
            print('robot.base.rotate_by', delay * self.delay_to_translate, direction)

    def get_delay(self, deviation, direction):
        deviation = abs(deviation)
        if (direction == 'f' or direction == 'b'):
            if (deviation >= 0.3):
                d = 0.1
            elif (deviation >= 0.2 and deviation < 0.30):
                d = 0.075
            elif (deviation >= 0.15 and deviation < 0.2):
                d = 0.045
            else:
                d = 0.035
            d = d*1.3
        else:
            if (deviation >= 0.4):
                d = 0.080
            elif (deviation >= 0.35 and deviation < 0.40):
                d = 0.070
            elif (deviation >= 0.30 and deviation < 0.35):
                d = 0.060
            elif (deviation >= 0.25 and deviation < 0.30):
                d = 0.050
            elif (deviation >= 0.20 and deviation < 0.25):
                d = 0.040
            else:
                d = 0.030
            d = d*1.3

        return d

    def append_text_img1(self, cv2_im, objs, labels, arr_dur, arr_track_data):
        height, width = cv2_im.shape[:2]
        font = cv2.FONT_HERSHEY_SIMPLEX
        tolerance = self.dev_tolerance

        # draw black rectangle on top
        cv2_im = cv2.rectangle(cv2_im, (0, 0), (width, 24), (0, 0, 0), -1)

        # write processing durations
        cam = round(arr_dur[0] * 1000, 0)
        inference = round(arr_dur[1] * 1000, 0)
        other = round(arr_dur[2] * 1000, 0)
        text_dur = 'Camera: {}ms   Inference: {}ms   other: {}ms'.format(cam, inference, other)
        cv2_im = cv2.putText(cv2_im, text_dur, (int(width / 4) - 30, 16), font, 0.4, (255, 255, 255), 1)

        # write FPS
        total_duration = cam + inference + other
        fps = round(1000 / total_duration, 1)
        text1 = 'FPS: {}'.format(fps)
        cv2_im = cv2.putText(cv2_im, text1, (10, 20), font, 0.7, (150, 150, 255), 2)

        # draw black rectangle at bottom
        cv2_im = cv2.rectangle(cv2_im, (0, height - 24), (width, height), (0, 0, 0), -1)

        # write deviations and tolerance
        str_tol = 'Tol : {}'.format(tolerance)
        cv2_im = cv2.putText(cv2_im, str_tol, (10, height - 8), font, 0.55, (150, 150, 255), 2)

        x_dev = arr_track_data['x_deviation']
        str_x = 'X: {}'.format(x_dev)
        if (abs(x_dev) < tolerance):
            color_x = (0, 255, 0)
        else:
            color_x = (0, 0, 255)
        cv2_im = cv2.putText(cv2_im, str_x, (110, height - 8), font, 0.55, color_x, 2)

        # write command, tracking status and speed
        cmd = arr_track_data['cmd']
        cv2_im = cv2.putText(cv2_im, str(cmd), (int(width / 2) + 10, height - 8), font, 0.68, (0, 255, 255), 2)

        delay1 = arr_track_data['delay']
        str_sp = 'Speed: {}%'.format(round(delay1 / (0.1) * 100, 1))
        cv2_im = cv2.putText(cv2_im, str_sp, (int(width / 2) + 185, height - 8), font, 0.55, (150, 150, 255), 2)

        if (cmd == 0) or len(cmd)==0:
            str1 = "No Human"
        elif (cmd == 'Stop'):
            str1 = 'Stop'
        elif str(cmd) in ['Move Left','Move Right','Move Forward','Move Backward']:
            str1 = str(cmd)
        else:
            str1 = 'Tracking'
        cv2_im = cv2.putText(cv2_im, str1, (width - 140, 18), font, 0.7, (0, 255, 255), 2)

        # draw center cross lines
        cv2_im = cv2.rectangle(cv2_im, (0, int(height / 2) - 1), (width, int(height / 2) + 1), (255, 0, 0), -1)
        cv2_im = cv2.rectangle(cv2_im, (int(width / 2) - 1, 0), (int(width / 2) + 1, height), (255, 0, 0), -1)

        # draw the center red dot on the object
        x_center = arr_track_data['x_center']
        y_center = arr_track_data['y_center']
        z_center = arr_track_data['z_center']
        cv2_im = cv2.circle(cv2_im, (int(x_center * width), int(y_center * height)), 7, (0, 0, 255), -1)

        # draw the tolerance box
        cv2_im = cv2.rectangle(cv2_im, (int(width / 2 - tolerance * width), 0),
                               (int(width / 2 + tolerance * width), height), (0, 255, 0), 2)

        for obj in objs:
            x0, y0, x1, y1 = list(obj.bbox)
            x0, y0, x1, y1 = int(x0 * width), int(y0 * height), int(x1 * width), int(y1 * height)
            percent = int(100 * obj.score)

            box_color, text_color, thickness = (0, 150, 255), (0, 255, 0), 1
            depth_value = z_center
            text3 = '{}% {}, [{},{},{}]'.format(percent, labels.get(obj.id, obj.id),x_center * width,y_center * height, depth_value)
            if (labels.get(obj.id, obj.id) == "person"):
                cv2_im = cv2.rectangle(cv2_im, (x0, y0), (x1, y1), box_color, thickness)
                cv2_im = cv2.putText(cv2_im, text3, (x0, y1 - 5), font, 0.5, text_color, thickness)

        return cv2_im

    def run_track(self):
        print('begin run_track')
        arr_dur = [0, 0, 0]
        i_frame = 0
        while True:
            i_frame += 1
            if i_frame >= self.max_frames and self.max_frames>0:
                break

            start_time = time.time()

            # ----------------Capture Camera Frame-----------------
            start_t0 = time.time()
            ret, frame, depth_frame = self.get_video()
            if not ret:
                print('Warning !!! No any frames !!!!!')
                break

            cv2_im = frame
            # cv2_im = cv2.flip(cv2_im, 0)
            # cv2_im = cv2.flip(cv2_im, 1)

            cv2_im_rgb = cv2.cvtColor(cv2_im, cv2.COLOR_BGR2RGB)
            pil_im = Image.fromarray(cv2_im_rgb)
            pil_im = pil_im.rotate(-90)
            cv2_im = np.asarray(pil_im)
            cv2_im = cv2.cvtColor(cv2_im, cv2.COLOR_RGB2BGR)

            depth_frame1 = Image.fromarray(depth_frame)
            depth_frame1 = depth_frame1.rotate(-90)
            depth_image = np.asarray(depth_frame1)

            arr_dur[0] = time.time() - start_t0

            # -------------------Inference---------------------------------
            start_t1 = time.time()
            cm.set_input(self.interpreter, pil_im)
            self.interpreter.invoke()
            objs = cm.get_output(self.interpreter, score_threshold=self.threshold, top_k=self.top_k)

            arr_dur[1] = time.time() - start_t1

            # -----------------other------------------------------------
            start_t2 = time.time()
            ret = self.track_object(objs, self.labels, depth_image)  # tracking  <<<<<<<
            if ret is None:
                continue

            if self.save_demo:
                cv2_im = self.append_text_img1(cv2_im, objs, self.labels, arr_dur, ret)
                cv2.imwrite('demo/human_track_{}.png'.format(i_frame), cv2_im)
                cv2.imshow('Stream', cv2_im)
                self.outvideo.write(cv2_im)

            arr_dur[2] = time.time() - start_t2
            fps = round(1.0 / (time.time() - start_time), 1)
            print(i_frame, "*********FPS: ", fps, "************")
            '''
            if i_frame==10:
                self.NEXT_POS='left'
            if i_frame == 30:
                self.NEXT_POS = 'back'
            if i_frame == 50:
                self.NEXT_POS = 'right'
            '''
            if self.CURRENT_POS != self.NEXT_POS:
                time.sleep(1)
                self.robot.head.move_to('head_pan', 0)
                obj_z_center = self.arr_track_data['z_center']
                if self.CURRENT_POS=='back':
                    current_dis = min(obj_z_center+100, self.dist_tolerance[1])/1000
                    next_dis = 0.75
                    self.robot.head.move_to('head_tilt', -np.pi / 10)
                else:
                    depth = obj_z_center/1000

                    current_dis = max(0.55, min(depth, 0.9))
                    next_dis = (self.dist_tolerance[0] + self.dist_tolerance[1]+200)/2/1000
                    self.robot.head.move_to('head_tilt', -np.pi / 20)
                move(self.robot, self.CURRENT_POS, self.NEXT_POS, current_dis, next_dis)
                self.CURRENT_POS = self.NEXT_POS
        print('done !!!')

def get_robot():
    # -----initialise motor speed-----------------------------------
    robot=stretch_body.robot.Robot()
    robot.startup()
    if not robot.is_calibrated():
        robot.home() #blocking
        robot.stow()
        robot.head.move_to('head_pan', 0)
    #robot.pretty_print()
    return robot

def get_camera():
    # Video streaming configuration.
    # Note: the actual fps will probably be lower than the target, especially if larger resolutions are used or multiple videos are saved.
    fps_color = 30 # FPS for color-only videos and for color+depth videos
    fps_depth_downsample_factor = 3 # The depth frame rate will be fps_color/fps_depth_downsample_factor.
                                    # Only used for the depth-only video stream (combined color+depth videos will be at the color fps).
    resolution_color = [640, 480]   # [1920, 1080], [1280, 720], [640, 480]
    resolution_depth = [640, 480]   # [1280, 720], [640, 480]
    frameBuffer_limit_s = 5 # How many seconds of frames to keep in the buffer - probably something short to avoid memory usage issues
                             # (different from save_limit, which limits the overall video capture length and is set by the command line argument above).
    # Some image processing options.
    apply_local_histogram_equalization = False
    apply_global_histogram_equalization = False

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, resolution_depth[0], resolution_depth[1], rs.format.z16,fps_color)  # note that the fps downsampling will be applied later
    config.enable_stream(rs.stream.color, resolution_color[0], resolution_color[1], rs.format.bgr8, fps_color)
    frame_width_colorAndDepth = resolution_color[1] + resolution_depth[1]
    frame_height_colorAndDepth = max(resolution_color[0], resolution_depth[0])
    frame_width_color = resolution_color[1]
    frame_height_color = resolution_color[0]
    frame_width_depth = resolution_depth[1]
    frame_height_depth = resolution_depth[0]
    frameBuffer_color = []
    frameBuffer_depth = []
    frameBuffer_colorAndDepth = []
    frameBuffer_limit_color = round(frameBuffer_limit_s * fps_color)
    frameBuffer_limit_depth = round(frameBuffer_limit_s * fps_color / fps_depth_downsample_factor)
    capture_limit_color = round(fps_color * 60 * 60)

    pipeline.start(config)
    start_capture_time_s = time.time()
    end_capture_time_color_s = start_capture_time_s
    end_capture_time_depth_s = start_capture_time_s
    frame_count_color = 0
    frame_count_depth = 0
    print("Begin to capture video...")
    return pipeline

def main():
    human_follower = Human_Follower(max_frames=MAX_FRAMES)
    #human_follower.run_track()
    try:
        thread = Thread(target=human_follower.run_track)
        thread.start()
        time.sleep(1)
        print('start')
        '''
        if Turn Command:
            human_follower.set_pos('left')
        if Stop:
            human_follower.stop_run()
            teleop(human_follower.robot)
        '''
        thread.join()
        human_follower.finish_run()
    except:
        human_follower.finish_run()


if __name__ == '__main__':
    main()

    #except:
    #    pipeline.stop()
    #    robot.stop()

