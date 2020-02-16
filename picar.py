# -*- coding: UTF-8 -*-
import logging
import cv2
import datetime
import actuator
from road_lane_detection import RoadlaneFollower

class Picar(object):

    #获取屏幕大小（px）
    capture_width = 320
    capture_height = 240
    servo_angle = 90
    motor_speed = 0

    def __init__(self):
        logging.info('creating Picar...')
        self.picar = actuator.Actuator()

        logging.info('setup camera...')
        self.camera = cv2.VideoCapture(-1)
        self.camera.set(3, self.capture_width)
        self.camera.set(4, self.capture_height)

        self.lane_follower = RoadlaneFollower(self)
        #self.traffic_sign_processor = ObjectsOnRoadProcessor(self)

        #获取视频流
        self.fourcc = cv2.VideoWriter_fourcc(*'XVID')
        datestr = datetime.datetime.now().strftime("%y%m%d_%H%M%S")#视频文件格式
        self.video_orig = self.create_video_record('./data/video/car_video%s.avi' % datestr)
        self.video_lane = self.create_video_record('./data/video/car_video_lane%s.avi' % datestr)
        #self.video_objs = self.create_video_record('./data/video/car_video_objs%s.avi' % datestr)

    def __exit__(self, exc_type, exc_val, exc_tb):
        if exc_tb is not None:
            logging.info('Exiting with statement with exception %s' % exc_tb)
        self.clean()

    def clean(self):
        logging.info('Picar stoptting...')
        self.servo_angle=90
        self.motor_speed=0
        self.picar.stop()
        self.camera.release()
        self.video_orig.release()
        self.video_lane.release()
        cv2.destroyAllWindows()

    def drive(self,speed=0,angle=15.5):
        logging.info('starting to drive speed %s' %speed)
        self.motor_speed = speed
        self.servo_angle = angle

        i = 0
        while self.camera.isOpened():
            _, image_lane = self.camera.read()

            i+=1

            self.video_orig.write(image_lane)

            self.picar.speed(self.motor_speed)
            self.picar.steer( self.servo_angle)
            self.picar.forward()
            #TODO 车道检测
            image_lane = self.follow_lane(image_lane)
            self.video_lane.write(image_lane)
            show_image('车道线', image_lane)

            #TODO 交通指示牌检测

            #如果摄像头退出则停止
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.clean()
                break

    def follow_lane(self, image):
        image = self.lane_follower.follow_lane(image)
        return image

    #录制视频
    def create_video_record(self, path):
        return cv2.VideoWriter(path, self.fourcc, 20.0, (self.capture_width, self.capture_height))


def show_image(title, frame, show=True):
    if show:
        cv2.imshow(title, frame)


def main():
    car = Picar();
    car.drive(13)

if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO, format='%(levelname)-5s:%(asctime)s: %(message)s')

    main()