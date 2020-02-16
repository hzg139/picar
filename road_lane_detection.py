# -*- coding: UTF-8 -*-
import logging
import cv2
import numpy as np
import math
import datetime
import sys

class RoadlaneFollower(object):
    def __init__(self,picar = None):
        self.picar = picar
        self.curr_steering_angle = 90 #初始舵机角度，居中
        #self.IS_STOP = False #停车/启动

    #车道识别
    def follow_lane(self,frame):
        show_image('原始',frame)
        lane_lines, frame = detect_lane(frame)
        final_frame = self.steer(frame, lane_lines)

        return final_frame

    #舵机转向
    def steer(self, frame, lane_lines):
        logging.debug('steering...')
        if len(lane_lines) == 0:
            logging.error('没有检测到车道线.')
            #停止运行
            #self.IS_STOP = True
            return frame

        new_steering_angle = compute_steering_angle(frame, lane_lines)
        self.curr_steering_angle = stabilize_steering_angle(self.curr_steering_angle, new_steering_angle,
                                                            len(lane_lines))

        if self.picar is not None:
            #转向角
            #self.car(self.curr_steering_angle)
            self.picar.servo_angle = self.curr_steering_angle
            logging.info('速度：%s , 转向角度: %s' % (self.picar.motor_speed,self.picar.servo_angle))

        curr_heading_image = display_heading_line(frame, self.curr_steering_angle)
        show_image("自动驾驶模式", curr_heading_image)

        return curr_heading_image
#-------------------------
#道路识别：
# 1.将RGB转换为hsv（色相/饱和度/值）色彩空间，转换后进入一定的颜色范围，通过inRange提升蓝色渲染遮罩代码，后续可使用Canney函数检测图像的边缘，绘制车道 detect_edges
# 2.设置感兴趣的区域，仅聚焦屏幕下半部分 region_of_interest
# 3.线段检测，使用霍夫变换提取图像中的线段 detect_line_segments
# 4.将检测的线段合并为车道线 average_slope_intercept
# 5.绘制车道display_lines
#-------------------------
def detect_lane(frame):
    logging.debug('detecting lane lines...')

    edges = detect_edges(frame)
    show_image('edges', edges)

    cropped_edges = region_of_interest(edges)
    show_image('edges cropped', cropped_edges)

    line_segments = detect_line_segments(cropped_edges)
    line_segment_image = display_lines(frame, line_segments)
    show_image("line segments", line_segment_image)

    lane_lines = average_slope_intercept(frame, line_segments)
    lane_lines_image = display_lines(frame, lane_lines)
    show_image("lane lines", lane_lines_image)

    return lane_lines, lane_lines_image

def detect_edges(frame):
    # filter for blue lane lines
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    show_image("hsv", hsv)
    lower_blue = np.array([30, 40, 0])
    upper_blue = np.array([150, 255, 255])
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    show_image("blue mask", mask)

    # 边缘检测
    edges = cv2.Canny(mask, 200, 400)

    return edges

def region_of_interest(canny):
    height, width = canny.shape
    mask = np.zeros_like(canny)

    # 仅聚焦屏幕下半部分

    polygon = np.array([[
        (0, height * 1 / 2),
        (width, height * 1 / 2),
        (width, height),
        (0, height),
    ]], np.int32)
    #合并MASK
    cv2.fillPoly(mask, polygon, 255)
    show_image("mask", mask)
    masked_image = cv2.bitwise_and(canny, mask)
    return masked_image

def detect_line_segments(cropped_edges):
    # 霍夫变换，tuning min_threshold, minLineLength, maxLineGap is a trial and error process by hand
    rho = 1  # 距离精度, i.e. 1 pixel
    angle = np.pi / 180  # 弧度的角度精度, i.e. 1 degree
    min_threshold = 10  # 视为一个线段的投票数，投票数越高，越可能是线段
    #minLineLength是线段的最小长度
    #maxLineGap是两个可以分开但仍被视为单个线段的线段的最大值（以像素为单位）。例如，如果我们有虚线车道标记，则通过指定合理的最大线间距，Hough Transform会将整个虚线车道线视为一条直线，这是理想的。
    line_segments = cv2.HoughLinesP(cropped_edges, rho, angle, min_threshold, np.array([]), minLineLength=8,
                                    maxLineGap=4)

    if line_segments is not None:
        for line_segment in line_segments:
            logging.debug('detected line_segment:')
            logging.debug("%s of length %s" % (line_segment, length_of_line_segment(line_segment[0])))

    return line_segments


def average_slope_intercept(frame, line_segments):
    """
    将线段合并为一条或两条车道线
    如果所有线斜率 < 0: 我们仅检测到左侧车道
    如果所有线斜率 > 0: 我们仅检测到右侧车道
    """
    lane_lines = []
    if line_segments is None:
        logging.info('没有发现车道线...')
        return lane_lines

    height, width, _ = frame.shape
    left_fit = []
    right_fit = []

    boundary = 1/3
    left_region_boundary = width * (1 - boundary)  # 左车道线段应该是在屏幕的左侧三分之二
    right_region_boundary = width * boundary # 右车道线段应位于屏幕的左2/3

    for line_segment in line_segments:
        for x1, y1, x2, y2 in line_segment:
            if x1 == x2:
                logging.info('边线重合 (slope=inf): %s' % line_segment)
                continue
            fit = np.polyfit((x1, x2), (y1, y2), 1)
            slope = fit[0]
            intercept = fit[1]
            if slope < 0:
                if x1 < left_region_boundary and x2 < left_region_boundary:
                    left_fit.append((slope, intercept))
            else:
                if x1 > right_region_boundary and x2 > right_region_boundary:
                    right_fit.append((slope, intercept))

    left_fit_average = np.average(left_fit, axis=0)
    if len(left_fit) > 0:
        lane_lines.append(make_points(frame, left_fit_average))

    right_fit_average = np.average(right_fit, axis=0)
    if len(right_fit) > 0:
        lane_lines.append(make_points(frame, right_fit_average))

    logging.debug('lane lines: %s' % lane_lines)  # [[[316, 720, 484, 432]], [[1009, 720, 718, 432]]]

    return lane_lines


#在车道线内定义小车转向角度
def compute_steering_angle(frame, lane_lines):
    """ 基于车道检测，在中线侧判断是否左、右即转向的角度
    """
    if len(lane_lines) == 0:
        logging.info('未检测到车道线...')
        return -90

    height, width, _ = frame.shape
    if len(lane_lines) == 1:
        logging.info('仅检测到一条车道线. %s' % lane_lines[0])
        x1, _, x2, _ = lane_lines[0][0]
        x_offset = x2 - x1
    else:
        _, _, left_x2, _ = lane_lines[0][0]
        _, _, right_x2, _ = lane_lines[1][0]
        camera_mid_offset_percent = 0.02 # 0.0 means car pointing to center, -0.03: car is centered to left, +0.03 means car pointing to right
        mid = int(width / 2 * (1 + camera_mid_offset_percent))
        x_offset = (left_x2 + right_x2) / 2 - mid

    # find the steering angle, which is angle between navigation direction to end of center line
    y_offset = int(height / 2)

    angle_to_mid_radian = math.atan(x_offset / y_offset)  # 与垂重中心线的角度
    angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)  # 角度 (in degrees) 距离中心线
    steering_angle = angle_to_mid_deg + 90  # 小车前轮转向的角度

    logging.info('坐标点：%s  - 行驶转向角: %s' % (str(x_offset)+','+str(y_offset),steering_angle))
    return steering_angle


# 转向稳定策略
# 如果新角度大于max_angle_deviation当前角度的max_angle_deviation度数，则将方向转向新角度
def stabilize_steering_angle(curr_steering_angle, new_steering_angle, num_of_lane_lines,
                             max_angle_deviation_two_lines=5, max_angle_deviation_one_lane=1):
    """
    使用最后的转向角来稳定转向角
    如果新角度与当前角度差异太大，
    仅转动max_angle_deviation度
    """
    #TODO 优化当仅检测到1条车道时，应及时转向发现车道
    if num_of_lane_lines == 2:
        # if both lane lines detected, then we can deviate more
        max_angle_deviation = max_angle_deviation_two_lines
    else:
        # if only one lane detected, don't deviate too much
        max_angle_deviation = max_angle_deviation_one_lane

    angle_deviation = new_steering_angle - curr_steering_angle
    logging.info('max_angle_deviation： %s,  - new_steering_angle： %s   ' % (str(max_angle_deviation),str(new_steering_angle)))
    if abs(angle_deviation) > max_angle_deviation:
        stabilized_steering_angle = int(curr_steering_angle
                                        + max_angle_deviation * angle_deviation / abs(angle_deviation))
    else:
        stabilized_steering_angle = new_steering_angle
    logging.info('修正前角度: %s, 修正后角度: %s' % (new_steering_angle, stabilized_steering_angle))
    return stabilized_steering_angle


def display_lines(frame, lines, line_color=(0, 255, 0), line_width=10):
    line_image = np.zeros_like(frame)
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(line_image, (x1, y1), (x2, y2), line_color, line_width)
    line_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)
    return line_image


def display_heading_line(frame, steering_angle, line_color=(0, 0, 255), line_width=5, ):
    heading_image = np.zeros_like(frame)
    height, width, _ = frame.shape

    # figure out the heading line from steering angle
    # heading line (x1,y1) is always center bottom of the screen
    # (x2, y2) requires a bit of trigonometry

    # Note: the steering angle of:
    # 0-89 degree: turn left
    # 90 degree: going straight
    # 91-180 degree: turn right
    steering_angle_radian = steering_angle / 180.0 * math.pi
    x1 = int(width / 2)
    y1 = height
    x2 = int(x1 - height / 2 / math.tan(steering_angle_radian))
    y2 = int(height / 2)

    cv2.line(heading_image, (x1, y1), (x2, y2), line_color, line_width)
    heading_image = cv2.addWeighted(frame, 0.8, heading_image, 1, 1)

    return heading_image


def length_of_line_segment(line):
    x1, y1, x2, y2 = line
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def make_points(frame, line):
    height, width, _ = frame.shape
    slope, intercept = line
    y1 = height  # bottom of the frame
    y2 = int(y1 * 1 / 2)  # make points from middle of the frame down

    # bound the coordinates within the frame
    x1 = max(-width, min(2 * width, int((y1 - intercept) / slope)))
    x2 = max(-width, min(2 * width, int((y2 - intercept) / slope)))
    return [[x1, y1, x2, y2]]

def show_image(title, frame, show=False):
    if show:
        cv2.imshow(title, frame)
