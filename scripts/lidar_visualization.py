import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float32
import numpy as np
import pygame
import time
import math
import sys
#from rdp import rdp
    
class LaserVisualization:
    def __init__(self, model, screen, dpa):
        self.sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.sub = rospy.Subscriber('/imu/mag', Vector3Stamped, self.imu_callback)
        scanArray = []
        self.model = model
        self.dpa = dpa
        self.screen = screen
        self.angle_min = 0
        self.angle_max = 0
        self.angle_increment = 0
        self.zeroIndex = 0
        self.ranges = []
        self.heading = 0
        self.heading_x = 0
        self.heading_y = 0
        self.scaling = 30
        self.tolerance = .1*self.scaling
        self.angle_threshold = .1
        self.cone_detection_angle = .05      #in radians
        self.cone_detection_threshold = .02 #in meters
        self.cone_detection_jump = .3 # in meters
        self.center = (self.model.width/2, self.model.height/2)
    def imu_callback(self, scan):
        if(not math.isnan(scan.vector.z)):
            #self.heading_y = scan.vector.y*200
            #self.heading_x = scan.vector.z*200
            self.heading = scan.vector.z
            pass
        #print self.heading_x
        self.heading_x = -int(math.cos(self.heading)*200)
        self.heading_y = int(math.sin(self.heading)*200)
        #print self.heading_x, self.heading_y


    def laser_callback(self, scan):
        self.angle_min = scan.angle_min
        self.angle_increment = scan.angle_increment
        self.zeroIndex = -self.angle_min/self.angle_increment
        self.ranges = scan.ranges
        self.draw()
    def draw(self):
        self.screen.fill(pygame.Color('grey'))
        xAndYArray = []
        prev_distance = 0
        max_point_difference = 0
        noise_index_array = []
        temp_index_array = []
        angle_started = False
        temp_angle = self.angle_min
        """
        for i in range(len(self.ranges)):
            if (self.ranges[i] > .4 and self.ranges[i-1] > .4):
                self.current_angle = i*self.angle_increment + self.angle_min

                if(abs(prev_distance - self.ranges[i]) > self.cone_detection_jump):
                    temp_index_array = []
                    angle_started = False
                else:
                    if(len(temp_index_array) > 0):
                        if(abs((temp_index_array[0] - i)*self.angle_increment) > self.cone_detection_angle):
                            noise_index_array.extend(temp_index_array)
                        else:
                            temp_index_array = []
                            angle_started = False
                #
                # if(not math.isnan(self.ranges[i])):
                #     point_difference = abs(prev_distance - self.ranges[i])
                #     if (point_difference > self.cone_detection_jump):
                #         pass
                #         #temp_index_array.append((i, point_difference))
                    

                #     if(len(temp_index_array) > 0):
                #         if(abs(point_difference) < self.cone_detection_threshold):
                #             temp_index_array.append(i)
                #         else:
                #             if((i - temp_index_array[0])*self.angle_increment < self.cone_detection_angle):
                #                 #if the angle of index is less than the cone detection angle, pass the temp index into the shit
                #                 noise_index_array.extend(temp_index_array)
                #                 temp_index_array = []
                if(not math.isnan(self.ranges[i])):
                    if(self.current_angle - temp_angle > self.angle_threshold):
                        angle_started = True
                    temp_angle = self.current_angle
                    if angle_started:
                        temp_index_array.append(i)
                #if()
                prev_distance = self.ranges[i]"""



        for i in range(len(self.ranges)):
            if i in noise_index_array:
                color = pygame.Color('red')
            else:
                color = pygame.Color('blue')
            self.current_angle = i*self.angle_increment + self.angle_min
            if(not math.isnan(self.ranges[i])):
                x = -int(self.ranges[i]*math.sin(self.current_angle)*self.scaling)
                y = -int(self.ranges[i]*math.cos(self.current_angle)*self.scaling)
                if(abs(x) > + self.tolerance and abs(y) > self.tolerance):
                    xAndYArray.append([x + self.model.width/2, y + self.model.height/2])
            """if(i in noise_index_array):
                point = ((self.center[0] + x*10), (self.center[1] + y*10))
                pygame.draw.line(self.screen, pygame.Color('green'), self.center, point, 3)"""
            pygame.draw.circle(self.screen, color, (x + self.model.width/2,y + self.model.height/2), 2)
        #print self.heading_x
        rdp_array = dpa.rdp(xAndYArray, 3)
        print rdp_array
        line_array = []
        for i in range(len(rdp_array)):
            #print rdp_array[i]
            x = rdp_array[i][0]
            y = rdp_array[i][1]
            pygame.draw.circle(self.screen, pygame.Color('green'), (x,y), 2)
            for j in range(i, len(rdp_array)):
                x1 = rdp_array[i][0]
                y1 = rdp_array[i][1]
                x2 = rdp_array[j][0]
                y2 = rdp_array[j][1]
                length = dpa.distance((x1, y1),(x2, y2))
                if((x2 -x1) != 0 and length < (10*self.scaling)and length > (.5*self.scaling)):
                    slope = (y2-y1)/(x2-x1)
                    line_array.append(Line(x1, y1, x2, y2, self.get_angle((y2, y1), (x2, x1))))
        #for i in renage(len(slope_array)):

        for i, line in enumerate(line_array):
            #line.reset_dots()
            for i, point in enumerate(xAndYArray):
                distance_to_line = dpa.point_line_distance((point[0], point[1]), (line.x1, line.y1), (line.x2, line.y2))
                distance_to_start = dpa.distance((point[0], point[1]),(line.x1, line.y1))
                if (distance_to_line < .14*self.scaling and distance_to_line > 0):
                    #print distance_to_line
                    line.number_of_dots += 1
                    line.total_distance_to_start += distance_to_start
            #print line.number_of_dots
            line_distance = dpa.distance((line.x1, line.y1), (line.x2, line.y2))
            if(line.number_of_dots > 5 and line_distance/line.number_of_dots < 1.5):
            #(line.number_of_dots > 30 and line.total_distance_to_start/line.number_of_dots < 1.5*self.scaling):
                pygame.draw.line(self.screen, pygame.Color('red'), (line.x1, line.y1), (line.x2, line.y2), 2)
                line.is_good_line = True
        number_of_lines = 0
        total = 0
        for i, line, in enumerate(line_array):
            if line_array[0].is_good_line:
                angle = line_array[0].angle#*(line.total_distance_to_start/line.number_of_dots)
                if (angle < 0):
                    angle += math.pi*2#*(line.total_distance_to_start/line.number_of_dots)
                total += angle
                number_of_lines += 1#*(line.total_distance_to_start/line.number_of_dots)
                #print angle
        average = 0
        if (number_of_lines != 0):
            average = total/number_of_lines
        if (average != 0):
            point3 = ((self.center[0] + math.cos(average)*100), (self.center[1] + math.sin(average)*100))
            print point3
            pygame.draw.line(self.screen, pygame.Color('black'), self.center, point3, 5)
                #print line.angle*180.0/math.pi
        pygame.display.update()

    def get_angle(self, point1, point2):
        return math.atan2((point2[1]-point1[1]),(point2[0]-point1[0]))

class Line(object):
    def __init__(self, x1, y1, x2, y2, angle):
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2
        self.angle = angle
        self.total_distance_to_start = 0
        self.number_of_dots = 0
        self.is_good_line = True
    def add_dots(self):
        self.number_of_dots += 1
    def add_distance(self, distance):
        self.total_distance_to_start += distance
    def reset_dots(self):
        self.number_of_dots = 0
        self.total_distance_to_start = 0

class LidarModel(object):
    def __init__(self):
        self.width = 1000
        self.height = 1000
        self.center = (self.width/2, self.height/2)
        self.size = (self.width, self.height)

class DPAlgorithm():
    def distance(self,  a, b):
        return  math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

    def point_line_distance(self,  point, start, end):
        if (start == end):
            return self.distance(point, start)
        else:
            n = abs(
                (end[0] - start[0]) * (start[1] - point[1]) - (start[0] - point[0]) * (end[1] - start[1])
            )
            d = math.sqrt(
                (end[0] - start[0]) ** 2 + (end[1] - start[1]) ** 2
            )
            return n / d

    def rdp(self, points, epsilon):
        """
        Reduces a series of points to a simplified version that loses detail, but
        maintains the general shape of the series.
        """
        dmax = 0.0
        index = 0
        i=1
        for i in range(1, len(points) - 1):
            d = self.point_line_distance(points[i], points[0], points[-1])
            if d > dmax :
                index = i
                dmax = d

        if dmax >= epsilon :
            results = self.rdp(points[:index+1], epsilon)[:-1] + self.rdp(points[index:], epsilon)
        else:
            results = [points[0], points[-1]]
        return results


if __name__ == '__main__':
    print 'start'
    rospy.init_node('lidar_visulization')
    model = LidarModel();
    dpa= DPAlgorithm();
    screen = pygame.display.set_mode(model.size)
    laser = LaserVisualization(model, screen, dpa)
    rospy.spin()