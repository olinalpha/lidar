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
        self.scaling = 40
        self.tolerance = 10
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
        rdp_array = []
        for i in range(len(self.ranges)):
            self.current_angle = i*self.angle_increment + self.angle_min
            if(not math.isnan(self.ranges[i])):
                x = -int(self.ranges[i]*math.sin(self.current_angle)*self.scaling)
                y = int(self.ranges[i]*math.cos(self.current_angle)*self.scaling)
                if(abs(x) > self.tolerance and abs(y) > self.tolerance):
                    rdp_array.append([x, y])
            pygame.draw.circle(self.screen, pygame.Color('blue'), (self.model.width/2 + x,self.model.height/2 - y), 2)
        #print self.heading_x
        rdp_array = dpa.rdp(rdp_array, .5)
        print rdp_array
        slope_array = []
        for i in range(len(rdp_array)):
            #print rdp_array[i]
            x = rdp_array[i][0]
            y = rdp_array[i][1]
            pygame.draw.circle(self.screen, pygame.Color('green'), (self.model.width/2 + x,self.model.height/2 - y), 2)
            for j in range(i, len(rdp_array)):
                x1 = rdp_array[i][0]
                y1 = rdp_array[i][1]
                x2 = rdp_array[j][0]
                y2 = rdp_array[j][1]
                length = dpa.distance((x1, y1),(x2, y2))
                if((x2 -x1) != 0 and length < (1*self.scaling)and length > (.5*self.scaling)):
                    slope = (y2-y1)/(x2-x1)
                    slope_array.append(((x1, y1), (x2, y2), self.get_angle((y2, y1), (x2, x1))))
        #for i in renage(len(slope_array)):

        for i in range(len(slope_array)):
            x1 = slope_array[i][0][0]
            y1 = slope_array[i][0][1]
            x2 = slope_array[i][1][0]
            y2 = slope_array[i][1][1]
            point1 = (x1 + self.model.width/2,self.model.height/2 - y1)
            point2 = (x2 + self.model.width/2, self.model.height/2 - y2)
            pygame.draw.line(self.screen, pygame.Color('red'), point1, point2, 2)
        pygame.display.update()
    def get_angle(self, point1, point2):
        return math.atan2((point2[1]-point1[1]),(point2[0]-point1[0]))


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