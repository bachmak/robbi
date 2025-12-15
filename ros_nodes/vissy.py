from operator import itemgetter
from pygame.locals import *
from rclpy.node import Node
from std_msgs.msg import String
import numpy as np
import rclpy
import sys, math, pygame, time, serial

class Point3D:
    def __init__(self, x = 0, y = 0, z = 0):
        self.x, self.y, self.z = float(x), float(y), float(z)

    def rotateX(self, angle):
        """ Rotates the point around the X axis by the given angle in radians. """
        rad = angle
        cosa = math.cos(rad)
        sina = math.sin(rad)
        y = self.y * cosa - self.z * sina
        z = self.y * sina + self.z * cosa
        return Point3D(self.x, y, z)

    def rotateY(self, angle):
        """ Rotates the point around the Y axis by the given angle in radians. """
        rad = angle
        cosa = math.cos(rad)
        sina = math.sin(rad)
        z = self.z * cosa - self.x * sina
        x = self.z * sina + self.x * cosa
        return Point3D(x, self.y, z)

    def rotateZ(self, angle):
        """ Rotates the point around the Z axis by the given angle in radians. """
        rad = angle
        cosa = math.cos(rad)
        sina = math.sin(rad)
        x = self.x * cosa - self.y * sina
        y = self.x * sina + self.y * cosa
        return Point3D(x, y, self.z)

    def project(self, win_width, win_height, fov, viewer_distance):
        """ Transforms this 3D point to 2D using a perspective projection. """
        factor = fov / (viewer_distance + self.z)
        x = self.x * factor + win_width / 2
        y = -self.y * factor + win_height / 2
        return Point3D(x, y, self.z)

class Vissy(Node):
    def __init__(self, win_width = 1080, win_height = 640):
        super().__init__('vissy')

        pygame.init()
        #Set the window characteristics
        self.screen = pygame.display.set_mode((win_width, win_height))
        pygame.display.set_caption("Visualization of IMU Orientation")

        self.clock = pygame.time.Clock()
        #Create the eight vertices for a rectangular prism
        self.vertices = [
            Point3D(-1, -0.2,1),
            Point3D(1, -0.2,1),
            Point3D(1, -0.2,-1),
            Point3D(-1, -0.2,-1),
            Point3D(-1, 0.2,1),
            Point3D(1, 0.2,1),
            Point3D(1, 0.2,-1),
            Point3D(-1, 0.2,-1)
        ]

        self.yaw = None
        self.pitch = None
        self.roll = None
        
        # Define the vertices that compose each of the 6 faces. These numbers are
        # indices to the vertices list defined above.
        self.faces  = [(0,1,2,3),(1,5,6,2),(5,4,7,6),(4,0,3,7),(0,4,5,1),(3,2,6,7)]

        # Define colors for each face
        #                down                   up
        self.colors = [(0,0,255),(0,255,0),(0,0,255),(0,255,0),(255,255,0),(255,255,0)]

        self.declare_parameter('topic', '/immy')

        topic = self.get_parameter('topic').value
        
        self.create_subscription(String, topic, self.subscription_callback, 10)
        self.get_logger().info("Vissy started")
    
    def subscription_callback(self, msg):
        yaw, pitch, roll = line.split()
        self.yaw = float(yaw)
        self.pitch = float(pitch)
        self.roll = float(roll)

        run()

    def run(self):
        """ Main Loop """
        angle = 0
        nowP = time.time()
        while 1:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()
            #Limit the speed of the program
            #self.clock.tick(int(1/self.sampTime))
            self.screen.fill((0,32,0))
            
            x = self.ser.readline().decode().rstrip()
            imuData = [float(value) for value in x.split(' ')]
            #time.sleep(0.003)
            
            # It will hold transformed vertices.
            t = []

            for v in self.vertices:
                # Rotate the point around X axis, then around Y axis and around Z axis.
                angleX = math.radians(imuData[1])
                angleY = math.radians(imuData[0])
                angleZ = math.radians(imuData[2])
                r = v.rotateX(angleX)
                r1 = r.rotateY(-angleY)
                r2 = r1.rotateZ(angleZ)
                
                # Transform the point from 3D to 2D
                p = r2.project(self.screen.get_width(), self.screen.get_height(), 256, 4)
                # Put the point in the list of transformed vertices
                t.append(p)
            
            # Calculate the average Z values of each face.
            avg_z = []
            i = 0
            for f in self.faces:
                z = (t[f[0]].z + t[f[1]].z + t[f[2]].z + t[f[3]].z)/4
                avg_z.append([i,z])
                i = i + 1

            # Draw the faces using the Painter's algorithm:
            # Distant faces are drawn before the closer ones.
            for tmp in sorted(avg_z,key=itemgetter(1),reverse=True):
                face_index = tmp[0]
                f = self.faces[face_index]
                pointlist = [(t[f[0]].x, t[f[0]].y), (t[f[1]].x, t[f[1]].y),
                             (t[f[1]].x, t[f[1]].y), (t[f[2]].x, t[f[2]].y),
                             (t[f[2]].x, t[f[2]].y), (t[f[3]].x, t[f[3]].y),
                             (t[f[3]].x, t[f[3]].y), (t[f[0]].x, t[f[0]].y)]
                pygame.draw.polygon(self.screen,self.colors[face_index],pointlist)

            pygame.display.flip()
            
            elapsed = time.time() - nowP
            if (elapsed > 0.5):
                print("Yaw = ", imuData[0], " Pitch = ", imuData[1], " Roll = ", imuData[2])
                nowP = time.time()

    def destroy(self):
        super().destroy_node()

def main(args=None):
        rclpy.init(args=args)
        node = Vissy()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
