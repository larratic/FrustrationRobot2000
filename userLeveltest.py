#!/usr/bin/env python
"""
Created on Tue Dec  1 11:14:18 2015

@author: Lauren Milliken
"""

fps                 = 10        #at most  this many frames per second
back_image          = 'back2_800_600.bmp'   #must have this file in same dir.
display_cols        = 800
display_rows        = 600
wall_thickness      = 5         #thickness in pixels
wall_color          = 'black'
food_color          = 'purple'


color_of_nothing    = 'white'
sim_version         = 'FrustrationRobot2000'

r_image          = 'robo2.bmp'  #must have this file in same dir.
r_edge           = 51       #edge of square surrounding robot (in pixels)
r_init_azi       = 0        #azimuth, in degrees (up is 0)
r_init_x         = 30       #must be >= wall_thickness
r_init_y         = 30

r_init_fwd_speed = 5        #pixels per simulation cycle
r_init_spin_speed= 3        #degrees per simulation cycle
r_transparency   = 0       #0 is totally transp., 255 totally opaque
r_visual_range   = 50      #measured from robot center
r_visual_angle   = 30       #in degrees, must divide 90 exactly!
r_visual_granularity = 5    #must be < wall_thickness for walls to be detected correctly!


user_inputF       = 0.0
user_inputR       = 0.0         #amount of influence of user control the human has on the system

Distance        = 0.0         # total Distance travelled by robot
minObsDist      = 1000.0      # closest the robot came to an obstacle
totalUserInput   = 0.0           # time of button press

list_obstacles = []
blocked_nodes = []
debug_path  = []
path = None

#import everything
import os, pygame
from pygame.locals import *
import math
import random
import time
import numpy
import csv
from astar import PathManager
from HMM import HMM
from pomdp import *

#POMDP files
filename_env =  'POMDPu/userlevel2.pomdp'
filename_policy = 'POMDPu/out.policy'
pomdp = POMDP(filename_env, filename_policy, np.array([[0.15],[0.35],[0.15],[0.35]]))

prevOb = 4


main_dir = os.path.split(os.path.abspath(__file__))[0]
screen = pygame.display.set_mode((display_cols, display_rows))
list_traces = []
R = 0
numTrials = 5
o = np.zeros(numTrials)

class Trace():
    def __init__(self, from_rect, start_angle, stop_angle):
        self.rect       = from_rect
        self.start_angle= start_angle
        self.stop_angle = stop_angle

class Obstacle(pygame.Rect):       #for now just colored rectangles
    def __init__(self, x_topleft, y_topleft, width, height, color):
        self.x_topleft  = x_topleft
        self.y_topleft  = y_topleft
        
        self.width      = width
        self.height     = height
        self.color      = pygame.Color(color)
        self.rect       = pygame.Rect(x_topleft, y_topleft, width, height)
        
        self.r          = width
        
        self.x          = self.rect.center[0]
        self.y          = self.rect.center[1]
        
        self.force      = 0.0
        self.theta      = 0.0
        self.detected   = False
        self.dist       = 1000.0
        self.xCollide   = 0.0
        self.yCollide   = 0.0
        
        
    def nodes(self):
        nodes = []
        buff  = 15
        
        for y in range(self.y_topleft -buff, self.y_topleft + self.height + buff):
            for x in range(self.x_topleft-buff, self.x_topleft + self.width + buff ):
                nodes.append((int(x), int(y)))
        return nodes

''' Changes alpha for surfaces with per-pixel alpha; only for small surfaces!
    Sets alpha for WHITE pixels to new_alpha.
    The alpha value is an integer from 0 to 255, 0 is fully transparent and
    255 is fully opaque. '''
def change_alpha_for_white(surface,new_alpha):
    size = surface.get_size()
    if size[0]>300 or size[1]>300:
        print 'change_alpha_for_white-> size = ', size, ' IMAGE TOO LARGE!'
        return surface
    for y in xrange(size[1]):
	for x in xrange(size[0]):
	    r,g,b,a = surface.get_at((x,y))
	    if r==255 and g==255 and b==255:
                surface.set_at((x,y),(r,g,b,new_alpha))
    return surface

''' Changes alpha for surfaces with per-pixel alpha; only for small surfaces!
    Sets alpha for pixels with alpha == 0 to new_alpha. It is needed b/c
    transform.smoothscale pads image with alpha=0. '''
def change_alpha_for_alpha(surface,new_alpha):
    size = surface.get_size()
    for y in xrange(size[1]):
	for x in xrange(size[0]):
	    r,g,b,a = surface.get_at((x,y))
	    if a<200:
                surface.set_at((x,y),(r,g,b,new_alpha))
    return surface

class Goal(pygame.sprite.Sprite):
    def __init__(self, image):
        pygame.sprite.Sprite.__init__(self) #call Sprite initializer
        #Sprites must have an image and a rectangle
        self.image          = image
        self.image_original = self.image    #unchanging copy, for rotations
        self.rect           = image.get_rect()

        self.x = random.randint(60,display_cols-60)
        self.y = random.randint(60,display_rows-60)
        self.rect.center = (self.x, self.y) # set starting position
        
        if self.inObstacle:
            self.getNew()
        
    def getNew(self):
        self.x = random.randint(60,display_cols-60)
        self.y = random.randint(60,display_rows-60)
        self.rect.center = (self.x, self.y) # set starting position
        
        while self.inObstacle():
             self.x = random.randint(60,display_cols-60)
             self.y = random.randint(60,display_rows-60)
             self.rect.center = (self.x, self.y) # set starting position
                
        
    def inObstacle(self):
        for ob in list_rect_obstacles: 
            if ob.colliderect(self.rect):
                return True
        return False
    


#Create list of obstacles (walls+others)
#First 2 args are x and y of top-left corner, next two width and height, next color

w01 = Obstacle(0,0,display_cols,wall_thickness, wall_color)                          #top wall
list_obstacles.append(w01)
w02 = Obstacle(display_cols-wall_thickness,0,wall_thickness,display_rows,wall_color) #right wall
list_obstacles.append(w02)
w03 = Obstacle(0,display_rows-wall_thickness,display_cols,wall_thickness,wall_color) #bottom wall
list_obstacles.append(w03)
w04 = Obstacle(0,0,wall_thickness,display_rows, wall_color)                          #left wall
list_obstacles.append(w04)
w05 = Obstacle(display_cols/2,display_rows/2,wall_thickness,display_rows/2,wall_color)
list_obstacles.append(w05)
w06 = Obstacle(display_cols/6,display_rows/2,display_rows/4,wall_thickness,wall_color)
list_obstacles.append(w06)


### create random obastacles
#obs = random.randint(10,20)
#for i in range(1,obs):
#    x = random.randint(100,display_cols)
#    y = random.randint(100,display_rows)
# 
#    r = random.randint(20,50)
#    print x,y,r,r
#    obstacle = Obstacle(x,y,r,r,food_color)
#    list_obstacles.append(obstacle)

# read obstalces from the file
with open('obstacles.csv', 'rb') as csvfile:
  obs = csv.reader(csvfile, delimiter=',', quotechar='|')
  for row in obs:
    print row
    x = int(row[0])
    y = int(row[1])
    a = int(row[2])
    b = int(row[3])
    print x, y, a ,b
    obstacle = Obstacle(x,y,a,b,food_color)
    list_obstacles.append(obstacle)



#for collision-checking
#so for speed a stripped-down list of rectangles is built:
list_rect_obstacles = []
for ob in list_obstacles:
    list_rect_obstacles.append(ob.rect)
    
    for node in ob.nodes():
        blocked_nodes.append(node)
        
# create manager to handle all pathing requests
path_manager = PathManager(obstacles=blocked_nodes)



class Robot(pygame.sprite.Sprite):
    def __init__(self, image, x, y, azimuth, fwd_speed, spin_speed,\
                 visual_range, visual_angle, goal,pomdp):
        pygame.sprite.Sprite.__init__(self) #call Sprite initializer
        #Sprites must have an image and a rectangle
        self.image          = image
        self.image_original = self.image    #unchanging copy, for rotations
        self.rect           = image.get_rect()
        self.rect_original  = self.rect     #unchanging copy, for rotations
        self.x = x # created because self.rect.center does not hold
        self.y = y # decimal values but these do
        self.rect.center = (self.x, self.y) # set starting position
        self.fwd_speed      = fwd_speed
        self.spin_speed     = spin_speed
        self.azi            = azimuth       #in degrees
        self.collided       = False
        self.opmode         = 0             #0=tele, 1=(random)walk 2=auto
        self.spin_angle_left= 0             #relative angle left to spin
        
        self._d = {'w':-1, 's':1, 'a':1, 'd':-1, 'N':0} 
        self.speed          = 0.0
        self.rotspeed       = 5.0
        self.bearing        = 0.0
        self.direction       = 'N'
        self.maxAccel       = 10.0
        
        self.goal           = goal 
        self.spin_angle_left = 0.0
        self.found_goal     = 0
        
        self.path           = None
        self.obsWarn        = False
        self.frontDist      = 0.0
        
        self.userInf        = 0.0
        self.pomdp      = pomdp
        self.action = pomdp.get_best_action()[0]
        self.numCollision = 0
        
        self.currentSpeed = -1.0
        self.currentAngle = 48.0
        self.prevSpinAngle =0.0

        
        #these are the parameters of the range-sensing system
        self.visual_range   = visual_range
        self.visual_angle   = visual_angle
        self.nr_sensors     = 2*90/self.visual_angle+1
        self.retina         = list([self.visual_range, pygame.Color(color_of_nothing)]\
                                   for i in range(self.nr_sensors))

    def printRetina(self):
        """Prints the content of the retina list"""
        for s in self.retina:
            if (s[0] == self.visual_range): #this really means >=, since sense() func. caps distances
                                            #to visual_range
                print '>'+str(self.visual_range)
            else:       #obstacle detected
                print s
        print '\n'

    def update(self):
        """All sprites have an update() method. This function is
        typically called once per frame. IMPORTANT: All actions in
        here execute AFTER the ones called directly in the event loop.
        """
        if   (self.opmode == 0): self.mode_0_tele()     #teleop mode    
        elif (self.opmode == 1): self.mode_1_auto()     #autonomous
        elif (self.opmode == 2): self.mode_2_assist()   #assist 
        elif (self.opmode == 3): self.mode_3_pomdp()   #assist 
        elif (self.opmode == 4): self.mode_4_hmm()   #assist 
        else:
            print 'ERROR! Undefined operation mode!'
            
        
            
    def mode_0_tele(self):
        self.sense()   
        global R
        if self.collided:
            o[R] = 1.
            print 'update-->self.opmode==0 THAT HURT!'
            self.collided = False
            self.speed = 0.0

  
        dtheta = 0       
                
                
        if self.direction == 'w':
            if self.speed >-self.maxAccel:
                self.speed -= 1
        elif self.direction == 's':
            if self.speed < self.maxAccel:
                self.speed += 1
        elif self.direction == 'd' or self.direction == 'a':
            dtheta = self.rotspeed*self._d[self.direction]
            
          
        dx = self.speed*math.sin(self.bearing*math.pi/180)
        dy = self.speed*math.cos(self.bearing*math.pi/180)
        
        drift = random.choice([0,1],1)
        cx = drift*2*self.currentSpeed*math.sin(self.currentAngle*math.pi/180)
        cy = drift*2*self.currentSpeed*math.cos(self.currentAngle*math.pi/180)

        self.move(dx+cx,dy+cy,dtheta)


    
    
    ########A*########

    def mode_1_auto(self):
        """A*"""
        self.sense()
        dtheta = 0.0
        global user_inputF,user_inputR
      
        UF =user_inputF
        UR = user_inputR
        
        if self.collided:
            print 'update-->self.opmode==0 THAT HURT!'
            self.collided = False
            self.speed = 0.0

        dtheta1 = 0.0
        if self.direction == 'w':
            if self.speed >-self.maxAccel:
                self.speed -= 2*UF
        elif self.direction == 's':

            if self.speed<self.maxAccel:
                self.speed += 2*UF
        elif self.direction == 'd' or self.direction == 'a':
            dtheta1 = UR*self.rotspeed*self._d[self.direction]
            print dtheta1

        # generate A* path
        self.path = path_manager.generate_path((self.x,self.y),(self.goal.x,self.goal.y))      
        debug_path = self.path.get_path()

        if len(debug_path) > 1:
            current = debug_path[1]
            dx = current[0] - self.x
            dy = current[1] - self.y


            angle =  int((math.atan2(dx,dy))*180/math.pi)
            angle = angle+180
            if angle  >= 360:         #keep theta between -360..360
                angle -= 360
            if angle <= -360:
                angle += 360
                
        else:
            angle = 0.0
            
        self.spin_angle_left = angle - self.bearing
        
        
        if self.spin_angle_left >=180.0:
            self.spin_angle_left = self.spin_angle_left - 360.0
        if self.spin_angle_left <=-180.0:
            self.spin_anlge_left = self.spin_angle_left + 360.0

        if self.spin_angle_left != 0:     #must finish SPIN
            if math.fabs(self.spin_angle_left) <= self.rotspeed:
                dtheta = int((1-UR)*(sign(self.spin_angle_left))+dtheta1)
            else:
                dtheta = int((1-UR)*(sign(self.spin_angle_left)*(self.rotspeed))+dtheta1)
                
            if self.spin_angle_left > 45 or self.spin_angle_left < -45:
                if self.speed < 0 :
                    self.speed += 1*(1-UF)
                elif self.speed > 0:
                    self.speed -= 1*(1-UF)
                

        if self.obsWarn :
            if self.spin_angle_left != 0:
                if self.speed < 0:
                    if self.speed > -1:
                        self.speed = 0.0
                    else:
                        self.speed += 1.0
                

 
            elif self.speed < -4 :
                self.speed += 1
                
            elif self.speed > -4:
                self.speed -= 1
                
            elif self.frontDist < 35 :
                if self.speed < 2 :
                    self.speed += 2
                else :
                    self.speed = 1
                    
            else:
                self.speed -= 1
                       
        else :
            if self.speed >-self.maxAccel :
                self.speed -= 1*(1-UF)
                    
        print 'speed: ' + str(self.speed)
        moveX = self.speed*math.sin(self.bearing*math.pi/180)
        moveY = self.speed*math.cos(self.bearing*math.pi/180)
        self.move(moveX,moveY,dtheta)            

            

            
    def mode_2_assist(self):
##### potential field method
        self.sense()   
        dtheta = 0 
        global user_inputF,user_inputR
        if user_inputF > 1:
            user_inputF = 1
            
        UF =user_inputF
        UR = user_inputR
        dtheta1 = 0.0

        if self.collided:
          print 'update-->self.opmode==0 THAT HURT!'
          self.collided = False
          self.speed = 0.0
          self.numCollision  += 1
        
        if self.direction == 'w':
          if self.speed >-self.maxAccel:
              self.speed -= 2*UF
        elif self.direction == 's':
            if self.speed < self.maxAccel:
                self.speed += 2*UF
        elif self.direction == 'd' or self.direction == 'a':
            dtheta1 =  UF*self.rotspeed*self._d[self.direction]
            print dtheta1

            
        
        dx,dy = self.potentialField()
        if math.isnan(dx) or math.isnan(dy):
          angle = self.bearing
        else:
          angle = int(math.atan2(dx,dy)* 180/math.pi) + 180
        if angle  >= 360:         #keep theta between -360..360
            angle -= 360
        if angle <= -360:
            angle += 360

        self.spin_angle_left = angle - self.bearing
        dtheta = dtheta+dtheta1
         
        if self.spin_angle_left != 0:     #must finish SPIN
            if math.fabs(self.spin_angle_left) <= self.rotspeed:
                dtheta = int((1-UR)*sign(self.spin_angle_left))+dtheta1
            else:
                dtheta = int((1-UR)*sign(self.spin_angle_left)*(1))+dtheta1
                
            if self.spin_angle_left > 45 or self.spin_angle_left < -45:
                if self.speed < 0 :
                    self.speed += 1*(1-UF)
                elif self.speed > 0:
                    self.speed -= 1*(1-UF)
       
        if self.obsWarn :
            if self.spin_angle_left != 0:
                if self.speed < 0 :
                    self.speed += 2
                    
            elif self.speed < -2 :
                 self.speed += 2*(1-UF)
#            else :
#              print 'hey'
#              self.speed = -2*(1-U)
#               
#                if self.frontDist < 30:
#                    if self.speed < 2 :
#                        self.speed += 2
#                    else :
#                        self.speed = 2
                        
                         
#        else :
#            if self.spin_angle_left > 45:
#                if self.speed < -2 :
#                    self.speed += 2 *(1-U)
#
#            else:
#
#                if self.speed >-self.maxAccel :
#                    self.speed -= 2*(1-U)
                    
                    
        print 'speed: ' + str(self.speed)
        print dtheta
        movex = self.speed*math.sin(self.bearing*math.pi/180)
        movey = self.speed*math.cos(self.bearing*math.pi/180)
        self.move(movex,movey,dtheta)
        
    def mode_3_pomdp(self):
      global pomdp,prevOb
      if self.bearing < 0:
        self.bearing +=360

      if self.action == 0:
        self.moreHelp()
      elif self.action == 1:
        self.mostHelp()
      elif self.action == 2:
        self.leastHelp()
      elif self.action == 3:
        self.someHelp()
      else:
        self.sense()   
        if self.collided:
            print 'update-->self.opmode==0 THAT HURT!'
            self.collided = False
            self.speed = 0.0
            self.numCollision  += 1
     
        dtheta = 0       
  
  
        if self.direction == 'w':
            if self.speed >-self.maxAccel:
                self.speed -= 1
        elif self.direction == 's':
            if self.speed < self.maxAccel:
                self.speed += 1
        elif self.direction == 'd' or self.direction == 'a':
            dtheta = self.rotspeed*self._d[self.direction]
      
          
        dx = self.speed*math.sin(self.bearing*math.pi/180)
        dy = self.speed*math.cos(self.bearing*math.pi/180)
  
        self.move(dx,dy,dtheta)
        
        
    def mostHelp(self):
      self.sense()   
      dtheta = 0 
      self.maxAccel = 10.0
      global user_inputF,user_inputR
      if user_inputF > 1:
          user_inputF = 1
      if user_inputR > 1:
          user_inputR = 1
          
      UF =user_inputF
      UR = user_inputR
      dtheta1 = 0

      if self.collided:
              print 'update-->self.opmode==0 THAT HURT!'
              self.collided = False
              self.speed = 0.0
              self.numCollision  += 1
      
      if self.direction == 'w':
          if self.speed >-self.maxAccel:
              self.speed -= 2*UF
      elif self.direction == 's':
          if self.speed < self.maxAccel:
              self.speed += 2*UF
      elif self.direction == 'd' or self.direction == 'a':
          dtheta1 =  UR*self.rotspeed*self._d[self.direction]
 
      dx,dy = self.potentialField()
      
      if math.isnan(dx) or math.isnan(dy):
        angle = self.bearing
      else:
        angle = int(math.atan2(dx,dy)* 180/math.pi) + 180
      if angle  >= 360:         #keep theta between -360..360
          angle -= 360
      if angle <= -360:
          angle += 360
      self.spin_angle_left = (int(angle - self.bearing))
      if(self.spin_angle_left) == (int(self.prevSpinAngle - 180)-5) or (self.spin_angle_left) == (int(self.prevSpinAngle + 180)+5) or (self.spin_angle_left) == (int(self.prevSpinAngle + 180)-5) or (self.spin_angle_left) == (int(self.prevSpinAngle - 180)+5):     
        self.spin_angle_left = 0

      else:
        self.prevSpinAngle = self.spin_angle_left

      
        
      if self.spin_angle_left  > 180:
          self.spin_angle_left -= 360
      if self.spin_angle_left < -180:
          self.spin_angle_left +=360
#      if math.fabs(self.spin_angle_left) == 180:
#        self.spin_angle_left = 0
      dtheta = int(dtheta+dtheta1)
      if self.spin_angle_left != 0:     #must finish SPIN
          if math.fabs(self.spin_angle_left) <= self.rotspeed:
            dtheta = int((1-UF*0.1)*(1-UR)*sign(self.spin_angle_left)+dtheta1)
          else:
            dtheta = int((1-UF*0.1)*(1-UR)*sign(self.spin_angle_left)*(self.rotspeed)+dtheta1)
          if self.spin_angle_left > 45 or self.spin_angle_left < -45:
              if self.speed < 0 :
                  self.speed += 1*(1-UF)
              elif self.speed > 0:
                  self.speed -= 1*(1-UF)
     
      if self.obsWarn :
          if self.spin_angle_left != 0:
              if self.speed < 0 :
                  self.speed += .95*abs(self.speed)
                  
          elif self.speed < -2 :
               self.speed += .95*abs(self.speed)


                  
      
      movex = self.speed*math.sin(self.bearing*math.pi/180)
      movey = self.speed*math.cos(self.bearing*math.pi/180)
      self.move(movex,movey,dtheta)

        
    def moreHelp(self):
      self.sense()   
      dtheta = 0 
      self.maxAccel = 15.0
      
      global user_inputF,user_inputR
      if user_inputF > 1:
          user_inputF = 1
      if user_inputR > 1:
          user_inputR = 1
          
      UF =user_inputF
      UR = user_inputR
      dtheta1 = 0

      if self.collided:
              print 'update-->self.opmode==0 THAT HURT!'
              self.collided = False
              self.speed = 0.0
              self.numCollision  += 1
      
      if self.direction == 'w':
          if self.speed >-self.maxAccel:
              self.speed -= 2*UF
      elif self.direction == 's':
          if self.speed < self.maxAccel:
              self.speed += 2*UF
      elif self.direction == 'd' or self.direction == 'a':
          dtheta1 =  UR*self.rotspeed*self._d[self.direction]
 
      dx,dy = self.potentialField()
      
      if math.isnan(dx) or math.isnan(dy):
        angle = self.bearing
      else:
        angle = int(math.atan2(dx,dy)* 180/math.pi) + 180
      if angle  >= 360:         #keep theta between -360..360
          angle -= 360
      if angle <= -360:
          angle += 360

      self.spin_angle_left = (int(angle - self.bearing))

      if(self.spin_angle_left) == (int(self.prevSpinAngle - 180)-5) or (self.spin_angle_left) == (int(self.prevSpinAngle + 180)+5) or (self.spin_angle_left) == (int(self.prevSpinAngle + 180)-5) or (self.spin_angle_left) == (int(self.prevSpinAngle - 180)+5):     
        self.spin_angle_left = 0

      else:
        self.prevSpinAngle = self.spin_angle_left

      
        
      if self.spin_angle_left  > 180:
          self.spin_angle_left -= 360
      if self.spin_angle_left < -180:
          self.spin_angle_left +=360
#      if math.fabs(self.spin_angle_left) == 180:
#        self.spin_angle_left = 0
      dtheta = int(dtheta+dtheta1)

       
      if self.spin_angle_left != 0:     #must finish SPIN
          if math.fabs(self.spin_angle_left) <= self.rotspeed:
            dtheta = int((1-UF*0.1)*(1-UR)*sign(self.spin_angle_left)+dtheta1)
          else:
            dtheta = int((1-UF*0.1)*(1-UR)*sign(self.spin_angle_left)*(self.rotspeed)+dtheta1)
          if self.spin_angle_left > 45 or self.spin_angle_left < -45:
              if self.speed < 0 :
                  self.speed += 1*(1-UF)
              elif self.speed > 0:
                  self.speed -= 1*(1-UF)
     
      if self.obsWarn :
          if self.spin_angle_left != 0:
              if self.speed < 0 :
                  self.speed += .95*abs(self.speed)
                  
          elif self.speed < -2 :
               self.speed += .95*abs(self.speed)


                  
      
      movex = self.speed*math.sin(self.bearing*math.pi/180)
      movey = self.speed*math.cos(self.bearing*math.pi/180)
      self.move(movex,movey,dtheta)
      
    def someHelp(self):
      self.sense()   
      self.maxAccel = 15.0
      if self.collided:
          print 'update-->self.opmode==0 THAT HURT!'
          self.collided = False
          self.speed = 0.0
          self.numCollision  += 1
   
      dtheta = 0       


      if self.direction == 'w':
          if self.speed >-self.maxAccel:
              self.speed -= 2
      elif self.direction == 's':
          if self.speed < self.maxAccel:
              self.speed += 2
      elif self.direction == 'd' or self.direction == 'a':
          dtheta = self.rotspeed*self._d[self.direction]
      
      if self.obsWarn :
        if self.spin_angle_left != 0:
            if self.speed < 0 :
                self.speed += .95*abs(self.speed)
                
        elif self.speed < -2 :
             self.speed += .95*abs(self.speed)
        
      dx = self.speed*math.sin(self.bearing*math.pi/180)
      dy = self.speed*math.cos(self.bearing*math.pi/180)
      
      

      self.move(dx,dy,dtheta)
        
    def leastHelp(self):
      self.sense()   
      if self.collided:
          print 'update-->self.opmode==0 THAT HURT!'
          self.collided = False
          self.speed = 0.0
          self.numCollision  += 1
   
      dtheta = 0       


      if self.direction == 'w':
          if self.speed >-self.maxAccel:
              self.speed -= 2
      elif self.direction == 's':
          if self.speed < self.maxAccel:
              self.speed += 2
      elif self.direction == 'd' or self.direction == 'a':
          dtheta = self.rotspeed*self._d[self.direction]
    
        
      dx = self.speed*math.sin(self.bearing*math.pi/180)
      dy = self.speed*math.cos(self.bearing*math.pi/180)

      self.move(dx,dy,dtheta)

      
    def mode_4_hmm(self):
      self.sense()   
      global R
      if self.collided:
          o[R] = 1.
          print 'update-->self.opmode==0 THAT HURT!'
          self.collided = False
          self.speed = 0.0


      dtheta = 0       
              
              
      if self.direction == 'w':
          if self.speed >-self.maxAccel:
              self.speed -= 1
      elif self.direction == 's':
          if self.speed < self.maxAccel:
              self.speed += 1
      elif self.direction == 'd' or self.direction == 'a':
          dtheta = self.rotspeed*self._d[self.direction]
          
        
      dx = self.speed*math.sin(self.bearing*math.pi/180)
      dy = self.speed*math.cos(self.bearing*math.pi/180)

      self.move(dx,dy,dtheta)
    
      
      

        
        
    # show the path for A*        
    def showGoalPath(self,target_surf):
        if self.path:
            for node in self.path.get_closed_nodes():
                pygame.draw.circle(target_surf, (255, 0, 0), node, 1, 1)
            for node in self.path.get_open_nodes():
                pygame.draw.circle(target_surf, (255, 255, 0), node, 1, 1)
        
        
            
        
    def move(self,dx,dy,dtheta):
        self.spin(dtheta)
        previous_rect = self.rect           #re1member in case undo is necessary       
        self.rect = self.rect.move(dx,dy)
        if self.rect.collidelist(list_rect_obstacles) != -1:#if collision exists
            print 'mode  -->I collided with wall(s)',\
                  self.rect.collidelistall(list_rect_obstacles)
            self.rect = previous_rect                   #undo the move
            self.collided = True
            self.speed = 0.0
        else:                   #if there was no collision
            self.x,self.y = self.rect.center
            global Distance
            Distance += math.sqrt(dx*dx + dy*dy)
            
    #potential field generation for obstacle avoidance      
    def potentialField(self):
        RadiusOfInfluence   = 50.0
        Kobjs               = 1.0
        Kgoal               = 0
        self.infl_obst = []
        for ob in list_obstacles:
          #ob.dist = math.sqrt(math.pow(self.x - ob.x,2)+math.pow(self.y - ob.y,2))
          if ob.detected and ob.dist <= RadiusOfInfluence: 
              self.infl_obst.append(ob)
        Distance = np.zeros((1,len(self.infl_obst)))
        V        = np.zeros((2,len(self.infl_obst)))
        count = 0
        if self.infl_obst != []:
            for ob in self.infl_obst:
                Distance[0][count] = ob.dist #- ob.r
                V[0][count]     = (ob.xCollide - self.x) 
                V[1][count]     = (ob.yCollide - self.y)
                count +=1
            
            rho = np.repeat(Distance,2,0)
            DrhoDx = -V/rho
            F = (1/rho - 1/RadiusOfInfluence)* 1/np.square(rho)*DrhoDx
            FObjects = Kobjs*numpy.sum(F) 

            
        else :
            FObjects = np.array([0,0])
            
        #GoalError = np.array([self.speed*math.sin(self.bearing*math.pi/180)-self.x,self.speed*math.cos(self.bearing*math.pi/180)-self.y])    
        GoalError = np.array([self.goal.x - self.x, self.goal.y - self.y])
        normGoal = (math.sqrt(math.pow(GoalError[0],2)+math.pow(GoalError[1],2)))
        Fgoal = Kgoal * GoalError/normGoal
        Ftotal = FObjects + Fgoal
        normTotal = (math.sqrt(math.pow(Ftotal[0],2)+math.pow(Ftotal[1],2)))
        Magnitude = numpy.min((1,normTotal))
        Ftotal = Ftotal/normTotal * Magnitude

        return Ftotal
            
            
      


    def spin(self,dtheta):
        self.bearing += dtheta
        if self.bearing >= 360:         #keep theta between -360..360
            self.bearing -= 360
        if self.bearing <= -360:
            self.bearing += 360
        original_rect = self.image_original.get_rect()
        rotated_image = pygame.transform.rotate(self.image_original, self.bearing)
        rotated_rect  = original_rect.copy()
        rotated_rect.center = rotated_image.get_rect().center
        self.image = rotated_image.subsurface(rotated_rect).copy()
        self.image = change_alpha_for_alpha(self.image, r_transparency)

    
    
    #this function's job is to place in self.retina the range sensed by each sensor
    def sense(self):
        self.obsWarn = False
        self.obsDist = 500.0
        n = (self.nr_sensors - 1)/2     #the "natural" sensor range is -n to +n
        granu = r_visual_granularity    #must be at least as large as the wall thickness!!
        for i in range(-n,n+1):         #sense with each of the 2n+1 range sensors
            ang = (self.bearing - i*self.visual_angle)*math.pi/180
            for distance in range(granu, self.visual_range+granu, granu):
                x = self.rect.center[0]-distance*math.sin(ang)  #endpoint coordinates
                y = self.rect.center[1]-distance*math.cos(ang)
                nr_collisions = 0
                count = -1          #needed to coordinate the two lists, to extract color after loop
                for ob in list_obstacles:  #use the stripped-down list of rectangles for speed
                    count = count + 1
                    if i == 0:
                        minDist = distance 
                        if minDist < self.obsDist:
                            self.obsDist = minDist       
                    if ob.rect.collidepoint(x,y):
                      nr_collisions = 1
                      ob.detected = True
                      if distance < ob.dist:
                        
 
                        ob.xCollide = x
                        ob.yCollide = y
                        ob.dist = distance


                if nr_collisions:   #non-zero collision
                    break           #breaks out of distance loop
                    
                dist = math.sqrt(math.pow(self.x -self.goal.x,2)+math.pow(self.y -self.goal.y,2))
                if dist < self.visual_range:
                    self.found_goal = 1
                else:
                    self.found_goal = 0
                    
            #distance now has the min. between the visual range and the first collision
            self.retina[i+n][0] = distance
            if nr_collisions:       #nr_collisions is 1 if a collision has occurred
                if i == 0:
                    self.obsWarn = True
                    
                self.retina[i+n][1] = list_obstacles[count].color #color comes form the larger list
            else:
                self.retina[i+n][1] = pygame.Color(color_of_nothing)
            self.frontDist = self.retina[n][0]

          
    def draw_rays(self, target_surf):
        n = (self.nr_sensors - 1)/2 #the "natural" sensor range -n to +n
        for i in range(-n,n+1):     #draw the 2n+1 rays of the range sensors
            ang = (self.bearing - i*self.visual_angle)*math.pi/180
            x = self.rect.center[0]-self.retina[i+n][0]*math.sin(ang)
            y = self.rect.center[1]-self.retina[i+n][0]*math.cos(ang)
            pygame.draw.line(target_surf, (0,0,0), self.rect.center, (x,y))
########end of Robot class########

def load_image(name):
    path = os.path.join(main_dir, name)
    temp_image = pygame.image.load(path).convert_alpha()  #need this if using ppalpha
    return change_alpha_for_white(temp_image, r_transparency)  
    


###########################################
###########################################
def main():
    global leave_trace, list_traces,pomdp
    if r_visual_granularity > wall_thickness:
        print 'PARAMETER ERROR: r_visual_granularity exceeds wall_thickness!'
        print 'This can cause wall detection errors!'
    if r_init_x<wall_thickness or r_init_y<wall_thickness:
        print 'PARAMETER ERROR: starting position overlaps wall!'
        print 'Check r_init_x|y_topleft and wall_thickness'
    pygame.init()           #also calls display.init()   
    startTime = time.time()
    caption = (sim_version + ' \tmode: teleoperation  ' )
    pygame.display.set_caption(caption+ str(startTime))
    r_sprite = load_image(r_image)
    g_sprite = load_image('goal.bmp')
    background  = load_image(back_image)


    #prepare simulation objects
    clock = pygame.time.Clock()
    screen.blit(background, (0, 0))
    goal = Goal(g_sprite)
    r = Robot(r_sprite, r_init_x, r_init_y,r_init_azi, r_init_fwd_speed,\
              r_init_spin_speed, r_visual_range, r_visual_angle,goal,pomdp)

    robotSprite = pygame.sprite.Group(r)
    goalSprite = pygame.sprite.Group(goal)

    #display the environment once, right before event loop
    
    count = -1
    for ob in list_obstacles:
        count = count + 1
        s = pygame.display.get_surface()
        s.fill(ob.color, list_rect_obstacles[count])
    r.draw_rays(screen) 
    r.showGoalPath(screen)
    pygame.display.flip() 
    
    #recording time
    going = True
    time_down = 0.0
    time_elapsed = 0.0
    T1 = -1
    T2 = -1
    
    #HMM initalize  
    hmm = HMM()
    hmm.pi = np.array([0.5, 0.5])
    hmm.A = np.array([[0.5, 0.5],[0.5, 0.5]])
    hmm.B = np.array([[0.3, 0.7],[0.99, 0.01]])
    
    
    while going:
        clock.tick(fps)      #at most that many fps
        time_elapsed = 0.0
        #Event loop################################
        global user_inputF,user_inputR, totalUserInput, o,R,numTrials
        for event in pygame.event.get():
            if event == QUIT:
                going = False
            elif event.type == KEYDOWN:
                if event.key == K_ESCAPE:
                    going = False
                 
                elif event.key == K_w:
                    T1 = 1
                    r.direction = 'w'
                    time_down = pygame.time.get_ticks()
                elif event.key == K_d:
                    T2 = 1
                    r.direction = 'd'
                    time_down = pygame.time.get_ticks()
                elif event.key == K_a:
                    T2 = 1
                    r.direction = 'a'
                    time_down = pygame.time.get_ticks()
                elif event.key == K_s:
                    T1 = 1
                    r.direction = 's'
                    time_down = pygame.time.get_ticks()

              
                if event.key == K_SPACE:
                    r.opmode = 0            #teleop mode
                    caption = sim_version + ' \tmode: teleoperation  '
                if event.key == K_1:
                    r.opmode = 1            #autonomous navigation mode
                    caption = (sim_version + ' \tmode: autonomous  ')
                if event.key == K_2:
                    r.opmode = 2            #autonomous navigation mode
                    caption = (sim_version + ' \tmode: assist  ')
                if event.key == K_3:
                    r.opmode = 3            #autonomous navigation mode
                    caption = (sim_version + ' \tmode: pomdp  ')
                if event.key == K_4:
                    r.opmode = 4           #autonomous navigation mode
                    caption = (sim_version + ' \tmode: hmm  ')
                    

            elif event.type == KEYUP:
                
                if event.key == K_w:
                    time_elapsed = pygame.time.get_ticks() - time_down
                    T1 = -1
                    r.direction = 'N'
                elif event.key == K_d:
                    time_elapsed = pygame.time.get_ticks() - time_down
                    T2 = -1
                    r.direction = 'N'
                elif event.key == K_a:
                    time_elapsed = pygame.time.get_ticks() - time_down
                    T2 = -1
                    r.direction = 'N'
                elif event.key == K_s:
                    time_elapsed = pygame.time.get_ticks() - time_down
                    T1 = -1
                    r.direction = 'N'
                    
        totalUserInput += time_elapsed/1000.0
            

        
        user_inputF += T1*0.1

        if user_inputF > 1.0:
            user_inputF = 1.0
        elif user_inputF < 0.0:
            user_inputF = 0.0
            
        user_inputR += T2*0.1

        if user_inputR > 1.0:
            user_inputR = 1.0
        elif user_inputR < 0.0:
            user_inputR = 0.0

                
        pygame.display.set_caption(caption)
         
        #slow down when stopped moving               
        if r.speed > 0.0:
          r.speed -= 0.5
          if r.speed < 0.5:
            r.speed = 0.0
        elif r.speed < 0.0:
          r.speed += 0.5
          if r.speed >0.5:
            r.speed = 0.0
        
        
            
        
            
        # Find if goal reached 
        if pygame.sprite.spritecollide(r, goalSprite, False) != []:    
            print 'You made it to the goal'
            print r.numCollision
            R=R+1

            if len(list_obstacles) > 30:
              mapLevel = 3
            else:
              mapLevel = 0
              
            if r.numCollision > 5:
              observation = 0+mapLevel
            elif r.numCollision > 0:
              observation = 1+mapLevel
            else:
              observation = 2+mapLevel
            r.pomdp.update_belief(r.action,observation)
            r.action = r.pomdp.get_best_action()[0] 
            print 'Most likely state: ' + r.pomdp.pomdpenv.states[np.argmax(r.pomdp.belief)]
            print 'Observation: ' +  pomdp.pomdpenv.observations[observation]
            print 'Action: ' + pomdp.pomdpenv.actions[r.action]
            prevOb = observation
            print r.pomdp.belief
            
            r.numCollision = 0

#            if R == numTrials:
#                hmm.train(o,0.0001,graphics=False)
#                print 'probabilities\n',hmm.pi
#                print 'state transition probabililities\n',hmm.A
#                print 'observation probabililities\n',hmm.B
#                R = 0
#                o = np.zeros(numTrials)
            goal.getNew()
            startTime = time.time()

  
        robotSprite.update()
        goalSprite.update()
        screen.blit(background, (0, 0))  #redraws the entire bkgrnd.
        #screen.fill((255,255,255)) # white background
        #screen.blit(red_block, (100,100))
        count = -1
        for ob in list_obstacles:
            count = count + 1
            s = pygame.display.get_surface()
            s.fill(ob.color, list_rect_obstacles[count])
            ob.detected = False
            ob.dist = 1000.0
            ob.xCollide = 0.0
            ob.yCollide = 0.0
        r.draw_rays(screen)
        r.showGoalPath(screen)

        robotSprite.draw(screen)
        goalSprite.draw(screen)
        

        
        #pygame.display.update()
        pygame.display.flip()   #all changes are drawn at once (double buffer)
        #pygame.time.delay(100)
    pygame.quit()               #also calls display.quit()
    f = open('results','w')

    f.write(str(Distance)+',')
    f.write(str(totalUserInput)+',')
    f.write(str(minObsDist))

if __name__ == '__main__':
    main()
