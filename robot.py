import pygame
from pygame.locals import *
import math
import random

color_of_nothing    = 'white'
r_image          = 'robo2.bmp'  #must have this file in same dir.
r_edge           = 51       #edge of square surrounding robot (in pixels)
r_init_azi       = 0        #azimuth, in degrees (up is 0)
r_init_x = 100       #must be >= wall_thickness
r_init_y = 100
r_step_tele      = 10       #steps for teleop, equal in x or y (in pixels)
r_step_theta     = 7.5      #step for teleop, in azimuth
r_init_fwd_speed = 5        #pixels per simulation cycle
r_init_spin_speed= 3        #degrees per simulation cycle
r_transparency   = 0       #0 is totally transp., 255 totally opaque
r_visual_range   = 200      #measured from robot center
r_visual_angle   = 30       #in degrees, must divide 90 exactly!
r_visual_granularity = 5    #must be < wall_thickness for walls to be detected correctly!


class Robot(pygame.sprite.Sprite):
    def __init__(self, image, x, y, azimuth, fwd_speed, spin_speed,\
                 visual_range, visual_angle, goal):
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
        self.acceleration   = 0.0
        self.bearing        = 0.0
        self.direction       = 'N'
        self.maxAccel       = 10.0
        
        self.goal           = goal 
        self.spin_angle_left = 0.0
        self.found_goal     = 0

        
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
        elif (self.opmode == 1): self.mode_1_walk()     #RW (random walk)
        elif (self.opmode == 2): self.mode_2_auto()     #autonomous
        elif (self.opmode == 3): self.mode_3_assist()   #assist 
        else:
            print 'ERROR! Undefined operation mode!'
            
        
            
    def mode_0_tele(self):
        self.sense()    
        if self.collided:
                print 'update-->self.opmode==0 THAT HURT!'
                self.collided = False
                self.acceleration = 0.0

        if self.direction == 'w':
            if self.acceleration >-self.maxAccel:
                self.acceleration -= 2
        elif self.direction == 's':
            if self.acceleration<self.maxAccel:
                self.acceleration += 2
        elif self.direction == 'd' or self.direction == 'a':
            dtheta = self.rotspeed*self._d[self.direction]
            self.spin(dtheta)
        
        self.speed = self.acceleration
        dx = self.speed*math.sin(self.bearing*math.pi/180)
        dy = self.speed*math.cos(self.bearing*math.pi/180)

        self.move(dx,dy)

  
            
    def mode_1_walk(self):
        self.sense()
        if self.collided:       #collision in prev. cycle --> start SPIN
                print 'update-->self.opmode==1 THAT HURT!'
                self.collided = False
                walk_dazi = random.randint(-180, 180)
                if math.fabs(walk_dazi) <= self.spin_speed:    #can spin in 1 cycle
                    self.spin(walk_dazi)
                else:
                    if walk_dazi > 0:   #calculate the angle's sign
                        sign = 0.5
                    else:
                        sign = -0.5
                    self.spin(sign*self.spin_speed)
                    self.spin_angle_left = walk_dazi-sign*self.spin_speed                
        else:                   #not collided --> finish SPIN, or MOVE fwd
            if math.fabs(self.spin_angle_left) > 0:     #must finish SPIN
                if math.fabs(self.spin_angle_left) <= self.spin_speed:    #can spin in 1 cycle
                    self.spin(self.spin_angle_left)
                    self.spin_angle_left = 0
                else:
                    if self.spin_angle_left > 0:   #calculate the angle's sign
                        sign = 1
                    else:
                        sign = -1
                    self.spin(sign*self.spin_speed)
                    self.spin_angle_left -= sign*self.spin_speed 
            else:                            
                if self.acceleration<self.maxAccel:
                    self.acceleration += 2
           
                self.speed = self.acceleration
                dx = self.speed*math.sin(self.bearing*math.pi/180)
                dy = self.speed*math.cos(self.bearing*math.pi/180)
                self.move(dx, dy)
    ########end mode_1_walk(self)########

    def mode_2_auto(self):
        """The Autonomous mode to reach the goal"""
        self.sense()
        if self.collided:
            print 'update-->self.opmode==0 THAT HURT!'
            self.collided = False
            self.acceleration = 0.0
        
        moveY = self.x - self.goal.x 
        moveX = self.y - self.goal.y 

        angle =  round((math.atan2(moveY,moveX))*180/math.pi)

        self.spin_angle_left = angle - self.bearing

       # print self.spin_angle_left
        if self.spin_angle_left != 0:     #must finish SPIN
            if math.fabs(self.spin_angle_left) < self.rotspeed:
                self.spin(1)
            else:
                self.spin(sign(self.spin_angle_left)*self.rotspeed)
        else :
            self.sense()
            self.direction = 'w'
            if self.acceleration >-self.maxAccel:
                self.acceleration -= 2
                
            self.speed = self.acceleration
            dx = self.speed*math.sin(self.bearing*math.pi/180)
            dy = self.speed*math.cos(self.bearing*math.pi/180)

            self.move(dx,dy)
            
    ########end mode_2_auto(self)########
            
    def mode_3_assist(self):
        """Robot assists user to the goal"""
        self.sense()
        if self.collided:
            print 'update-->self.opmode==0 THAT HURT!'
            self.collided = False
            self.acceleration = 0.0
        
        moveY = self.x - self.goal.x 
        moveX = self.y - self.goal.y 
        
        goalx = 0.0
        goaly = 0.0

        angle =  round((math.atan2(moveY,moveX))*180/math.pi)

        self.spin_angle_left = angle - self.bearing
        
        if self.direction == 'w':
            if self.acceleration >-self.maxAccel:
                self.acceleration -= 2
        elif self.direction == 's':
            if self.acceleration<self.maxAccel:
                self.acceleration += 2
        elif self.direction == 'd' or self.direction == 'a':
            dtheta = self.rotspeed*self._d[self.direction]
            self.spin(dtheta)
        
        self.speed = self.acceleration
        dx = self.speed*math.sin(self.bearing*math.pi/180)
        dy = self.speed*math.cos(self.bearing*math.pi/180)

        
        if self.found_goal:
           # print self.spin_angle_left
            if self.spin_angle_left != 0:     #must finish SPIN
                self.spin(sign(self.spin_angle_left))
            else :
                if self.acceleration >-self.maxAccel:
                    self.acceleration -= 2
                    
                self.speed = self.acceleration
                goalx = 0.001*self.speed*math.sin(self.bearing*math.pi/180)
                goaly = 0.001*self.speed*math.cos(self.bearing*math.pi/180)
        
        x = dx 
        y = dy 
        self.move(x,y)
            
    def showGoalPath(self,target_surf):
        pygame.draw.line(target_surf, (255,0,0), (self.x, self.y), (self.goal.x, self.goal.y))
        
        
            
        
    def move(self,dx,dy):
        previous_rect = self.rect           #re1member in case undo is necessary       
        self.rect = self.rect.move(dx,dy)
        if self.rect.collidelist(list_rect_obstacles) != -1:#if collision exists
            print 'mode  -->I collided with wall(s)',\
                  self.rect.collidelistall(list_rect_obstacles)
            self.rect = previous_rect                   #undo the move
            self.collided = True
        else:                   #if there was no collision
            self.x,self.y = self.rect.center
            if leave_trace:     #update trace list
                tr = self.rect.inflate(trace_decrease, trace_decrease)
                list_traces.append(Trace(tr, 90+self.bearing-trace_arc, 90+self.bearing+trace_arc))


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
        if leave_trace:     #update trace list
            tr = self.rect.inflate(trace_decrease, trace_decrease)
            list_traces.append(Trace(tr, 90+self.bearing-trace_arc, 90+self.bearing+trace))
    
    
    #this function's job is to place in self.retina the range sensed by each sensor
    def sense(self):
        n = (self.nr_sensors - 1)/2     #the "natural" sensor range is -n to +n
        granu = r_visual_granularity    #must be at least as large as the wall thickness!!
        for i in range(-n,n+1):         #sense with each of the 2n+1 range sensors
            ang = (self.bearing - i*self.visual_angle)*math.pi/180
            for distance in range(granu, self.visual_range+granu, granu):
                x = self.rect.center[0]-distance*math.sin(ang)  #endpoint coordinates
                y = self.rect.center[1]-distance*math.cos(ang)
                nr_collisions = 0
                count = -1          #needed to coordinate the two lists, to extract color after loop
                for ob in list_rect_obstacles:  #use the stripped-down list of rectangles for speed
                    count = count + 1
                    if ob.collidepoint(x,y):
                        nr_collisions = 1
                        break       #breaks out of wall loop
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
                self.retina[i+n][1] = list_obstacles[count].color #color comes form the larger list
            else:
                self.retina[i+n][1] = pygame.Color(color_of_nothing)
        
        #print 'sense -->retina is:\n', self.retina
        #self.printRetina()
          
    def draw_rays(self, target_surf):
        n = (self.nr_sensors - 1)/2 #the "natural" sensor range -n to +n
        for i in range(-n,n+1):     #draw the 2n+1 rays of the range sensors
            ang = (self.bearing - i*self.visual_angle)*math.pi/180
            x = self.rect.center[0]-self.retina[i+n][0]*math.sin(ang)
            y = self.rect.center[1]-self.retina[i+n][0]*math.cos(ang)
            #use aaline for smoother (but slower) lines
            pygame.draw.line(target_surf, (0,0,0), self.rect.center, (x,y))
########end of Robot class########