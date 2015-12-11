#!/usr/bin/env python
"""
Created on Tue Dec  1 11:14:18 2015

@author: lauren
"""

fps                 = 10        #at most  this many frames per second
back_image          = 'back2_800_600.bmp'   #must have this file in same dir.
display_cols        = 800
display_rows        = 600
wall_thickness      = 5         #thickness in pixels
wall_color          = 'black'
food_color          = 'purple'
trace_color         = 'blue'
trace_arc           = 10        #in degrees, shows on both sides of r.azi
trace_decrease      = -17       #negative, subtracts from robot size to make a smaller trace
trace_width         = 1
leave_trace         = 0         #default mode is not to leave traces

color_of_nothing    = 'white'
sim_version         = 'FrustrationRobot2000'

r_image          = 'robo2.bmp'  #must have this file in same dir.
r_edge           = 51       #edge of square surrounding robot (in pixels)
r_init_azi       = 0        #azimuth, in degrees (up is 0)
r_init_x         = 30       #must be >= wall_thickness
r_init_y         = 30
r_step_tele      = 10       #steps for teleop, equal in x or y (in pixels)
r_step_theta     = 7.5      #step for teleop, in azimuth
r_init_fwd_speed = 5        #pixels per simulation cycle
r_init_spin_speed= 3        #degrees per simulation cycle
r_transparency   = 0       #0 is totally transp., 255 totally opaque
r_visual_range   = 200      #measured from robot center
r_visual_angle   = 30       #in degrees, must divide 90 exactly!
r_visual_granularity = 5    #must be < wall_thickness for walls to be detected correctly!

R                = .010

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
from astar import PathManager

main_dir = os.path.split(os.path.abspath(__file__))[0]
screen = pygame.display.set_mode((display_cols, display_rows))
list_traces = []

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
        
        self.force      = 0.0
        self.theta      = 0.0
        
    def nodes(self):
        nodes = []
        for y in range(self.y_topleft -10, self.y_topleft + self.height + 10):
            for x in range(self.x_topleft-10, self.x_topleft + self.width + 10 ):
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

def draw_traces(target_surf):
    for t in list_traces:
        pygame.draw.arc(target_surf, pygame.Color(trace_color), t.rect,\
                        t.start_angle*math.pi/180, t.stop_angle*math.pi/180, trace_width)
                        
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
obs = random.randint(45,50)
for i in range(1,obs):
    x = random.randint(100,display_cols)
    y = random.randint(100,display_rows)
 
    width = random.randint(5,100)
    height = random.randint(5,100)
    obstacle = Obstacle(x,y,width,height,food_color)
    list_obstacles.append(obstacle)



#f01 = Obstacle(300,200,20,20,food_color)        #food pellet
#f02 = Obstacle(500,300,50,50,food_color)
#
#list_obstacles.append(f01)
#list_obstacles.append(f02)

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
        self.bearing        = 0.0
        self.direction       = 'N'
        self.maxAccel       = 20.0
        
        self.goal           = goal 
        self.spin_angle_left = 0.0
        self.found_goal     = 0
        
        self.path           = None

        
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
        else:
            print 'ERROR! Undefined operation mode!'
            
        
            
    def mode_0_tele(self):
        self.sense()    
        if self.collided:
                print 'update-->self.opmode==0 THAT HURT!'
                self.collided = False
                self.speed = 0.0
#                
        dtheta = 0       
                
                
        if self.direction == 'w':
            if self.speed >-self.maxAccel:
                self.speed -= 1
        elif self.direction == 's':
            if self.speed < self.maxAccel:
                self.speed += 1
        elif self.direction == 'd' or self.direction == 'a':
            dtheta = self.rotspeed*self._d[self.direction]
            #self.spin(dtheta)
        
        
        dx = self.speed*math.sin(self.bearing*math.pi/180)
        dy = self.speed*math.cos(self.bearing*math.pi/180)

        self.move(dx,dy,dtheta)


    
    
    ########end mode_1_walk(self)########

    def mode_1_auto(self):
        """The Autonomous mode to reach the goal"""
        self.sense()
        dtheta = 0.0
        if self.collided:
            print 'update-->self.opmode==0 THAT HURT!'
            self.collided = False
            self.speed = 0.0
            
        
        
        self.path = path_manager.generate_path((self.x,self.y),(self.goal.x,self.goal.y))      
        debug_path = self.path.get_path()

        if len(debug_path) > 1:
            current = debug_path[1]
            dx = current[0] - self.x
            dy = current[1] - self.y
            nx = debug_path[0][0]
            ny = debug_path[0][1]
            

            angle =  round((math.atan2(dx,dy))*180/math.pi)
            angle = angle+180
            if angle  >= 360:         #keep theta between -360..360
                angle -= 360
            if angle <= -360:
                angle += 360
            
            self.spin_angle_left = angle - self.bearing
            
            
            if self.spin_angle_left >=180.0:
                self.spin_angle_left = self.spin_angle_left - 360.0
            if self.spin_angle_left <=-180.0:
                self.spin_anlge_left = self.spin_angle_left + 360.0

            if self.spin_angle_left != 0:     #must finish SPIN
                if math.fabs(self.spin_angle_left) <= self.rotspeed:
                    dtheta = (sign(self.spin_angle_left))
                    if self.speed > 0.0:
                        self.speed -= 2
                    elif self.speed < 0.0:
                        self.speed += 2
                else:
                    dtheta = (sign(self.spin_angle_left)*(self.rotspeed))
                    if self.speed > 0.0:
                        self.speed -= 2
                    elif self.speed < 0.0:
                        self.speed += 2
                    
            else:
                if self.speed >-self.maxAccel :
                    self.speed -= 1
            
            moveX = self.speed*math.sin(self.bearing*math.pi/180)
            moveY = self.speed*math.cos(self.bearing*math.pi/180)
            self.move(moveX,moveY,dtheta)            

            
    ########end mode_2_auto(self)########
            
    def mode_2_assist(self):
        """Robot assists user to the goal"""
        self.sense()
        if self.collided:
            print 'update-->self.opmode==0 THAT HURT!'
            self.collided = False
            self.speed = 0.0
        
############ potential field method
        avoidX = 0
        avoidY = 0
        for obs in list_obstacles:
            center = obs.rect.center
            d = math.sqrt(math.pow(self.x-(center[0]+obs.width/2),2)+math.pow(self.y-(center[1]+obs.width/2),2))
            obs.force = R * math.exp(-.01*d)*60
            dx = center[0] - self.x
            dy = center[1] - self.y
            
            angle =  round((math.atan2(dx,dy)))
            obs.theta = self.bearing*math.pi/180 - angle
            
            avoidX += obs.force * math.cos(obs.theta)
            avoidY += obs.force * math.sin(obs.theta)
        print avoidX
        
########### if then method
        
#        theta = 0.0
#        for obs in list_obstacles:
#            center = obs.rect.center
#            
#            d = math.sqrt(math.pow(self.x-(center[0]+obs.width/2),2)+math.pow(self.y-(center[1]+obs.width/2),2))
#            print d
#            if d < 70.0 :
#                center = obs.rect.center
#                dx = center[0] - self.x
#                dy = center[1] - self.y
#            
#                angle =  round((math.atan2(dx,dy)))*180/math.pi
#                theta = self.bearing - angle
#
#                #self.speed = math.pow(d,-1)*self.speed*2
#                print self.speed
#                break
        
        
        moveY = self.x - self.goal.x 
        moveX = self.y - self.goal.y 
        
        goalx = 0.0
        goaly = 0.0

        angle =  round((math.atan2(moveY,moveX))*180/math.pi)
        
        self.spin_angle_left = math.fabs(angle-theta) - self.bearing 
        if self.spin_angle_left >=180.0:
            self.spin_angle_left = self.spin_angle_left - 360.0
        if self.spin_angle_left <=-180.0:
            self.spin_anlge_left = self.spin_angle_left + 360.0

        
        
        if self.direction == 'w':
            if self.speed >-self.maxAccel:
                self.speed -= 2
        elif self.direction == 's':
            if self.speed<self.maxAccel:
                self.speed += 2
        elif self.direction == 'd' or self.direction == 'a':
            dtheta = self.rotspeed*self._d[self.direction]
            self.spin(dtheta)
        
        
        
        #self.speed -=  avoidX
        #self.spin_angle_left -= int(avoidY)
        dx = self.speed*math.sin(self.bearing*math.pi/180)
        dy = self.speed*math.cos(self.bearing*math.pi/180)

        
        if self.found_goal:
           # print self.spin_angle_left
            if self.spin_angle_left != 0:     #must finish SPIN
                if math.fabs(self.spin_angle_left) <= self.rotspeed:
                    self.spin(sign(self.spin_angle_left))
                else:
                    self.spin(sign(self.spin_angle_left)*self.rotspeed)
   
   
                goalx = 0.001*self.speed*math.sin(self.bearing*math.pi/180)
                goaly = 0.001*self.speed*math.cos(self.bearing*math.pi/180)
        
        x = dx
        y = dy
        self.move(x,y,0)
            
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
        else:                   #if there was no collision
            self.x,self.y = self.rect.center
            
            
      


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

def load_image(name):
    path = os.path.join(main_dir, name)
    temp_image = pygame.image.load(path).convert_alpha()  #need this if using ppalpha
    return change_alpha_for_white(temp_image, r_transparency)  
    


###########################################
###########################################
def main():
    global leave_trace, list_traces
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
              r_init_spin_speed, r_visual_range, r_visual_angle,goal)

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
    

    going = True
    
    
    while going:
        clock.tick(fps)      #at most that many fps

        #Event loop################################

        for event in pygame.event.get():
            if event == QUIT:
                going = False
            elif event.type == KEYDOWN:
                if event.key == K_ESCAPE:
                    going = False
                    
                elif event.key == K_w:
                    r.direction = 'w'
                elif event.key == K_d:
                    r.direction = 'd'
                elif event.key == K_a:
                    r.direction = 'a'
                elif event.key == K_s:
                    r.direction = 's'
                    
               
                if event.key == K_SPACE:
                    r.opmode = 0            #teleop mode
                    caption = sim_version + ' \tmode: teleoperation  '
                if event.key == K_1:
                    r.opmode = 1            #autonomous navigation mode
                    caption = (sim_version + ' \tmode: autonomous  ')
                if event.key == K_2:
                    r.opmode = 2            #autonomous navigation mode
                    caption = (sim_version + ' \tmode: assist  ')
                    
                                    
                    
                    
                if event.key == K_t:        #toggles the tracing mode
                    if leave_trace:
                        leave_trace = 0
                        list_traces = list()
                        print 'changing leave_trace from 1 to 0'
                    else:
                        leave_trace = 1
                        print 'changing leave_trace from 0 to 1'
            elif event.type == KEYUP:
                if event.key == K_w:
                    r.direction = 'N'
                elif event.key == K_d:
                    r.direction = 'N'
                elif event.key == K_a:
                    r.direction = 'N'
                elif event.key == K_s:
                    r.direction = 'N'
                
        pygame.display.set_caption(caption + str(time.time() - startTime))
                        
        #Redrawing
        if r.speed > 0.0:
            r.speed -= 0.5
        elif r.speed < 0.0:
            r.speed += 0.5
            
        
            
        # Find if goal reached 
        if pygame.sprite.spritecollide(r, goalSprite, False) != []:
            print 'You made it to the goal'
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
        r.draw_rays(screen)
        r.showGoalPath(screen)
#       
        
        draw_traces(screen)
        robotSprite.draw(screen)
        goalSprite.draw(screen)
        

        
        #pygame.display.update()
        pygame.display.flip()   #all changes are drawn at once (double buffer)
        #pygame.time.delay(100)
    pygame.quit()               #also calls display.quit()


if __name__ == '__main__':
    main()

