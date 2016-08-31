import numpy as np
from copy import deepcopy

dir_sensors = {'u': ['l', 'u', 'r'], 'r': ['u', 'r', 'd'],
               'd': ['r', 'd', 'l'], 'l': ['d', 'l', 'u'],
               'up': ['l', 'u', 'r'], 'right': ['u', 'r', 'd'],
               'down': ['r', 'd', 'l'], 'left': ['d', 'l', 'u']}
dir_move = {'u': [0, 1], 'r': [1, 0], 'd': [0, -1], 'l': [-1, 0],
            'up': [0, 1], 'right': [1, 0], 'down': [0, -1], 'left': [-1, 0]}
dir_reverse = {'u': 'd', 'r': 'l', 'd': 'u', 'l': 'r',
               'up': 'd', 'right': 'l', 'down': 'u', 'left': 'r'}
dir_compare ={'uu':'u','ur':'r','ru':'l','ud':'b',
              'll':'u','lu':'r','ul':'l','du':'b',
              'rr':'u','dl':'r','ld':'l','lr':'b',
              'dd':'u','rd':'r','dr':'l','rl':'b',}
dir_value={'u':1,'r':2,'d':4,'l':8}
dir_heading={(0,1):'u',(1,0):'r',(0,-1):'d',(-1,0):'l'}
dir_rotation={'l':-90,'u':0,'r':90}

class Robot2(object):
    def __init__(self, maze_dim):
        '''
        Use the initialization function to set up attributes that your robot
        will use to learn and navigate the maze. Some initial attributes are
        provided based on common information, including the size of the maze
        the robot is placed in.
        '''
        self.location = [0, 0]
        self.heading = 'u'
        self.maze_dim = maze_dim
        self.orientation=[0,1]        
        self.goal_bounds=[self.maze_dim/2 - 1,self.maze_dim/2]
        self.skip_signal=False
        self.movement=0
        self.rotation=0
        self.junctions=[]
        self.cell_points=np.zeros([maze_dim,maze_dim],dtype='uint8')
        self.cell_points[0,0]=1
        self.steering=None
        self.i=-1 #used for second running's step counting
    def next_move(self, sensors):
        '''
        Use this function to determine the next move the robot should make,
        based on the input from the sensors after its previous move. Sensor
        inputs are a list of three distances from the robot's left, front, and
        right-facing sensors, in that order.

        Outputs should be a tuple of two values. The first value indicates
        robot rotation (if any), as a number: 0 for no rotation, +90 for a
        90-degree rotation clockwise, and -90 for a 90-degree rotation
        counterclockwise. Other values will result in no rotation. The second
        value indicates robot movement, and the robot will attempt to move the
        number of indicated squares: a positive number indicates forwards
        movement, while a negative number indicates backwards movement. The
        robot may move a maximum of three units per turn. Any excess movement
        is ignored.

        If the robot wants to end a run (e.g. during the first training run in
        the maze) then returing the tuple ('Reset', 'Reset') will indicate to
        the tester to end the run and return the robot to the start.
        '''
        
        if self.steering !=None:
            self.i=self.i+1
            return self.steering[self.i][0],self.steering[self.i][1]
                
        #here the robot finished exploring the whole maze
        if manhattan_dist_to_goal(self.location,self.goal_bounds)==0:
            rotation='Reset'
            movement='Reset'
            self.location = [0, 0]
            self.heading = 'u'
            path=repeat_a_star(self.cell_points,10)
            self.steering=to_steering(path)
            return rotation, movement
            

        if self.skip_signal:
            self.movement=1
            self.rotation=90
            self.skip_signal=False
        else:
            passable=list(map(lambda x:x>0 ,sensors))
            passable_num=passable.count(True)
            if passable_num==0:
                self.turn_back()
            elif passable_num==1:
                if passable[0]==True:
                    self.turn_left()
                elif passable[1]==True:
                    self.go_forward()
                elif passable[2]==True:
                    self.turn_right()
            elif passable_num==2:
                need_create=True
                operation=''
                for junction in self.junctions:
                    if junction.location==self.location:
                        operation=junction.direct(dir_reverse[self.heading])
                        need_create=False
                        break
                if need_create:
                    temp_junction=Junction_3(self.location,dir_reverse[self.heading],passable)
                    operation=dir_compare[self.heading+temp_junction.right_branch]
                    self.junctions.append(temp_junction)
                if operation=='l':
                    self.turn_left()
                elif operation=='u':
                    self.go_forward()
                elif operation=='r':
                    self.turn_right()
                elif operation=='b':
                    self.turn_back()
                    
            elif passable_num==3:
                need_create=True
                for junction in self.junctions:
                    if junction.location==self.location:
                        operation=junction.direct(dir_reverse[self.heading])
                        need_create=False
                if need_create:
                    temp_junction=Junction_4(self.location,dir_reverse[self.heading])
                    operation=dir_compare[self.heading+temp_junction.right_branch]
                    self.junctions.append(temp_junction)
                if operation=='l':
                    self.turn_left()
                elif operation=='u':
                    self.go_forward()
                elif operation=='r':
                    self.turn_right()
                elif operation=='b':
                    self.turn_back()
            if self.cell_points[self.location[0],self.location[1]]==0:
                self.cell_points[self.location[0],self.location[1]]=self.to_point_value(self.heading,passable)
        
        self.move(self.movement,self.rotation)
        return self.rotation, self.movement
        
        #process movement
    def move(self,movement,rotation):
        temp=self.orientation[:]
        if rotation==-90:
            self.orientation[0]=-temp[1]
            self.orientation[1]=temp[0]
            self.heading=dir_sensors[self.heading][0]
        elif rotation==90:
            self.orientation[0]=temp[1]
            self.orientation[1]=-temp[0]
            self.heading=dir_sensors[self.heading][2]

        self.location[0]=self.location[0]+movement*self.orientation[0]
        self.location[1]=self.location[1]+movement*self.orientation[1]
    
    def turn_back(self):
        self.skip_signal=True
        self.movement=0
        self.rotation=90
    def turn_left(self):
        self.movement=1
        self.rotation=-90
    def turn_right(self):
        self.movement=1
        self.rotation=90
    def go_forward(self):
        self.movement=1
        self.rotation=0
    def to_point_value(self,heading,passable):
        orientations=dir_sensors[heading][:]
        orientations.append(dir_reverse[heading])
        temp=[]
        for i in orientations:
            temp=temp+[dir_value[i]]
        passable_list=passable
        passable_list.append(True)
        value=sum(list(map(lambda x:x[0]*x[1],zip(temp,passable_list))))
        return value
    
    
class Junction_3:
    def __init__(self,location,come_from,passable):
        self.location=location[:]
        self.come_from=come_from
        orientations=dir_sensors[dir_reverse[come_from]][:]
        if passable[0]==False:
            self.left_branch=orientations[1]
            self.right_branch=orientations[2]
        if passable[1]==False:
            self.left_branch=orientations[0]
            self.right_branch=orientations[2]
        if passable[2]==False:
            self.left_branch=orientations[0]
            self.right_branch=orientations[1]
        self.left_branch_visited=False
    def direct(self,oncoming):
        operation=''
        if oncoming==self.right_branch:
            if self.left_branch_visited:
                operation=dir_compare[dir_reverse[oncoming]+self.come_from]
            else:
                operation=dir_compare[dir_reverse[oncoming]+self.left_branch]
                self.left_branch_visited=True
        elif oncoming==self.left_branch:
            if self.left_branch_visited:
                operation=dir_compare[dir_reverse[oncoming]+self.come_from]
            else:
                operation='b'
                self.left_branch_visited=True
        return operation
        
class Junction_4:
    def __init__(self,location,come_from):
        self.location=location[:]
        self.come_from=come_from
        orientations=dir_sensors[dir_reverse[come_from]][:]
        self.left_branch=orientations[0]
        self.forward_branch=orientations[1]
        self.right_branch=orientations[2]
        self.left_branch_visited=False
        self.forward_branch_visited=False
        self.mode=None
    def direct(self,oncoming):
        operation=''
        if oncoming==self.right_branch:
            if self.mode==None:
                self.mode=1
                operation=dir_compare[dir_reverse[oncoming]+self.forward_branch]
                self.forward_branch_visited=True                
            elif self.mode==2:
                operation=dir_compare[dir_reverse[oncoming]+self.come_from]                
        elif oncoming==self.forward_branch:
            if self.mode==None:
                self.mode=2
                operation='r'
                self.left_branch_visited=True
                self.forward_branch_visited=True
            elif self.mode==1:
                if self.left_branch_visited:
                    operation='u'
                else:
                    operation='r'
                    self.left_branch_visited=True
            elif self.mode==2:
                operation='r'            
        elif oncoming==self.left_branch:
            if self.mode==None:
                self.mode=2
                operation='l'
                self.left_branch_visited=True
                self.forward_branch_visited=True                
            elif self.mode==1:
                if not self.left_branch_visited:
                    operation='b'
                    self.left_branch_visited=True
            elif self.mode==2:
                operation='l'            
        return operation
#used for A* alghorithm
class Point:
    def __init__(self,location,cell_data,goal_bounds):
        self.location = location[:]
        self.came_from = None
        self.g_score = 1000
        self.h_score = manhattan_dist_to_goal(self.location,goal_bounds) 
        #check each orientation is passable or not
        self.up_passable=(cell_data&1!=0)
        self.right_passable=(cell_data&2!=0)
        self.down_passable=(cell_data&4!=0)
        self.left_passable=(cell_data&8!=0)
        #append the neighbor position to the neighbors list
        self.neighbors=[]
        if self.up_passable:
            self.neighbors.append([self.location[0],self.location[1]+1])
        if self.right_passable:
            self.neighbors.append([self.location[0]+1,self.location[1]])
        if self.down_passable:
            self.neighbors.append([self.location[0],self.location[1]-1])
        if self.left_passable:
            self.neighbors.append([self.location[0]-1,self.location[1]])
#repeat A* for several times to find a quickest path.               
def repeat_a_star(maze_data,times):
    path=a_star(maze_data)
    for i in range(times-1):
        temp=a_star(maze_data)
        #print temp
        if temp[-1].g_score<path[-1].g_score:
            path=deepcopy(temp)
    return path

def a_star(maze_data):
    dim=maze_data.shape[0]
    goal_bounds = [dim/2 - 1, dim/2]
    start_location=[0,0]
    open_set = set()
    closed_set = set()
    start_point = Point(start_location,maze_data[tuple(start_location)],goal_bounds)
    start_point.g_score = 0
    open_set.add(start_point)
    while len(open_set) > 0:
        current = min(open_set,key=lambda o:o.g_score + o.h_score)
        if manhattan_dist_to_goal(current.location,goal_bounds)==0:
            path = []
            while current.came_from:
                path.append(current)
                current = current.came_from 
            # the start node does not have a parent
            path.append(current)
            return path[::-1]

        open_set.remove(current)
        closed_set.add(current)
        for neighbor in current.neighbors: #here neighbor is position data,not Point object
            if neighbor in list(map(lambda x:x.location,closed_set)):
                continue
            #here is keypoint 
            if straight_step_num(current)<2 and is_straight(current,neighbor):
                tentative_g_score=current.g_score
            else:
                tentative_g_score=current.g_score+1
            #unexplored position seem as not passable
            if neighbor not in list(map(lambda x:x.location,open_set)):# and maze_data[tuple(neighbor)]>0:
                neighbor_point=Point(neighbor,maze_data[tuple(neighbor)],goal_bounds)
                open_set.add(neighbor_point)
            else:
                neighbor_point=filter(lambda x:x.location==neighbor,open_set)[0]
                if tentative_g_score>neighbor_point.g_score:
                    continue
            neighbor_point.came_from=current
            neighbor_point.g_score=tentative_g_score
            

#calculate distance between location and goal aera.here goal_bounds is limit of x,y           
def manhattan_dist_to_goal(location,goal_bounds):
    x=y=0
    if location[0]<=goal_bounds[0]:
        x=goal_bounds[0]-location[0]
    else:
        x=location[0]-goal_bounds[1]
    if location[1]<=goal_bounds[0]:
        y=goal_bounds[0]-location[1]
    else:
        y=location[1]-goal_bounds[1]
    return x+y
#calculte how many more straight steps has been taken since counting start.
def straight_step_num(point):
    i=0
    temp=point
    while(temp.came_from!=None and temp.g_score==temp.came_from.g_score):
        i=i+1
        temp=temp.came_from
    return i
#check whether s step is going straightly
def is_straight(point,neighbor):
    if (point.came_from!=None 
        and neighbor[0]-point.location[0]==point.location[0]-point.came_from.location[0]
        and neighbor[1]-point.location[1]==point.location[1]-point.came_from.location[1]):
        return True
    else:
        return False
#translate path to steering sequent.
def to_steering(path):
    steering=[]
    temp=path[0]
    temp.heading='u'
    rotation=0
    movement=0
    for point in path[1:]:
        point.heading = dir_heading[tuple(map(lambda x:x[0]-x[1],zip(point.location,temp.location)))]
        if point.g_score>temp.g_score:
            steering.append([rotation,movement])
            turning=dir_compare[temp.heading+point.heading]
            rotation=dir_rotation[turning]
            movement=1
        else:
            movement+=1
        temp=point
    steering.append([rotation,movement])
    return steering[1:]            

