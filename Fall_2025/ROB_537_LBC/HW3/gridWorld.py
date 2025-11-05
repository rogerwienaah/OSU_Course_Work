import random
from copy import deepcopy
from math import sqrt

def distance(position_a, position_b):
    delta_x = position_a[0]-position_b[0]
    delta_y = position_a[1]-position_b[1]
    return sqrt( (delta_x)**2 + (delta_y)**2 )

class gridWorld:
    def __init__(self,option_1=False,option_2=False,option_3=False):
        # put gridsize and obstacle positions here
        self.grid_size=(10,5)
        # self.obstacle_positions=[[0,2],[1,2],[2,2],[3,2]]


        self.init_door_position=[1,4]
        self.agent_position=None
        self.door_position=None
        self.num_steps=0

        # For part B
        self.option_1 = option_1 # Move door
        self.option_2 = option_2 # Use local observations
        self.option_3 = option_3 # Use custom reward

    def reset(self):
        self.door_position=self.init_door_position
        position=[-1,-1]
        while not self.isValid(position):
            position[0]=random.randint(0,9)
            position[1]=random.randint(0,4)
        self.agent_position=position
        if self.option_2:
            return self.build_observation()
        else:
            return deepcopy(self.agent_position)

    def take_action(self,action,position,check_valid=True):
        x,y=position
        if action=="up":
            y+=1
        if action=="down":
            y-=1
        if action=="right":
            x+=1
        if action=="left":
            x-=1
        if not check_valid:
            return [x,y]
        if self.isValid([x,y]):
            return [x,y]
        return position
    
    def generate_default_reward(self):
        if self.agent_position[0]==self.door_position[0] and self.agent_position[1]==self.door_position[1]:
            reward=20
        else:
            reward=-1
        return reward
    
    def move_door(self):
        # ---------------------------------------------------------------------
        # TODO (Part B, Option 1): Move the door every even timestep.
        # You can either use the code below or modify it to create your own version.
        # Example modifications:
        #   - Only move left/right or up/down
        #   - Follow a fixed path rather than a random path
        # Make sure you explain how the door moves and any modifications.
        # ---------------------------------------------------------------------
        # Every even timestep (Ex: 0,2,4,6,...), the door takes a random action
        if self.num_steps%2==0: 
            door_action=random.choice(['up','down','left','right','stay'])
            new_door_position=self.take_action(door_action,self.door_position)
            self.door_position=new_door_position
        
    def build_observation(self):
        # ---------------------------------------------------------------------
        # TODO (Part B, Option 2): Build a local observation for the agent's "state".
        # You can either use the code below or modify it to create your own version.
        # Example modifications:
        #   - Look two-steps away rather than one-step away
        #   - Use floating-point values to represent how far away a door/obstacle is
        # Make sure you explain how the observation works and any modifications.
        # ---------------------------------------------------------------------
        # We generate temp_position and check what is left, right, up, and down from the agent
        # With this information we build a virtual_sensor that reads like so
        # [door_detection_left, door_detection_right, door_detection_up, door_detection_down,
        #     obstacle_detection_left, obstacle_detection_right, obstacle_detection_up, obstacle_detection_down]
        # Each value corresponds to a direction, and whether or not we sensed something there.
        # For example, if the door is 'up' from the agent and an obstacle is 'down' from the agent, 
        #     then our sensor reads [0, 0, 1, 0, 0, 0, 0, 1]
        door_detections=[]
        for action in ['left','right','up','down']:
            temp_position=self.take_action(action,self.agent_position,check_valid=False)
            door_detected=0
            if temp_position[0]==self.door_position[0] and temp_position[1]==self.door_position[1]:
                door_detected=1
            door_detections.append(door_detected)
        obstacle_detections=[]
        for action in ['left','right','up','down']:
            temp_position=self.take_action(action,self.agent_position,check_valid=False)
            obstacle_detected=0
            if not self.isValid(temp_position):
                obstacle_detected=1
            obstacle_detections.append(obstacle_detected)
        virtual_sensor=door_detections+obstacle_detections
        return virtual_sensor
        
    def generate_custom_reward(self,previous_position,action):
        # ---------------------------------------------------------------------
        # TODO (Part B, Option 3): Generate a custom reward.
        # You can either use the code below or modify it to create your own version.
        # Example modifications:
        #   - Give negative reward if you are near an obstacle
        #   - Give higher reward if you are close to the door (+20 for being at the door, +10 for being a step away from the door, etc)
        # Make sure you explain how the reward works and any modifications.
        # ---------------------------------------------------------------------
        # If the agent is at the door, then give +20
        # If the agent is getting closer to the door, then give +1
        # In any other case, -1
        if self.agent_position[0]==self.door_position[0] and self.agent_position[1]==self.door_position[1]:
            reward=20
        elif distance(self.agent_position, self.door_position)<distance(previous_position, self.door_position):
            reward=1
        else:
            reward=-1
        return reward


    def step(self,action):
        # Book-keeping. 
        # Track the number of steps we have made
        # Track the previous position of our agent
        self.num_steps+=1
        previous_position=deepcopy(self.agent_position)

        # Agent takes our specified action
        self.agent_position=self.take_action(action,self.agent_position)
        
        # By default, we leave the door in-place. 
        # If we set self.option_1=True, then move the door according to self.move_door().
        if self.option_1:
            self.move_door()

        # By default, the state of our agent is its position.
        # If we set self.option_2=True, then build a local observation to represent our agent's "state".
        if self.option_2:
            state=self.build_observation()
        else:
            state=deepcopy(self.agent_position)
        
        # By default, generate the reward using self.generate_default_reward().
        # If we set self.option_3=True, then use your own custom reward function.
        # In your custom reward function, you have access to the agent's previous position and its action,
        #     so you get flexibility in what information you want to incorporate in your reward.
        if self.option_3:
            reward=self.generate_custom_reward(previous_position, action)
        else:
            reward=self.generate_default_reward()

        return state,reward

    def isValid(self,position):
        x,y=position
        if x<0 or x>9:      #out of x bounds
            return False
        if y<0 or y>4:      #out of y bounds
            return False
        if x<=3 and y==2:    #inside obstacles
            return False
        return True

if __name__=="__main__":
    # example usage for a gym-like environment 
    # state: [x,y] position of the agent
    # actions: ["up","down","left","right","stay"] directions the agent can move
    # For part B, you can set your chosen option to True. For example:
    # env=gridWorld(option_1=True)
    env=gridWorld()
    for learning_epoch in range(100):
        state=env.reset()              #every episode, reset the environment to the original configuration
        for time_step in range(20):
            action=["up","down","left","right","stay"][0] #learner chooses one of these actions
            next_state,reward=env.step(action)            #the action is taken, a reward and new state is returned
