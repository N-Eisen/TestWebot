from deepbots.supervisor.controllers.robot_supervisor_env import RobotSupervisorEnv
from utilities import normalize_to_range
from PPO_agent import PPOAgent, Transition
from final_package.follow_joint_trajectory_client import FollowJointTrajectoryClient

from gym.spaces import Box, Discrete
import numpy as np

GOAL = {
    'joint_names': [
        'shoulder_pan_joint',
        'shoulder_lift_joint',
        'elbow_joint',
        'wrist_1_joint',
        'finger_1_joint_1',
        'finger_2_joint_1',
        'finger_middle_joint_1'
    ],
    'points': [
        {
            'positions': [0.63, -2.26, -1.88, -2.14, 0.0495, 0.0495, 0.0495],
            'time_from_start': {'sec': 6, 'nanosec': 0}
        }
    ]
}

START = {
    'joint_names': [
        'shoulder_pan_joint',
        'shoulder_lift_joint',
        'elbow_joint',
        'wrist_1_joint',
        'finger_1_joint_1',
        'finger_2_joint_1',
        'finger_middle_joint_1'
    ],
    'points': [
        {
            'positions': [0.0, 0.0, 0.0, 0.0, 0.0495, 0.0495, 0.0495],
            'time_from_start': {'sec': 0, 'nanosec': 0}
        }
    ]
}

class Ur5eWithGripper(RobotSupervisorEnv):
    def __init__(self):
        super.__init__()

       # self.observation_space

        self.action_space = Discrete(4)

        self.robot = self.getSelf()
        # Sensoren z.B. position
        #self.position_sensor = self.getDevice("polePosSensor")
        #self.position_sensor.enable(self.timestep)

        #self.pole_endpoint = self.getFromDef("POLE_ENDPOINT")
        #self.wheels = []
        #for wheel_name in ['wheel1', 'wheel2', 'wheel3', 'wheel4']:
         #   wheel = self.getDevice(wheel_name)  # Get the wheel handle
        #    wheel.setPosition(float('inf'))  # Set starting position
         #   wheel.setVelocity(0.0)  # Zero out starting velocity
        #   self.wheels.append(wheel)

        self.steps_per_episode = 200
        self.episode_score = 0
        self.episode_score_list = []

    def get_observations(self):   
        # abstand objekt
        # punkt im raum
        return []
    
    def get_default_observation(self):
        # This method just returns a zero vector as a default observation
        return []
    
    def get_reward(self, action=None):
        # Reward is +1 for every step the episode hasn't ended
        return 1
    
    def is_done(self):
        if self.episode_score == 1:
            return True
        
        #arm is at goal point
        #cart_position = round(self.robot.getPosition()[0], 2)  # Position on x-axis
        #if abs(cart_position) > 0.39:
            #return True

        return False
    
    def solved(self):
        if len(self.episode_score_list) > 100:  # Over 100 trials thus far
            if np.mean(self.episode_score_list[-100:]) > 0.95:  # Last 100 episodes' scores average value
                return True
        return False
    
    def get_info(self):
        return None

    def render(self, mode='human'):
        pass

    def apply_action(self, action):
        action = int(action[0])

        # 4 action varianten
        if action == 0:
            motor_speed = 5.0
        else:
            motor_speed = -5.0

        #move(up/down,..)
    

def main(args=None):
    controller = FollowJointTrajectoryClient('ur5e_controller', '/ur5e/ur_joint_trajectory_controller')
    env = Ur5eWithGripper()
    agent = PPOAgent(number_of_inputs=env.observation_space.shape[0], number_of_actor_outputs=env.action_space.n)

    solved = False
    episode_count = 0
    episode_limit = 2000 

    while not solved and episode_count < episode_limit:
        observation = env.reset()  # Reset robot and get starting observation
        controller.send_goal(START, 1)
        env.episode_score = 0
    #Randomize Object and spawn it, only objects possible to clear


        controller.send_goal(GOAL, 1)
        for step in range(env.steps_per_episode):
            # In training mode the agent samples from the probability distribution, naturally implementing exploration
            selected_action, action_prob = agent.work(observation, type_="selectAction")
            # Step the supervisor to get the current selected_action's reward, the new observation and whether we reached
            # the done condition
            new_observation, reward, done, info = env.step([selected_action])

            # Save the current state transition in agent's memory
            trans = Transition(observation, selected_action, action_prob, reward, new_observation)
            agent.store_transition(trans)

        #if goal is achieved:
            #env.episode_score += reward  # Accumulate episode reward

            if done:
             # Save the episode's score
                env.episode_score_list.append(env.episode_score)
                agent.train_step(batch_size=step + 1)
                solved = env.solved()  # Check whether the task is solved
                break

        
            observation = new_observation  # observation for next step is current step's new_observation

        print("Episode #", episode_count, "score:", env.episode_score)
        episode_count += 1  # Increment episode counter
    if not solved:
        print("Task is not solved")
    elif solved:
        print("Task is solved")




    controller.send_goal(GOAL, 1)
    

if __name__ == '__main__':
    main()