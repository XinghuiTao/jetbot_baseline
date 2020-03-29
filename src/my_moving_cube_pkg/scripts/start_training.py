#!/usr/bin/env python
import os
import gym
from baselines import deepq
import rospy
import rospkg
import my_one_disk_walk # importing our robot task environment

def main():
    rospy.init_node('movingcube_onedisk_walk_gym', anonymous=True, log_level=rospy.WARN)

    # Set the path where learned model will be saved
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('my_moving_cube_pkg')
    models_dir_path = os.path.join(pkg_path, "models_saved")
    if not os.path.exists(models_dir_path):
        os.makedirs(models_dir_path)

    out_model_file_path = os.path.join(models_dir_path, "movingcube_model.pkl")

    max_timesteps = rospy.get_param("/moving_cube/max_timesteps")
    buffer_size = rospy.get_param("/moving_cube/buffer_size")
    # We convert to float becase if we are using Ye-X notation, it sometimes treats it like a string.
    lr = float(rospy.get_param("/moving_cube/lr"))

    exploration_fraction = rospy.get_param("/moving_cube/exploration_fraction")
    exploration_final_eps = rospy.get_param("/moving_cube/exploration_final_eps")
    print_freq = rospy.get_param("/moving_cube/print_freq")

    env = gym.make("MyMovingCubeOneDiskWalkEnv-v0")

    # start the learning
    act = deepq.learn(
        env,
        network='mlp',
        lr=lr,
        total_timesteps=max_timesteps,
        buffer_size=buffer_size,
        exploration_fraction=exploration_fraction,
        exploration_final_eps=exploration_final_eps,
        print_freq=print_freq, # how many apisodes until you print total rewards and info
        param_noise=False,
        callback=None,
        load_path=None # indicate here a previous saved model (if you want to continue training it)
    )

    env.close()

    # Save the learned model
    rospy.logwarn("Saving model to movingcube_model.pkl")
    act.save(out_model_file_path)


if __name__ == '__main__':
    main()