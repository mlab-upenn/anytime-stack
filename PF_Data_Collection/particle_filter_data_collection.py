import numpy as np
import os 
import signal
import subprocess
import yaml
import time

# Finished initializing, waiting on messages...

def editConfigFile(N):

    with open("/home/schmidd/ros_ws/src/pf_test/config/amcl.yaml", "r") as file:
        dict_file = yaml.load(file, Loader=yaml.FullLoader)

    dict_file['particle_filter']['ros__parameters']['max_particles'] = N

    with open("/home/schmidd/ros_ws/src/pf_test/config/amcl.yaml", 'w') as file:
        documents = yaml.dump(dict_file, file)

kill_cmd = 'kill -9 $(pidof /usr/bin/python3 /opt/ros/foxy/bin/ros2 bag record)'

commands = [
    'colcon build --symlink-install --packages-select pf_test', 
    'ros2 launch pf_test pf_test.launch.py',
    """ros2 topic pub -t 5 -r 1 /initialpose geometry_msgs/PoseWithCovarianceStamped '{ header: {frame_id: "map"}, pose: { pose: {position: {x: 3.3079, y: 0.1519, z: 0.0}, orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0058}}, } }'""",
    'ros2 bag record -o pf_data_test /pf/pose/odom',
    'ros2 bag play lidar_odom_short/'
    ]

workdirs = [
    '/home/schmidd/ros_ws/', 
    '/home/schmidd/',
    '/home/schmidd/',
    '/home/schmidd/Documents/PF_Dataset/',
    '/home/schmidd/Documents/ROS_Bag_Data/',
    ]

K = 20
J = 20

Ts = ((np.arange(K)/K)*20e-3 + 12.5e-3)

k = 0
j = 0

for k in range(K):

    for j in range(J):

        print("Max Particles: " + str(k*200+100) + " and Iteration: " + str(j))

        editConfigFile(k*200 + 100)

        pf_string = 'ros2 bag record -o pf_data_particles_' + str(k) + '_' + str(j) + ' /pf/pose/odom'
        # pf_string = 'ros2 bag record -o pf_data_sync_' + str(j) + ' /pf/pose/odom'

        p1 = subprocess.Popen(commands[0], shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, cwd=workdirs[0]).wait()

        print("Colcon Build Done")

        time.sleep(2.00)

        p2 = subprocess.Popen(commands[1], shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, cwd=workdirs[1], preexec_fn=os.setsid)

        print("Launching PF Test")

        time.sleep(5.00)

        print("Publishing Initial Pose")

        p3 = subprocess.Popen(commands[2], shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, cwd=workdirs[2]).wait()

        p4 = subprocess.Popen(pf_string, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, cwd=workdirs[3], preexec_fn=os.setsid)

        print("Start Recording")

        time.sleep(5.00)

        print("Start Playback")

        p5 = subprocess.Popen(commands[4], shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, cwd=workdirs[4]).wait()

        print("Playback Done, Kill All Processes")

        time.sleep(3.00)

        for i in range(20):

            os.killpg(os.getpgid(p4.pid), signal.SIGTERM)

            os.killpg(os.getpgid(p2.pid), signal.SIGTERM)

            p4.terminate()

            p4.kill()

            time.sleep(0.1)
        
        time.sleep(5.00)

# p4.kill()
# p2.kill()



# for k in range(K):

#     for j in range(J):

#         print("Current Frequency: " + str(1/Ts[k]) + "Hz and Iteration: " + str(j))

#         editConfigFile(k/K*20e-3 + 12.5e-3)

#         pf_string = 'ros2 bag record -o pf_data_test_' + str(k) + '_' + str(j) + ' /pf/pose/odom'

#         p1 = subprocess.Popen(commands[0], shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, cwd=workdirs[0]).wait()

#         print("Colcon Build Done")

#         time.sleep(2.00)

#         p2 = subprocess.Popen(commands[1], shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, cwd=workdirs[1])

#         print("Launching PF Test")

#         time.sleep(5.00)

#         print("Publishing Initial Pose")

#         p3 = subprocess.Popen(commands[2], shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, cwd=workdirs[2]).wait()

#         p4 = subprocess.Popen(pf_string, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, cwd=workdirs[3])

#         print("Start Recording")

#         time.sleep(5.00)

#         print("Start Playback")

#         p5 = subprocess.Popen(commands[4], shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, cwd=workdirs[4]).wait()

#         print("Playback Done, Kill All Processes")

#         p4.kill()
#         p2.kill()




# editConfigFile(50e-3)

# p1 = subprocess.Popen(commands[0], shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, cwd=workdirs[0]).wait()

# p2 = subprocess.Popen(commands[1], shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, cwd=workdirs[1])

# time.sleep(10.00)

# p3 = subprocess.Popen(commands[2], shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, cwd=workdirs[2]).wait()

# p4 = subprocess.Popen(commands[3], shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, cwd=workdirs[3])

# time.sleep(5.00)

# p5 = subprocess.Popen(commands[4], shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, cwd=workdirs[4])

