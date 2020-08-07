import subprocess
import time
#Spawn background vehicle
command_1 = ['python', 'carla_functions/spawn_npc.py',"-n", "80"]
subprocess.Popen(command_1)
time.sleep(10)


#set up bridge
command_2 = ['python3', 'bridge.py']
subprocess.call(command_2)

