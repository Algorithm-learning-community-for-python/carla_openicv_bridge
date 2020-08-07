
# %%
# the TestEnv environment is used to simply simulate the network
from flow.envs import TestEnv

# the Experiment class is used for running simulations
from flow.core.experiment import Experiment

# the base network class
from flow.networks import Network

# all other imports are standard
from flow.core.params import VehicleParams
from flow.core.params import NetParams
from flow.core.params import InitialConfig
from flow.core.params import EnvParams
from flow.core.params import SumoParams
# create some default parameters parameters
import numpy as np
import os
from lxml import etree
net_xml=etree.parse('/home/yining/CARLA_0.9.9/Co-Simulation/Sumo/examples/net/Town04.net.xml')
root=net_xml.getroot()


edge_id_list=root.xpath('//edge/@id')
print(edge_id_list)
#edge_id_list.remove('0.0.00')
#edge_id_list.remove('17.0.00')
rts=np.array(len(edge_id_list))

#for i in range(1,len(edge_id_list)):
#    rts[i]=":"+edge_id_list[i]+": [:"+edge_id_list+"]"
print(rts)


env_params = EnvParams()
initial_config = InitialConfig()

# %%
Sumo_dir = "/home/yining/CARLA_0.9.9/Co-Simulation/Sumo/"

# %% [markdown]
# ## 3. Sumo Network Files
# 
# Sumo generates several network and simulation-specifc template files prior to starting a simulation. This procedure when creating custom networks and networks from OpenStreetMap is covered by the network class. Three of these files (\*.net.xml, \*.rou.xml, and vtype.add.xml) can be imported once again via the network class to recreate a previously designed network.
# 
# We start by creating the simulation parameters:

# %%


sim_params = SumoParams(render=True, sim_step=0.05,num_clients=1)

# %%
# specify the edges vehicles can originate on
initial_config = InitialConfig(
    edges_distribution=edge_id_list
)


# specify the routes for vehicles in the network
class TemplateNetwork(Network):

    def specify_routes(self, net_params):
        return {":317_6": [":317_6"]}

# %% [markdown]
# The simulation can then be executed as follows:

# %%
net_params = NetParams(
    template={
        # network geometry features
        "net": os.path.join(Sumo_dir, "examples/net/Town04.net.xml"),
        # features associated with the properties of drivers
        "vtype": os.path.join(Sumo_dir, "data/carlavtypes.add.xml"),
        # features associated with the routes vehicles take
        "rou": os.path.join(Sumo_dir, "examples/rou/Town04_flow.rou.xml")
               
    }
)

# we no longer need to specify anything in VehicleParams
vehicles = VehicleParams()
#vehicles.add(veh_id='human',num_vehicles=10)


# %% [markdown]
# #### 3.2.3 Running the Modified Simulation
# 
# Finally, the fully imported simulation can be run as follows. 
# 
# **Warning**: the network takes time to initialize while the departure positions and times and vehicles are specified.

# %%
flow_params = dict(
    exp_tag='Town04',
    env_name=TestEnv,
    network=Network,
    simulator='traci',
    sim=sim_params,
    env=env_params,
    net=net_params,
    veh=vehicles,
    initial=initial_config,
)

# number of time steps
flow_params['env'].horizon = 500
exp = Experiment(flow_params)

# run the sumo simulation
_ = exp.run(1)


