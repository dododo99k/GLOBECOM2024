import os, sys
import math, time
from shapely.geometry import shape, Point, LineString, mapping
import numpy as np
import copy
from random import choice
from rtree import index
from itertools import tee
from uplink.path_loss import path_loss_calculator
from uplink.generate_hex import generate_single_site_and_areas
from parameters import *

## wireless hard setting here #############################################
PARAMETERS = {
        'iterations': 1,
        'seed_value1': 1,
        'seed_value2': 2,
        'indoor_users_percentage': 50,
        'los_breakpoint_m': 500,
        'tx_macro_baseline_height': 30,
        'tx_macro_power': 40,
        'tx_macro_gain': 16,
        'tx_macro_losses': 1,
        'tx_micro_baseline_height': 10,
        'tx_micro_power': 24,
        'tx_micro_gain': 5,
        'tx_micro_losses': 1,
        'rx_gain': 4,
        'rx_losses': 4,
        'rx_misc_losses': 4,
        'rx_height': 1.5,
        'building_height': 5,
        'street_width': 20,
        'above_roof': 0,
        'network_load': 50,
        'percentile': 50,
        'sectorization': 3,
        'mnos': 2,
        'asset_lifetime': 10,
        'discount_rate': 3.5,
        'opex_percentage_of_capex': 10,
    }

SPECTRUM_PORTFOLIO = [
    (0.7, 10, '5G'),
    (0.8, 10, '4G'),
    (1.8, 10, '4G'),
    (2.6, 10, '4G'),
    (3.5, 40, '5G'),
    (3.7, 40, '5G'),
    (26, 200, '5G'),
]


MODULATION_AND_CODING_LUT =[
    # ETSI. 2018. ‘5G; NR; Physical Layer Procedures for Data
    # (3GPP TS 38.214 Version 15.3.0 Release 15)’. Valbonne, France: ETSI.
    # CQI Index	Modulation	Coding rate
    # Spectral efficiency (bps/Hz) SINR estimate (dB)
    ('4G', 1, 'QPSK', 78,	0.1523, -6.7),
    ('4G', 2, 'QPSK', 120, 0.2344, -4.7),
    ('4G', 3, 'QPSK', 193, 0.377, -2.3),
    ('4G', 4, 'QPSK', 308, 0.6016, 0.2),
    ('4G', 5, 'QPSK', 449, 0.877, 2.4),
    ('4G', 6, 'QPSK', 602, 1.1758, 4.3),
    ('4G', 7, '16QAM', 378, 1.4766, 5.9),
    ('4G', 8, '16QAM', 490, 1.9141, 8.1),
    ('4G', 9, '16QAM', 616, 2.4063, 10.3),
    ('4G', 10, '64QAM', 466, 2.7305, 11.7),
    ('4G', 11, '64QAM', 567, 3.3223, 14.1),
    ('4G', 12, '64QAM', 666, 3.9023, 16.3),
    ('4G', 13, '64QAM', 772, 4.5234, 18.7),
    ('4G', 14, '64QAM', 973, 5.1152, 21),
    ('4G', 15, '64QAM', 948, 5.5547, 22.7),
    ('5G', 1, 'QPSK', 78, 0.1523, -6.7),
    ('5G', 2, 'QPSK', 193, 0.377, -4.7),
    ('5G', 3, 'QPSK', 449, 0.877, -2.3),
    ('5G', 4, '16QAM', 378, 1.4766, 0.2),
    ('5G', 5, '16QAM', 490, 1.9141, 2.4),
    ('5G', 6, '16QAM', 616, 2.4063, 4.3),
    ('5G', 7, '64QAM', 466, 2.7305, 5.9),
    ('5G', 8, '64QAM', 567, 3.3223, 8.1),
    ('5G', 9, '64QAM', 666, 3.9023, 10.3),
    ('5G', 10, '64QAM', 772, 4.5234, 11.7),
    ('5G', 11, '64QAM', 873, 5.1152, 14.1),
    ('5G', 12, '256QAM', 711, 5.5547, 16.3),
    ('5G', 13, '256QAM', 797, 6.2266, 18.7),
    ('5G', 14, '256QAM', 885, 6.9141, 21),
    ('5G', 15, '256QAM', 948, 7.4063, 22.7),
]

ENVIRONMENTS = ['urban', 'suburban', 'rural']

class Uplink:
    def __init__(self, environment='urban', ant_type='micro', 
                 site_coord=[0,0], site_radius=500, 
                 frequency=3.5, bandwidth=1, generation='5G', 
                 simulation_parameters=PARAMETERS, modulation_coding=MODULATION_AND_CODING_LUT):
        # parameters for simulation
        self.frequency = frequency
        self.bandwidth = bandwidth
        self.generation = generation
        self.environment = environment
        self.ant_type = ant_type
        self.site_coord = site_coord
        self.site_radius = site_radius
        self.time = 0
        self.parameters = simulation_parameters
        self.modulation_coding = modulation_coding
        # self.broadcast_buffer = 0

        # print simulation information
        # print('simulation with: env {}, ant {}, range {}, gen {}, feq {}, bw {}'
        #       .format(self.environment, self.ant_type, self.site_radius, 
        #               self.generation, self.frequency, self.bandwidth)
        #      )

        ################################# generate network setups ########################

        # the transmitter is the site (base station), which is stationary 
        transmitter, interfering_transmitters, site_area, interfering_site_areas = \
            generate_single_site_and_areas(self.site_coord, self.site_radius)   

        self.transmitter = Transmitter(transmitter, ant_type, simulation_parameters)
        self.interfering_transmitters = {}
        self.receivers = {}
        self.site_area = SiteArea(site_area[0])

        for interfering_transmitter in interfering_transmitters:
            site_id = interfering_transmitter['properties']["site_id"]
            site_object = InterferingTransmitter(interfering_transmitter, ant_type, simulation_parameters)
            self.interfering_transmitters[site_id] = site_object

    ########################## update information of wireless ############################################
    def add_in_receiver(self, vid, coords):
        receiver = create_single_receiver_coord(PARAMETERS, vid, coords, self.site_coord)
        receiver_id = str(receiver['properties']["ue_id"])
        receiver = Receiver(receiver, self.parameters)
        self.receivers[receiver_id] = receiver

    def enqueue_task(self, task): # queue task 
        self.receivers[str(task.vid)].tasks.append(task) 

    # def update_broadcast_buffer(self, incoming_size):
    #     self.broadcast_buffer += incoming_size

    def get_num_of_total_receivers(self):
        return len(self.receivers)

    def update_receiver_task_buffer(self, vid, task_size):
        num_of_tasks_in_vehicle = len(self.receivers[str(vid)].tasks)
        for task in self.receivers[str(vid)].tasks: # multi task share data rate of this vehicle
            task.remain_size_ul = np.clip(task.remain_size_ul - task_size/num_of_tasks_in_vehicle, 0, None)

    def update_receiver_path_loss(self,):
        for receiver in self.receivers.values():
            path_loss, _, _, _ = self.estimate_path_loss(
                receiver, self.frequency, self.environment, self.parameters
            )
            receiver.path_loss = path_loss 

    def update_receivers_activations(self,):
        count = 0
        for receiver in self.receivers.values():
            if len(receiver.tasks) > 0:
                receiver.active = True
                count += 1
            else:
                receiver.active = False
    
        return count

    ########################## run simulation of wireless ################################################
    def step(self,):

        self.time += 1

        # increase total time for each task in the queue
        for receiver in self.receivers.values():
            for task in receiver.tasks:
                task.total_time += 1
                task.time_ul_transmit += 1

        self.update_receiver_path_loss() # get path loss for all receivers

        trans_completion = []

        num_of_active_users = self.update_receivers_activations()
        if num_of_active_users <= 0: return trans_completion
        indiv_bandwidth = self.bandwidth / num_of_active_users

        # runt he simulation and get the data rate
        results = self.estimate_link_budget(
            self.frequency, indiv_bandwidth, self.generation, self.ant_type,
            self.environment, self.modulation_coding, self.parameters
            )

        for result in results:
            vid = result['id']
            outgoing_size = result["capacity_mbps"] * 1000 # calculate outgoing size # this is Mbits, and time unit is millisecond, the data size is kbits
            self.update_receiver_task_buffer(vid, outgoing_size)
            # only output the receiver that has buffer size at the begining of current time
            
            # check if all receivers are broadcasted
            receiver = self.receivers[str(vid)]
            for task in receiver.tasks:
                if task.remain_size_ul <= 0:
                    trans_completion.append(copy.deepcopy(task))
                    # reset the reciever metric
                    receiver.tasks.remove(task)

        np.random.seed(int(time.time()))
        return trans_completion

    def estimate_link_budget(self, frequency, bandwidth,
        generation, ant_type, environment,
        modulation_and_coding_lut, simulation_parameters):
        """

        Takes propagation parameters and calculates link budget capacity.

        Parameters
        ----------
        frequency : float
            The carrier frequency for the chosen spectrum band (GHz).
        bandwidth : int
            The bandwidth of the carrier frequency (MHz).
        generation : string
            Either 4G or 5G dependent on technology.
        ant_type : str
            Type of antenna (macro, small etc.).
        environment : string
            Either urban, suburban or rural.
        modulation_and_coding_lut : list of tuples
            A lookup table containing modulation and coding rates,
            spectral efficiencies and SINR estimates.
        simulation_parameters : dict
            A dict containing all simulation parameters necessary.

        Returns
        -------
        results : List of dicts
            Each dict is an individual simulation result.

        """

        results = []

        for receiver in self.receivers.values():

            path_loss, r_model, r_distance, type_of_sight = self.estimate_path_loss(
                receiver, frequency, environment, simulation_parameters
            )

            received_power = self.estimate_received_power(self.transmitter,
                receiver, path_loss
            ) + int(np.random.rand()*50)/10 # TODO small scale emulation only with random

            interference, i_model, ave_distance, ave_inf_pl = self.estimate_interference(
                receiver, frequency, environment, simulation_parameters)

            noise = self.estimate_noise(
                bandwidth
            )

            f_received_power, f_interference, f_noise, i_plus_n, sinr = \
                self.estimate_sinr(received_power, interference, noise,
                simulation_parameters
                )

            spectral_efficiency = self.estimate_spectral_efficiency(
                sinr, generation, modulation_and_coding_lut
            )

            capacity_mbps, capacity_mbps_km2 = (
                self.estimate_average_capacity(
                bandwidth, spectral_efficiency)
            )

            results.append({
                'id': receiver.id,
                'path_loss': path_loss,
                'r_model': r_model,
                'type_of_sight': type_of_sight,
                'ave_inf_pl': ave_inf_pl,
                'received_power': f_received_power,
                'distance': r_distance,
                'interference': np.log10(f_interference),
                'i_model': i_model,
                'network_load': simulation_parameters['network_load'],
                'ave_distance': ave_distance,
                'noise': f_noise,
                'i_plus_n': np.log10(i_plus_n),
                'sinr': sinr,
                'spectral_efficiency': spectral_efficiency,
                'capacity_mbps': capacity_mbps,
                'capacity_mbps_km2': capacity_mbps_km2,
                'receiver_x': receiver.coordinates[0],
                'receiver_y': receiver.coordinates[1],
                })

        return results


    def estimate_path_loss(self, receiver, frequency,environment,
        simulation_parameters):
        """

        Function to calculate the path loss between a transmitter
        and receiver.

        Parameters
        ----------
        receiver : object
            Receiving User Equipment (UE) item.
        frequency : float
            The carrier frequency for the chosen spectrum band (GHz).
        environment : string
            Either urban, suburban or rural.
        seed_value : int
            Set seed value for quasi-random number generator.
        iterations : int
            The number of stochastic iterations for the specific point.
        los_breakpoint_m : int
            The breakpoint over which propagation becomes non line of sight.

        Returns
        -------
        path_loss : float
            Estimated path loss in decibels between the transmitter
            and receiver.
        model : string
            Specifies which propagation model was used.
        strt_distance : int
            States the straight line distance in meters between the
            transmitter and receiver.
        type_of_sight : string
            Either Line of Sight or None Line of Sight.

        """
        temp_line = LineString(
            [
                (receiver.coordinates[0],
                receiver.coordinates[1]),
                (self.transmitter.coordinates[0],
                self.transmitter.coordinates[1])
            ]
        )

        strt_distance = temp_line.length

        if strt_distance < 20:
            strt_distance = 20

        ant_height = self.transmitter.ant_height
        ant_type =  self.transmitter.ant_type

        if strt_distance < simulation_parameters['los_breakpoint_m'] :
            type_of_sight = 'los'
        else:
            type_of_sight = 'nlos'

        path_loss, model = path_loss_calculator(
            frequency,
            strt_distance,
            ant_height,
            ant_type,
            simulation_parameters['building_height'],
            simulation_parameters['street_width'],
            environment,
            type_of_sight,
            receiver.ue_height,
            simulation_parameters['above_roof'],
            receiver.indoor,
            simulation_parameters['seed_value1'],
            simulation_parameters['iterations']
        )

        return path_loss, model, strt_distance, type_of_sight


    def estimate_received_power(self, transmitter, receiver, path_loss):
        """

        Calculate received power based on transmitter and receiver
        characteristcs, and path loss.

        Equivalent Isotropically Radiated Power (EIRP) = (
            Power + Gain - Losses
        )

        Parameters
        ----------
        transmitter : object
            Radio transmitter.
        receiver : object
            Receiving User Equipment (UE) item.
        path_loss : float
            Estimated path loss in decibels between the transmitter
            and receiver.

        Returns
        -------
        received_power : float
            UE received power.

        """
        #calculate Equivalent Isotropically Radiated Power (EIRP)
        eirp = (
            float(self.transmitter.power) +
            float(self.transmitter.gain) -
            float(self.transmitter.losses)
        )

        received_power = ( eirp -
            path_loss -
            receiver.misc_losses +
            receiver.gain -
            receiver.losses
        )

        return received_power


    def estimate_interference(self, receiver, frequency, environment,
        simulation_parameters):
        """
        Calculate interference from other sites.

        closest_sites contains all sites, ranked based
        on distance, meaning we need to select sites 1-3 (as site 0
        is the actual site in use)

        Parameters
        ----------
        receiver : object
            Receiving User Equipment (UE) item.
        frequency : float
            The carrier frequency for the chosen spectrum band (GHz).
        environment : string
            Either urban, suburban or rural.
        seed_value : int
            Set seed value for quasi-random number generator.
        iterations : int
            The number of stochastic iterations for the specific point.

        Returns
        -------
        interference : List
            Received interference power in decibels at the receiver.
        model : string
            Specifies which propagation model was used.
        ave_distance : float
            The average straight line distance in meters between the
            interfering transmitters and receiver.
        ave_pl : string
            The average path loss in decibels between the interfering
            transmitters and receiver.

        """
        interference = []

        ave_distance = 0
        ave_pl = 0

        for interfering_transmitter in self.interfering_transmitters.values():


            temp_line = LineString(
                [
                    (receiver.coordinates[0],
                    receiver.coordinates[1]),
                    (interfering_transmitter.coordinates[0],
                    interfering_transmitter.coordinates[1])
                ]
            )

            interference_strt_distance = temp_line.length
            if interference_strt_distance < 20:
                interference_strt_distance == 20

            ant_height = interfering_transmitter.ant_height
            ant_type =  interfering_transmitter.ant_type

            if interference_strt_distance < simulation_parameters['los_breakpoint_m']:
                type_of_sight = 'los'
            else:
                type_of_sight = 'nlos'

            path_loss, model = path_loss_calculator(
                frequency,
                interference_strt_distance,
                ant_height,
                ant_type,
                simulation_parameters['building_height'],
                simulation_parameters['street_width'],
                environment,
                type_of_sight,
                receiver.ue_height,
                simulation_parameters['above_roof'],
                receiver.indoor,
                simulation_parameters['seed_value2'],
                simulation_parameters['iterations'],
            )

            received_interference = self.estimate_received_power(
                interfering_transmitter,
                receiver,
                path_loss
            )

            ave_distance += interference_strt_distance
            ave_pl += path_loss

            interference.append(received_interference)

        ave_distance = (
            ave_distance / len(self.interfering_transmitters.values())
        )

        ave_pl = (
            ave_pl / len(self.interfering_transmitters.values())
        )

        return interference, model, ave_distance, ave_pl


    def estimate_noise(self, bandwidth):
        """

        Estimates the potential noise at the UE receiver.

        Terminal noise can be calculated as:

        “K (Boltzmann constant) x T (290K) x bandwidth”.

        The bandwidth depends on bit rate, which defines the number
        of resource blocks. We assume 50 resource blocks, equal 9 MHz,
        transmission for 1 Mbps downlink.

        Required SNR (dB)
        Detection bandwidth (BW) (Hz)
        k = Boltzmann constant
        T = Temperature (Kelvins) (290 Kelvin = ~16 degrees celcius)
        NF = Receiver noise figure (dB)

        NoiseFloor (dBm) = 10log10(k * T * 1000) + NF + 10log10BW

        NoiseFloor (dBm) = (
            10log10(1.38 x 10e-23 * 290 * 1x10e3) + 1.5 + 10log10(10 x 10e6)
        )

        Parameters
        ----------
        bandwidth : int
            The bandwidth of the carrier frequency (MHz).

        Returns
        -------
        noise : float
            Received noise at the UE receiver in decibels

        """
        k = 1.38e-23
        t = 290
        BW = bandwidth*1000000

        noise = 10 * np.log10(k * t * 1000) + 1.5 + 10 * np.log10(BW)

        return noise


    def estimate_sinr(self, received_power, interference, noise,
        simulation_parameters):
        """

        Calculate the Signal-to-Interference-plus-Noise-Ratio (SINR).

        Parameters
        ----------
        received_power : float
            UE received power in decibels.
        interference : List
            Received interference power in decibels at the receiver.
        noise : float
            Received noise at the UE receiver in decibels
        simulation_parameters : dict
            A dict containing all simulation parameters necessary.

        Returns
        -------
        received_power : float
            UE received power in decibels.
        raw_sum_of_interference : float
            Linear values of summed interference at the receiver in decibels.
        noise : float
            Received noise at the UE receiver in decibels.
        i_plus_n : float
            Linear sum of interference plus noise in decibels.
        sinr : float
            Signal-to-Interference-plus-Noise-Ratio (SINR) in decibels.

        """
        raw_received_power = 10**received_power

        interference_list = []
        for value in interference:
            output_value = 10**value
            interference_list.append(output_value)

        interference_list.sort(reverse=True)
        interference_list = interference_list[:3]

        network_load = simulation_parameters['network_load']
        i_summed = sum(interference_list)
        raw_sum_of_interference = i_summed * (network_load/100)

        raw_noise = 10**noise

        i_plus_n = (raw_sum_of_interference + raw_noise)

        sinr = round(np.log10(
            raw_received_power / i_plus_n
            ),2)

        return received_power, raw_sum_of_interference, noise, i_plus_n, sinr


    def estimate_spectral_efficiency(self, sinr, generation,
        modulation_and_coding_lut):
        """
        Uses the SINR to determine spectral efficiency given the relevant
        modulation and coding scheme.

        Parameters
        ----------
        sinr : float
            Signal-to-Interference-plus-Noise-Ratio (SINR) in decibels.
        generation : string
            Either 4G or 5G dependent on technology.
        modulation_and_coding_lut : list of tuples
            A lookup table containing modulation and coding rates,
            spectral efficiencies and SINR estimates.

        Returns
        -------
        spectral_efficiency : float
            Efficiency of information transfer in Bps/Hz

        """
        spectral_efficiency = 0.1
        for lower, upper in pairwise(modulation_and_coding_lut):
            if lower[0] and upper[0] == generation:

                lower_sinr = lower[5]
                upper_sinr = upper[5]

                if sinr >= lower_sinr and sinr < upper_sinr:
                    spectral_efficiency = lower[4]
                    return spectral_efficiency

                highest_value = modulation_and_coding_lut[-1]
                if sinr >= highest_value[5]:

                    spectral_efficiency = highest_value[4]
                    return spectral_efficiency


                lowest_value = modulation_and_coding_lut[0]

                if sinr < lowest_value[5]:

                    spectral_efficiency = 0
                    return spectral_efficiency


    def estimate_average_capacity(self, bandwidth, spectral_efficiency):
        """
        Estimate link capacity based on bandwidth and received signal.

        Parameters
        ----------
        bandwidth : int
            Channel bandwidth in MHz
        spectral_efficiency : float
            Efficiency of information transfer in Bps/Hz

        Returns
        -------
        capacity_mbps : float
            Average link budget capacity in Mbps.
        capacity_mbps_km2 : float
            Average site area capacity in Mbps km^2.

        """
        bandwidth_in_hertz = bandwidth * 1e6 #MHz to Hz

        capacity_mbps = (
            (bandwidth_in_hertz * spectral_efficiency) / 1e6
        )

        capacity_mbps_km2 = (
            capacity_mbps / (self.site_area.area / 1e6)
        )

        return capacity_mbps, capacity_mbps_km2


    def receiver_density(self):
        """

        Calculate receiver density per square kilometer (km^2)

        Returns
        -------
        receiver_density : float
            Density of receivers per square kilometer (km^2).

        Notes
        -----
        Function returns `0` when no receivers are configered to the area.

        """
        if not self.receivers:
            return 0

        postcode_sector_area = (
            [round(a.area) for a in self.area.values()]
            )[0]

        receiver_density = (
            len(self.receivers) / (postcode_sector_area/1000000)
            )

        return receiver_density


class Transmitter(object):
    """

    Radio transmitter object.

    Parameters
    ----------
    data : dict
        Contains all object data parameters.
    simulation_parameters : dict
        A dict containing all simulation parameters necessary.

    """
    def __init__(self, data, ant_type, simulation_parameters):

        self.id = data['properties']['site_id']
        self.coordinates = data['geometry']['coordinates']

        self.ant_type = ant_type

        if ant_type == 'macro':
            self.ant_height = simulation_parameters['tx_macro_baseline_height']
            self.power = simulation_parameters['tx_macro_power']
            self.gain = simulation_parameters['tx_macro_gain']
            self.losses = simulation_parameters['tx_macro_losses']

        if ant_type == 'micro':
            self.ant_height = simulation_parameters['tx_micro_baseline_height']
            self.power = simulation_parameters['tx_micro_power']
            self.gain = simulation_parameters['tx_micro_gain']
            self.losses = simulation_parameters['tx_micro_losses']


class InterferingTransmitter(object):
    """

    A site object is specific site.

    Parameters
    ----------
    data : dict
        Contains all object data parameters.
    simulation_parameters : dict
        A dict containing all simulation parameters necessary.

    """
    def __init__(self, data, ant_type, simulation_parameters):

        self.id = data['properties']['site_id']
        self.coordinates = data['geometry']['coordinates']

        self.ant_type = ant_type

        if ant_type == 'macro':
            self.ant_height = simulation_parameters['tx_macro_baseline_height']
            self.power = simulation_parameters['tx_macro_power']
            self.gain = simulation_parameters['tx_macro_gain']
            self.losses = simulation_parameters['tx_macro_losses']

        if ant_type == 'micro':
            self.ant_height = simulation_parameters['tx_micro_baseline_height']
            self.power = simulation_parameters['tx_micro_power']
            self.gain = simulation_parameters['tx_micro_gain']
            self.losses = simulation_parameters['tx_micro_losses']


class Receiver(object):
    """

    Radio receiver object (UE).

    Parameters
    ----------
    data : dict
        Contains all object data parameters.
    simulation_parameters : dict
        A dict containing all simulation parameters necessary.

    """
    def __init__(self, data, simulation_parameters):
        self.id = data['properties']['ue_id']
        self.coordinates = data['geometry']["coordinates"]

        self.ue_height = data['properties']['ue_height']
        self.gain = data['properties']['gain']
        self.losses = data['properties']['losses']
        self.misc_losses = data['properties']['misc_losses']
        self.indoor = data['properties']['indoor']

        self.active = data['information']['active']
        self.tasks = data['tasks']

class SiteArea(object):
    """

    site area object.

    Parameters
    ----------
    data : dict
        Contains all object data parameters.

    """
    def __init__(self, data):
        self.id = data['properties']['site_id']
        self.geometry = data['geometry']
        self.coordinates = data['geometry']['coordinates']
        self.area = self._calculate_area(data)


    def _calculate_area(self, data):
        polygon = shape(data['geometry'])
        area = polygon.area
        return area


def pairwise(iterable):
    """

    Return iterable of 2-tuples in a sliding window

    Parameters
    ----------
    iterable: list
        Sliding window

    Returns
    -------
    list of tuple
        Iterable of 2-tuples

    Example
    -------
        >>> list(pairwise([1,2,3,4]))
            [(1,2),(2,3),(3,4)]

    """
    a, b = tee(iterable)
    next(b, None)
    return zip(a, b)


def create_single_receiver_coord(parameters, vid, coord, centroid=[0,0]):
    """

    Generate receiver locations as points.

    Parameters
    ----------
    parameters : dict
        Contains all necessary simulation parameters.

    Output
    ------
    receivers : List of dicts
        Contains the quantity of desired receivers.

    """

    path = LineString([(coord), (centroid)])
    length = int(path.length)
    receiver = {
        'type': "Feature",
        'geometry': {
            "type": "Point",
            "coordinates": coord,
        },
        'properties': {
            'ue_id': vid,
            "misc_losses": parameters['rx_misc_losses'],
            "gain": parameters['rx_gain'],
            "losses": parameters['rx_losses'],
            "ue_height": float(parameters['rx_height']),
            "indoor": (False),
        },
        'information': {
            'active': True,
        },
        'tasks': [],
    }

    return receiver

def generate_receivers_w_coords(parameters, coordinates, centroid=[0,0]):
    """

    Generate receiver locations as points.

    Parameters
    ----------
    parameters : dict
        Contains all necessary simulation parameters.

    Output
    ------
    receivers : List of dicts
        Contains the quantity of desired receivers.

    """
    receivers = []

    id_number = 0
    for coord in coordinates:
        path = LineString([(coord), (centroid)])
        length = int(path.length)
        receivers.append({
            'type': "Feature",
            'geometry': {
                "type": "Point",
                "coordinates": coord,
            },
            'properties': {
                'ue_id': "id_{}".format(id_number),
                "misc_losses": parameters['rx_misc_losses'],
                "gain": parameters['rx_gain'],
                "losses": parameters['rx_losses'],
                "ue_height": float(parameters['rx_height']),
                "indoor": (False),
            },
            'information': {
                'buffer':0,
                'active': True,
            }
        })
        id_number += 1

    return receivers

    ##############
    # def add_in_receiver(self, vid, coord):
    #     # create a receiver based on coords an vid
    #     receiver = create_single_receiver_coord(PARAMETERS, vid, initial_coords, self.site_coord)
    #     self.curr_id += 1 # increase the 
    #     self.simulator.add_in_receiver(receiver) # add the receiver into self.simulator

    # def add_in_buffer(self, vid, task_size):
    #     # assert(len(task_size) == self.max_users)
    #     # add the receivers into the simulator class, mandatory
    #     self.simulator.update_receiver_buffer(vid, task_size)

    # def update_location(self, vid, location):
    #     # assert(len(locations) == self.max_users)
    #     # add the receivers into the simulator class, mandatory
    #     self.simulator.update_receiver_location(vid, location)

    # def run_simulator_get_datarate(self,):
    #     assert(len(self.simulator.receivers)) > 0) # make sure there are at least a receiver

    #     bandwidth_per_user = self.bandwidth/self.simulator.get_num_of_active_receivers()

    #     # runt he simulation and get the data rate
    #     results = self.simulator.estimate_link_budget(
    #         self.frequency, bandwidth_per_user, self.generation, self.ant_type,
    #         self.environment,
    #         MODULATION_AND_CODING_LUT,
    #         PARAMETERS
    #         )

    #     # sort out the simulation results
    #     throughput, distance, path_loss, interference = [], [], [], []
    #     for result in results:
    #         throughput.append(result["capacity_mbps"])
    #         distance.append(result["distance"])
    #         path_loss.append(result["path_loss"])
    #         interference.append(result["interference"])

    #     # print the simulation results
    #     print("average throughput, distance are: ", np.mean(throughput), np.mean(distance))

    #     # update buffer size since we have transmitted some of them
    #     outgoing_size = np.array(throughput)*1*1000 # since throughput is Mbps

    #     self.simulator.update_receivers_buffers(-outgoing_size)

    #     remaining_buffers = np.array([receiver.buffer for receiver in self.simulator.receivers.values()])

    #     print("avg remaining buffers is :", np.mean(remaining_buffers))

    #     trans_completion = remaining_buffers == 0
        
    #     return trans_completion # remaining_buffers  # return number of bits sent for each user


##################################################################################################

# wireless = Wireless()

# num_of_users = 100

# # initial all vehicles
# vids = []
# for idx in range(num_of_users):
#     vids.append("id_"+str(idx))
#     wireless.add_in_receiver(vids[idx], [0,0]) 



# avg_trans_completion = []

# for _ in range(10):
#     Coords = np.random.randint(0, 700, [num_of_users, 2])
#     Sizes = np.random.randint(0, 1000, num_of_users) # unit is number of bits

#     for idx in range(num_of_users):
#         wireless.update_receiver_location(vids[idx],Coords[idx])
#         wireless.update_receiver_buffer(vids[idx], Sizes[idx])

#     avg_trans_completion.append(wireless.run_simulator_get_datarate())


# avg_trans_completion = np.array(avg_trans_completion)

# print(np.mean(avg_trans_completion, axis=1))