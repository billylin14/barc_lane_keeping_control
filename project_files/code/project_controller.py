#!/usr/bin python3

# Your import statements here
import numpy as np
from mpclab_common.pytypes import VehicleState, VehiclePrediction
from mpclab_common.track import get_track
from mpclab_controllers.abstract_controller import AbstractController

# The ProjectController class will be instantiated when creating the ROS node.
class ProjectController(AbstractController):
    def __init__(self, dt: float, print_method=print):
        # The control interval is set at 10 Hz
        self.dt = dt

        # If printing to terminal, use self.print_method('some string').
        # The ROS print method will be passed in when instantiating the class
        if print_method is None:
            self.print_method = lambda s: None
        else:
            self.print_method = print_method

        # The state and input prediction object
        self.state_input_prediction = VehiclePrediction()

        # The state and input reference object
        self.state_input_reference = VehiclePrediction()

        # Load the track used in the MPC Lab. The functions for transforming between
        # Global (x, y, psi) and Frenet (s, e_y, e_psi) frames is contained in the returned
        # object. i.e. global_to_local and local_to_global
        self.track = get_track('L_track_barc')
        self.L = self.track.track_length
        self.W = self.track.track_width

        # Example for obtaining the x-y points of the track boundaries
        track_xy = self.track.get_track_xy()
        bound_in_xy = track_xy['bound_in']
        bound_out_xy = track_xy['bound_out'] 

        # Convert x-y points to frenet frame
        bound_in_sey = []
        for _x, _y in zip(bound_in_xy['x'], bound_in_xy['y']):
            _s, _ey, _, = self.track.global_to_local((_x, _y, 0))
            bound_in_sey.append([_s, _ey])

        bound_out_sey = []
        for _x, _y in zip(bound_out_xy['x'], bound_out_xy['y']):
            _s, _ey, _, = self.track.global_to_local((_x, _y, 0))
            bound_out_sey.append([_s, _ey])

    # This method will be called upon starting the control loop
    def initialize(self, vehicle_state: VehicleState):
        self.t0 = vehicle_state.t

    # This method will be called once every time step, make sure to modify the vehicle_state
    # object in place with your computed control actions for acceleration (m/s^2) and steering (rad)
        
    def step(self, vehicle_state: VehicleState):
        #CHANGED BY VINCENT :):):):):)):)):):)
        t = vehicle_state.t - self.t0
        k = self.track.get_curvature(s)
        # Example transformation from global to Frenet frame coordinates
        s, e_y, e_psi = self.track.global_to_local((vehicle_state.x.x, vehicle_state.x.y, vehicle_state.e.psi))
        
        # Modify the vehicle state object in place to pass control inputs to the ROS node
        #P Control
        kp = 1 # p-gain
        x_la = 1 # lookahead term

        g = 9.81   #gravity
        l_f = l_r = 0.13 #left and right axle to CoM
        C_f = C_r = 2.28    #cornering stiffness
        a_y = vehicle_state.a.a_tran 
        W_f = (m*l_r)/(2*L) 
        W_r = (m*l_f)/(2*L)
        R = 1/k     
        L = 0.37 # vehicle length in (m)
        m = 2.2187 # mass in (kg) 
        Kus = W_f/C_f - W_r/C_r # Understeer gradient
        ff = L/R + Kus*(a_y/g)
        beta_ss = l_r*k - k*((l_f*m*vehicle_state.v.v_long**2)/(2*C_r*L))

    
        accel = -1*(vehicle_state.v.v_long - 3)
        steer = -0.85*(e_y + x_la*(e_psi+beta_ss)) + ff

        vehicle_state.u.u_a = accel
        vehicle_state.u.u_steer = steer


        # Example of printing
        self.print_method(f's: {s} | e_y: {e_y} | e_psi: {e_psi}')
        self.print_method(f'Accel: {accel} | Steering: {steer}')

        return

    # This method will be called once every time step. If you would like to visualize
    # the predictions made by your controller, make sure to populate the state_input_prediction
    # object
    def get_prediction(self):
        return self.state_input_prediction

    # This method will be called once every time step. If you would like to visualize
    # some user defined reference, make sure to populate the state_input_reference
    # object
    def get_reference(self):
        return self.state_input_reference
