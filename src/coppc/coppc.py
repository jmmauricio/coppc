#!/usr/bin/env python3
'''
PPC python3 /home/cocoon/shared/cocoon/coppc/src/coppc/coppc.py
'''
import argparse
import logging
from multiprocessing import Process
from pymodbus.client import ModbusTcpClient
from pymodbus.server import StartTcpServer
from pymodbus.datastore import ModbusSequentialDataBlock, ModbusSlaveContext, ModbusServerContext
from threading import Thread
import json
import http.client
import time
import copy
import numpy as np
from colinker.modbus import modbus_client
from fastapi import FastAPI, Response
from coppc.modbus.modbus_client import Modbus_client
from colinker.colinker import Linker
import logging
import uvicorn

# Define a new log level called VERBOSE (between DEBUG and INFO)
VERBOSE_LEVEL = 15  # Midway between DEBUG (10) and INFO (20)
logging.addLevelName(VERBOSE_LEVEL, "VERBOSE")

# Define a custom function to log at the VERBOSE level
def verbose(self, message, *args, **kwargs):
    if self.isEnabledFor(VERBOSE_LEVEL):
        self._log(VERBOSE_LEVEL, message, args, **kwargs)

# Add the custom level to the logging.Logger class
logging.Logger.verbose = verbose

# # Set up basic configuration for logging
# logging.basicConfig(level=logging.DEBUG,   # Set the minimum logging level
#                     format='%(asctime)s - %(levelname)s - %(message)s')

# Get the logger instance
logger = logging.getLogger(__name__)


class PPC(Linker):

    def __init__(self, cfg_dev, cfg_ctrl='') -> None:
        name = 'PPC'
        super().__init__(name, cfg_dev, cfg_ctrl=cfg_ctrl)   
        self.Dt_mid = 0.25 
        self.Dt_meas = 0.05 
        self.Dt_record = 0.02
        self.setup_multiple_device()
        self.setpoints_dict = {'P_POI_ref':1.0, 'Q_POI_ref':0.0, 
                               'record':False, 'stop_ppc':False,
                               'Q_POI_sin_ref_amplitude':0.0,
                               'Q_POI_sin_ref_hz':0.1}
        self.recording = False
        self.N_gen = 2
        self.S_base = 100e6
        self.S_vsc = 1e6
        self.S_plant = 2e6

    def start_api(self):
        app = FastAPI()    

        @app.post("/setpoints")
        async def set_setpoints(received: dict):
            self.setpoints_dict.update(received)

            return Response(content= 'OK', media_type='text/plain')

        print('run uvicorn')
        port = 8200
        uvicorn.run(app,host="0.0.0.0", port = port, log_level='critical')
        print('uvicorn running at port {port}')

    def start_modbus_clients(self):

        # Modbus clients open
        for device in self.devices_list:
            ip = device['modbus_ip']
            port = device['modbus_port']
            mb = Modbus_client(ip,port=port)
            mb.start()
            device.update({'modbus_client':mb})

    def stop_modbus_clients(self):

        # Modbus clients close
        for device in self.devices_list:
            device['modbus_client'].close()


    def p_ctrl_ini(self):
        '''
        Reactive power at POI control initialization
        '''
        self.K_pp = 0.0
        self.K_pi = 0.25
        self.xi_p = 0.0 
        self.p_prev = 0.0 
        self.p_ppc = 0.0 

    def p_ctrl(self, p_ref, p):
        '''
        Reactive power at POI control
        '''
        # Control Law
        epsilon_p = p_ref - p
        self.xi_p += self.Dt_mid*epsilon_p
        self.p_ppc = p_ref + self.p_prev
        self.p_prev = self.K_pp*epsilon_p + self.K_pi*self.xi_p

    def q_ctrl_ini(self):
        '''
        Reactive power at POI control initialization
        '''
        self.K_qp = 0.0
        self.K_qi = 0.5
        self.xi_q = 0.0 
        self.q_prev = 0.0 
        self.q_ppc = 0.0
        self.q_ref_prev = 0.0

    def q_ctrl(self, q_ref, q):
        '''
        Reactive power at POI control
        '''
        # Control Law
        epsilon_q = self.q_ref_prev - q
        self.q_ref_prev = q_ref
        self.q_ppc = self.K_qp*epsilon_q + self.K_qi*self.xi_q + q_ref
        self.xi_q += self.Dt_mid*epsilon_q
        

    def vrs_ini(self):
        '''
        Voltage Regulation System at POI initialization
        '''
        self.K_p = 0.0
        self.K_i = 1
        self.xi_v = 0.0 
        self.q_vrs = 0.0

    def vrs(self, V_ref, V):
        '''
        Voltage Regulation System at POI
        '''
        # Control Law
        V_POI_ref = V_ref
        epsilon_v = V_POI_ref - V
        self.xi_v += self.Dt_mid*epsilon_v
        self.q_vrs = self.K_p*epsilon_v + self.K_i*self.xi_v

    def run(self):

        self.start_modbus_clients()

        N_gen = 2
        S_b = 2e6
        
        t_mid = 0.0
        t_meas = 0.0
        t_rec = 0.0
        t_0 = time.perf_counter()
 
        self.vrs_ini()
        self.p_ctrl_ini()
        self.q_ctrl_ini()
        P_POI = 0.0
        Q_POI = 0.0

        while True:
            
            t = time.perf_counter() - t_0
            if self.setpoints_dict['stop_ppc']:
                print('PPC stopped by user')
                break

            # Setpoints
            p_ref_pu = self.setpoints_dict['P_POI_ref']
            q_ref_sin_pu = self.setpoints_dict['Q_POI_sin_ref_amplitude']*np.sin(2*np.pi*self.setpoints_dict['Q_POI_sin_ref_hz']*t)
            q_ref_pu = self.setpoints_dict['Q_POI_ref'] + q_ref_sin_pu
            

            if t>t_meas:
                t_meas = t + self.Dt_meas
                # Measurements
                for device in self.devices_list:
                    if device['emec_id'] == 'POI':
                        for meas in device['measurements']:
                            if meas['ing_name'] == 'VoltageAVG':
                                modbus_value = device['modbus_client'].read( meas['address'], meas['type'], format = meas['format'])
                                V_POI = modbus_value*meas['emec_scale']
                            if meas['ing_name'] == 'ActivePower':
                                modbus_value = self.S_base*device['modbus_client'].read( meas['address'], meas['type'], format = meas['format'])
                                P_POI = modbus_value*meas['emec_scale'] # W
                            if meas['ing_name'] == 'ReactivePower':
                                modbus_value = self.S_base*device['modbus_client'].read( meas['address'], meas['type'], format = meas['format'])
                                Q_POI = modbus_value*meas['emec_scale'] # var
                            
                            #print(f"ing_name = {meas['ing_name']}, modbus_value = {modbus_value}, {meas['address']}@{device['modbus_ip']}:{device['modbus_port']}")

                V_POI_ref = 1.0
                P_POI_pu = P_POI/self.S_plant
                Q_POI_pu = Q_POI/self.S_plant

            # Powers setpoints
            if t > t_mid:
                t_mid = t + self.Dt_mid

                
                #self.vrs(V_POI_ref, V_POI) 
                self.p_ctrl(p_ref_pu, P_POI_pu) 
                p_ppc_pu = self.p_ppc 

                self.q_ctrl(q_ref_pu, Q_POI_pu) 
                q_ppc_pu = self.q_ppc 


                for device in self.devices_list:
                    for setp in device['setpoints']:
                        if setp['ing_name'] == 'SetActivePower':
                            p_ppc = p_ppc_pu*1e6
                            modbus_value = int(p_ppc)
                            device['modbus_client'].write(modbus_value, setp['address'], setp['type'], format = setp['format'])

                        if setp['ing_name'] == 'SetReactivePower':
                            q_ppc = q_ppc_pu*1e6
                            modbus_value = int(q_ppc)
                            device['modbus_client'].write(modbus_value, setp['address'], setp['type'], format = setp['format'])
                        
                        #print(f"ing_name = {setp['ing_name']}, modbus_value = {modbus_value}, {setp['address']}@{device['modbus_ip']}:{device['modbus_port']}")

                print(f't = {time.perf_counter() - t_0:6.3f}, P_POI = {P_POI/1e6:4.2f} MW, Q_POI = {Q_POI/1e6:4.2f} Mvar, V_POI = {V_POI:0.3f} pu, p_ref_pu = {p_ref_pu:0.2f}, q_ref_pu = {q_ref_pu:0.2f}, recording = {self.recording}')
                

            if t > t_rec:
                t_rec = t + self.Dt_record

         
                if self.setpoints_dict['record'] and not self.recording:
                    self.record = []
                    self.recording = True

                if self.recording:
                    self.record += [{'t':t, 'P_POI':P_POI, 'Q_POI':Q_POI, 'p_ref_pu':p_ref_pu, 'q_ref_pu':q_ref_pu, 'p_ppc':self.p_ppc, 'q_ppc':self.q_ppc}]

                if not self.setpoints_dict['record'] and self.recording:
                    self.recording = False
                    print('Stop recording')
                    with open('record.json', 'w') as fobj:
                        json.dump(self.record, fobj, indent=4)  # 'indent' makes the file readable
    
        self.stop_modbus_clients()

    def start_ctrl(self):
        self.ctrl_loop_thread = Thread(target = self.run)
        self.ctrl_loop_thread.start()
        print('PPC running')
        

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("-cfg_dev", help="id name of the device")
    parser.add_argument("-cfg_ctrl", help="id name of the device")
    args = parser.parse_args()
    cfg_dev_path = args.cfg_dev
    cfg_ctrl_path = args.cfg_ctrl

    ppc = PPC(r'./local_local/config_devices_local_local.json',cfg_ctrl=r'config_controller.json') 
    #ppc = PPC(cfg_dev_path,cfg_ctrl=cfg_ctrl_path) 
    ppc.start_ctrl()
    ppc.start_api()
