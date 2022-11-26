"""Abstraction model for interfacing with Waveshare SIM7080X NB-IoT HAT
"""
import logging
import math
import os
from enum import Enum
from time import sleep
from typing import TypedDict

import serial
from gpiozero import DigitalOutputDevice

_log = logging.getLogger(__name__)


class DataService(Enum):
    """Data service aka ip_type used for +CGDCONT and +CGNCFG"""
    DUAL_PDN: 0
    IP_V4: 1
    IP_V6: 2
    NONIP: 3
    EX_NONIP: 4
    
    def __repr__(self):
        if self.name == 'NONIP':
            return 'Non-IP'
        elif self.name == 'IP_V4':
            return 'IP'
        elif self.name == 'IP_V6':
            return 'IPV6'
        elif self.name == 'DUAL_PDN':
            return 'IPV4V6'
    
    @property
    def is_ip(self) -> bool:
        if self.name in ['NONIP', 'EX_NONIP']:
            return False
        return True


class AuthType(Enum):
    NONE: 0
    PAP: 1
    CHAP: 2
    PAP_CHAP: 3


class PdpContext:
    def __init__(self,
                 id: int,
                 apn: str,
                 data_service: DataService,
                 ) -> None:
        self.id: int = id or 0
        self.apn: str = apn or ''
        self.data_service: DataService = data_service or DataService.IP
        self.ip_address: 'str|None' = None
        self.configured: bool = False
        self.username: str = None
        self.password: str = None
        self.auth: AuthType = None
    
    @property
    def is_ip(self) -> bool:
        return self.data_service.is_ip


class PdpAction(Enum):
    DEACTIVATE: 0
    ACTIVATE: 1
    AUTO_ACTIVATE: 2


class WaveshareNbiotHat:
    """A Waveshare SIM7080X NB-IoT HAT."""
    def __init__(self, uart: str = '/dev/ttyS0', power_pin: int = 4) -> None:
        self._baudrate: int = 9600
        self.uart = serial.Serial(uart, self._baudrate, timeout=1.0)
        self.power_pin = DigitalOutputDevice(power_pin)
        self._ready: bool = None
        self._rf_enabled: bool = None
        self._pdp_contexts: dict = {}
        self._active_pdp_context: int = None
        self._mqtt_connected: bool = True
        # self._at_queue = None
    
    @property
    def active_pdp_context(self) -> int:
        return self._active_pdp_context
    
    @property
    def pdp_contexts(self) -> dict:
        return self._pdp_contexts
    
    def power_on(self):
        _log.debug('Powering on SIM7080G')
        self.power_pin.blink(1, 5, n=1)
    
    def power_off(self):
        _log.debug('Powering off SIM7080G')
        self.power_pin.blink(2, 5, n=1)
    
    def initialize(self, max_attempts: int = 3) -> bool:
        responsive = False
        attempt = 0
        while not responsive and attempt < max_attempts:
            attempt += 1
            res = self.at_command('AT', timeout=1)
            if 'OK' in res:
                responsive = True
            else:
                _log.debug('Attempting module power on')
                self.power_on()
        self._ready = responsive
        return self._ready
    
    @property
    def ready(self) -> bool:
        if self._ready is None:
            self.initialize()
        return self._ready
        
    def at_command(self, command: str, timeout: float = 0.1) -> 'list[str]':
        if self.uart.in_waiting:
            buffer = self.uart.read(self.uart.in_waiting)
            debug = buffer.decode().replace('\r', '<cr>').replace('\n', '<lf>')
            _log.warning(f'Buffer contained: {debug}')
        self.uart.reset_input_buffer()
        self.uart.write(f'{command}\r'.encode())
        sleep(timeout)
        response: 'list[str]' = []
        while self.uart.in_waiting:
            line = self.uart.read_until('\r\n'.encode()).decode()
            if line.strip():
                response.append(line.strip())
        _log.debug(f'Command {command} response: {response}')
        return response
    
    def disable_rf(self) -> bool:
        res = self.at_command('AT+CFUN=0')
        if 'OK' not in res:
            _log.error(f'Error: {res}')
            return False
        if '+CPIN: NOT READY' not in res:
            return False
        return True
            
    def enable_rf(self) -> bool:
        res = self.at_command('AT+CFUN=1')
        if 'OK' not in res:
            _log.error(f'Error: {res}')
            return False
        if '+CPIN: READY' not in res:
            return False
        return True
            
    def pdp_context_define(self,
                           id: int,
                           apn: str,
                           data_service: DataService = DataService.IP_V4) -> None:
        if not isinstance(id, int) or id not in range(0,4):
            raise ValueError('PDP context ID must be 0-3')
        if not isinstance(apn, str) or not apn:
            raise ValueError('Invalid apn name')
        if not isinstance(data_service, DataService):
            raise ValueError('Invalid data_service')
        # TODO: +CGDCONT seems redundant to +CNCFG
        if not self.disable_rf():
            raise RuntimeError('Could not disable RF')
        res = self.at_command(f'AT+CGDCONT={id},"{str(data_service)}","{apn}"')
        # TODO: allows specification of static PDP_addr, d_comp, h_comp, ipv4_ctrl, emergency_flag
        if 'OK' not in res:
            raise ValueError(f'Error: {res}')
        if not self.enable_rf():
            raise RuntimeError('Could not enable RF')
        if data_service.is_ip:
            res = self.at_command('AT+GCATT?')
            if '+CGATT: 1' not in res:
                raise SystemError('Packet service not attached')
        # TODO: unclear if +GGNAPN is required seems to indicate registered
        res = self.at_command('AT+GCNAPN')
        if f'+CGNAPN: {id},"{apn}"' not in res:
            raise ValueError(f'Error: {res}')
        
    def pdp_context_configure(self,
                              id: int,
                              apn: str = None,
                              data_service: DataService = None,
                              username: str = '',
                              password: str = '',
                              auth: 'AuthType|None' = None) -> None:
        if id not in self._pdp_contexts:
            raise ValueError(f'PDP context ID {id} not defined')
        pdp_context: PdpContext = self._pdp_contexts[id]
        if not apn:
            apn = pdp_context.apn
        if not data_service:
            data_service = pdp_context.data_service
        # TODO: unclear if +CGCFG required if no username/password/auth
        config_command = f'AT+CNCFG={id},"{str(data_service)}","{apn}"'
        if username:
            config_command += f',{username}'
            if password:
                config_command += f',{password}'
        if auth:
            if not username and not password:
                config_command += ',,'
            config_command += f',{auth.value}'
        res = self.at_command(config_command)
        if 'OK' not in res:
            raise ValueError(f'Error: {res}')
        pdp_context.username = username
        pdp_context.password = password
        pdp_context.auth = auth
        pdp_context.configured = True
    
    def pdp_context_activate(self,
                             pdp_context_id: int = 0,
                             action: PdpAction = PdpAction.ACTIVATE) -> bool:
        if pdp_context_id not in self._pdp_contexts:
            raise ValueError(f'PDP context ID {pdp_context_id} not defined')
        pdp_context: PdpContext = self._pdp_contexts[pdp_context_id]
        if not pdp_context.configured:
            raise ValueError(f'PDP context ID {pdp_context_id} not configured')
        res = self.at_command(f'AT+CNACT={pdp_context_id},{action.value}')
        if f'+APP PDP: {pdp_context_id},{action.name}' not in res:
            _log.error(f'Failed to confirm PDP context actvation: {res}')
        if pdp_context.is_ip:
            if action == PdpAction.DEACTIVATE:
                pdp_context.ip_address = None
                self._active_pdp_context = None
            else:
                res = self.at_command('AT+CNACT?')
                for line in res:
                    if line.startswith(f'+CNACT: {pdp_context_id}'):
                        pdp_context.ip_address = (
                            line.split(',')[2].replace('"', ''))
                        self._active_pdp_context = pdp_context_id
                        break
    
    def _put_file_in_flash(self, filename: str, flashname: str) -> None:
        """"""
        res = self.at_command('AT+CFSINIT')
        if not 'OK' in res:
            raise SyntaxError(f'Could not initialize filesystem: {res}')
        size = os.path.getsize(filename)
        res = self.at_command(f'AT+CFSWFILE=3,"{flashname}",0,{size},1000')
        if 'DOWNLOAD' not in res:
            raise SystemError(f'Error opening download: {res}')
        self.uart.write(open(filename, 'rb').read())
        sleep(math.ceil(size * 8 / self._baudrate))   #: TODO is this necessary?
        if self.uart.in_waiting:
            res = self.uart.read(self.uart.in_waiting).decode().strip()
            if 'OK' not in res:
                raise SystemError(f'Error downloading file: {res}')
        res = self.at_command('AT+CFSTERM')
        if not 'OK' in res:
            raise SystemError(f'Failed to close file buffer: {res}')
        
    def pdp_context_configure_ssl(self,
                                  ca_file: str,
                                  cert_file: str,
                                  key_file: str):
        """"""
        if not os.path.isfile(ca_file):
            raise FileNotFoundError(f'{ca_file} root CA not found')
        if not os.path.isfile(cert_file):
            raise FileNotFoundError(f'{cert_file} certificate not found')
        if not os.path.isfile(key_file):
            raise FileNotFoundError(f'{key_file} key not found')
        if not self._active_pdp_context:
            raise SystemError('No active PDP context')
        if not self._pdp_contexts[self._active_pdp_context].ip_address:
            raise SystemError('No valid IP address')
        sync = False
        res = self.at_command('AT+CCLK?')
        for line in res:
            if '+CCLK: "' in line:
                sync = True
                break
        if not sync:
            raise SystemError(f'Time not synchronized: {res}')
        self._put_file_in_flash(ca_file, 'ca.crt')
        self._put_file_in_flash(cert_file, 'client.crt')
        self._put_file_in_flash(key_file, 'client.key')
        res = self.at_command('AT+CSSLCFG="CONVERT",2,"ca.crt"')
        if not 'OK' in res:
            raise SystemError(f'Failed to configure CA cert: {res}')
        res = self.at_command('AT+CSSLCFG="CONVERT",1,"client.crt","client.key"')
        if not 'OK' in res:
            raise SystemError(f'Failed to configure client/key: {res}')
        
    def mqtt_connect(self,
                     server_url: str,
                     server_port: int,
                     client_id: str,
                     keepalive: int = 60,
                     ssl: bool = True) -> None:
        """"""
        if not self._active_pdp_context:
            raise ValueError('No valid PDP context')
        if not self._pdp_contexts[self._active_pdp_context].ip_address:
            raise ValueError('No valid IP address in context')
        res = self.at_command(f'AT+SMCONF="URL",{server_url},{server_port}')
        if not 'OK' in res:
            raise SystemError(f'Failed to configure MQTT server: {res}')
        res = self.at_command(f'AT+SMCONF="KEEPTIME",{keepalive}')
        if not 'OK' in res:
            raise SystemError(f'Failed to configure MQTT keepalive: {res}')
        res = self.at_command(f'AT+SMCONF="CLEANSS",1')
        if not 'OK' in res:
            raise SystemError(f'Failed to configure MQTT clean session: {res}')
        res = self.at_command(f'AT+SMCONF="CLIENTID",{client_id}')
        if not 'OK' in res:
            raise SystemError(f'Failed to configure MQTT client ID: {res}')
        if ssl:
            self.pdp_context_configure_ssl()
            res = self.at_command('AT+SMSSL=1,"ca.crt","client.crt"')
        res = self.at_command(f'AT+SMCONN')
        if not 'OK' in res:
            raise SystemError(f'Failed to connect to MQTT broker: {res}')
        self._mqtt_connected = True
    
    def mqtt_disconnect(self):
        res = self.at_command('AT+SMDISC')
        if not 'OK' in res:
            raise SystemError(f'Failed to disconnect MQTT: {res}')
        self._mqtt_connected = False
    
    def mqtt_susbscribe(self, topic: str, qos: int = 0):
        """"""
        res = self.at_command(f'AT+SMSUB="{topic}",{qos}')
        if not 'OK' in res:
            raise SystemError(f'Failed to subscribe to {topic}: {res}')
    
    def mqtt_unsubscribe(self, topic: str):
        res = self.at_command(f'AT+SMUNSUB="{topic}"')
        if not 'OK' in res:
            raise SystemError(f'Failed to unsubscribe from {topic}: {res}')
    
    def mqtt_publish(self,
                     topic: str,
                     payload: str,
                     qos: int = 0,
                     retain: bool = False):
        """"""
        retain = 1 if retain is True else 0
        res = self.at_command(f'AT+SMPUB="{topic}",{len(payload)},{qos},{retain}')
        if '>' not in res:
            raise SystemError(f'No prompt for MQTT payload: {res}')
        self.uart.write(payload.encode())
        sleep(math.ceil(len(payload) / self._baudrate))
        if self.uart.in_waiting:
            res = self.uart.read(self.uart.in_waiting).encode().strip()
            if 'OK' not in res:
                raise SystemError(f'Failed to publish MQTT: {res}')
