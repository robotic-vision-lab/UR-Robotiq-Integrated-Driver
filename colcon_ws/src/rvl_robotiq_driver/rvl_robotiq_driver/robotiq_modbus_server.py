from pymodbus.client.sync import ModbusSerialClient
from pymodbus.register_read_message import ReadHoldingRegistersResponse
from pymodbus.exceptions import ModbusIOException

import pymodbus

from math import ceil

class RobotiqRTUClient:
    def __init__(self, unit_id = 0x0009, input_addr = 0x03E8, output_addr = 0x07D0):
        self.client = None
        self.unit_id = unit_id
        self.command_register = input_addr
        self.status_registers = output_addr
        print(pymodbus.__version__)

    def connect(self, device_addr):
        self.client = ModbusSerialClient(method = 'rtu',
                                         port = device_addr,
                                         stopbits = 1,
                                         bytesize = 8,
                                         baudrate = 115200,
                                         timeout = 1)
        # this does not verify connection?
        return self.client.connect()

    def disconnect(self):
        self.client.close()

    def send_command(self, command):
        # make sure data has an even number of elements
        if(len(command) % 2 == 1):
            command.append(0)
        # initiate message as an empty list
        message = []
        # fill message by combining two bytes in one register
        for i in range(0, len(command)//2):
            message.append((command[2*i] << 8) + command[2*i+1])
        # sending the command
        try:
            reply = self.client.write_registers(self.command_register, message, unit=self.unit_id)
        except Exception as e:
            print(e)
            return False
        # check if the reply is valid
        if isinstance(reply, ModbusIOException):
            print('ModbusIOException occured!')
            print(reply)
            return False
        # if the reply is valid, the command is PROBABLY accepted
        return True

    def request_status(self, nbytes = 6):
        nregs = int(ceil(nbytes/2.0))
        raw_status = self.client.read_holding_registers(self.status_registers, nregs, unit = self.unit_id, timeout = 3)
        if isinstance(raw_status, ReadHoldingRegistersResponse):
            return self.parse_registers(raw_status, nregs)
        elif isinstance(raw_status, ModbusIOException):
            print('ModbusIOException occured!')
            print(raw_status)
            return None
        else:
            print('Unknown error case occured!')
            return None

    def parse_registers(self, recv_regs, nregs):
        output = []
        for i in range(nregs):
            output.append((recv_regs.getRegister(i) & 0xFF00) >> 8)
            output.append( recv_regs.getRegister(i) & 0x00FF)
        return output
