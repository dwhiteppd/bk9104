# Pica Product Development, 2026
# Devon White
# BK Precision 9104 Power Supply Control Module

import re
from typing import List, Union
import serial
import time

MAX_WATTAGE = 320
MAX_VOLTAGE = 84
MAX_CURRENT = 10


class BK9104:
    def __init__(self, port: str, baudrate: int = 9600, timeout: int = 1, connect: bool = True):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None
        self.voltage = 0.0
        self.current = 0.0
        if connect:
            self.connect()

    def update(self, channel:int = 0, quiet:bool = False):
        voltage, current = 0.0, 0.0
        self._transmit(f'GETS{channel}\r\n', quiet=quiet)
        response = self._receive()
        pattern = re.compile(r"^(?P<voltage>\d{4})(?P<current>\d{4})$")
        for line in response:
            match = pattern.match(line)
            if match:
                voltage = int(match["voltage"]) / 100.0
                current = int(match["current"]) / 100.0
                print(f"Channel {channel} - Voltage: {voltage} V, Current: {current} A, Power: {voltage * current} W")

        self.voltage = voltage
        self.current = current

    def connect(self) -> bool:
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            print(f"Connected to BK9104 on {self.port} at {self.baudrate} baud.")
            return True
        except serial.SerialException as e:
            print(f"Error connecting to BK9104: {e}")
            return False

    def cmd(self, command: str):
        self._transmit(command + '\r\n')
        return self._receive()

    def _transmit(self, command: str, quiet:bool = False) -> bool:
        if self.ser and self.ser.is_open:
            rc = self.ser.write(command.encode())
            if rc:
                if not quiet:
                    print(f"Sent --> {command}")
                return True
            else:
                print(f"Failed to send command: {command}")
                return False
        else:
            print("Serial connection is not open.")
            return False

    def _receive(self, quiet:bool = False) -> List[str]:
        time.sleep(1)
        response_list = []
        lines = []
        if self.ser and self.ser.is_open:
            while self.ser.in_waiting > 0:
                lines.append(self.ser.readline().decode().strip())
            for line in lines:
                sub_lines = line.split('\r')
                response_list.extend(sub_lines)
            if not quiet:
                for line in response_list:
                    print(f"Recv <-- {line}")
        else:
            print("Serial connection is not open.")

        return response_list

    def set_voltage(self, v:float, channel:int = 0) -> bool:
        self.update(channel, quiet=True)
        if v * self.current > MAX_WATTAGE:
            print(f"Cannot set voltage: Max wattage restricted to {MAX_WATTAGE}")
            return False
        self._transmit(f"VOLT{channel}{int(v * 100):04d}\r\n")
        self.voltage = v
        return True

    def set_current(self, c:float, channel:int = 0) -> bool:
        self.update(channel, quiet=True)
        if c * self.voltage > MAX_WATTAGE:
            print(f"Cannot set current: Max wattage restricted to {MAX_WATTAGE}")
            return False
        self._transmit(f"CURR{channel}{int(c * 100):04d}\r\n")
        self.current = c
        return True

    def set_enable(self, enable:bool = True) -> bool:
        self._transmit(f"SOUT{'1' if enable else '0'}\r\n")
        return self.get_enable() == enable

    def get_enable(self) -> int:
        self._transmit("GOUT\r\n")
        response_list = self._receive()
        for line in response_list:
            if line.strip() == "1":
                return 1
            elif line.strip() == "0":
                return 0
            if line.startswith("OK"):
                break
        return -1

def main():
    bkp = BK9104(port='/dev/ttyUSB0', baudrate=9600, connect=False)
    bkp.connect()
    bkp.update(channel=0)
    bkp.set_current(0.5, channel=0)
    bkp.set_voltage(12.0, channel=0)
    bkp.set_enable(True)
    
    while True:
        bkp.set_voltage(12.0, channel=0)
        time.sleep(5)
        bkp.set_voltage(16.0, channel=0)
        time.sleep(5)
        bkp.set_voltage(20.0, channel=0)
        time.sleep(5)

if __name__ == "__main__":
    main()