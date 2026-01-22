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
        """Initialize the BK9104 power supply interface

        Args:
            port: COM port or device path.
            baudrate: Baud rate for the serial connection. Defaults to 9600.
            timeout: Read timeout in seconds. Defaults to 1.
            connect: Whether to connect immediately on initialization. Defaults to True.
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None
        self.voltage = 0.0
        self.current = 0.0
        if connect:
            self.connect()

    def update(self, channel:int = 0, quiet:bool = False):
        """Update the voltage and current readings from the power supply

        Args:
            channel: The channel to read from. Defaults to 0.
            quiet: If True, suppress output. Defaults to False.
        """
        voltage, current = 0.0, 0.0
        response = self.cmd(f'GETS{channel}', quiet=quiet)
        pattern = re.compile(r"^(?P<voltage>\d{4})(?P<current>\d{4})$")
        for line in response:
            match = pattern.match(line)
            if match:
                voltage = int(match["voltage"]) / 100.0
                current = int(match["current"]) / 100.0
                enabled = self.get_enable()
                out_string = 'ON' if enabled == 1 else 'OFF' if enabled == 0 else 'UNKNOWN'
                print(f"Channel {channel} | {voltage} V | {current} A | {voltage * current} W | OUTPUT: {out_string}")

        self.voltage = voltage
        self.current = current

    def connect(self) -> bool:
        """Connect to the BK9104 power supply

        Returns:
            True if connection was successful, False otherwise.
        """
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            print(f"Connected to BK9104 on {self.port} at {self.baudrate} baud.")
            return True
        except serial.SerialException as e:
            print(f"Error connecting to BK9104: {e}")
            return False

    def disconnect(self) -> bool:
        """Disconnect from the BK9104 power supply

        Returns:
            True if disconnection was successful, False otherwise.
        """
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("Disconnected from BK9104.")
            return True
        else:
            print("Serial connection is not open.")
            return False

    def cmd(self, command: str, quiet:bool = False) -> List[str]:
        """Send a command to the BK9104 power supply and receive the response

        Args:
            command: The command string to send.
            quiet: If True, suppress output. Defaults to False.
        Returns:
            A list of response lines from the power supply.
        """
        command = command.strip()
        if not quiet:
            print("--------------------------------")
            print(f"Sending: {command}")
        self._transmit(command + '\r\n')
        ret = self._receive()
        if not quiet:
            print("Received:")
            for line in ret:
                print(f"  {line}")
            print("--------------------------------\r\n")
        return ret

    def _transmit(self, command: str) -> bool:
        """Transmit a command to the BK9104 power supply

        Args:
            command: The command string to send.
        Returns:
            True if the command was successfully transmitted, False otherwise.
        """
        if self.ser and self.ser.is_open:
            rc = self.ser.write(command.encode())
            time.sleep(0.1)
            if rc:
                return True
            else:
                print(f"Failed to send command: {command}")
                return False
        else:
            print("Error: Serial connection is not open.")
            return False

    def _receive(self) -> List[str]:
        """Receive response lines from the BK9104 power supply

        Returns:
            A list of response lines from the power supply.
        """
        response_list = []
        lines = []
        if self.ser and self.ser.is_open:
            while self.ser.in_waiting > 0:
                lines.append(self.ser.readline().decode().strip())
            for line in lines:
                sub_lines = line.split('\r')
                response_list.extend(sub_lines)
        else:
            print("Error: Serial connection is not open.")

        return response_list

    def set_voltage(self, v:float, channel:int = 0, quiet:bool = False) -> bool:
        """Set the voltage for a specific channel

        Args:
            v: The voltage value to set.
            channel: The channel number. Defaults to 0.
        Returns:
            True if the voltage was successfully set, False otherwise.
        """
        self.update(channel, quiet=True)
        if v * self.current > MAX_WATTAGE:
            print(f"Cannot set voltage: Max wattage restricted to {MAX_WATTAGE}")
            return False
        self.cmd(f"VOLT{channel}{int(v * 100):04d}")
        self.voltage = v
        return True

    def set_current(self, c:float, channel:int = 0, quiet:bool = False) -> bool:
        self.update(channel, quiet=quiet)
        if c * self.voltage > MAX_WATTAGE:
            print(f"Cannot set current: Max wattage restricted to {MAX_WATTAGE}")
            return False
        self.cmd(f"CURR{channel}{int(c * 100):04d}", quiet=quiet)
        self.current = c
        return True

    def set_enable(self, enable:bool = True, quiet:bool = False) -> bool:
        self.cmd(f"SOUT{'1' if enable else '0'}", quiet=quiet)
        return self.get_enable() == enable

    def get_enable(self) -> int:
        self.cmd("GOUT")
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
    ONE_HOUR = 3600
    bkp = BK9104(port='/dev/ttyUSB0', baudrate=9600, connect=False)
    bkp.connect()
    bkp.update(channel=0)
    bkp.set_current(0.5, channel=0, quiet=True)
    bkp.set_voltage(0.0, channel=0, quiet=True)
    bkp.set_enable(True, quiet=True)
    step_length = ONE_HOUR
    
    try:
        while True:
            bkp.set_voltage(12.0, channel=0)
            time.sleep(step_length)
            bkp.set_voltage(24.0, channel=0)
            time.sleep(step_length)
            bkp.set_voltage(36.0, channel=0)
            time.sleep(step_length)
            bkp.set_voltage(48.0, channel=0)
            time.sleep(step_length)
    except KeyboardInterrupt:
        print("Interrupted by user. Shutting down.")
        bkp.set_voltage(0.0, channel=0)
        bkp.set_enable(False)
        bkp.disconnect()
        quit()


if __name__ == "__main__":
    main()