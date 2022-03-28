"""
Pixhawk'ta bulunan basınç sensörü verisinin istenmesi.
"""

from pymavlink import mavutil
master=mavutil.mavlink_connection('/dev/ttyACM0',baud=115200) # Raspberry Pi ile Pixhawk'ın iletişim kurabilmesi için.

def request_message_interval(message_id: int, frequency_hz: float):
    """
    Request MAVLink message in a desired frequency,
    documentation for SET_MESSAGE_INTERVAL:
        https://mavlink.io/en/messages/common.html#MAV_CMD_SET_MESSAGE_INTERVAL

    Args:
        message_id (int): MAVLink message ID
        frequency_hz (float): Desired frequency in Hz
    """
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        message_id, # The MAVLink message ID
        1e6 / frequency_hz, # The interval between two messages in microseconds. Set to -1 to disable and 0 to request default rate.
        0, 0, 0, 0, # Unused parameters
        0, # Target address of message stream (if message has target address fields). 0: Flight-stack default (recommended), 1: address of requestor, 2: broadcast.
    )

# Configure ATTITUDE message to be sent at 2Hz
request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_SCALED_PRESSURE, 2)

def get_pressure_data():
    while True:
        msg = master.recv_match()
        if not msg:
            continue
        if msg.get_type() == 'SCALED_PRESSURE':
            data = str(msg)
            try:
                data = data.split(":")
                press_abs = data[2].split(",")[0]
                kPa = float(press_abs)/10
            except:
                print(data)

            return kPa
while True:
    print("Absolute pressure: {} kPa".format(get_pressure_data()))
