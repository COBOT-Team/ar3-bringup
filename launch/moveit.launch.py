from launch_ros.actions import Node
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from ament_index_python import get_package_share_directory
import os
import serial
import struct

#                                                                                                  #
# ======================================= COBOT Messaging ======================================== #
#                                                                                                  #


FIRMWARE_VERSION = 5

CRC_TABLE = [
    0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15, 0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,
    0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65, 0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D,
    0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5, 0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,
    0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85, 0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD,
    0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2, 0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA,
    0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2, 0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A,
    0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32, 0x1F, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A,
    0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42, 0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A,
    0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C, 0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,
    0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC, 0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4,
    0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C, 0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44,
    0x19, 0x1E, 0x17, 0x10, 0x05, 0x02, 0x0B, 0x0C, 0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34,
    0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B, 0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63,
    0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B, 0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13,
    0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB, 0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83,
    0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB, 0xE6, 0xE1, 0xE8, 0xEF, 0xFA, 0xFD, 0xF4, 0xF3
]


def crc8ccitt(data):
    """
    Compute the CRC-8-CCITT checksum of a byte array.

    Args:
        data: The byte array over which to compute the checksum.

    Returns:
        The CRC-8-CCITT checksum of the byte array.
    """

    val = 0
    for b in data:
        val = CRC_TABLE[(val ^ b) & 0xFF]
    return val & 0xFF


def frame(msg):
    """
    Frame a message for transmission to the COBOT.

    Args:
        msg: The message to frame.

    Returns:
        The framed message.
    """

    header = [0x24, len(msg) & 0xFF, crc8ccitt(msg) & 0xFF]
    return bytes(header) + msg


def wait_for_response(ser):
    """
    Wait for a response from the COBOT.

    Args:
        ser: The serial port to read from.

    Raises:
        Exception: If the response is not a valid response or the COBOT returns an error.

    Returns:
        The response type, response ID, and message.
    """

   # Wait for the start byte
    while ser.read(1) != b'$':
        pass

    # Read the length and checksum
    length = ser.read(1)[0]
    checksum = ser.read(1)[0]

    # Read the rest of the message
    msg = ser.read(length)

    # Verify the checksum
    if crc8ccitt(bytes([length]) + bytes([checksum]) + msg) != 0:
        raise Exception('Checksum error')

    # If the message is not a response, skip it
    if msg[0] != 0x01:
        return wait_for_response(ser)

    # Extract the response type and ID
    response_type = msg[1]
    response_id = struct.unpack('<I', msg[2:6])[0]

    # If the response is an error, raise an exception
    if response_type == 0x02:
        error_code = msg[6]
        error_str_len = msg[7]
        error_str = msg[8:8+error_str_len].decode('utf-8')
        raise Exception(f'COBOT Error {error_code}: {error_str}')

    # Return the response
    return response_type, response_id, msg[6:]


def wait_for_ack(ser, response_id):
    """
    Wait for an ACK from the COBOT.

    Args:
        ser: The serial port to read from.
        response_id: The ID of the message to wait for an ACK for.

    Raises:
        Exception: If the response is not an ACK or is not for the expected message.

    Returns:
        None
    """

    response_type, actual_id, msg = wait_for_response(ser)
    if response_type != 0x00:
        raise Exception(f'Expected ACK, got {response_type}')
    if response_id != actual_id:
        raise Exception(f'Expected ACK for message \
                        {response_id}, got {actual_id}')


def wait_for_done(ser, response_id):
    """
    Wait for a DONE from the COBOT.

    Args:
        ser: The serial port to read from.
        response_id: The ID of the message to wait for a DONE for.

    Raises:
        Exception: If the response is not a DONE or is not for the expected message.

    Returns:
        The response code.
    """

    response_type, actual_id, msg = wait_for_response(ser)
    if response_type != 0x01:
        raise Exception(f'Expected DONE, got {response_type}')
    if response_id != actual_id:
        raise Exception(f'Expected DONE for message \
                        {response_id}, got {actual_id}')
    return msg[0]


def calibrate_cobot(port, baudrate):
    """
    Calibrate the COBOT.

    Args:
        port: The serial port to use.
        baudrate: The baudrate to use.

    Raises:
        Exception: If an error occurs during calibration.

    Returns:
        None
    """

    init_msg = frame(b'\x00' + struct.pack('<I', 0) +
                     struct.pack('<I', FIRMWARE_VERSION))
    calibrate_msg = frame(b'\x01' + struct.pack('<I', 1) +
                          struct.pack('B', 0b111111))

    with serial.Serial(port, baudrate) as ser:
        print("Initializing COBOT...")
        ser.write(init_msg)
        wait_for_ack(ser, 0)
        print("Calibrating COBOT...")
        ser.write(calibrate_msg)
        wait_for_ack(ser, 1)
        wait_for_done(ser, 1)
        print('Calibration complete')


#                                                                                                  #
# ====================================== Launch description ====================================== #
#                                                                                                  #


def generate_launch_description():

    # Arguments
    serial_port_launch_arg = DeclareLaunchArgument(
        "port", default_value=TextSubstitution(text="/dev/ttyCobot0")
    )
    baudrate_launch_arg = DeclareLaunchArgument(
        "baudrate", default_value=TextSubstitution(text="115200")
    )
    calibrate_launch_arg = DeclareLaunchArgument(
        "calibrate", default_value=TextSubstitution(text="true")
    )

    # Perform calibration in the launch file before starting the controller. This is done so that
    # the controller does not start before the COBOT is calibrated.
    if calibrate_launch_arg.default_value == "true":
        calibrate_cobot(
            serial_port_launch_arg.default_value, baudrate_launch_arg.default_value)

    # start another turtlesim_node in the turtlesim2 namespace
    # and use args to set parameters
    turtlesim_node_with_parameters = Node(
        package='turtlesim',
        namespace='turtlesim2',
        executable='turtlesim_node',
        name='sim',
        parameters=[{
            "background_r": LaunchConfiguration('background_r'),
            "background_g": LaunchConfiguration('background_g'),
            "background_b": LaunchConfiguration('background_b'),
        }]
    )

    # perform remap so both turtles listen to the same command topic
    forward_turtlesim_commands_to_second_turtlesim_node = Node(
        package='turtlesim',
        executable='mimic',
        name='mimic',
        remappings=[
            ('/input/pose', '/turtlesim1/turtle1/pose'),
            ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
        ]
    )

    return LaunchDescription([
        background_r_launch_arg,
        background_g_launch_arg,
        background_b_launch_arg,
        chatter_py_ns_launch_arg,
        chatter_xml_ns_launch_arg,
        chatter_yaml_ns_launch_arg,
        launch_include,
        launch_py_include_with_namespace,
        launch_xml_include_with_namespace,
        launch_yaml_include_with_namespace,
        turtlesim_node,
        turtlesim_node_with_parameters,
        forward_turtlesim_commands_to_second_turtlesim_node,
    ])
