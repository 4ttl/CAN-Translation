import board
import digitalio
import busio
from time import sleep, monotonic
import json
from canio import CAN as CAN1, Message
from adafruit_mcp2515 import MCP2515 as CAN2

# CAN bus initialization
# CAN1 setup (using built-in CAN)
can1 = CAN1(rx=board.IO6, tx=board.IO7, baudrate=500_000, auto_restart=True)

# CAN2 setup (using MCP2515)
cs = digitalio.DigitalInOut(board.IO10)
spi = busio.SPI(board.IO12, board.IO11, board.IO13)
can2 = CAN2(spi, cs, baudrate=500_000, loopback=False, silent=False)
can2.auto_restart = True

# Load the DBC JSON files
def load_dbc_json(path):
    with open(path, 'r') as file:
        return json.load(file)

input_db_json = load_dbc_json('/sd/input_dbc.json')
output_db_json = load_dbc_json('/sd/output_dbc.json')

# Apply scale and offset to a signal value
def apply_scale_and_offset(value, factor, offset):
    return (value * factor) + offset

# Extract and process a single signal from the data
def bytes_to_int(bytes_value):
    result = 0
    for b in bytes_value:
        result = (result << 8) | b
    # Pad to 64 bits
    result = result << (8 * (8 - len(bytes_value)))
    return result

# Extract and process a single signal from the data
def extract_signal(message_data, start_bit, bit_length, is_signed, factor=1, offset=0):
    print(f"Extracting signal: start_bit={start_bit}, bit_length={bit_length}, is_signed={is_signed}")

    # Convert message_data to a 64-bit integer
    message_int = bytes_to_int(message_data)
    print(f"Message as int: {message_int:X}")

    # Calculate start byte and bit within byte
    start_byte = start_bit // 8
    bit_within_byte = start_bit % 8

    # Calculate the shift to align the least significant bit of the signal
    shift = 64 - (start_byte + 1) * 8 + bit_within_byte

    # Extract the signal
    mask = (1 << bit_length) - 1
    value = (message_int >> shift) & mask

    print(f"After extraction: {value:X}")

    # Handle signed values
    if is_signed and (value & (1 << (bit_length - 1))):
        value -= (1 << bit_length)

    # Apply factor and offset
    final_value = value * factor + offset
    print(f"Final value: {final_value}")
    return final_value

# Find the corresponding output signal configuration by name
def find_output_signal(signal_name, output_message_config):
    return output_message_config["signals"].get(signal_name)

# Translate and send messages based on signal names
def format_output_message(output_db, message_name, signals):
    """
    Format the extracted signals into a new CAN message based on the output database.
    """
    message_config = output_db[message_name]
    message_id = message_config["id"]

    # Check if length is specified, if not, use a default value or calculate from signals
    if "length" not in message_config:
        print(f"Warning: 'length' not specified for message {message_name}. Calculating from signals.")
        message_length = max((signal["start_bit"] + signal["length"] + 7) // 8 for signal in message_config["signals"].values())
    else:
        message_length = message_config["length"]

    print(f"Formatting output message: {message_name}, ID: {message_id:x}, Length: {message_length}")

    # Initialize the output message with zeros
    output_data = [0] * message_length

    for signal_name, value in signals.items():
        if signal_name not in message_config["signals"]:
            print(f"Warning: Signal {signal_name} not found in output message configuration. Skipping.")
            continue

        signal_config = message_config["signals"][signal_name]
        start_bit = signal_config["start_bit"]
        bit_length = signal_config["length"]
        factor = signal_config.get("factor", 1)
        offset = signal_config.get("offset", 0)

        # Reverse the scaling
        raw_value = int((value - offset) / factor)

        print(f"Formatting signal: {signal_name}, Value: {value}, Raw: {raw_value}, Start: {start_bit}, Length: {bit_length}")

        # Place the value in the correct position in the message
        for i in range(bit_length):
            byte_index = (start_bit + i) // 8
            bit_index = (start_bit + i) % 8
            if raw_value & (1 << i):
                output_data[byte_index] |= (1 << bit_index)

    return message_id, bytes(output_data)

def print_can2_diagnostics():
    print("CAN2 Diagnostics:")
    print(f"Baudrate: {can2.baudrate}")
    print(f"Loopback: {can2.loopback}")
    print(f"Silent: {can2.silent}")
    print(f"State: {can2.state}")
    print(f"Unread Message Count: {can2.unread_message_count}")

def translate_and_send(message, input_db, output_db, can_in, can_out):
    print(f"Translating message from {'CAN1' if can_in == can1 else 'CAN2'}: ID={message.id:x} Data={message.data.hex()}")

    # Find the input message configuration
    input_message_config = None
    for cfg in input_db.values():
        if cfg["id"] == message.id:
            input_message_config = cfg
            break

    if not input_message_config:
        print(f"No input configuration found for message ID {message.id:x}")
        return

    extracted_signals = {}
    for signal_name, signal in input_message_config["signals"].items():
        value = extract_signal(message.data, signal["start_bit"], signal["length"],
                               signal.get("is_signed", False), signal.get("factor", 1),
                               signal.get("offset", 0))
        extracted_signals[signal_name] = value
        print(f"Extracted {signal_name}: {value} {signal.get('unit', '')}")

    # Find the corresponding output message
    output_message_name = None
    for name, cfg in output_db.items():
        if all(signal in cfg["signals"] for signal in extracted_signals):
            output_message_name = name
            break

    if not output_message_name:
        print("No matching output message found")
        return

    try:
        # Format the output message
        output_id, output_data = format_output_message(output_db, output_message_name, extracted_signals)

        # Send the message on the opposing bus
        output_message = Message(id=output_id, data=output_data)
        print(f"Attempting to send on {'CAN1' if can_out == can1 else 'CAN2'}: ID={output_id:x} Data={output_data.hex()}")

        send_result = can_out.send(output_message)
        print(f"Send result: {send_result}")

        if can_out == can2:
            print_can2_diagnostics()
            # Try to read any pending messages
            while can2.unread_message_count > 0:
                try:
                    received = can2.read_message()
                    if received:
                        print(f"Message read from CAN2: ID={received.id:x} Data={received.data.hex()}")
                    else:
                        print("No message read from CAN2")
                except Exception as read_error:
                    print(f"Error reading message from CAN2: {type(read_error).__name__}: {str(read_error)}")
                    break

    except Exception as e:
        print(f"Error sending output message: {type(e).__name__}: {str(e)}")

# Main loop
while True:
    try:
        # Listen on CAN1
        with can1.listen(timeout=0.1) as can1_listener:
            message = can1_listener.receive()
            if isinstance(message, Message):
                print(f"CAN1 received: ID={message.id:x} Data={message.data.hex()}")
                translate_and_send(message, input_db_json, output_db_json, can1, can2)

        # Listen on CAN2
        with can2.listen(timeout=0.1) as can2_listener:
            message = can2_listener.receive()
            if isinstance(message, Message):
                print(f"CAN2 received: ID={message.id:x} Data={message.data.hex()}")
                translate_and_send(message, input_db_json, output_db_json, can2, can1)

        # Periodically check CAN2 status
        if monotonic() % 20 < 0.1:  # Every 20 seconds approximately
            print_can2_diagnostics()

    except Exception as e:
        print(f"Error during CAN operation: {type(e).__name__}: {str(e)}")
        # If there's an error, try to restart CAN2
        try:
            can2.restart()
            print("CAN2 restarted after error")
        except Exception as restart_error:
            print(f"Error restarting CAN2: {type(restart_error).__name__}: {str(restart_error)}")

    sleep(0.1)  # Short delay to prevent tight looping
