import board
import digitalio
import busio
from time import sleep
import json
from canio import CAN as CAN1, Message
from adafruit_mcp2515 import MCP2515 as CAN2

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
def extract_signal(data, start_bit, length, byte_order, is_signed):
    num_bits = start_bit + length
    num_bytes = (num_bits + 7) // 8  # Calculate the number of bytes needed

    # Ensure there is enough data
    if len(data) < num_bytes:
        raise ValueError(f"Not enough data to extract the required bytes. Data length={len(data)}, Required bytes={num_bytes}")

    # Extract the relevant bytes
    byte_start = start_bit // 8
    byte_end = (start_bit + length - 1) // 8
    extracted_bytes = data[byte_start:byte_end + 1]

    # Handle byte order
    if byte_order == "Motorola":
        extracted_bytes = extracted_bytes[::-1]  # Reverse for Motorola

    # Convert bytes to integer
    bit_start = start_bit % 8
    bit_end = bit_start + length
    extracted_value = int.from_bytes(extracted_bytes, byteorder="big" if byte_order == "Motorola" else "little")

    # Mask out the bits
    mask = (1 << length) - 1
    extracted_value = (extracted_value >> bit_start) & mask

    # Handle signed values
    if is_signed and (extracted_value & (1 << (length - 1))):
        extracted_value -= (1 << length)

    return extracted_value

def inject_signal(output_data, value, start_bit, length, byte_order):
    num_bits = start_bit + length
    num_bytes = (num_bits + 7) // 8

    if len(output_data) < num_bytes:
        raise ValueError(f"Not enough space in output data. Data length={len(output_data)}, Required bytes={num_bytes}")

    byte_start = start_bit // 8
    byte_end = (start_bit + length - 1) // 8

    # Handle byte order
    if byte_order == "Motorola":
        value_bytes = value.to_bytes(byte_end - byte_start + 1, byteorder="big")
        value_bytes = value_bytes[::-1]  # Reverse for Motorola
    else:
        value_bytes = value.to_bytes(byte_end - byte_start + 1, byteorder="little")

    # Inject value into output data
    bit_start = start_bit % 8
    for i in range(len(value_bytes)):
        shift = bit_start if i == 0 else 0
        mask = (0xFF >> shift) if i == 0 else 0xFF
        if i == len(value_bytes) - 1:
            mask = (mask & (0xFF >> (8 - (length - (len(value_bytes) - 1) * 8))))  # Ensure masking the correct bits in the last byte
        output_data[byte_start + i] = (output_data[byte_start + i] & ~mask) | ((value_bytes[i] >> shift) & mask)

def process_can_message(can_id, data, input_dbc, output_dbc):
    if can_id not in input_dbc:
        print(f"No config found for message with ID {can_id}")
        return None

    input_message = input_dbc[can_id]
    output_message = None

    output_data = bytearray(8)  # Initialize with zeros

    for signal_name, signal_config in input_message["signals"].items():
        try:
            start_bit = signal_config["start_bit"]
            length = signal_config["length"]
            byte_order = signal_config["byte_order"]
            is_signed = signal_config["is_signed"]

            extracted_value = extract_signal(data, start_bit, length, byte_order, is_signed)

            for out_msg, out_cfg in output_dbc.items():
                if signal_name in out_cfg["signals"]:
                    out_signal = out_cfg["signals"][signal_name]

                    value = int((extracted_value - out_signal.get("offset", 0)) / out_signal.get("factor", 1))

                    out_start_bit = out_signal["start_bit"]
                    out_length = out_signal["length"]
                    out_byte_order = out_signal["byte_order"]

                    inject_signal(output_data, value, out_start_bit, out_length, out_byte_order)

                    output_message = out_cfg["id"]

        except Exception as e:
            print(f"Error during signal extraction or processing for {signal_name}: {e}")
            continue

    if output_message is not None:
        return output_message, output_data
    else:
        return None

# Translate and send messages based on signal names
def translate_and_send(message, bus, input_db, output_db, can_out):
    try:
        print(f"Received message: ID={message.id} Data={message.data.hex()}")

        input_message_config = None
        for name, config in input_db.items():
            if config["id"] == message.id:
                input_message_config = config
                print(f"Message config found for {name} with ID {message.id}")
                break

        if not input_message_config:
            print(f"No input message configuration found for ID {message.id} (hex: {hex(message.id)})")
            return

        translated_data = bytearray(8)

        for signal_name, signal in input_message_config["signals"].items():
            print(f"Processing signal: {signal_name}")

            output_message_config = None
            for name, config in output_db.items():
                if signal_name in config["signals"]:
                    output_message_config = config
                    print(f"Output signal configuration found for {signal_name} in {name}")
                    break

            if not output_message_config:
                print(f"No output message configuration found for signal {signal_name}")
                continue

            output_signal = output_message_config["signals"].get(signal_name)
            if not output_signal:
                print(f"Output signal {signal_name} not found in output configuration for {output_message_config}")
                continue

            start = signal["start_bit"]
            length = signal["length"]
            is_signed = signal.get("is_signed", False)
            byte_order = signal.get("byte_order", "Motorola")

            print(f"Extracting signal {signal_name}: Start={start}, Length={length}, Byte Order={byte_order}")

            try:
                raw_value = extract_signal(message.data, start, length, byte_order, is_signed)
                if raw_value is not None:
                    print(f"Raw value extracted for {signal_name}: {raw_value}")

                    factor = signal.get("factor", 1)
                    offset = signal.get("offset", 0)
                    scaled_value = apply_scale_and_offset(raw_value, factor, offset)
                    print(f"Scaled value for {signal_name}: {scaled_value} (Factor: {factor}, Offset: {offset})")

                    # Calculate the byte start and bit positions for inserting the value
                    byte_start = output_signal["start_bit"] // 8
                    bit_start = output_signal["start_bit"] % 8
                    bit_end = bit_start + output_signal["length"]
                    num_bytes = (bit_end + 7) // 8

                    print(f"Inserting value into translated_data: Byte Start={byte_start}, Bit Start={bit_start}, Bit End={bit_end}, Num Bytes={num_bytes}")

                    # Ensure that we are inserting correctly
                    mask = (1 << output_signal["length"]) - 1
                    packed_value = (int(scaled_value) & mask) << bit_start

                    for i in range(num_bytes):
                        if i + byte_start < len(translated_data):
                            translated_data[i + byte_start] |= (packed_value >> (i * 8)) & 0xFF

                    print(f"Translated data after processing {signal_name}: {translated_data.hex()}")

            except Exception as e:
                print(f"Error during signal extraction or processing for {signal_name}: {e}")
                continue

        translated_id = None
        for name, config in output_db.items():
            if any(signal_name in config["signals"] for signal_name in input_message_config["signals"]):
                translated_id = config["id"]
                print(f"Determined output ID {translated_id} for message with ID {message.id}")
                break

        if not translated_id:
            print(f"No output ID found for message with ID {message.id}")
            return

        translated_msg = Message(id=translated_id, data=translated_data[:8], extended=False)
        can_out.send(translated_msg)
        print(f"Translated message sent: ID={translated_id}, Data={translated_data[:8].hex()} to bus {bus}")

    except Exception as e:
        print(f"Error translating and sending message: {e}")

# Initialize CAN1 (Primary CAN using canio)
can1 = CAN1(rx=board.IO6, tx=board.IO7, baudrate=500_000, auto_restart=True)

# Initialize CAN2 (Secondary CAN using MCP2515)
cs = digitalio.DigitalInOut(board.IO10)
cs.direction = digitalio.Direction.OUTPUT
spi = busio.SPI(board.IO12, board.IO11, board.IO13)
can2 = CAN2(spi, cs, baudrate=500_000)

while True:
    try:
        # Listen on CAN1
        with can1.listen(timeout=1.0) as can1_listener:
            message_count = can1_listener.in_waiting()
            if message_count:
                for _ in range(message_count):
                    msg = can1_listener.receive()
                    if isinstance(msg, Message):
                        # Translate and send to CAN2
                        translate_and_send(msg, 2, input_db_json, output_db_json, can2)

        # Listen on CAN2
        with can2.listen(timeout=1.0) as can2_listener:
            message_count = can2_listener.in_waiting()
            if message_count:
                for _ in range(message_count):
                    msg = can2_listener.receive()
                    if isinstance(msg, Message):
                        # Translate and send to CAN1
                        translate_and_send(msg, 1, input_db_json, output_db_json, can1)

    except Exception as e:
        print(f"Error during CAN operation: {e}")

    sleep(1)
