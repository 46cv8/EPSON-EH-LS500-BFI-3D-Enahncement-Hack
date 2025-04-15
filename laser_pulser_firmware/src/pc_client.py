import asyncio
import json
from bleak import BleakClient, BleakScanner

# Mapping for abbreviated keys (used in the BLE JSON) to full parameter names
PARAMETER_MAP = {
    "pwm": "enable_pwm_mitm",
    "fd": "frame_delay",
    "fr": "frame_duration",
    "di": "disable_pwm_during_active_interval",
    "es": "enable_serial_mitm",
    "sb": "serial_mitm_brightness_override",
    "ls": "log_mcu_serial",
    "g3": "gpio_3d_mode_active",
    "gs": "gpio_3d_signal_high",
    "gt": "gpio_3d_signal_last_timestamp",
    "cs": "current_active_interval_start_time",
    "ce": "current_active_interval_stop_time",
}

SERVICE_UUID = "12345678-1234-1234-1234-1234567890ab"
CHAR_UUIDS = {
    "ota": "12345678-1234-1234-1234-1234567890ac",  # OTA update characteristic
    "combined_update": "12345678-1234-1234-1234-1234567890b1",
    "update_eeprom": "12345678-1234-1234-1234-1234567890b8",
    "authenticate": "12345678-1234-1234-1234-1234567890b9",
    "combined_read_only": "12345678-1234-1234-1234-1234567890b7",
    "reset": "12345678-1234-1234-1234-1234567890c0",
    "dump_mcu_serial_log": "12345678-1234-1234-1234-1234567890c1"
}

DEVICE_NAME = "ESP32_3D_PWM"
SECRET = "MYSECRET"  # Must match the ESP32 firmware secret

async def discover_device():
    print("Scanning for BLE devices...")
    devices = await BleakScanner.discover()
    for d in devices:
        if DEVICE_NAME in d.name:
            print(f"Found device: {d.name} ({d.address})")
            return d
    print("Device not found.")
    return None

async def read_characteristic(client, name):
    uuid = CHAR_UUIDS.get(name)
    if not uuid:
        print(f"Characteristic '{name}' not found.")
        return None
    try:
        value = await client.read_gatt_char(uuid)
        decoded = value.decode().strip()
        return decoded
    except Exception as e:
        print(f"Error reading {name}: {e}")
        return None

async def write_characteristic(client, name, value):
    uuid = CHAR_UUIDS.get(name)
    if not uuid:
        print(f"Characteristic '{name}' not found.")
        return
    try:
        await client.write_gatt_char(uuid, value.encode())
        print(f"Set {name} to {value}")
    except Exception as e:
        print(f"Error writing {name}: {e}")

# New helper: Write OTA data without encoding if already bytes.
async def write_ota_characteristic(client, data):
    uuid = CHAR_UUIDS.get("ota")
    if not uuid:
        print("OTA characteristic not found.")
        return
    try:
        # If data is bytes, write directly; otherwise encode.
        if isinstance(data, bytes):
            await client.write_gatt_char(uuid, data)
        else:
            await client.write_gatt_char(uuid, data.encode())
        print("OTA characteristic updated.")
    except Exception as e:
        print(f"Error writing OTA characteristic: {e}")

def print_params(title, params_str):
    print(title)
    try:
        data = json.loads(params_str)
        # Create a mapped dict using full parameter names where available.
        mapped_data = {}
        for key, value in data.items():
            full_key = PARAMETER_MAP.get(key, key)
            mapped_data[full_key] = value
        for full_key, value in mapped_data.items():
            print(f"  {full_key}: {value}")
    except Exception as e:
        print("  Failed to parse parameters:", e)
        print("  Raw value:", params_str)

async def unified_dump(client):
    updateable_str = await read_characteristic(client, "combined_update")
    readonly_str = await read_characteristic(client, "combined_read_only")
    print("\nUnified Parameters Dump:")
    if updateable_str:
        print_params("Updateable Parameters:", updateable_str)
    else:
        print("  Could not read updateable parameters.")
    if readonly_str:
        print_params("Read-only Parameters:", readonly_str)
    else:
        print("  Could not read read-only parameters.")

async def send_firmware_update(client, firmware_path):
    try:
        with open(firmware_path, "rb") as f:
            firmware_data = f.read()
        print(f"Firmware file read successfully, size: {len(firmware_data)} bytes")
    except Exception as e:
        print("Error reading firmware file:", e)
        return

    chunk_size = 512  # Adjust the chunk size if necessary

    print("Starting OTA update...")
    # Send OTA_START command
    await write_ota_characteristic(client, "OTA_START")
    await asyncio.sleep(0.5)  # small delay to let device prepare

    # Send firmware data in chunks
    for i in range(0, len(firmware_data), chunk_size):
        chunk = firmware_data[i:i+chunk_size]
        await write_ota_characteristic(client, chunk)
        progress = (i + len(chunk)) / len(firmware_data) * 100
        print(f"Sent {i + len(chunk)} / {len(firmware_data)} bytes ({progress:.1f}%)")
        await asyncio.sleep(0.01)  # slight delay between chunks

    # Send OTA_END command to finalize the update
    await write_ota_characteristic(client, "OTA_END")
    print("OTA update completed. Device should restart if update was successful.")

async def interactive_menu(client):
    while True:
        print("\nMenu:")
        print("1: Dump all parameters")
        print("2: Update one parameter")
        print("3: Update EEPROM")
        print("4: Reset ESP32")
        print("5: OTA Firmware Update")
        print("6: Dump MCU Serial Log")
        print("7: Exit")
        choice = input("Enter your choice: ").strip()
        
        if choice == "1":
            await unified_dump(client)
        
        elif choice == "2":
            current = await read_characteristic(client, "combined_update")
            if not current:
                print("Could not retrieve current parameters.")
                continue
            try:
                data = json.loads(current)
            except Exception as e:
                print("Failed to parse current parameters:", e)
                continue

            print("Current updateable parameters:")
            print(f"1: enable_pwm_mitm: {data.get('pwm')}")
            print(f"2: frame_delay: {data.get('fd')}")
            print(f"3: frame_duration: {data.get('fr')}")
            print(f"4: disable_pwm_during_active_interval: {data.get('di')}")
            print(f"5: enable_serial_mitm: {data.get('es')}")
            print(f"6: serial_mitm_brightness_override: {data.get('sb')}")
            print(f"7: log_mcu_serial: {data.get('ls')}")
            
            sel = input("Enter the number of the parameter you want to update: ").strip()
            if sel == "1":
                new_val = input("Enter new value for enable_pwm_mitm (0 or 1): ").strip()
                data["pwm"] = int(new_val)
            elif sel == "2":
                new_val = input("Enter new value for frame_delay: ").strip()
                data["fd"] = int(new_val)
            elif sel == "3":
                new_val = input("Enter new value for frame_duration: ").strip()
                data["fr"] = int(new_val)
            elif sel == "4":
                new_val = input("Enter new value for disable_pwm_during_active_interval (0 or 1): ").strip()
                data["di"] = int(new_val)
            elif sel == "5":
                new_val = input("Enter new value for enable_serial_mitm (0 or 1): ").strip()
                data["es"] = int(new_val)
            elif sel == "6":
                new_val = input("Enter new 4-digit hex value for serial_mitm_brightness_override (A888-AFFF): ").strip()
                if len(new_val) != 4 or not all(c in "0123456789ABCDEFabcdef" for c in new_val):
                    print("Invalid hex format. Must be 4 hex digits.")
                    continue
                try:
                    brightness = int(new_val, 16)
                except Exception as e:
                    print("Failed to convert hex value:", e)
                    continue
                if brightness < 0xA888 or brightness > 0xAFFF:
                    print("Value out of range. Must be between A888 and AFFF.")
                    continue
                data["sb"] = new_val.upper()
            elif sel == "7":
                new_val = input("Enter new value for log_mcu_serial (0 or 1): ").strip()
                data["ls"] = int(new_val)
            else:
                print("Invalid selection.")
                continue
            
            payload = json.dumps(data, separators=(',', ':'))
            await write_characteristic(client, "combined_update", payload)

        elif choice == "3":
            print("Triggering EEPROM update on the device...")
            await write_characteristic(client, "update_eeprom", "1")
            
        elif choice == "4":
            print("Sending reset command to device...")
            await write_characteristic(client, "reset", "1")
            print("Reset command sent. Waiting for device to restart...")
            await asyncio.sleep(3)
            return True  # Indicate reset was triggered
        
        elif choice == "5":
            print("Do not do this unless you know what your doing.")
            print("Applying a firmware update while the projector is running is risky.")
            print("When the ESP32 resets on completion, it will cacuse the projector to detect a laser driver error and go into error mode with orange light source error and flashing blue projecto leds.")
            print("The only way to recover after it finishes is to remove the power plug and plug it back in, it should still work though.")
            print("Don't forget to clear your bluetooth device cache on your PC after an update or the software may not work properly.")
            firmware_path = input("Enter the path to the firmware file: ").strip()
            await send_firmware_update(client, firmware_path)

        elif choice == "6":
            serial_data = ""
            while True:
                chunk = await read_characteristic(client, "dump_mcu_serial_log")
                serial_data += chunk + '\n'
                # If the received chunk is less than 64 bytes, assume this is the final part.
                if len(chunk) < 64:
                    break
            if serial_data:
                print("Dump MCU Serial Log:")
                print(serial_data)
            else:
                print("  Could not read dump MCU serial log.")

            
        elif choice == "7":
            print("Exiting...")
            return False
        
        else:
            print("Invalid choice.")

async def main():
    while True:
        device = await discover_device()
        if not device:
            return

        try:
            async with BleakClient(device.address) as client:
                if not client.is_connected:
                    print("Failed to connect.")
                    return

                print("Connected to device.")
                # Automatically send authentication
                await write_characteristic(client, "authenticate", SECRET)
                await unified_dump(client)
                reset_triggered = await interactive_menu(client)
                if reset_triggered:
                    print("Reset triggered. Reconnecting...")
                    await asyncio.sleep(2)
                    continue
                else:
                    break
        except Exception as e:
            print("Connection error:", e)
            await asyncio.sleep(2)

if __name__ == "__main__":
    asyncio.run(main())
