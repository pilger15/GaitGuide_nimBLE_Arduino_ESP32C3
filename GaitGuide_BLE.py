import asyncio
from bleak import BleakScanner
from bleak import BleakClient
import time

BLE_DURATION_MED_CHARACTERISTIC_UUID = "1113",
BLE_DURATION_LAT_CHARACTERISTIC_UUID = "1114",

#'34:85:18:04:2E:A6'
timeout = 5
async def run():
    devices = await BleakScanner.discover()
    for d in devices:
        if d.name=='GaitGuide':
            print('Device found - MAC [', d.address,']')
            client = BleakClient(d.address)

            await client.connect()
            print('Connected [', d.address,']')
            services = await client.get_services()
            while client.is_connected:
                print("MEDIAL")
                await client.write_gatt_char(BLE_DURATION_MED_CHARACTERISTIC_UUID, 120)
                time.sleep(1) # Sleep for 3 seconds
                print("LATERAL")
                await client.write_gatt_char(BLE_DURATION_LAT_CHARACTERISTIC_UUID, 120)
                time.sleep(1) # Sleep for 3 seconds
            
            

async def connectOLF(address):
    """
    Connect to the BLE device. 
    If the device address is provided, try to connect to it directly,
    If address is not provided, it will scan for a device with the name "GaitGuide"
    and connect to it.
    :return: True if the connection is successful, False otherwise.
    """
    try:
        # Connect to the device directly if an address is provided
        if address:
            client = BleakClient(address)
        else:
            # Scan for available BLE devices and get the first device with the name "GaitGuide"
            devices = await BleakClient.discover(timeout=timeout, device="GaitGuide")
            await BleakClient.discover()
            if not devices:
                print("Could not find device with name GaitGuide")
                return False
            address = devices[0].address
            client = BleakClient(address)
            
        # Connect to the device
        await client.connect()
        # Enable reliable writes 
        await client.write_gatt_descriptor(0x2902, b"\x01\x00")
        
        return True
    except Exception as e:
        print(f"Error occured while connecting: {e}")
        return False

loop = asyncio.get_event_loop()
loop.run_until_complete(run())