import asyncio
from bleak import BleakScanner, BleakClient
import time
import struct
import datetime

BLE_DURATION_STIM_SERVICE_UUID = '1111'
BLE_AMPLITUDE_CHARACTERISTIC_UUID = '1112'  # these need to be chaned at some point for BLE specificatin reasons '48e47602-1b27-11ee-be56-0242ac120002'
BLE_DURATION_RIGHT_CHARACTERISTIC_UUID = '1113'  # these need to be chaned at some point for BLE specificatin reasons '48e47602-1b27-11ee-be56-0242ac120002'
BLE_DURATION_LEFT_CHARACTERISTIC_UUID = '1114'  # '63bae092-1b27-11ee-be56-0242ac120002'

BLE_BATTERY_SERVICE_UUID = '180F'
BLE_BATTERY_LEVEL_CHARACTERISTIC_UUID = '2A19'

timeout = 5

async def connect_to_device():
    devices = await BleakScanner.discover()
    for d in devices:
        if d.name == 'GaitGuide':
            print('Device found - MAC [', d.address, ']')
            client = BleakClient(d.address)
            await client.connect(timeout=timeout)
            print('Connected [', d.address, ']')
            return client

async def get_characteristic(service, characteristic_uuid):
    characteristic = service.get_characteristic(characteristic_uuid)
    return characteristic

async def write_characteristic(client, characteristic, value):
    value_bytes = struct.pack('<H', value)
    await client.write_gatt_char(characteristic, value_bytes)

async def read_characteristic(client, characteristic):
    value = await client.read_gatt_char(characteristic)
    return value


async def set_amp(client, characteristic, value):
    await client.write_gatt_char(characteristic,  bytearray([value]))


async def run():
    

    GaitGuide = await connect_to_device()
    LRA_service = GaitGuide.services.get_service(BLE_DURATION_STIM_SERVICE_UUID)
    BAT_service = GaitGuide.services.get_service(BLE_BATTERY_SERVICE_UUID)

    if LRA_service:
        Right = await get_characteristic(LRA_service, BLE_DURATION_RIGHT_CHARACTERISTIC_UUID)
        Left = await get_characteristic(LRA_service, BLE_DURATION_LEFT_CHARACTERISTIC_UUID)
        Ampl = await get_characteristic(LRA_service, BLE_AMPLITUDE_CHARACTERISTIC_UUID)
    else:
        print("GaitGuide was not found. Make sure the device is turned on and the blue LED is pulsing slowly.")
        exit()
    if BAT_service:    
        Bat = await get_characteristic(BAT_service, BLE_BATTERY_LEVEL_CHARACTERISTIC_UUID)
    else:
        print("Battery level not found - reading is ignored")

    await set_amp(GaitGuide, Ampl, 127)
    count = 0
    while (GaitGuide.is_connected):# and count < 11):
        await write_characteristic(GaitGuide, Right, 120)
        time.sleep(1)  # Sleep for 1 second


        await write_characteristic(GaitGuide, Left, 120)
        time.sleep(1)  # Sleep for 1 second

        if Bat: 
            batteryLevel = await read_characteristic(GaitGuide, Bat)
            batteryLevel_int = int.from_bytes(batteryLevel, "little")  # use "big" for big-endian

        current_time = datetime.datetime.now()  # get the current date and time

        print(f'{current_time}: Bat_Level = [{batteryLevel_int}%]')

        count = count +1

    await GaitGuide.disconnect()
    
loop = asyncio.get_event_loop()
loop.run_until_complete(run())