import asyncio
from bleak import BleakScanner, BleakClient
import time

BLE_DURATION_STIM_SERVICE_UUID = '1111'
BLE_DURATION_RIGHT_CHARACTERISTIC_UUID = '1113'  # these need to be chaned at some point for BLE specificatin reasons '48e47602-1b27-11ee-be56-0242ac120002'
BLE_DURATION_LEFT_CHARACTERISTIC_UUID = '1114'  # '63bae092-1b27-11ee-be56-0242ac120002'

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
    await client.write_gatt_char(characteristic, bytearray([value]))

async def run():
    

    GaitGuide = await connect_to_device()
    service = GaitGuide.services.get_service(BLE_DURATION_STIM_SERVICE_UUID)

    if service:
        Right = await get_characteristic(service, BLE_DURATION_RIGHT_CHARACTERISTIC_UUID)
        Left = await get_characteristic(service, BLE_DURATION_LEFT_CHARACTERISTIC_UUID)

    count = 0
    while (GaitGuide.is_connected and count < 11):
        await write_characteristic(GaitGuide, Right, 120)
        time.sleep(1)  # Sleep for 1 second

        await write_characteristic(GaitGuide, Left, 120)
        time.sleep(1)  # Sleep for 1 second

        count = count +1

    await GaitGuide.disconnect()
    print('Disconnected [', GaitGuide.address, ']')

loop = asyncio.get_event_loop()
loop.run_until_complete(run())