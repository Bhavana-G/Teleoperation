import asyncio
import xdc

# xdc code here, e.g.:

## DEVICE/CONNECTION LAYER: ##

# scan for all BLE devices the computer can see
#
# returns a list of `bleak.backends.device.BLEDevice`
#xdc.scan_all()

# scan for all DOT devices the computer can see
#
# returns a list of `bleak.backends.device.BLEDevice`
#xdc.scan()

# take an element from the scan list (`bleak.backends.device.BLEDevice`)
device = xdc.scan()[0]

# take the address (a string) of the device
#
# handy, because you can save the string to a config file etc. and use it
# to reconnect to the device after reboots etc.
address = device.address

# finds a BLE device by an address string
#
# returns `bleak.backends.device.BLEDevice`
xdc.find_by_address(address)

# same as above, but ensures the address points to a DOT by checking
# whether the device is a DOT after establishing the connection
#
# returns `bleak.backends.device.BLEDevice`
dot = xdc.find_dot_by_address(address)

## EXAMPLE CHARACTERISTIC USE (see source code for more examples) ##

# read the "Device Info Characteristic" for the given DOT
#
# returns `xdc.DeviceInfoCharacteristic`
xdc.device_info_read(dot)

# read the "Device Control Characteristic" for the given DOT
#
# returns `xdc.DeviceControlCharacteristic`
control_chr = xdc.device_control_read(dot)

control_chr.output_rate = 4  # modify it

# write a (potentially, modified) `xdc.DeviceControlCharacteristic` to
# the DOT. This enables controlling the device
xdc.device_control_write(dot, control_chr)


## HIGH-LEVEL CONVENIENCE API (see source code for implementation details) ##

# make the DOT flash its LED light a little bit, so that you can identify it
xdc.identify(dot)

# turn the DOT off (requires maybe pressing the button or shaking it afterwards to turn it back on)
xdc.power_off(dot)

# enable powering the DOT on whenever the micro-USB charger is plugged in
xdc.enable_power_on_by_usb_plug_in(dot)

# (opposite of the above): disable powering the DOT on whenever the micro-USB charger is plugged in
xdc.disable_power_on_by_usb_plug_in(dot)

# set the output rate of the DOT
#
# this is the frequency at which the reporting characteristic (i.e. the thing that is emitted whenever
# the DOT reports telemetry) reports
#
# must be 1, 4, 10, 12, 15, 20, 30, 60, 120 (see official XSens spec: Device Control Characteristic)
xdc.set_output_rate(dot, 10)

# reset the output rate to the default rate
xdc.reset_output_rate(dot)


## READING DATA FROM THE DOT ##
#
# Once you enable reporting, the DOT will asynchronously send telemetry data to the computer.
#
# Robust downstream code should assume that notifications sometimes go missing (e.g. due to
# connection issues)

# a callback function that is called whenever the DOT sends a device report notification
#
# after using `device_report_start_notify`, this will be called by the backend - the caller
# should handle the message bytes as appropriate (e.g. by pumping them into a parser)
def on_device_report(message_id, message_bytes):
    parsed = xdc.DeviceReportCharacteristic.from_bytes(message_bytes)
    print(parsed)

# a callback function that is called whenever the DOT sends a long payload report notifcation
#
# after using `device
def on_long_payload_report(message_id, message_bytes):
    print(message_bytes)

def on_medium_payload_report(message_id, message_bytes):
    print(message_bytes)

def on_short_payload_report(message_id, message_bytes):
    print(message_bytes)

def on_battery_report(message_id, message_bytes):
    print(message_bytes)

## SYNCHRONOUS API (simpler, but not exactly how the communication actually works)

with xdc.Dot(dot) as device:
    # subscribe to notifications
    device.device_report_start_notify(on_device_report)
    device.long_payload_start_notify(on_long_payload_report)
    device.medium_payload_start_notify(on_medium_payload_report)
    device.short_payload_start_notify(on_short_payload_report)

    # make the calling (synchronous) pump the asynchronous event queue forever
    #
    # this is required, because the main thread is responsible for pumping the
    # message queue that contains the above notifications. If you don't pump
    # the queue then you won't see the notifications
    xdc.pump_forever()


## ASYNCHRONOUS API (this is actually how communication with the `bleak` backend actually works)

# define an asynchronous function that should be used as the entrypoint for the asynchronous
# event loop (`asyncio.run_until_complete`)
async def arun():
    async with xdc.Dot(dot) as device:
        # asynchronously subscribe to notifications
        await device.adevice_report_start_notify(on_device_report)
        await device.along_payload_start_notify(on_long_payload_report)
        await device.amedium_payload_start_notify(on_medium_payload_report)
        await device.ashort_payload_start_notify(on_short_payload_report)

        # sleep for some amount of time, while pumping the message queue
        #
        # note: this differs from python's `sleep` function, because it doesn't cause the
        #       calling (asynchronous) thread to entirely sleep - it still processes any
        #       notifications that come in, unlike the synchronous API
        await asyncio.sleep(10)

        # (optional): unsubscribe to the notifications
        await device.adevice_report_stop_notify()
        await device.along_payload_stop_notify()
        await device.amedium_payload_stop_notify()
        await device.ashort_payload_stop_notify()

# start running the async task from the calling thread (by making the calling thread fully
# pump the event loop until the task is complete)

loop = asyncio.get_event_loop()
loop.run_until_complete(run())
