import subprocess

#dostajemy id usb i liczbe krokow
usb='usb:1-3'
steps=400

devices_command = 'adb devices -l'
devices_result = subprocess.run(devices_command, shell=True, capture_output=True)

devices_output_bytes = devices_result.stdout
devices_output_string = devices_output_bytes.decode('utf-8')

devices_output_list = devices_output_string.splitlines()
devices_output_list=devices_output_list[1:-1]

devices_dict={}

for device_output in devices_output_list:
    device_output= device_output.split()
    #print(device_output)
    device_usb=device_output[-2]
    device_id=(device_output[-1].split(":"))[-1]
    devices_dict[device_usb]=device_id


rotate_command="adb -t" +devices_dict[usb]+" shell motor_rotate "+str(steps)
subprocess.run(rotate_command, shell=True)
