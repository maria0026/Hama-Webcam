import subprocess
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from functools import partial

class CameraRotator(Node):

    def __init__(self):
        super().__init__('camera_rotator')
        cameras_dict= self.detect_cameras()

        self.subscribers={}
        for camera_usb, camera_id in cameras_dict.items():
            usb_nr= (camera_usb.split(":"))[-1]
            usb_nr=usb_nr.replace("-", "_")
            topic_name=f"/camera_rotator/usb_{usb_nr}"
            self.subscribers[camera_usb]= self.create_subscription(
                Int32,
                topic_name,
                partial(self.listener_callback, camera_id=camera_id),
                10)
        
            self.subscribers[camera_usb]  #prevent unused variable warning
                
    def detect_cameras(self):
        cameras_detection_command = 'adb devices -l'
        cameras_result = subprocess.run(cameras_detection_command, shell=True, capture_output=True)

        cameras_output_bytes = cameras_result.stdout
        devices_output_string = cameras_output_bytes.decode('utf-8')

        cameras_output_list = devices_output_string.splitlines()
        cameras_output_list=cameras_output_list[1:-1]

        cameras_dict={}
        self.cameras_positions={}

        for camera_output in cameras_output_list:
            camera_output= camera_output.split()
            camera_usb=camera_output[-2]
            #wybranie id po :
            camera_id=(camera_output[-1].split(":"))[-1]
            cameras_dict[camera_usb]=camera_id

            init_command=f"adb -t {camera_id} shell motor_init"
            subprocess.run(init_command, shell=True)

            self.cameras_positions[camera_id]=0

            
        return cameras_dict


    def listener_callback(self, msg, camera_id):
        final_position=int(msg.data*(4096/360))
        if final_position>180:
            final_position=180
        elif final_position<-180:
            final_position=-180

        steps=final_position-self.cameras_positions[camera_id]
        
        rotate_command=f"adb -t {camera_id} shell motor_rotate {steps}"
        self.cameras_positions[camera_id]+=steps
        #print(self.cameras_positions[camera_id])
        subprocess.run(rotate_command, shell=True)


def main(args=None):
    rclpy.init(args=args)
    camera_rotator = CameraRotator()
    rclpy.spin(camera_rotator)
    camera_rotator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


