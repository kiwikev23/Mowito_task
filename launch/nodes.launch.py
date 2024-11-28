from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()



    #Intialisation of usb_cam node
    camera_node = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        name="usb_cam_node",
        output="screen"
    )


    #Intialisation of service node
    server_node = Node(
        package="mowito_task",
        executable="image_converter_node",
        name="image_conversion",
        output="screen",

        #Configuring params
        parameters=[
            {"input_topic": "/image_raw"},
            {"output_topic": "/converted_image"}
        ]
    )

    ld.add_action(camera_node)
    ld.add_action(server_node)

    return ld
