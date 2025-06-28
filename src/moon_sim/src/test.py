from gz.transport13 import Node
from gz.msgs10.world_control_pb2 import WorldControl
from gz.msgs10.world_reset_pb2 import WorldReset
from gz.msgs10.boolean_pb2 import Boolean

def main():
    node = Node()
    service_name = "/world/moon_flat_world/control"

    # Create a WorldControl message
    request = WorldControl()

    # Create the WorldReset message
    reset_msg = WorldReset()
    reset_msg.all = True  # Reset everything (poses, velocities, etc.)

    # Set it into the WorldControl message
    request.reset.CopyFrom(reset_msg)

    timeout = 5000  # milliseconds

    # Call the service
    success, response = node.request(
        service_name, request, WorldControl, Boolean, timeout
    )

    print("Service call success:", success)
    if success:
        print("Response from Gazebo:", response.data)
    else:
        print("Request failed or timed out")

if __name__ == "__main__":
    main()
