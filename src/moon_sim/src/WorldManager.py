from gz.transport13 import Node
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.boolean_pb2 import Boolean

import time

class WorldManager:
    def __init__(self, world: str = "moon_flat_world"):
        self.world = world
        self.node = Node()

    def spawn_model(self, model_uri: str = "model://lander", model_name: str = "apollo_lander",
                    x: float = 0.0, y: float = 0.0, z: float = 50.0):
        msg = EntityFactory()
        msg.sdf_filename = model_uri
        msg.name = model_name
        msg.allow_renaming = False

        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0

        service = f"/world/{self.world}/create"
        success, response = self.node.request(service, msg, EntityFactory, Boolean, timeout=2000)

        if success:
            print(f"[INFO] Spawned model '{model_name}' at ({x}, {y}, {z})")
        else:
            print(f"[ERROR] Failed to spawn model. Response: {response}")

    def destroy_model(self, model_name: str = "apollo_lander"):
        msg = Entity()
        msg.name = model_name
        msg.type = Entity.MODEL  # Set type explicitly (MODEL = 1)

        service = f"/world/{self.world}/remove"
        success, response = self.node.request(service, msg, Entity, Boolean, timeout=2000)

        if success:
            print(f"[INFO] Destroyed model '{model_name}'")
        else:
            print(f"[ERROR] Failed to destroy model. Response: {response}")

# if __name__ == "__main__":
#     wm = WorldManager()
#     while True:
#         wm.spawn_model()
#         time.sleep(10)
#         print("destroying")
#         wm.destroy_model()
#         time.sleep(1)
