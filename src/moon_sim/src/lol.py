from gz.msgs10.world_control_pb2 import WorldControl
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.world_reset_pb2 import WorldReset
print("WorldControl fields:")
for field in WorldControl.DESCRIPTOR.fields:
    print(f"- {field.name} ({field.type})")
print("WorldReset fields:")
for field in WorldReset.DESCRIPTOR.fields:
    print(f"- {field.name} ({field.type})")
