import re
import bpy
scene = bpy.context.scene
scene.cycles.device = 'GPU'
prefs = bpy.context.preferences
prefs.addons['cycles'].preferences.get_devices()
cprefs = prefs.addons['cycles'].preferences
print(cprefs)
for compute_device_type in ('CUDA', 'OPENCL', 'NONE'):
  try:
    cprefs.compute_device_type = compute_device_type
    print('Device found:', compute_device_type)
    break
  except TypeError:
    pass
for device in cprefs.devices:
  if not re.match('intel', device.name, re.I):
    print('Activating', device)
    device.use = True
  else:
    device.use = False
