from libcamera import CameraManager
cm = CameraManager.singleton()
for i, cam in enumerate(cm.cameras):
    print(i, cam.id)
