from onvif import ONVIFCamera
import time

# Camera connection details
CAMERA_IP = '192.168.0.95'
PORT = 80
USERNAME = 'admin'
PASSWORD = '123456'

def main():
    # Create camera object
    mycam = ONVIFCamera(CAMERA_IP, PORT, USERNAME, PASSWORD)

    # Create media service to get profiles
    media_service = mycam.create_media_service()
    profiles = media_service.GetProfiles()
    profile = profiles[0]

    # Create PTZ service
    ptz_service = mycam.create_ptz_service()

    # Get PTZ status
    status = ptz_service.GetStatus({'ProfileToken': profile.token})
    print(f"Current PTZ status: {status}")

    # Create continuous move request
    request = ptz_service.create_type('ContinuousMove')
    request.ProfileToken = profile.token

    # Zoom in speed: positive value zooms in, negative zooms out
    request.Velocity = {'Zoom': {'x': 0.5}}

    # Start zoom in
    print("Starting zoom in...")
    ptz_service.ContinuousMove(request)

    # Let it zoom for 2 seconds
    time.sleep(2)

    # Stop movement
    ptz_service.Stop({'ProfileToken': profile.token})
    print("Zoom stopped.")

if __name__ == '__main__':
    main()