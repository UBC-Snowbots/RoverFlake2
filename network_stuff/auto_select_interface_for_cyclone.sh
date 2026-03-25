# Auto-select DDS interface
if ip addr show eno1 | grep -q "192.168.1.50"; then
    export CYCLONEDDS_URI=file:///home/rv/RoverFlake2/network_stuff/dds_main.xml
elif ip addr show ztc3qyu2l6 | grep -q "172.30.0.21"; then
    export CYCLONEDDS_URI=file:///home/rv/RoverFlake2/network_stuff/dds_fallback.xml
else
    # Fallback to auto-determine
    export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface autodetermine="true"/></Interfaces></General></Domain></CycloneDDS>'
fi
