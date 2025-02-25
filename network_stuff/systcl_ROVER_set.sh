echo "net.ipv4.ipfrag_time=3" | sudo tee -a /etc/sysctl.conf
echo "net.ipv4.ipfrag_high_thresh=134217728" | sudo tee -a /etc/sysctl.conf
echo "net.core.rmem_max=2147483647" | sudo tee -a /etc/sysctl.conf 
