ffplay -f live_flv -fast -x 2560 -y 1440 -fflags nobuffer -flags low_delay -strict experimental -vf "setpts=N/60/TB" -af "asetpts=N/60/TB" -noframedrop -i "rtmp://192.168.0.95:1935"
#ffplay -f live_flv -i "rtmp://192.168.0.95:1935/live"
