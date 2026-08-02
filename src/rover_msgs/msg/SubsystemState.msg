# Observed (reaped) state of one heart-managed child. Never a stored belief.
uint8 STOPPED=0
uint8 RUNNING=1
uint8 STOPPING=2
uint8 CRASHED=3

string name
uint8 state
int32 pid          # -1 when not running
int32 exit_code    # valid in CRASHED/STOPPED after a reap; 128+signal if killed by signal
uint32 uptime_s    # 0 unless RUNNING
