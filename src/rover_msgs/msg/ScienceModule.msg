# Command-only contract: the science hardware has no feedback sensors, so every
# field is a commanded state, not a measured one.
int16 sequenceselection
int16 sv1status
int16 sv2status
int16 sv3status
int16 sv4status

int16 p1status

int16 largestatus
int16 smallstatus
int16 carouselindex      # target chamber, 0-2 (servo-rotated, 3 chambers)
int16 dropservo          # dirt drop servo: 0=closed, 1=open

int16 agpowerstatus
int16 spectrostatus
int16 light1status
int16 light2status

int16 drilldir           # hold-to-rotate: -1=counter-clockwise, 0=stop, 1=clockwise
int16 drillstepperdir    # -1=raise, 0=hold, 1=lower
int16 vacuumstatus       # 0=off, 1=on
