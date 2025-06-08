#enviroment variables for control base

#using 59.94 instead of 60 hoping for better reliability
export cb_REFRESH_RATE=59.94

# For usb c nuc "sucs"
#middle screen
export cb_MIDDLE_SCREEN_ID=DP-1
export cb_MIDDLE_SCREEN_RES=1920x1080

#left screen
export cb_LEFT_SCREEN_ID=DP-2-9
export cb_LEFT_SCREEN_RES=1920x1080

#right screen
export cb_RIGHT_SCREEN_ID=DP-2-8
export cb_RIGHT_SCREEN_RES=1920x1080

export cb_scripts_PATH="${HOME}/RoverFlake2/src/rover_manager/screenScripts"

alias cbs_connect_screens="sh ${cb_scripts_PATH}/xrandr_setup.sh"
alias cbs_startup="sh ${cb_scripts_PATH}/startup_system.sh"

# for rowan's laptop:
# export cb_MIDDLE_SCREEN_ID=DP-1-0
# export cb_MIDDLE_SCREEN_RES=1920x1080

# #left screen
# export cb_LEFT_SCREEN_ID=DP-1-2
# export cb_LEFT_SCREEN_RES=1920x1080

# #right screen
# export cb_RIGHT_SCREEN_ID=HDMI-A-0
# export cb_RIGHT_SCREEN_RES=1920x1080
