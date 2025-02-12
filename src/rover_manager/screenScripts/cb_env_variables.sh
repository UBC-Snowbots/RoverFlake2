#enviroment variables for control base

#middle screen
export cb_MIDDLE_SCREEN_ID=DP-1-0
export cb_MIDDLE_SCREEN_RES=1920x1080

#left screen
export cb_LEFT_SCREEN_ID=DP-1-2
export cb_LEFT_SCREEN_RES=1920x1080

#right screen
export cb_RIGHT_SCREEN_ID=HDMI-A-0
export cb_RIGHT_SCREEN_RES=1920x1080

export cb_scripts_PATH="${HOME}/RoverFlake2/src/rover_manager/screenScripts/"

alias cbs_init="sh ${cb_scripts_PATH}/xrandr_setup.sh"