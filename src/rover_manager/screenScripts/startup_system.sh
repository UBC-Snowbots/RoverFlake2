#To startup the whole system
source cb_env_variables.sh
clear
echo CONTROL BASE SYSTEM START | figlet | lolcat
sleep 0.5
echo MIDDLE SCREEN: $cb_MIDDLE_SCREEN_ID, $cb_MIDDLE_SCREEN_RES
echo LEFT SCREEN: $cb_LEFT_SCREEN_ID, $cb_LEFT_SCREEN_RES
echo RIGHT SCREEN: $cb_RIGHT_SCREEN_ID, $cb_RIGHT_SCREEN_RES

i3-msg "workspace 1; move workspace to output $cb_LEFT_SCREEN_ID"
i3-msg "workspace 2; move workspace to output $cb_MIDDLE_SCREEN_ID"
i3-msg "workspace 3; move workspace to output $cb_RIGHT_SCREEN_ID"

# i3-msg "workspace 1; exec firefox"
# i3-msg "workspace 2; exec code"
# i3-msg "workspace 1; exec kitty --hold -e $cb_PATH/screenScripts/left_screen_startup.bash"
i3-msg "workspace 1; exec kitty -e bash '$cb_PATH/screenScripts/left_screen_startup.bash'"
i3-msg "workspace 2; exec kitty -e bash '$cb_PATH/screenScripts/middle_screen_startup.sh'"
i3-msg "workspace 3; exec kitty -e bash '$cb_PATH/screenScripts/right_screen_startup.sh'"
