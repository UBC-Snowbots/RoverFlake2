# xrandr --output HDMI-A-0 --mode 1920x1080 --right-of eDP
# echo ${ROVERFLAKE_PATH}
echo ${cb_LEFT_SCREEN_ID}
echo ${cb_MIDDLE_SCREEN_ID}
echo ${cb_RIGHT_SCREEN_ID}

# source ${cb_scripts_PATH}/cb_env_variables.sh
xrandr --output $cb_MIDDLE_SCREEN_ID --mode $cb_MIDDLE_SCREEN_RES --primary --pos 1920x0 \
       --output $cb_LEFT_SCREEN_ID --mode $cb_LEFT_SCREEN_RES --pos 0x0 \
       --output $cb_RIGHT_SCREEN_ID --mode $cb_RIGHT_SCREEN_RES --pos 3840x0


#for rowans laptop:
# xrandr --output $cb_MIDDLE_SCREEN_ID --mode $cb_MIDDLE_SCREEN_RES --primary --pos 1920x0 \
#        --output $cb_LEFT_SCREEN_ID --mode $cb_LEFT_SCREEN_RES --pos 0x0 \
#        --output $cb_RIGHT_SCREEN_ID --mode $cb_RIGHT_SCREEN_RES --pos 3840x0 \
#        --output eDP --mode 2560x1440 --pos 5760x0