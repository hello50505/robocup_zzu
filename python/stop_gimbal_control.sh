kill -9 $(ps -ef|grep gimbal_control.py|gawk '$0 !~/grep/ {print $2}' |tr -s '\n' ' ')
