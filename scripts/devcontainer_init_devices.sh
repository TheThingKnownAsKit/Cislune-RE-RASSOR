devices=()
for d in /dev/ttyUSB0 /dev/ttyACM0 /dev/video0 /dev/dri ; do
  [[ -e $d ]] && devices+=( "--device=$d:$d" )
done
echo "${devices[@]}"