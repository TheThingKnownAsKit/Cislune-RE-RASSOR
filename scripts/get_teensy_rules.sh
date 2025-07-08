# Keeps installing the file in the repository for some reason. TODO fix later
curl -sSL https://www.pjrc.com/teensy/00-teensy.rules \
| sudo tee /etc/udev/rules.d/00-teensy.rules >/dev/null

sudo udevadm control --reload-rules && sudo udevadm trigger

sudo usermod -aG dialout $USER