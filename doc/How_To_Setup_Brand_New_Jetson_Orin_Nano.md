1. Download the Nvidia sdkmanager from [https://developer.nvidia.com/sdk-manager](https://developer.nvidia.com/sdk-manager)  
2. Connect the orin to your computer using a usb to usb-c   
   1. Make sure the orin has the sd card in it  
3. Open the sdkmanager  
   1. Deselect host computer  
   2. Select Jetson Orin Nano Developer Kit Version  
   3. Continue to step 2. The rest should be self explanatory. Install only CUDA and NVIDIA Container Runtime
   4. **If sdkmanager cannot find the board, see the below section on putting it in recovery mode.**  
4. After step 3.c is finished and everything is installed it should be good to go with ubuntu and everything on it  
   1. It probably initially installs Ubuntu 22.04, but we want Ubuntu 24.04. To upgrade this, open a terminal and run the commands below in order  
      1. sudo apt update  
      2. sudo apt full-upgrade  
         1. Will probably need to reboot  
      3. sudo do-release-upgrade

FORCED RECOVERY MODE
1. Make sure the orin is NOT plugged into a power source  
2. Get a female to female jumper wire (or wire cap, both work) and connect the FC REC and GND pins together
    - Note that the pins are located on the side opposite all the ports and underneath the fan and raised board. Easy to miss
3. Plug the orin into its power source  
4. Go back to step 3 with sdkmanager, it should show up as a discoverable board now