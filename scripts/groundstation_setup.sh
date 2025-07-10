






# Check to see if the GPU is nvidia (blank if not)
gpu=$(lspci | grep -i 'vga' | grep -i 'nvidia')
if [[ -n "$gpu" ]]; then
    stuff
else
    stuff
fi