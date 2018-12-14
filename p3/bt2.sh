echo "Bluetooth connection script (BT module 2)"
hcitool scan
sudo rfcomm connect 0 00:06:66:45:DC:6C 1
sudo rfcomm release 0
echo "rfcomm released!..."

