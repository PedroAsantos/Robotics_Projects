echo "Bluetooth connection script (BT module 1)"
hcitool scan
sudo rfcomm connect 0 00:06:66:45:DC:6B 1
sudo rfcomm release 0
echo "rfcomm released!..."

