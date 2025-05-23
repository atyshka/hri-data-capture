set -Eeuo pipefail
wget -N https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4a1.4-dev/libk4a1.4-dev_1.4.1_amd64.deb
wget -N https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4a1.4/libk4a1.4_1.4.1_amd64.deb
sudo apt install ./libk4a1.4_1.4.1_amd64.deb ./libk4a1.4-dev_1.4.1_amd64.deb
wget -N https://raw.githubusercontent.com/microsoft/Azure-Kinect-Sensor-SDK/refs/heads/develop/scripts/99-k4a.rules
sudo mv 99-k4a.rules /etc/udev/rules.d/