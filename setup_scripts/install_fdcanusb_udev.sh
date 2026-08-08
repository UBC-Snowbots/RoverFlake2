#!/usr/bin/env bash
# Optional: stable /dev/fdcanusb symlink + open permissions for the mjbots
# CAN-FD USB adapter. The arm driver and tview auto-detect the adapter with
# or without this — the symlink only stops ttyACM renumbering from stranding
# a long-running driver on a dead port. Safe to skip on machines without the
# arm hardware.
set -e
echo 'SUBSYSTEM=="tty", ATTRS{manufacturer}=="mjbots", ATTRS{product}=="fdcanusb", MODE="0666", SYMLINK+="fdcanusb"' \
  | sudo tee /etc/udev/rules.d/70-fdcanusb.rules > /dev/null
sudo udevadm control --reload-rules
sudo udevadm trigger
echo "Done. Re-plug the fdcanusb and check: ls -l /dev/fdcanusb"
