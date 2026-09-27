# AFC bag recording — install (once, on the Pi)

Files go in the repo at `rpi_ros2/tools/`: `afc_record.sh`, `afc_rec`,
`afc-record@.service`.

```bash
cd ~/afc-drone/rpi_ros2/tools
chmod +x afc_record.sh afc_rec
sudo cp afc-record@.service /etc/systemd/system/
sudo systemctl daemon-reload
ln -sf ~/afc-drone/rpi_ros2/tools/afc_rec ~/.local/bin/afc_rec   # or add tools/ to PATH
mkdir -p ~/bags
```

If `~/.bashrc` sets `ROS_DOMAIN_ID` or `RMW_IMPLEMENTATION`, copy the same values
into the `Environment=` lines of the unit (then `daemon-reload`), or the recorder
won't see the graph.

Optional — no password prompt for start/stop (edit with `sudo visudo -f /etc/sudoers.d/afc-record`):

```
pi ALL=(root) NOPASSWD: /usr/bin/systemctl start afc-record@*, /usr/bin/systemctl stop afc-record@*
```

## Use

```bash
afc_rec start T-C5     # new bag ~/bags/afc_<date>_<time>_T-C5
afc_rec status         # recording? size, free disk
afc_rec stop           # clean close + `ros2 bag info`
afc_rec ls             # all bags with sizes
```

Recording survives the SSH session dropping (it runs under systemd, not your
shell). It never starts at boot. Start it before or after the stack — the
recorder picks the topics up when they appear.

Pull bags to the desktop: `scp -r pi@10.223.250.2:~/bags/afc_<name> .`

---

# Bridge + rosbridge as a boot service (S14)

The agent already runs as `microxrce-agent.service`. This adds the bridge and
rosbridge as a second, independent service, so the whole stack comes up at boot
and restarting the bridge can never cycle the agent.

```bash
cd ~/afc-drone/rpi_ros2/tools
chmod +x afc_bridge_run.sh
sudo cp afc-bridge.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now afc-bridge
journalctl -u afc-bridge -f          # expect "opened /dev/ttyAMA1 (exclusive)" and the 1 Hz [health] line
```

Same `Environment=` caveat as the recorder. To run the bridge by hand (debugging),
`sudo systemctl stop afc-bridge` first -- it holds the Tiny's UART exclusively.

Checks: reboot the Pi and confirm the dashboard connects with no manual steps;
`sudo systemctl restart afc-bridge` while armed recovers PRIMARY without an FC
reboot (repeat of T-C2); `systemctl status microxrce-agent` shows the agent's
uptime unchanged by bridge restarts.
