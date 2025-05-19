Ran on Isaac Sim v4.2.0

```
Set-Alias -Name isaac_python -Value "C:\users\<yourname>\appdata\local\ov\pkg\isaac-sim-4.2.0\python.bat"
```

- `app` contains the robot arm python app to pick and place objects
  - NOTE: depends on `kit-exts-project`
```bash
isaac_python main.py
```
- `kit-exts-project` contains an Isaac extension to forward message bus events to MQTT
```bash
isaac_python -m pip install paho-mqtt
```
-  `test_zed` contains a scene to test out the Zed camera object position