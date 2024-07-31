KB explorer
==========

To use
------

Install and start `rosbridge_server`:

```
sudo apt install ros-humble-rosbridge-server
ros2 run rosbridge_server rosbridge_websocket
```

Then:

```
python3 -m http.server
```

And open http://localhost:8000/ in your webbrowser.

To compile and test
-------------------

```
npm install
npm run build
python3 -m http.server
```
