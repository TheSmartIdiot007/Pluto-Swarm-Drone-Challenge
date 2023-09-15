# How to get keyboard control
### Run the following command in terminal to get keyboard control in the directory of keyboard_control.py

```basic
python3 keyboard_control.py
 ```

|key    | action|
|-------|-------|
|W      | Throttle High|
|S      | Throttle Low|
|A      | Yaw Left|
|D      | Yaw Right|
|Arrow up| Pitch Forward|
|Arrow down| Pitch Backward|
|Arrow left| Roll Left|
|Arrow right| Roll Right|
|Space| Toggle Arm|
|R| Hover|

If no key pressed then the drone will reset it's roll pitch to zero and yaw rate to zero <br>
R key reset the drone completly to hover mode



## Pipeline
 * GetKey class is responsible for logging keys pressed
 * With Pluto class in python_wrapper keyboard_control.py is able to send commands to Pluto using all the different methods avilable in Pluto Class

* We update the crrosponding RC values in the Protocol of Pluto class and send them to the drone using the writerFunc() method

  ```python 
  Pluto.writerFunc() #is being used to write RC values to the drone for control
  ```