# GetKey Class API Documentation

## Class: GetKey

### `__init__(self)`
Initializes the class with the following attributes:
- `key`: empty string
- `settings`: terminal settings obtained using `termios.tcgetattr(sys.stdin)`
- `reg_time`: current time obtained using `time()`

### `getKey(self)`
Waits for and stores a key press.
- Sets terminal to raw input mode using `tty.setraw(sys.stdin.fileno())`.
- Listens for input with a timeout of 0.1 seconds using `select.select()`.
- If input was received, stores the first character as `self.key`.
- If the first character is the escape key, stores the next two characters.
- Flushes the `stdin` buffer.
- Restores terminal settings using `termios.tcsetattr()`.
- If the user pressed Ctrl-C, prints "Exit" and exits the program.

### `__call__(self)`
Updates the following class attributes:
- `key`: latest key press obtained using `getKey()`.
- `reg_time`: current time obtained using `time()`.

## Variables:
- `key`: latest key press (string)
- `reg_time`: time of latest key press (float)
- `settings`: terminal settings (list)


# KeyboardControl class

## Overview
This class is used to control a Pluto object using keyboard commands. It is initialized with an arguments object and creates a Pluto object. It also creates a key logger and key listener thread to track keyboard input.

## Methods

### `__init__`
Initializes the Pluto object and arms it. Initializes the key logger object and key listener thread.

### `reset`
Resets the Pluto object.

### `incAlt`
Increases the altitude of the Pluto object.

### `decAlt`
Decreases the altitude of the Pluto object.

### `yawLeft`
Turns the Pluto object to the left.

### `yawRight`
Turns the Pluto object to the right.

### `forward`
Moves the Pluto object forward.

### `backward`
Moves the Pluto object backward.

### `right`
Moves the Pluto object to the right.

### `left`
Moves the Pluto object to the left.

### `toggleArm`
Toggles the arm state of the Pluto object.

### `_key_listener`
Continuously checks for new keystrokes.

# KeyboardControl

This class is used to control a Pluto object using a keyboard.

## Class Members

### `__init__(self, args)`

This method is the constructor for the `KeyboardControl` class. It initializes the Pluto object, key logger object and the key listener thread. It also sets the time of the last key press to the current time and sets the reset flag to False. It also creates a dictionary `keyMap` that maps keys to functions.

### `justPass(self)`

This function does nothing and is called for unknown keys.

### `reset(self)`

This function resets the Pluto object. It will only reset if it hasn't been called before.

### `hardReset(self)`

This function does a hard reset of the Pluto object.

### `incAlt(self)`

This function increases the altitude of the Pluto object.

### `decAlt(self)`

This function decreases the altitude of the Pluto object.

### `yawLeft(self)`

This function makes the Pluto object yaw to the left.

### `yawRight(self)`

This function makes the Pluto object yaw to the right.

### `forward(self)`

This function makes the Pluto object move forward.

### `backward(self)`

This function makes the Pluto object move backward.

### `right(self)`

This function makes the Pluto object move to the right.

### `left(self)`

This function makes the Pluto object move to the left.

### `toggleArm(self)`

This function toggles the arm state of the drone. It arms or disarms the drone depending on its current state.

### `_key_listener(self)`

This function continuously checks for new keystrokes.

### `__call__(self)`

This method is called when the object is called as a function. It sleeps for a short time before checking for new keystrokes. If a new keystroke has been registered, the corresponding method from the `keyMap` dictionary is called. If the key that was pressed is not an empty string, the `reset_called` flag is reset.
