

## capture_checkerboard.py

- `detect_checker_board` function:
    - **Input**:
        - `image`: current frame captured from the camera
        - `grayImage`: grayscale version of the current frame
        - `criteria`: termination criteria used for cornerSubPix function
        - `boardDimension`: dimensions of the checkerboard (9, 6)
    - **Output**:
        - `image`: current frame with corners of the checkerboard drawn on it
        - `ret`: a flag indicating whether a checkerboard was detected in the image
    - This function detects the checkerboard in the current frame and draws the corners of the checkerboard on the frame.

- Main loop:

    - Initializes the Camera object with the resolution 1920x1080
    - Checks if the "images" directory exists, if not creates it
    - Continuously captures frames from the camera and displays it
    - Waits for user input:
  
        - if "q" is pressed, the loop breaks and the program ends
        - if "s" is pressed and a checkerboard was detected in the current frame, the code saves the current frame in the "images" directory
    - The number of saved images is displayed after the program ends.

## calibrate_camera.py

- The code is for camera calibration using the checkerboard pattern.
- `CHESS_BOARD_DIM` = (23, 15): Specifying the dimensions of the checkerboard pattern.

- `SQUARE_SIZE` = 35: Specifying the size of each square on the checkerboard pattern in millimeters.

- `calib_data_path` = "../calib_data": Specifying the path to the directory where the calibration data will be saved.

- `CHECK_DIR` = os.path.isdir(calib_data_path): Checking if the `calib_data_path` directory exists.

- if not `CHECK_DIR`: os.makedirs(`calib_data_path`): If the directory does not exist, it creates a new directory with the specified path.


- `image_dir_path` = "images": Specifying the path to the directory where the images used for calibration are stored.

- `cv.findChessboardCorners(image, CHESS_BOARD_DIM, None)`: Finding the corners of the checkerboard pattern in the image.
- `cv.drawChessboardCorners(image, CHESS_BOARD_DIM, corners2, ret)`: Drawing the corners of the checkerboard pattern on the image.
- `cv.calibrateCamera(obj_points_3D, img_points_2D, grayScale.shape[::-1], None, None,flags=cv.CALIB_RATIONAL_MODEL
)`: Calibrating the camera using the checkerboard pattern.
