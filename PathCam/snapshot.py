import sensor, image, lcd, time, os
from maix import KPU
import gc
from modules import ybkey

# Determine the maximum image index on the SD card
image_index = 0
sd_path = "/sd/"

try:
    print("Reading SD card file list...")
    file_list = os.listdir(sd_path)  # Get all files from SD card
    print("SD card file list:", file_list)  # Debug information

    image_files = [f for f in file_list if f.startswith("image_") and f.endswith(".jpg")]

    # Extract the maximum index of existing image files
    if image_files:
        image_indices = [int(f[6:-4]) for f in image_files if f[6:-4].isdigit()]
        if image_indices:
            image_index = max(image_indices) + 1  # Get max index and add 1
            print("Detected {} images, current max index: {}".format(len(image_files), image_index - 1))
        else:
            print("No valid image indices detected, starting from 0.")
except Exception as e:
    print("Error reading SD card:", e)

# Initialize the button module
KEY = ybkey()  # Initialize the button using ybkey module

# Camera initialization
print("Initializing camera...")
sensor.reset()
sensor.set_pixformat(sensor.RGB565)  # Set to color image
sensor.set_framesize(sensor.QVGA)  # Set resolution to 320x240
# sensor.set_vflip(1)  # Flip vertically if needed
sensor.run(1)  # Start the camera
print("Camera initialized successfully.")

# LCD display initialization
print("Initializing LCD display...")
lcd.init()
print("LCD display initialized successfully.")

# Main loop
while True:
    # Capture an image frame
    img = sensor.snapshot()  # Take a picture

    # Display image in real-time on LCD
    lcd.display(img)

    # Check if the button is pressed
    if KEY.is_press():
        # If button is pressed, save the image to SD card
        file_name = "/sd/image_{}.jpg".format(image_index)
        img.save(file_name)  # Save the picture
        print("Image saved: {}".format(file_name))

        # Update image index
        image_index += 1

        # Delay to avoid repeated triggers
        time.sleep_ms(200)  # Delay 200ms to prevent bouncing

    # Ensure timely memory release
    gc.collect()

    # Delay to simulate lower refresh rate
    time.sleep_ms(50)
