import numpy as np
import cv2

# Canvas dimensions
canvas_height = 501
canvas_width = 1201
clearance_color = (127, 127, 127)
obstacle_color = (0, 0, 0)
free_space_color = (255, 255, 255)
clearance_distance = 5

# Initialize a white canvas
canvas = np.ones((canvas_height, canvas_width, 3), dtype="uint8") * 255

def obstacles(node):
    x, y = node
    Hex_center = (650, 250)
    Xc, Yc = Hex_center
    y = abs(y - canvas_height)
    side_length = 150
    R = np.cos(np.pi / 6) * side_length
    obstacles = [
        (x >= 100 and x <= 175 and y >= 100 and y <= 500),
        (x >= 275 and x <= 350 and y >= 0 and y <= 400),
        (x >= 900 and x <= 1100 and y >= 50 and y <= 125),
        (x >= 900 and x <= 1100 and y >= 375 and y <= 450),
        (x >= 1020 and x <= 1100 and y >= 50 and y <= 450),
        (x >= Xc - R and x <= Xc + R and y <= ((np.pi/6)*(x-(Xc-R)))+325 and y <= -((np.pi/6)*(x-(Xc+R)))+325 and y >= -((np.pi/6)*(x-(Xc-R)))+175 and y >= ((np.pi/6)*(x-(Xc+R)))+175),
        
    ]
    return any(obstacles)

def clearance(x, y, clearance):
    Hex_center = (650, 250)
    Xc, Yc = Hex_center
    y = abs(y - canvas_height)
    side_length = 150 
    R = (np.cos(np.pi / 6) * side_length)  + clearance
    clearance_zones = [
        (x >= 100 - clearance and x <= 175 + clearance and y >= 100 - clearance and y <= 500 + clearance),
        (x >= 275 - clearance and x <= 350 + clearance and y >= 0 - clearance and y <= 400 + clearance),
        (x >= 900 - clearance and x <= 1100 + clearance and y >= 50 - clearance and y <= 125 + clearance),
        (x >= 900 - clearance and x <= 1100 + clearance and y >= 375 - clearance and y <= 450 + clearance),
        (x >= 1020 - clearance and x <= 1100 + clearance and y >= 50 - clearance and y <= 450 + clearance),
        (x >= Xc - R and x <= Xc + R and y <= ((np.pi/6)*(x-(Xc-R)))+325 + clearance and y <= -((np.pi/6)*(x-(Xc+R)))+325 + clearance and y >= -((np.pi/6)*(x-(Xc-R)))+175 - clearance and y >= ((np.pi/6)*(x-(Xc+R)))+175 - clearance),
        (x <= clearance or x >= canvas_width - clearance or y <= clearance or y >= canvas_height - clearance),
    ]
    return any(clearance_zones)

for x in range(canvas_width):
    for y in range(canvas_height):
        if clearance(x, y, clearance_distance):
            canvas[y, x] = clearance_color
        if obstacles((x, y)):
            canvas[y, x] = obstacle_color



cv2.imshow('Canvas', canvas)
cv2.waitKey(0)
cv2.destroyAllWindows()
