"""
Copyright (c) 2026 Chiara Ferraioli

This module defines constants for the Spot robot, including coordinate frame 
names and camera intrinsic parameters.
"""

# ===== FRAME NAMES =====
BODY_FRAME = "body"
ODOM_FRAME = "odom"
VISION_FRAME = "vision"
HAND_FRAME = "hand"
HEAD_FRAME = "head"

# Intermediate Frames
FRONTRIGHT_FRAME = "frontright"
FRONTLEFT_FRAME = "frontleft"
RIGHT_FRAME = "right"
LEFT_FRAME = "left"
BACK_FRAME = "back"

# Cameras Frames
BACK_CAMERA_FRAME = "back_fisheye"
FRONTLEFT_CAMERA_FRAME = "frontleft_fisheye"
FRONTRIGHT_CAMERA_FRAME = "frontright_fisheye"
LEFT_CAMERA_FRAME = "left_fisheye"
RIGHT_CAMERA_FRAME = "right_fisheye"

# ===== CAMERA INTRINSICS =====

CAMERA_INTRINSICS = {
    FRONTLEFT_CAMERA_FRAME: {
        'fx': 330.7691955566406,
        'fy': 330.9757080078125,
        'cx': 318.3247375488281,
        'cy': 237.50799560546875
    },
    FRONTRIGHT_CAMERA_FRAME: {
        'fx': 329.4346008300781,
        'fy': 329.8037414550781,
        'cx': 314.97747802734375,
        'cy': 237.4246063232422
    },
    LEFT_CAMERA_FRAME: {
        'fx': 329.6697692871094,
        'fy': 329.885009765625,
        'cx': 319.45611572265625,
        'cy': 235.0216827392578
    },
    RIGHT_CAMERA_FRAME: {
        'fx': 331.18707275390625,
        'fy': 331.398681640625,
        'cx': 309.7239074707031,
        'cy': 242.5687713623047
    },
    BACK_CAMERA_FRAME: {
        'fx': 329.2738037109375,
        'fy': 329.7651672363281,
        'cx': 327.1336975097656,
        'cy': 238.9302215576172
    },
}
