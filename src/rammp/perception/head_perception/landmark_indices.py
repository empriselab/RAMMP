"""Selection of MediaPipe face-mesh landmarks used for rigid head tracking.

The non-rigid regions (lips, eyes, eyebrows, irises) move with facial
expression and are excluded so that, e.g., opening the mouth does not shift
the estimated head pose.

RIGID_LANDMARK_INDICES was derived once as ``set(range(468))`` minus every
index appearing in MediaPipe's FACEMESH_LIPS, FACEMESH_LEFT_EYE,
FACEMESH_RIGHT_EYE, FACEMESH_LEFT_EYEBROW, FACEMESH_RIGHT_EYEBROW,
FACEMESH_LEFT_IRIS and FACEMESH_RIGHT_IRIS connection sets. It is baked in as
a literal so the runtime does not depend on MediaPipe's deprecated
``solutions`` module (removed in mediapipe 0.10.x).
"""

# Name of the blendshape that scores how open the jaw/mouth is (0..1).
JAW_OPEN_BLENDSHAPE = "jawOpen"

# Rigid face-mesh landmark indices (the 468-point mesh minus lips, eyes,
# eyebrows, and irises). 376 indices.
RIGID_LANDMARK_INDICES: list[int] = [
    1, 2, 3, 4, 5, 6, 8, 9, 10, 11, 12, 15, 16, 18, 19, 20, 21, 22, 23, 24,
    25, 26, 27, 28, 29, 30, 31, 32, 34, 35, 36, 38, 41, 42, 43, 44, 45, 47,
    48, 49, 50, 51, 54, 56, 57, 58, 59, 60, 62, 64, 67, 68, 69, 71, 72, 73,
    74, 75, 76, 77, 79, 83, 85, 86, 89, 90, 92, 93, 94, 96, 97, 98, 99, 100,
    101, 102, 103, 104, 106, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117,
    118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132,
    134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 147, 148, 149, 150, 151,
    152, 156, 162, 164, 165, 166, 167, 168, 169, 170, 171, 172, 174, 175, 176,
    177, 179, 180, 182, 183, 184, 186, 187, 188, 189, 190, 192, 193, 194, 195,
    196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210,
    211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 224, 225,
    226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240,
    241, 242, 243, 244, 245, 247, 248, 250, 251, 252, 253, 254, 255, 256, 257,
    258, 259, 260, 261, 262, 264, 265, 266, 268, 271, 272, 273, 274, 275, 277,
    278, 279, 280, 281, 284, 286, 287, 288, 289, 290, 292, 294, 297, 298, 299,
    301, 302, 303, 304, 305, 306, 307, 309, 313, 315, 316, 319, 320, 322, 323,
    325, 326, 327, 328, 329, 330, 331, 332, 333, 335, 337, 338, 339, 340, 341,
    342, 343, 344, 345, 346, 347, 348, 349, 350, 351, 352, 353, 354, 355, 356,
    357, 358, 359, 360, 361, 363, 364, 365, 366, 367, 368, 369, 370, 371, 372,
    376, 377, 378, 379, 383, 389, 391, 392, 393, 394, 395, 396, 397, 399, 400,
    401, 403, 404, 406, 407, 408, 410, 411, 412, 413, 414, 416, 417, 418, 419,
    420, 421, 422, 423, 424, 425, 426, 427, 428, 429, 430, 431, 432, 433, 434,
    435, 436, 437, 438, 439, 440, 441, 442, 443, 444, 445, 446, 447, 448, 449,
    450, 451, 452, 453, 454, 455, 456, 457, 458, 459, 460, 461, 462, 463, 464,
    465, 467,
]
