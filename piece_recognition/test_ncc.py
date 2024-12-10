import cv2
import numpy as np
from matplotlib import pyplot as plt
import time

def rotate_image(image, angle):
    """
    Rotate an image by the specified angle.
    """
    h, w = image.shape[:2]
    center = (w // 2, h // 2)
    rotation_matrix = cv2.getRotationMatrix2D(center, angle, 1.0)
    rotated = cv2.warpAffine(image, rotation_matrix, (w, h), flags=cv2.INTER_LINEAR)
    return rotated

def rotation_invariant_template_matching(image, template, angles, threshold=0.8):
    """
    Perform template matching with rotation invariance.
    """
    best_matches = []
    
    for angle in angles:
        # Rotate the template
        rotated_template = rotate_image(template, angle)

        # Perform template matching
        result = cv2.matchTemplate(image, rotated_template, cv2.TM_CCOEFF_NORMED)
        (yCoords, xCoords) = np.where(result >= threshold)
        
        # Check if the match is above the threshold
        for (x, y) in zip(xCoords, yCoords):
            best_matches.append({
                "location": (x, y),
                "angle": angle,
                "score": result[y,x]
            })
    
    return best_matches

# Load the chessboard image and template
chessboard_img = cv2.imread("./images/boards/board_1.png", cv2.IMREAD_GRAYSCALE)
# piece_templates = []
pawn_template = cv2.imread("./images/pieces/pawn.png", cv2.IMREAD_GRAYSCALE)
# bishop_template = cv2.imread("./images/pieces/bishop.png", cv2.IMREAD_GRAYSCALE)
# rook_template = cv2.imread("./images/pieces/rook.png", cv2.IMREAD_GRAYSCALE)
# queen_template = cv2.imread("./images/pieces/queen.bmp", cv2.IMREAD_GRAYSCALE)

# Define angles to test (e.g., every 15 degrees)
angles = range(0, 360, 5)

# Perform rotation-invariant matching
t0 = time.time()
pawn_matches = rotation_invariant_template_matching(chessboard_img, pawn_template, angles, threshold=0.8)
t1 = time.time()

print(t1-t0)
# Display results
for match in pawn_matches:
    print(f"Detected at {match['location']} with angle {match['angle']} (score: {match['score']:.2f})")

    # Optionally draw a rectangle around the match
    h, w = pawn_template.shape[:2]
    top_left = match["location"]
    bottom_right = (top_left[0] + w, top_left[1] + h)
    cv2.rectangle(chessboard_img, top_left, bottom_right, 255, 2)

# Show the image with matches
# chessboard_img = cv2.resize(chessboard_img, (720, 1280))
# plt.imshow("Detected Matches", chessboard_img)
plt.imshow(chessboard_img, 'gray'), plt.show()
cv2.waitKey(0)
cv2.destroyAllWindows()
