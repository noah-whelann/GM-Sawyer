import cv2
import numpy as np
from matplotlib import pyplot as plt
import time
import concurrent.futures

white_pawn = cv2.imread("./images/pieces/white_pawn.bmp", cv2.IMREAD_GRAYSCALE)
white_bishop = cv2.imread("./images/pieces/white_bishop.bmp", cv2.IMREAD_GRAYSCALE)
white_rook = cv2.imread("./images/pieces/white_rook.bmp", cv2.IMREAD_GRAYSCALE)
white_queen = cv2.imread("./images/pieces/white_queen.bmp", cv2.IMREAD_GRAYSCALE)
white_knight = cv2.imread("./images/pieces/white_knight.bmp", cv2.IMREAD_GRAYSCALE)
white_king = cv2.imread("./images/pieces/white_king.bmp", cv2.IMREAD_GRAYSCALE)
black_pawn = cv2.imread("./images/pieces/black_pawn.bmp", cv2.IMREAD_GRAYSCALE)
black_bishop = cv2.imread("./images/pieces/black_bishop.bmp", cv2.IMREAD_GRAYSCALE)
black_rook = cv2.imread("./images/pieces/black_rook.bmp", cv2.IMREAD_GRAYSCALE)
black_queen = cv2.imread("./images/pieces/black_queen.bmp", cv2.IMREAD_GRAYSCALE)
black_knight = cv2.imread("./images/pieces/black_knight.bmp", cv2.IMREAD_GRAYSCALE)
black_king = cv2.imread("./images/pieces/black_king.bmp", cv2.IMREAD_GRAYSCALE)

piece_templates = {
    'P': white_pawn,
    'B': white_bishop,
    'R': white_rook,
    'Q': white_queen,
    'N': white_knight,
    'K': white_king,
    'p': black_pawn,
    'b': black_bishop,
    'r': black_rook,
    'q': black_queen,
    'n': black_knight,
    'k': black_king
}

piece_colors = {
    'P': (255, 255, 255),  # White Pawn - White
    'B': (0, 255, 0),      # White Bishop - Green
    'R': (0, 0, 255),      # White Rook - Red
    'Q': (255, 255, 0),    # White Queen - Cyan
    'N': (255, 0, 255),    # White Knight - Magenta
    'K': (255, 165, 0),    # White King - Orange
    'p': (128, 128, 128),  # Black Pawn - Gray
    'b': (0, 100, 0),      # Black Bishop - Dark Green
    'r': (42, 42, 165),    # Black Rook - Brown
    'q': (128, 0, 128),    # Black Queen - Purple
    'n': (203, 192, 255),  # Black Knight - Pink
    'k': (255, 182, 193),  # Black King - Light Blue
}

thresholds = {
    'P': 0.8,
    'B': 0.8,
    'R': 0.75,
    'Q': 0.85,
    'N': 0.75,
    'K': 0.7,
    'b': 0.915,
    'p': 0.875,
    'r': 0.875,
    'q': 0.9,
    'n': 0.92,
    'k': 0.9
}

def rotate_image(image, angle):
    """
    Rotate an image by the specified angle.
    """
    h, w = image.shape[:2]
    center = (w // 2, h // 2)
    rotation_matrix = cv2.getRotationMatrix2D(center, angle, 1.0)
    rotated = cv2.warpAffine(image, rotation_matrix, (w, h), flags=cv2.INTER_LINEAR)
    return rotated



def rotation_invariant_template_matching(image, template, piece, angles, threshold=0.8):
    """
    Perform template matching with rotation invariance.
    """

    def worker(angle):
        # Rotate the template
        rotated_template = rotate_image(template, angle)

        # Perform template matching
        result = cv2.matchTemplate(image, rotated_template, cv2.TM_CCOEFF_NORMED)
        (yCoords, xCoords) = np.where(result >= threshold)
        
        # Check if the match is above the threshold
        matches = []
        for (x, y) in zip(xCoords, yCoords):
            matches.append({
                "location": (x, y),
                "angle": angle,
                "score": result[y,x],
                "piece": piece
            })
        return matches
    
    executor = concurrent.futures.ThreadPoolExecutor(10)
    futures = [executor.submit(worker, angle) for angle in angles]

    best_matches = []
    for future in concurrent.futures.as_completed(futures):
        best_matches.extend(future.result())

    
    return best_matches

def find_chess_pieces(image, distance_threshold=50, angles=range(0, 360, 10)):
    """
    Finds all the chess pieces in an images, identifies them, and returns their location
    """
    matches = []

    # First blur the image
    image = cv2.GaussianBlur(image, (3,3), 0)
    
    # Perform rotation-invariant matching
    t0 = time.time()
    for piece in piece_templates.keys():
        matches += rotation_invariant_template_matching(image, piece_templates[piece], piece, angles, threshold=thresholds[piece])
    t1 = time.time()
    print("total matching time: " + str(t1-t0))

    # Remove any duplicate matches and only keep the best one
    matches = sorted(matches, key=lambda x: x["score"], reverse=True)
    filtered_matches = []
    while matches:
        # Pick the match with the highest score
        best_match = matches.pop(0)
        filtered_matches.append(best_match)
        
        # Remove duplicates within the threshold
        matches = [
            match for match in matches
            if np.linalg.norm(np.array(match["location"]) - np.array(best_match["location"])) > distance_threshold
        ]

    return filtered_matches

def draw_matches(image, matches, name):
    # Draw matches
    print(f"\n*******************\nDoing matches for {name}\n")
    image_color = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    for match in matches:
        h, w = piece_templates[match['piece']].shape[:2]
        print(f"Detected {match['piece']} at {match['location']} with angle {match['angle']} (score: {match['score']:.2f})")

        # Optionally draw a rectangle around the match
        top_left = match["location"]
        bottom_right = (top_left[0] + w, top_left[1] + h)
        cv2.rectangle(image_color, top_left, bottom_right, piece_colors[match['piece']], 2)
    print(f"Found {len(matches)} Matches")
    print("*******************\n")
    plt.imshow(cv2.cvtColor(image_color, cv2.COLOR_RGB2BGR), 'gray'), plt.show()



board_0 = cv2.imread("./images/boards/board_0.png", cv2.IMREAD_GRAYSCALE)
board_1 = cv2.imread("./images/boards/board_1.png", cv2.IMREAD_GRAYSCALE)
board_2 = cv2.imread("./images/boards/board_2.png", cv2.IMREAD_GRAYSCALE)
board_3 = cv2.imread("./images/boards/board_3.png", cv2.IMREAD_GRAYSCALE)
board_4 = cv2.imread("./images/boards/board_4.png", cv2.IMREAD_GRAYSCALE)
board_5 = cv2.imread("./images/boards/board_5.png", cv2.IMREAD_GRAYSCALE)
board_6 = cv2.imread("./images/boards/board_6.png", cv2.IMREAD_GRAYSCALE)
board_7 = cv2.imread("./images/boards/board_7.png", cv2.IMREAD_GRAYSCALE)
board_8 = cv2.imread("./images/boards/board_8.png", cv2.IMREAD_GRAYSCALE)
matches_0 = find_chess_pieces(board_0)
matches_1 = find_chess_pieces(board_1)
matches_2 = find_chess_pieces(board_2)
matches_3 = find_chess_pieces(board_3)
matches_4 = find_chess_pieces(board_4)
matches_5 = find_chess_pieces(board_5)
matches_6 = find_chess_pieces(board_6)
matches_7 = find_chess_pieces(board_7)
matches_8 = find_chess_pieces(board_8)
draw_matches(board_0, matches_0, "board_0")
draw_matches(board_1, matches_1, "board_1")
draw_matches(board_2, matches_2, "board_2")
draw_matches(board_3, matches_3, "board_3")
draw_matches(board_4, matches_4, "board_4")
draw_matches(board_5, matches_5, "board_5")
draw_matches(board_6, matches_6, "board_6")
draw_matches(board_7, matches_7, "board_7")
draw_matches(board_8, matches_8, "board_8")
