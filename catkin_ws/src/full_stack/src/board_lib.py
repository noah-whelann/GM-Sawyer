#!/usr/bin/env python

import cv2
from cv_bridge import CvBridge
import numpy as np

import rospkg

import time
import concurrent.futures

class BoardUpdater:
    def __init__(self):
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('full_stack')
        images_path = package_path + "/src/images"

        white_pawn = cv2.imread(images_path + "/white_pawn2.bmp", cv2.IMREAD_GRAYSCALE)
        white_bishop = cv2.imread(images_path + "/white_bishop.bmp", cv2.IMREAD_GRAYSCALE)
        white_rook = cv2.imread(images_path + "/white_rook.bmp", cv2.IMREAD_GRAYSCALE)
        white_queen = cv2.imread(images_path + "/white_queen.bmp", cv2.IMREAD_GRAYSCALE)
        white_knight = cv2.imread(images_path + "/white_knight.bmp", cv2.IMREAD_GRAYSCALE)
        white_king = cv2.imread(images_path + "/white_king.bmp", cv2.IMREAD_GRAYSCALE)
        black_pawn = cv2.imread(images_path + "/black_pawn.bmp", cv2.IMREAD_GRAYSCALE)
        black_bishop = cv2.imread(images_path + "/black_bishop.bmp", cv2.IMREAD_GRAYSCALE)
        black_rook = cv2.imread(images_path + "/black_rook.bmp", cv2.IMREAD_GRAYSCALE)
        black_queen = cv2.imread(images_path + "/black_queen.bmp", cv2.IMREAD_GRAYSCALE)
        black_knight = cv2.imread(images_path + "/black_knight.bmp", cv2.IMREAD_GRAYSCALE)
        black_king = cv2.imread(images_path + "/black_king.bmp", cv2.IMREAD_GRAYSCALE)

        self.piece_templates = {
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

        self.piece_colors = {
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

        self.thresholds = {
            'P': 0.675,
            'B': 0.65,
            'R': 0.7,
            'Q': 0.625,
            'N': 0.65,
            'K': 0.55,
            'b': 0.85,
            'p': 0.875,
            'r': 0.85,
            'q': 0.8,
            'n': 0.8,
            'k': 0.7
        }


    def get_tile_coords(self, imgmsg):
        bridge = CvBridge()
        
        image = bridge.imgmsg_to_cv2(imgmsg, "bgr8")

        ret, corners = cv2.findChessboardCorners(image, (7, 7), None)

        corners = corners.squeeze()

        all_corners = np.empty((0, 2))

        # top left corner
        corner = corners[0] + corners[0] - corners[8]
        cv2.circle(image, (int(corner[0]), int(corner[1])), 30, (0, 255, 0), 4)  # Draw the circle
        all_corners = np.append(all_corners, corner.reshape(1, -1), axis=0)

        # top right corner
        corner = corners[6] + corners[6] - corners[12]
        cv2.circle(image, (int(corner[0]), int(corner[1])), 30, (0, 255, 0), 4)  # Draw the circle
        all_corners = np.append(all_corners, corner.reshape(1, -1), axis=0)

        # bottom left corner
        corner = corners[42] + corners[42] - corners[36]
        cv2.circle(image, (int(corner[0]), int(corner[1])), 30, (0, 255, 0), 4)  # Draw the circle
        all_corners = np.append(all_corners, corner.reshape(1, -1), axis=0)
        
        # bottom right corner
        corner = corners[48] + corners[48] - corners[40]
        cv2.circle(image, (int(corner[0]), int(corner[1])), 30, (0, 255, 0), 4)  # Draw the circle
        all_corners = np.append(all_corners, corner.reshape(1, -1), axis=0)

        for i in range(49):
            color = (0, 255, 0)
            if i // 7 == 0: # first row
                corner = 2 * corners[i] - corners[i+7]
                cv2.circle(image, (int(corner[0]), int(corner[1])), 30, (0, 0, 255), 4)  # Draw the circle
                all_corners = np.append(all_corners, corner.reshape(1, -1), axis=0)
            
            elif i // 7 == 6:
                corner = 2 * corners[i] - corners[i-7]
                cv2.circle(image, (int(corner[0]), int(corner[1])), 30, (0, 0, 255), 4)  # Draw the circle
                all_corners = np.append(all_corners, corner.reshape(1, -1), axis=0)
            
            if i % 7 == 0: # left edge
                corner = 2 * corners[i] - corners[i+1]
                cv2.circle(image, (int(corner[0]), int(corner[1])), 30, (0, 0, 255), 4)  # Draw the circle
                all_corners = np.append(all_corners, corner.reshape(1, -1), axis=0)

            if i % 7 == 6: # right edge
                corner = 2 * corners[i] - corners[i-1]
                cv2.circle(image, (int(corner[0]), int(corner[1])), 30, (0, 0, 255), 4)  # Draw the circle
                all_corners = np.append(all_corners, corner.reshape(1, -1), axis=0)

            corner = corners[i]
            cv2.circle(image, (int(corner[0]), int(corner[1])), 30, (0, 0, 255), 4)  # Draw the circle
            all_corners = np.append(all_corners, corner.reshape(1, -1), axis=0)

        all_corners = all_corners.tolist()

        np_array = np.array(all_corners)
        np_array = np.round(np_array, 2)

        all_corners = np_array.tolist()

        all_corners.sort(key=lambda x: -x[1])

        board_corners = []
        for i in range(9):
            board_corners.append([])
            for j in range(9):
                board_corners[i].append(all_corners[i * 9 + j])

        for row in board_corners:
            row.sort(key=lambda y: -y[0])

        mapper = {}
        for alpha in range(ord('a'), ord('i')): # 8
            for num in range(1, 9):
                curr_key = f"{chr(alpha)}{num}"
                bottom_left_corner = board_corners[alpha - ord('a') + 1][num - 1]
                top_right_corner = board_corners[alpha - ord('a')][num]
                mapper[curr_key] = ((bottom_left_corner), (top_right_corner))

        cv2.imshow('Chessboard',image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        
        return mapper

    def get_piece_coords(self, imgmsg):
        bridge = CvBridge()

        image = bridge.imgmsg_to_cv2(imgmsg, "bgr8")
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        matches = self.find_chess_pieces(image)
        pieces_to_coords = {}
        for match in matches:
            pieces_to_coords[match["piece"]] = match["location"]

        processed_image = self.draw_matches(image, matches)

        cv2.imshow('Chessboard',processed_image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        return pieces_to_coords

    def find_chess_pieces(self, image, distance_threshold=50, angles=range(-16, 16, 2)):
        """
        Finds all the chess pieces in an images, identifies them, and returns their location
        """
        matches = []

        # First blur the image
        image = cv2.GaussianBlur(image, (11,11), 0)
        
        # Perform rotation-invariant matching
        t0 = time.time()
        for piece in self.piece_templates.keys():
            matches += self.rotation_invariant_template_matching(image, self.piece_templates[piece], piece, angles, threshold=self.thresholds[piece])
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

    def rotation_invariant_template_matching(self, image, template, piece, angles, threshold=0.8):
        """
        Perform template matching with rotation invariance.
        """

        def worker(angle):
            # Rotate the template
            rotated_template = self.rotate_image(template, angle)

            # Perform template matching
            result = cv2.matchTemplate(image, rotated_template, cv2.TM_CCOEFF_NORMED)
            (yCoords, xCoords) = np.where(result >= threshold)
            
            h, w = rotated_template.shape[:2]
            # Check if the match is above the threshold
            matches = []
            for (x, y) in zip(xCoords, yCoords):
                matches.append({
                    "location": (x + w//2, y + h//2),
                    "angle": angle,
                    "score": result[y,x],
                    "piece": piece
                })
            return matches
        
        executor = concurrent.futures.ThreadPoolExecutor(30)
        futures = [executor.submit(worker, angle) for angle in angles]

        best_matches = []
        for future in concurrent.futures.as_completed(futures):
            best_matches.extend(future.result())

    def rotate_image(self, image, angle):
        """
        Rotate an image by the specified angle.
        """
        h, w = image.shape[:2]
        center = (w // 2, h // 2)
        rotation_matrix = cv2.getRotationMatrix2D(center, angle, 1.0)
        rotated = cv2.warpAffine(image, rotation_matrix, (w, h), flags=cv2.INTER_LINEAR)
        return rotated
    
    def draw_matches(self, image, matches):
        # Draw matches
        print(f"\n*******************\nDoing matches\n")
        image_color = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        for match in matches:
            h, w = self.piece_templates[match['piece']].shape[:2]
            print(f"Detected {match['piece']} at {match['location']} with angle {match['angle']} (score: {match['score']:.2f})")

            # Optionally draw a rectangle around the match
            top_left = match["location"]
            top_left = (top_left[0] - w//2, top_left[1] - h//2)
            bottom_right = (top_left[0] + w, top_left[1] + h)
            cv2.rectangle(image_color, top_left, bottom_right, self.piece_colors[match['piece']], 2)
        print(f"Found {len(matches)} Matches")
        print("*******************\n")
        return cv2.cvtColor(image_color, cv2.COLOR_RGB2BGR)