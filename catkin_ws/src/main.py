#!/usr/bin/env python
## CHECKLIST BEFORE RUNNING DEMO ##
# RVIZ Open (?)
# Intera server running
# Camera screwed, wires ziptied or not in the way
# Validate Z position is correct (in case of shifted table)


# Step 1:
# Define piece pickup z coordinate (lowest point robot should go/pick up piece at)
#

'''
commands to run
roscore
roslaunch ar_tag_tracking logitech_c920.launch
roslaunch ar_tag_tracking ar_tracking.launch
roslaunch ar_tag_tracking sawyer_camera_track.launch
rosrun rviz rviz
rosrun chess_tracking find_initial_transform.py
rosrun chess_tracking piece_detect.py
rosrun chess_tracking transform_coordinates_service.py
rosrun chess_tracking board_service.py
rosrun intera_interface joint_trajectory_action_server.py 
roslaunch sawyer_moveit_config sawyer_moveit.launch electric_gripper:=true
python3 main.py
'''

from csv import Error
import numpy as np
from stockfish import Stockfish
from planning.chessboard import ChessBoard, TileObject
import rospy
from geometry_msgs.msg import Point, PointStamped
from move_arm.src.pickup_integ import pickup_and_place, calibrate_gripper, tuck
import rospkg
import requests
import json
# from chess_tracking.src.transform_coordinates import transform_point_to_base
from chess_tracking.srv import BoardString
from chess_tracking.srv import PieceMatches
from chess_tracking.srv import TransformPoint

rospy.wait_for_service("board_service")
rospy.wait_for_service("match_pieces")
rospy.wait_for_service("transform_camera_coordinates")


def get_next_move(fen):
    header = {fen}
    response = requests.post("https://chess-api.com/v1", json=header)
    response.raise_for_status()  # check for error

    data = json.loads(response.text)

    best_move = data['move']

    return best_move


def get_board_state(board: ChessBoard):  # return fen of current board state

    return_fen = []

    for row in range(8, 0, -1):  # iterate down the rows of the board
        empty = 0
        each_row = []
        for i in "ABCDEFGH":  # scan left to right top to bottom
            tile_name = f"{i}{row}"
            piece = board.chess_tiles[tile_name].piece if tile_name in board.chess_tiles else ""

            if piece == "":
                empty += 1
            else:
                if empty > 0:
                    each_row.append(str(empty))
                each_row.append(piece)

            return_fen.append("".join(each_row))

    # combine all rows
    piece_placement = "/".join(each_row)

    # placeholders w KQkq - 0 1
    # white to move, KQ -> black can castle kingside
    # "- -" means no one can castle
    # en passant stuff must be added later
    # last two numers are the halfmove number and fullmove number respectively
    return f"{piece_placement} w KQkq - 0 1"


def transform_camera_to_world(pixel_coords: Point) -> Point:
    transform_service = rospy.ServiceProxy("transform_camera_coordinates")

    # first turn pixel_coords into it's respective data type: "TransformPoint"
    request = TransformPoint()
    request.input_point.x = pixel_coords.x
    request.input_point.y = pixel_coords.y
    request.input_point.z = pixel_coords.z

    # real_coords is of type TransformPoint
    real_coords = transform_service(request)

    # exract the point from TransformPoint datatype
    return real_coords.transformed_point


def get_piece_location_on_tile(tile: str, board: ChessBoard) -> Point:
    pixel_coords = board.chess_tiles[tile].piece_location

    return transform_camera_to_world(pixel_coords)


def update_piece_locations(board: ChessBoard) -> None:
    call_piece_service = rospy.ServiceProxy("match_pieces", PieceMatches)
    matches = call_piece_service()
    for tile_name, tile in board.chess_tiles.items():
        tile.has_piece = False  # assume no tiles are occupied by a piece
        for i in len(matches.x):
            if (np.linalg.norm((tile.x, tile.y) - (matches.x[i], matches.y[i])) <= 50):
                tile.piece = matches.name[i]
                tile.piece_location.x = tile.x
                tile.piece_location.y = tile.y
                tile.has_piece = True

# simply updates all tiles on the board with their corresponding (x, y) coordinates


def update_tile_locations(board: ChessBoard) -> None:
    try:
        call_board_service = rospy.ServiceProxy("board_service", BoardString)

        resp = call_board_service().output

        # consits of a dicitions with mappings of tile names to their pixel coords in a tuple
        dictionary_mapping = json.loads(resp)

        for currTile in dictionary_mapping:
            bottom_left_corner = dictionary_mapping[currTile][0]
            top_right_corner = dictionary_mapping[currTile][1]

            center_x_coord = (bottom_left_corner[0] + top_right_corner[0]) / 2
            center_y_coord = (bottom_left_corner[1] + top_right_corner[1]) / 2

            board.chess_tiles[currTile].x = center_x_coord
            board.chess_tiles[currTile].y = center_y_coord

            # Create the TransformPoint request
            # point_request = Point()

            # point_request.x = center_x_coord
            # point_request.y = center_y_coord
            # point_request.z = 1.0  # Hardcoded depth

            # # Call the transform_coordinates service

            # # return type is of Point.transformed_point (consists of x, y, z)
            # real_coords = transform_service(point_request)

            # # Print the transformed point in the base frame
            # print(f"Transformed point for tile {currTile}: x={real_coords.transformed_point.x}, "
            #       f"y={real_coords.transformed_point.y}, z={real_coords.transformed_point.z}")
    except:
        return


def main():

    rospy.init_node("main_node", anonymous=True)

    # rospack = rospkg.RosPack()
    # package_path = rospack.get_path('chess_tracking')  # Replace with your package name

    # stockfish_path = package_path + "/src/stockfish-ubuntu-x86-64-vnni512"
    # print(stockfish_path)
    # stockfish = Stockfish(stockfish_path)  # init Stockfish
    # stockfish.set_skill_level(5)
    board = ChessBoard()  # initialize chessboard class
    board.create_board()

    input("press enter to calibrate gripper")
    calibrate_gripper()

    input("press enter to tuck robot")
    tuck()

    update_tile_locations(board)
    update_piece_locations(board)

    drop_off = (0.722, -0.013)  # piece drop off spot (after taking)

    gaming = True
    while gaming:
        update_tile_locations(board)
        update_piece_locations(board)
        # passes FEN of board state into stockfish, gets next move
        next_move = get_next_move(get_board_state())
        capture = False
        if next_move is None:  # probably needs a better game state
            gaming = False
            break

        pickup_tile = next_move[0:2]  # e2
        place_tile = next_move[2:]  # e4

        pickup_tile_coords = get_piece_location_on_tile(pickup_tile)
        # accesses board hashmap and grabs tile xy
        place_tile_coords = (Point(
            board.chess_tiles[place_tile].x, board.chess_tiles[place_tile].y, 0))

        if board.chess[place_tile].piece is not None:  # Taking a piece
            capture = True
            pickup_and_place(place_tile_coords, drop_off,
                             capture)  # move piece off board
            rospy.sleep(1.0)
            capture = False
        pickup_and_place(pickup_tile_coords, place_tile_coords,
                         capture)  # should be one fluid motion
        # robot then tucks after its move (automatically handled in pickup_integ.py)

        # wait for user to execute move
        input("Press enter when you have moved the piece")


if __name__ == '__main__':
    main()


# example main loop:
# convert board state to fen
# send fen state to stockfish
# get next move -> returns e2e4 for example
# grab piece on e2
    # find which piece is on e2
    # get that pieces location
    # transform that location to world coordinates
# pickup piece
# move to e4
    # find center of e4 tile
    # transform center of tile to world coordinates
    # move robot and place
# update new state of the board
# tuck robot
# wait for player to move piece (prompt user)
# user presses key indicating move has been done
# call get_board_state to update the state
# convert that to fen
# repeat
#
