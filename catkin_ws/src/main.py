## CHECKLIST BEFORE RUNNING DEMO ##
# RVIZ Open (?)
# Intera server running
# Camera screwed, wires ziptied or not in the way
# Validate Z position is correct (in case of shifted table)


# Step 1:
# Define piece pickup z coordinate (lowest point robot should go/pick up piece at)
#
from csv import Error
from stockfish import Stockfish
from planning.chessboard import ChessBoard, TileObject
import rospy
from geometry_msgs.msg import Point, PointStamped
from move_arm.src.pickup_integ import pickup_and_place
import rospkg
from chess_tracking.src.transform_coordinates import transform_point_to_base


def get_board_state():  # return fen of current board state
    return


# should return a Point() in terms of pixels
def get_piece_location_on_tile(tile: str):
    return


def get_piece_locations():  # returns a mapping from
    return


def get_tile_locations():  # returns a list of tuples, each tuple is for a tile, in order from a1 -> h8
    return


# Arguments are of type Point() -> doesn't return anything


def pickup_and_place_piece(from_tile, to_tile):

    # start_goal and end goal are both of type Point()
    start_goal = transform_point_to_base(from_tile)
    end_goal = transform_point_to_base(to_tile)

    pickup_and_place(start_goal, end_goal)

    return


def main():

    rospy.init_node("main_node", anonymous=True)

    rospack = rospkg.RosPack()
    package_path = rospack.get_path('chess_tracking')  # Replace with your package name

    stockfish_path = package_path + "/src/stockfish-ubuntu-x86-64-vnni512"
    stockfish = Stockfish(stockfish_path)  # init Stockfish
    stockfish.set_skill_level(5)
    board = ChessBoard()  # initialize chessboard class
    tiles = get_tile_locations()
    pieces = get_piece_locations()
    board.create_board(tiles, pieces)
    
    drop_off = (0.722, -0.013) #piece drop off spot (after taking)

    gaming = True
    while gaming:
        stockfish.set_fen(get_board_state())  # grab current board state
        next_move = stockfish.get_best_move()  # e2e4
        capture = False
        if next_move is None or stockfish.is_game_over():
            gaming = False
            break

        pickup_tile = next_move[0:2]  # e2
        place_tile = next_move[2:]  # e4

        pickup_tile_coords = get_piece_location_on_tile(pickup_tile)
        place_tile_coords = (board.chess_tiles[place_tile].x, board.chess_tiles[place_tile].y) #accesses board hashmap and grabs tile xy
        
        if board.chess[place_tile].piece is not None: #Taking a piece
            capture = True
            pickup_and_place(place_tile_coords, drop_off, capture) #move piece off board
            rospy.sleep(1.0)
            capture = False
        pickup_and_place(pickup_tile_coords, place_tile_coords, capture) # should be one fluid motion
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
