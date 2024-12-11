
# Step 1:
# Define piece pickup z coordinate (lowest point robot should go/pick up piece at)
#
from stockfish import Stockfish
from planning.chessboard import ChessBoard, TileObject


def get_board_state():  # return fen of current board state
    return


def get_piece_location_on_tile(tile):
    return


def get_piece_locations():  # returns a mapping from
    return


def get_tile_locations():  # returns a list of tuples, each tuple is for a tile, in order from a1 -> h8
    return


def move_robot(move_location):  # transform pixel coordinates to world coordinates
    transformed_coords = transformed_coordinates_service(move_location)
    move_arm_service(transformed_coords)
    # send over the world coordinates to move_arm function
    # that handles all the ik and planning
    return


def main():
    stockfish_path = ".."
    stockfish = Stockfish(stockfish_path)  # init Stockfish
    stockfish.set_skill_level(5)
    board = ChessBoard()  # initialize chessboard class
    tiles = get_tile_locations()
    pieces = get_piece_locations()
    board.create_board(tiles, pieces)

    gaming = True
    while gaming:
        stockfish.set_fen(get_board_state())  # grab current board state
        next_move = str(stockfish.get_best_move())  # e2e4

        pickup_tile = next_move[0:2]  # e2
        place_tile = next_move[2:]  # 4

        pickup_tile_coords = get_piece_location_on_tile(pickup_tile)
        place_tile_coords = tuple(place_tile.x, place_tile.y)

        move_robot(pickup_tile_coords)
        # robot then tucks after its move

        # wait for user to execute move
        input("Press enter when you have moved the piece")


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
