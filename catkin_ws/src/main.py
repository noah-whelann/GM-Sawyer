
# Step 1:
# Define piece pickup z coordinate (lowest point robot should go/pick up piece at)
# 

from stockfish import Stockfish
from planning.chessboard import ChessBoard, TileObject

def get_board_state(): #return fen of current board state
    return 

def get_movement_coords(move):
    return

def get_piece_locations(): #returns a list of 
    return

def get_tile_locations(): #returns a list of tuples, each tuple is for a tile, in order from a1 -> h8
    return

def main():
    stockfish_path = ".."
    stockfish = Stockfish(stockfish_path) #init Stockfish
    stockfish.set_skill_level(5) 
    board = ChessBoard() #initialize chessboard class
    tiles = get_tile_locations() 
    pieces = get_piece_locations()
    board.create_board(tiles, pieces)
    
    
    gaming = True
    while gaming:
        stockfish.set_fen(get_board_state()) #grab current board state
        next_move = stockfish.get_best_move()
        coords = get_movement_coords(next_move)
        