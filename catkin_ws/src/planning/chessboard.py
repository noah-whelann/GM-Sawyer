# origin point on chessboard is considered as tile a1, everything else is an offset off that
# 5.715 x 5.715 cm tile sizes
from geometry_msgs.msg import Point


class TileObject:
    # contains tile coordinate and the piece on it (if there is one)
    def __init__(self, x, y, piece="") -> None:
        self.x = x
        self.y = y
        self.tile_center = Point(x, y, 0)
        self.has_piece = False
        self.piece = piece
        self.piece_location = Point(0, 0, 0)


class ChessBoard:

    def __init__(self) -> None:
        self.white_castled = False
        self.black_castled = False
        self.chess_tiles = {}
        self.starting_tiles = {
            # White pieces
            # White major pieces
            'A1': 'R', 'B1': 'N', 'C1': 'B', 'D1': 'Q', 'E1': 'K', 'F1': 'B', 'G1': 'N', 'H1': 'R',
            'A2': 'P', 'B2': 'P', 'C2': 'P', 'D2': 'P', 'E2': 'P', 'F2': 'P', 'G2': 'P', 'H2': 'P',  # White pawns

            # Black pieces
            'A7': 'p', 'B7': 'p', 'C7': 'p', 'D7': 'p', 'E7': 'p', 'F7': 'p', 'G7': 'p', 'H7': 'p',  # Black pawns
            # Black major pieces
            'A8': 'r', 'B8': 'n', 'C8': 'b', 'D8': 'q', 'E8': 'k', 'F8': 'b', 'G8': 'n', 'H8': 'r'
        }

    def create_board(self):  # creates blank chessboard with grid locations
        for row in range(0, 8):
            col_counter = 0
            for col in "ABCDEFGH":
                tile_as_string = f"{col}{row + 1}"  # rows are 1-indexed
                piece = self.starting_tiles[tile_as_string] if self.starting_tiles.get(
                    tile_as_string) else ""  # if the tile has a starting piece assign it
                currTile = TileObject(0, 0, piece)
                self.chess_tiles[tile_as_string] = currTile
                col_counter += 1


# board = ChessBoard()

# board.create_board()
