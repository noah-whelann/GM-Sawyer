# origin point on chessboard is considered as tile a1, everything else is an offset off that
# 5.715 x 5.715 cm tile sizes
class TileObject:
    def init(self, x, y, piece = "") -> None: #contains tile coordinate and the piece on it (if there is one)
        self.x = x
        self.y = y
        self.piece = piece

class ChessBoard:

    def init(self) -> None:
        self.chess_tiles = {}
        self.starting_tiles = {
            # White pieces
            'A1': 'R', 'B1': 'N', 'C1': 'B', 'D1': 'Q', 'E1': 'K', 'F1': 'B', 'G1': 'N', 'H1': 'R',  # White major pieces
            'A2': 'P', 'B2': 'P', 'C2': 'P', 'D2': 'P', 'E2': 'P', 'F2': 'P', 'G2': 'P', 'H2': 'P',  # White pawns

            # Black pieces
            'A7': 'p', 'B7': 'p', 'C7': 'p', 'D7': 'p', 'E7': 'p', 'F7': 'p', 'G7': 'p', 'H7': 'p',  # Black pawns
            'A8': 'r', 'B8': 'n', 'C8': 'b', 'D8': 'q', 'E8': 'k', 'F8': 'b', 'G8': 'n', 'H8': 'r'   # Black major pieces
        }

    def create_board(self): #creates blank chessboard with grid locations

        for row in range(0,8):
            col_counter = 0
            for col in "ABCDEFGH":
                tile = f"{col}{row + 1}" #rows are 1-indexed
                tile_x = round(col_counter * 5.715 + self.origin_offset_x, 3) #using offsets of 5.715 (tile size). Each coordinate is represents the center of the tile
                tile_y = round(row * 5.715 + self.origin_offset_y, 3) #rounded to 3 decimal digits
                piece = self.starting_tiles[tile] if self.starting_tiles.get(tile) else "" #if the tile has a starting piece assign it
                self.chess_tiles[tile] = TileObject(tile_x, tile_y, piece)
                col_counter += 1
                print(tile,tile_x,tile_y, piece)



board = ChessBoard()

board.create_board()