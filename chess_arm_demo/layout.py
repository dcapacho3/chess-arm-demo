"""Where everything sits on the table (metres, robot base_link frame)."""
import chess

SQ = 0.055            # square size
BOARD_CX = 0.55       # board centre x; centre y is 0
TABLE_H = 0.72        # floor is this far below the table top, the world origin sits on the table
BOARD_TOP = 0.01      # board surface height
GRASP_Z = 0.03        # fingertip pad centre height while holding a piece
HOVER_Z = 0.16        # pad height when travelling (carried kings clear the pieces by 2 cm)
TCP_OFFSET = 0.17     # flange to pad centre along the tool axis

# (height, base radius) in metres of each piece's mesh in worlds/chess.sdf.
PIECE_SIZE = {chess.PAWN: (0.04998, 0.01159), chess.ROOK: (0.055, 0.01309),
              chess.KNIGHT: (0.06188, 0.01349),chess.BISHOP: (0.065, 0.01283),
              chess.QUEEN: (0.07345, 0.01501), chess.KING: (0.08749, 0.01699)}


def square_xy(sq):
    """Centre of a chess square index (0=a1 .. 63=h8). a-file is on the robot's left (+y)."""
    f, r = chess.square_file(sq), chess.square_rank(sq)
    return BOARD_CX + (r - 3.5) * SQ, (3.5 - f) * SQ


def grave_xy(k):
    """k-th slot for captured pieces, left of the board."""
    return 0.35 + (k % 6) * 0.07, 0.34 + (k // 6) * 0.07


def reserve_xy(color, k):
    """Spare queens for promotion, right of the board."""
    return 0.40 + k * 0.07, -0.34 if color == chess.WHITE else -0.41
