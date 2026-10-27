"""Turn a chess move into physical pick and place actions."""
from dataclasses import dataclass
from typing import Optional

import chess


@dataclass(frozen=True)
class Action:
    kind: str                  # 'move' | 'remove' | 'spare'
    src: Optional[int] = None  # square to pick from ('spare': None)
    dst: Optional[int] = None  # square to put on ('remove': None, goes to the grave)
    color: Optional[bool] = None  # 'spare' only: which colour's spare queen


def plan_actions(board, move):
    """Ordered actions that carry out `move` on `board` (board is not changed)."""
    if not board.is_legal(move):
        raise ValueError(f'illegal move {move.uci()} in {board.fen()}')
    acts = []
    if board.is_en_passant(move):
        acts.append(Action('remove', src=chess.square(chess.square_file(move.to_square), chess.square_rank(move.from_square))))
    elif board.is_capture(move):
        acts.append(Action('remove', src=move.to_square))
    acts.append(Action('move', move.from_square, move.to_square))
    if board.is_castling(move):
        rank = chess.square_rank(move.from_square)
        kingside = chess.square_file(move.to_square) == 6
        acts.append(Action('move', chess.square(7 if kingside else 0, rank), chess.square(5 if kingside else 3, rank)))
    if move.promotion:
        if move.promotion != chess.QUEEN:
            raise NotImplementedError('only queen promotion: the cell holds one spare queen per colour')
        acts.append(Action('remove', src=move.to_square))
        acts.append(Action('spare', dst=move.to_square, color=board.turn))
    return acts

