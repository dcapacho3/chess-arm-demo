import chess
import pytest

from chess_arm_demo.game import Action, plan_actions


def acts(fen, uci):
    b = chess.Board(fen)
    return plan_actions(b, chess.Move.from_uci(uci))


def sq(s):
    return chess.parse_square(s)


def test_quiet_move():
    assert acts(chess.STARTING_FEN, 'e2e4') == [Action('move', sq('e2'), sq('e4'))]


def test_capture_removes_victim_first():
    fen = 'rnbqkbnr/ppp1pppp/8/3p4/4P3/8/PPPP1PPP/RNBQKBNR w KQkq d6 0 2'
    assert acts(fen, 'e4d5') == [Action('remove', src=sq('d5')), Action('move', sq('e4'), sq('d5'))]


def test_en_passant_removes_pawn_beside():
    fen = 'rnbqkbnr/ppp1p1pp/8/3pPp2/8/8/PPPP1PPP/RNBQKBNR w KQkq f6 0 3'
    assert acts(fen, 'e5f6') == [Action('remove', src=sq('f5')), Action('move', sq('e5'), sq('f6'))]


def test_castling_moves_king_then_rook():
    fen = 'r3k2r/pppppppp/8/8/8/8/PPPPPPPP/R3K2R w KQkq - 0 1'
    assert acts(fen, 'e1g1') == [Action('move', sq('e1'), sq('g1')), Action('move', sq('h1'), sq('f1'))]
    assert acts(fen, 'e1c1') == [Action('move', sq('e1'), sq('c1')), Action('move', sq('a1'), sq('d1'))]
    assert acts(fen, 'e1g1')[1].src == sq('h1')


def test_black_castling_uses_rank_8():
    fen = 'r3k2r/pppppppp/8/8/8/8/PPPPPPPP/R3K2R b KQkq - 0 1'
    assert acts(fen, 'e8c8') == [Action('move', sq('e8'), sq('c8')), Action('move', sq('a8'), sq('d8'))]


def test_promotion_swaps_in_spare_queen():
    fen = '8/P6k/8/8/8/8/8/K7 w - - 0 1'
    assert acts(fen, 'a7a8q') == [Action('move', sq('a7'), sq('a8')), Action('remove', src=sq('a8')),
                                  Action('spare', dst=sq('a8'), color=chess.WHITE)]


def test_capture_promotion():
    fen = '1n5k/P7/8/8/8/8/8/K7 w - - 0 1'
    a = acts(fen, 'a7b8q')
    assert [x.kind for x in a] == ['remove', 'move', 'remove', 'spare']


def test_underpromotion_refused():
    with pytest.raises(NotImplementedError):
        acts('8/P6k/8/8/8/8/8/K7 w - - 0 1', 'a7a8n')


def test_illegal_move_refused():
    with pytest.raises(ValueError):
        acts(chess.STARTING_FEN, 'e2e5')
