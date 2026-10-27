"""Play a chess game with the arm: a move list chooses, the UR5e moves the pieces."""
import argparse
import json
import math
import os
import time

import chess
import rclpy
from std_msgs.msg import String

from . import layout as L
from .moveit_arm import MoveItArm
from .game import plan_actions
from .poses import is_upright, piece_poses

PLACE_TOL = 0.012       # m, a placed piece must end this close to the square centre
BYSTANDER_TOL = 0.015


class Player:
    def __init__(self, arm, fen=chess.STARTING_FEN):
        self.arm = arm
        self.board = chess.Board(fen)
        self.names = {sq: f'piece_{chess.square_name(sq)}' for sq in self.board.piece_map()}
        arm.scene_init([(f'piece_{chess.square_name(sq)}', p.piece_type, *L.square_xy(sq)) for sq, p in self.board.piece_map().items()]
                       + [(f"piece_spare_{'w' if c else 'b'}", chess.QUEEN, *L.reserve_xy(c, 0)) for c in (chess.WHITE, chess.BLACK)])
        self.grave = []
        self.log = []          # one dict per move
        self.status = arm.create_publisher(String, '/chess/status', 10)
        self.say({'text': 'ready'})

    def say(self, d):
        d.setdefault('fen', self.board.fen())
        d['ply'] = len(self.board.move_stack)
        self.status.publish(String(data=json.dumps(d)))

    # --- one physical transfer ---------------------------------------------------
    def transfer(self, name, src_xy, dst_xy):
        a = self.arm
        for attempt in (1, 2):
            a.open()
            a.line((*src_xy, L.HOVER_Z), 0.35)
            a.line((*src_xy, L.GRASP_Z), 0.10)
            a.close()
            if 0.014 < a.finger.sum() < 0.044 and a.finger.min() > 0.004:
                break
            a.open()                       # fingers closed on nothing or crushed: try again
            a.line((*src_xy, L.HOVER_Z), 0.15)
        else:
            raise RuntimeError(f'could not grip at {src_xy}')
        a.attach(name)
        a.line((*src_xy, L.HOVER_Z), 0.10)
        a.line((*dst_xy, L.HOVER_Z), 0.30)
        a.line((*dst_xy, L.GRASP_Z + 0.0015), 0.08)
        a.open()
        a.detach(name, *dst_xy)
        a.line((*dst_xy, L.HOVER_Z), 0.10)

    def do(self, action):
        if action.kind == 'move':
            self.transfer(self.names[action.src], L.square_xy(action.src), L.square_xy(action.dst))
        elif action.kind == 'remove':
            self.transfer(self.names[action.src], L.square_xy(action.src), L.grave_xy(len(self.grave)))
        else:
            self.transfer(f"piece_spare_{'w' if action.color else 'b'}", L.reserve_xy(action.color, 0), L.square_xy(action.dst))

    # --- whole move, then check against ground truth -------------------------------
    def play(self, move):
        san = self.board.san(move)
        before = piece_poses()
        acts = plan_actions(self.board, move)
        t0 = time.time()
        self.say({'text': f'{"White" if self.board.turn else "Black"}: {san}', 'san': san})
        touched = set()
        retries_before = len(self.arm.retries)
        plan_before = self.arm.plan_time
        for act in acts:
            if act.src is not None:
                touched.add(self.names[act.src])
            self.do(act)
            if act.kind == 'remove':   # keep the slot index right for the next removal
                self.grave.append(self.names.pop(act.src))
            elif act.kind == 'move':
                self.names[act.dst] = self.names.pop(act.src)
            else:
                self.names[act.dst] = f"piece_spare_{'w' if act.color else 'b'}"
                touched.add(self.names[act.dst])
        self.board.push(move)
        rec = self.verify(san, before, touched, time.time() - t0)
        rec['retries'] = self.arm.retries[retries_before:]
        rec['plan_seconds'] = round(self.arm.plan_time - plan_before, 2)
        self.log.append(rec)
        self.say({'text': f'{san}  placed within {rec["max_place_err_mm"]:.1f} mm' if rec['ok'] else f'{san}  FAILED: {rec["why"]}', 'san': san, 'ok': rec['ok']})
        return rec

    def verify(self, san, before, touched, secs):
        time.sleep(1.0)
        now = piece_poses()
        why, errs = [], []
        for sq, name in self.names.items():
            x, y = L.square_xy(sq)
            p = now[name]
            e = math.hypot(p[0] - x, p[1] - y)
            if name in touched:
                errs.append(e)
                if e > PLACE_TOL:
                    why.append(f'{name} {1000*e:.0f} mm off {chess.square_name(sq)}')
            if not is_upright(p) or abs(p[2] - L.TABLE_H - L.BOARD_TOP) > 0.004:
                why.append(f'{name} not standing')
        for name, p in before.items():
            if name in touched or name not in now:
                continue
            if name in self.grave:
                continue
            d = math.hypot(now[name][0] - p[0], now[name][1] - p[1])
            if d > BYSTANDER_TOL:
                why.append(f'bystander {name} moved {1000*d:.0f} mm')
        return {'move': san, 'ok': not why, 'why': '; '.join(why), 'seconds': round(secs, 1),
                'max_place_err_mm': round(1000 * max(errs), 1) if errs else 0.0}


DEMO_GAME = 'e2e4 d7d5 e4e5 f7f5 e5f6 g8f6 g1f3 c8g4 f1e2 b8c6 e1g1 e7e6 d2d4 d8d7 b1c3 e8c8'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--fen', default=chess.STARTING_FEN)
    ap.add_argument('--moves', default=DEMO_GAME, help='UCI moves, space separated (default: the demo game)')
    ap.add_argument('--out', default='/tmp/chess_result.json')
    args = ap.parse_args()

    rclpy.init()
    arm = MoveItArm()
    arm.wait_ready()
    player = Player(arm, args.fen)
    queue = [chess.Move.from_uci(m) for m in args.moves.split()]
    t0 = time.time()
    try:
        while queue and not player.board.is_game_over():
            rec = player.play(queue.pop(0))
            print(json.dumps(rec), flush=True)
            if not rec['ok']:
                break
    except Exception:
        import traceback
        traceback.print_exc()
    finally:
        ok = sum(r['ok'] for r in player.log)
        summary = {'moves': len(player.log), 'ok': ok, 'wall_seconds': round(time.time() - t0, 1),
                   'plan_seconds_total': round(arm.plan_time, 2), 'fallbacks_to_ompl': arm.fallbacks,
                   'final_fen': player.board.fen(), 'log': player.log}
        with open(args.out, 'w') as f:
            json.dump(summary, f, indent=1)
        print(f'{ok}/{len(player.log)} moves verified; result in {args.out}', flush=True)
        os._exit(0 if ok == len(player.log) and not queue else 1)


if __name__ == "__main__":
    main()
