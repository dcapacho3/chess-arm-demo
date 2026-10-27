import os
import xml.etree.ElementTree as ET

import chess

from chess_arm_demo import layout as L

WORLD = os.path.join(os.path.dirname(__file__), '..', 'worlds', 'chess.sdf')
ROOT = ET.parse(WORLD).getroot()


def test_world_has_every_piece_once():
    names = [m.get('name') for m in ROOT.iter('model') if m.get('name').startswith('piece_')]
    assert len(names) == len(set(names)) == 34          # 32 pieces and a spare queen per colour


def test_names_unique_inside_every_link():
    """sdformat rejects a link whose collisions and visuals share a name."""
    for link in ROOT.iter('link'):
        names = [e.get('name') for tag in ('collision', 'visual') for e in link.findall(tag)]
        assert len(names) == len(set(names)), [n for n in names if names.count(n) > 1]


def test_every_mesh_file_exists_next_to_the_world():
    uris = {u.text for u in ROOT.iter('uri')}
    assert len(uris) == 6
    for u in uris:
        assert os.path.exists(os.path.join(os.path.dirname(WORLD), u)), u


def test_scene_stands_on_the_floor():
    """Floor at world z=0; table, board and pieces TABLE_H above it (the GUI draws its grid at z=0)."""
    z = {m.get('name'): float(m.find('pose').text.split()[2]) for m in ROOT.iter('model') if m.find('pose') is not None}
    assert z['ground'] == 0.0
    assert z['table'] == z['board'] == L.TABLE_H
    assert abs(z['piece_e2'] - (L.TABLE_H + L.BOARD_TOP)) < 0.001


def test_tallest_piece_clears_the_pads_in_transit():
    tallest = max(h for h, _ in L.PIECE_SIZE.values())
    base_when_carried = L.HOVER_Z - (L.GRASP_Z - L.BOARD_TOP)    # a held piece hangs this far below the pad centre
    assert base_when_carried > L.BOARD_TOP + tallest + 0.02


def test_neighbouring_bases_leave_room_for_open_pads():
    widest = max(r for _, r in L.PIECE_SIZE.values())
    assert L.SQ - widest > 0.025 + 0.008              # a neighbour's base stays outside the open pad (25 mm + 8 mm)
