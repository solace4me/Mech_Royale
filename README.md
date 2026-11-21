# temi-survivor: a Mech Royale bot

Placement scores, not kills, so this bot plays to survive:

- **Zone first:** heads for a spot well inside the *next* circle early, pathing around buildings
  with a breadth-first distance field on a 4 m grid (computed a slice per tick).
- **Cover:** a 2 m grid of the static map (ground, walls, roofs) is rasterized at image build
  (`precompute.py`). It settles next to walls and, when hurt or shot at from range, moves to the
  nearest spot the attacker can't see, then lets HP regenerate.
- **Fights only when it should:** faces whoever is shooting (rear hits do 1.5x), fires the gun
  only when its lead-corrected aim is on target and the line of fire is clear, strafes
  unpredictably, and jinks when a laser is charging on it.
- **Loot:** takes nearby lightning or laser (no aim leading needed), cubes when it's quiet, and
  picks armor/plating-first upgrades. Leaves NPC drones alone unless they attack.

Check: `python3 check/check.py --map-dir "$PWD/arena" -- python3 -u bot.py`
