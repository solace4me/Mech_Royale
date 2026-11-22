"""Survivor bot for mechfight/1.

Placement scores, not kills. So: get into the safe zone early, stay next to
buildings, keep your face toward whoever is shooting you (rear hits do 1.5x),
return fire only with a clear line and a computed lead, break line of sight to
heal when hurt, and leave NPC drones alone unless they start it.
"""
import array
import json
import math
import os
import random
from collections import deque

import mechfight as mf

HERE = os.path.dirname(os.path.abspath(__file__))
UPGRADE_PREF = ["armor", "plating", "nanites", "gun_damage", "gun_rate", "lightning_efficiency",
                "laser_charge", "gyros", "gun_velocity", "laser_damage", "lightning_range",
                "gun_mag", "servos", "sensors", "jets", "capacitor", "reactor", "lightning_chain",
                "laser_pierce", "mortar_reload", "mortar_splash", "mortar_velocity"]
WEAPON_PREF = {"lightning": 3, "laser": 2, "mortar": 0}


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class Grid:
    """2 m map grid built at image build time (precompute.py)."""

    def __init__(self):
        h = json.load(open(os.path.join(HERE, "grid.json")))
        self.step, self.minx, self.miny, self.nx, self.ny = h["step"], h["minx"], h["miny"], h["nx"], h["ny"]
        n = self.nx * self.ny
        a = array.array("f")
        a.frombytes(open(os.path.join(HERE, "grid.bin"), "rb").read())
        self.g, self.base, self.top = a[:n], a[n:2 * n], a[2 * n:]
        # walkability on a 4 m nav grid: blocked if any 2 m sub-cell is a wall
        self.cnx, self.cny = (self.nx + 1) // 2, (self.ny + 1) // 2
        wall = bytearray(n)
        for k in range(n):
            g = self.g[k]
            if self.top[k] > g + 1.3 and self.base[k] < g + 9.0:
                wall[k] = 1
        self.wall = wall
        cb = bytearray(self.cnx * self.cny)
        for j in range(self.ny):
            row = j * self.nx
            cj = (j // 2) * self.cnx
            for i in range(self.nx):
                if wall[row + i]:
                    cb[cj + i // 2] = 1
        self.cblock = cb
        # "near cover": nav cells next to a wall cell
        cover = bytearray(self.cnx * self.cny)
        for cj in range(self.cny):
            for ci in range(self.cnx):
                k = cj * self.cnx + ci
                if cb[k]:
                    continue
                for dj in (-1, 0, 1):
                    for di in (-1, 0, 1):
                        jj, ii = cj + dj, ci + di
                        if 0 <= jj < self.cny and 0 <= ii < self.cnx and cb[jj * self.cnx + ii]:
                            cover[k] = 1
        self.cover = cover

    def cell(self, x, y):
        i = int((x - self.minx) / self.step)
        j = int((y - self.miny) / self.step)
        if 0 <= i < self.nx and 0 <= j < self.ny:
            return j * self.nx + i
        return -1

    def ccell(self, x, y):
        i = int((x - self.minx) / (2 * self.step))
        j = int((y - self.miny) / (2 * self.step))
        if 0 <= i < self.cnx and 0 <= j < self.cny:
            return j * self.cnx + i
        return -1

    def ccenter(self, k):
        j, i = divmod(k, self.cnx)
        return (self.minx + (i + 0.5) * 2 * self.step, self.miny + (j + 0.5) * 2 * self.step)

    def ground(self, x, y):
        k = self.cell(x, y)
        return self.g[k] if k >= 0 else 0.0

    def los(self, a, b):
        """Straight line a->b (3D) clear of buildings and terrain (trees don't stop shots)."""
        dx, dy, dz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
        d = math.hypot(dx, dy)
        n = int(d / (self.step * 0.75)) + 1
        for s in range(1, n):
            t = s / n
            x, y, z = a[0] + dx * t, a[1] + dy * t, a[2] + dz * t
            k = self.cell(x, y)
            if k < 0:
                return False
            if z < self.g[k] - 0.3:
                return False
            if self.base[k] - 0.3 < z < self.top[k] + 0.3:
                return False
        return True


class Nav:
    """Breadth-first distance field to a goal on the 4 m grid, computed a slice per tick."""

    def __init__(self, grid):
        self.grid = grid
        self.goal = None
        self.dist = None
        self.queue = None
        self.done = False

    def set_goal(self, x, y):
        g = self.grid
        k = g.ccell(x, y)
        if k < 0:
            return
        if self.goal is not None and abs(self.goal[0] - x) + abs(self.goal[1] - y) < 12:
            return
        # snap to nearest walkable cell
        if g.cblock[k]:
            best = None
            for r in range(1, 12):
                for dj in range(-r, r + 1):
                    for di in range(-r, r + 1):
                        j, i = divmod(k, g.cnx)
                        jj, ii = j + dj, i + di
                        if 0 <= jj < g.cny and 0 <= ii < g.cnx and not g.cblock[jj * g.cnx + ii]:
                            best = jj * g.cnx + ii
                            break
                    if best is not None:
                        break
                if best is not None:
                    break
            if best is None:
                return
            k = best
        self.goal = (x, y)
        self.dist = array.array("i", [-1]) * (g.cnx * g.cny)
        self.dist[k] = 0
        self.queue = deque([k])
        self.done = False

    def work(self, budget=2500):
        if self.done or self.queue is None:
            return
        g, dist, q = self.grid, self.dist, self.queue
        cnx, cny, cb = g.cnx, g.cny, g.cblock
        n = 0
        while q and n < budget:
            k = q.popleft()
            n += 1
            d = dist[k] + 1
            j, i = divmod(k, cnx)
            for dj, di in ((0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)):
                jj, ii = j + dj, i + di
                if 0 <= jj < cny and 0 <= ii < cnx:
                    kk = jj * cnx + ii
                    if dist[kk] < 0 and not cb[kk]:
                        if dj and di and (cb[j * cnx + ii] or cb[jj * cnx + i]):
                            continue  # no corner cutting
                        dist[kk] = d
                        q.append(kk)
        if not q:
            self.done = True

    def direction(self, x, y):
        """Unit move direction toward the goal, or None to fall back to a straight line."""
        if self.dist is None:
            return None
        g = self.grid
        k = g.ccell(x, y)
        if k < 0 or self.dist[k] < 0:
            return None
        j, i = divmod(k, g.cnx)
        best, bd = None, self.dist[k]
        # look two cells ahead for a smoother line
        for dj in range(-2, 3):
            for di in range(-2, 3):
                if not dj and not di:
                    continue
                jj, ii = j + dj, i + di
                if 0 <= jj < g.cny and 0 <= ii < g.cnx:
                    d = self.dist[jj * g.cnx + ii]
                    if 0 <= d and (d < bd or (d == bd and best is None and False)):
                        if max(abs(dj), abs(di)) == 2:
                            mid = (j + dj // 2) * g.cnx + (i + di // 2)
                            if g.cblock[mid]:
                                continue
                        bd, best = d, (jj, ii)
        if best is None:
            return None
        cx, cy = g.ccenter(best[0] * g.cnx + best[1])
        vx, vy = cx - x, cy - y
        n = math.hypot(vx, vy)
        return (vx / n, vy / n) if n > 1e-6 else None


class Bot:
    name = "temi-hunter"

    def __init__(self, hello):
        self.cfg = hello["config"]
        self.slot = hello["slot"]
        self.rng = random.Random(hello.get("seed", 0))
        self.grid = Grid()
        self.nav = Nav(self.grid)
        self.last_pos = None
        self.stuck = 0
        self.strafe = 1
        self.strafe_until = 0.0
        self.threat = None        # (bearing, time, attacker id)
        self.last_hit_t = -99.0
        self.enemies = {}         # id -> (pos, vel, t)
        self.scan_yaw = None
        self.laser_hold = 0.0
        self.home = None
        self.prev_dir = None

    # ---------- helpers ----------
    def lead(self, me_pos, mz, tgt, speed):
        px, py, pz = tgt["pos"]
        vx, vy, vz = tgt.get("vel", [0, 0, 0])
        # aim at the body's middle: mechs are 9 m tall (scaled by cubes), drones 3.5 m
        if tgt.get("kind") == "npc":
            tz = pz + 1.75
        else:
            tz = pz + 4.5 * (1 + 0.05 * min(tgt.get("cubes", 0) or 0, 40))
        t = 0.0
        for _ in range(3):
            ax, ay, az = px + vx * t, py + vy * t, tz + vz * t
            d = math.dist((ax, ay, az), (me_pos[0], me_pos[1], mz))
            t = d / speed
        ax, ay, az = px + vx * t, py + vy * t, tz + max(min(vz * t, 4), -4)
        yaw = math.atan2(ay - me_pos[1], ax - me_pos[0])
        pitch = math.atan2(az - mz, math.hypot(ax - me_pos[0], ay - me_pos[1]))
        return yaw, pitch, (ax, ay, az)

    def safe_goal(self, obs, me):
        z = obs["zone"]
        x, y = me["pos"][0], me["pos"][1]
        t = obs["t"]
        # aim for the circle that will be next; once its shrink is near, it is the one that counts
        cx, cy, r = z["next_center"][0], z["next_center"][1], z["next_radius"]
        if r < 1:
            return cx, cy, 0.0
        dx, dy = x - cx, y - cy
        d = math.hypot(dx, dy)
        keep = r * (0.55 if r > 100 else 0.4)
        if d <= keep:
            return None  # already comfortably inside
        return cx + dx / d * keep, cy + dy / d * keep, d - keep

    def zone_margin(self, obs, x, y, ahead):
        """Distance inside the circle `ahead` seconds from now (negative = outside)."""
        z = obs["zone"]
        t = obs["t"] + ahead
        cx, cy, r = z["center"][0], z["center"][1], z["radius"]
        if t > z["shrink_start"] and z["shrink_end"] > z["shrink_start"]:
            f = min((t - z["shrink_start"]) / (z["shrink_end"] - z["shrink_start"]), 1.0)
            cx += (z["next_center"][0] - cx) * f
            cy += (z["next_center"][1] - cy) * f
            r += (z["next_radius"] - r) * f
        return r - math.hypot(x - cx, y - cy)

    # ---------- main ----------
    def act(self, obs):
        me = obs["self"]
        t = obs["t"]
        x, y, z = me["pos"]
        mz = z + me["stats"].get("muzzle_height", 7)
        eye = z + me["stats"].get("eye_height", 8)
        act = {"tick": obs["tick"]}
        self.nav.work()

        # upgrades
        if obs.get("offer"):
            ch = obs["offer"]["choices"]
            ranks = [UPGRADE_PREF.index(c["id"]) if c["id"] in UPGRADE_PREF else 99 for c in ch]
            act["pick"] = ranks.index(min(ranks))

        # threats from damage events
        for ev in obs.get("events", []):
            if ev.get("type") == "damaged" and ev.get("by") is not None and ev.get("bearing") is not None:
                self.threat = (ev["bearing"], t, ev["by"], ev.get("weapon"))
                self.last_hit_t = t
            elif ev.get("type") == "damaged":
                self.last_hit_t = t

        mechs = [m for m in obs.get("mechs", []) if m["kind"] == "mech"]
        npcs = [m for m in obs.get("mechs", []) if m["kind"] == "npc"]
        for m in mechs:
            self.enemies[m["id"]] = (m["pos"], m.get("vel", [0, 0, 0]), t)
        hp_frac = me["hp"] / max(me["max_hp"], 1)
        late = obs["alive"] <= 4

        # ---- target choice ----
        target = None
        if mechs:
            def score(m):
                d = math.dist(m["pos"][:2], (x, y))
                s = d
                if self.threat and self.threat[2] == m["id"] and t - self.threat[1] < 4:
                    s -= 80  # whoever is shooting us first
                s -= (1 - m.get("hp", 1400) / max(m.get("max_hp", 1400), 1)) * 60
                return s
            target = min(mechs, key=score)
        npc_target = None
        if self.threat and self.threat[3] == "npc" and t - self.threat[1] < 3:
            for n in npcs:
                if n["id"] == self.threat[2]:
                    npc_target = n
        farm = (target is None and hp_frac > 0.5 and me["in_zone"]
                and not (self.threat and t - self.threat[1] < 4 and self.threat[3] != "npc"))
        if npc_target is None and farm:
            near_npcs = [n for n in npcs if math.dist(n["pos"][:2], (x, y)) < 110]
            if near_npcs:
                npc_target = min(near_npcs, key=lambda n: math.dist(n["pos"][:2], (x, y)))
        crate_target = None
        if npc_target is None and farm:
            crates = [c for c in obs.get("crates", []) if math.dist(c["pos"][:2], (x, y)) < 90]
            if crates:
                c = min(crates, key=lambda c: math.dist(c["pos"][:2], (x, y)))
                crate_target = {"pos": [c["pos"][0], c["pos"][1], c["pos"][2] - 3.3], "vel": [0, 0, 0],
                                "id": c["id"], "hp": 1, "max_hp": 1}

        # ---- movement goal ----
        move = None
        boost = False
        goal = self.safe_goal(obs, me)
        zone_urgent = (not me["in_zone"]) or (goal is not None and obs["t"] > obs["zone"]["shrink_start"] - goal[2] / 9.0 - 8)
        # the circle is about to catch us: nothing else matters
        zone_critical = (not me["in_zone"]) or self.zone_margin(obs, x, y, 4.0) < 12
        retreat = False
        laser_on_me = None
        for m in mechs:
            if m.get("charging"):
                to_me = math.atan2(y - m["pos"][1], x - m["pos"][0])
                if abs(wrap(m["aim"][0] - to_me)) < math.radians(12):
                    laser_on_me = m
        if target is not None:
            td = math.dist(target["pos"][:2], (x, y))
            their = target.get("hp", 1400) / max(target.get("max_hp", 1400), 1)
            winning = their < 0.35 and hp_frac > 0.6
            if not late and not winning:
                if hp_frac < 0.3 and their > hp_frac:
                    retreat = True
                elif td > 120 and t - self.last_hit_t < 3 and "laser" in target.get("weapons", []):
                    retreat = True  # a laser out-trades our gun at long range
        elif self.threat and t - self.threat[1] < 2.5 and hp_frac < 0.3:
            retreat = True

        if goal is not None:
            self.nav.set_goal(goal[0], goal[1])
        dir_goal = None
        if goal is not None and (zone_urgent or target is None or retreat is False):
            dir_goal = self.nav.direction(x, y)
            if dir_goal is None:
                ang = math.atan2(goal[1] - y, goal[0] - x)
                dir_goal = (math.cos(ang), math.sin(ang))

        if retreat:
            src = target["pos"] if target is not None else (x + math.cos(self.threat[0]) * 60, y + math.sin(self.threat[0]) * 60, z)
            key = (round(x / 8), round(y / 8), round(src[0] / 16), round(src[1] / 16))
            if getattr(self, "_cover_key", None) != key or t - getattr(self, "_cover_t", -9) > 1.0:
                self._cover = self.find_cover(x, y, z, src)
                self._cover_key, self._cover_t = key, t
            hide = self._cover
            if hide is not None:
                ang = math.atan2(hide[1] - y, hide[0] - x)
                move = (math.cos(ang), math.sin(ang))
                boost = me["energy"] > 25
            elif dir_goal is not None:
                move = dir_goal
        if move is None:
            if dir_goal is not None:
                move = dir_goal
                boost = zone_urgent and me["energy"] > 20
            elif target is not None or (self.threat and t - self.threat[1] < 3):
                move = None  # fight in place, strafing below
            else:
                # settle against a wall nearby, out of the open
                if not self.grid.cover[max(self.grid.ccell(x, y), 0)]:
                    c = self.nearest_cover(x, y)
                    if c is not None:
                        ang = math.atan2(c[1] - y, c[0] - x)
                        move = (math.cos(ang), math.sin(ang))

        # chase a nearly dead mech we can see
        if target is not None and not retreat and not zone_urgent:
            their = target.get("hp", 1400) / max(target.get("max_hp", 1400), 1)
            td = math.dist(target["pos"][:2], (x, y))
            if their < 0.35 and td > 25:
                ang = math.atan2(target["pos"][1] - y, target["pos"][0] - x)
                move = (math.cos(ang), math.sin(ang))
                boost = me["energy"] > 30

        # strafe while under fire or fighting
        fighting = target is not None or (self.threat and t - self.threat[1] < 2)
        if fighting and not retreat:
            if t > self.strafe_until:
                self.strafe = -self.strafe if self.rng.random() < 0.7 else self.strafe
                self.strafe_until = t + self.rng.uniform(0.4, 1.1)
            ref = target["pos"] if target is not None else (x + math.cos(self.threat[0]), y + math.sin(self.threat[0]))
            a = math.atan2(ref[1] - y, ref[0] - x) + self.strafe * math.pi / 2
            sx, sy = math.cos(a), math.sin(a)
            if move is not None:
                move = (move[0] * 0.75 + sx * 0.6, move[1] * 0.75 + sy * 0.6)
            else:
                move = (sx, sy)

        # a laser charging on us: jink sideways hard
        if laser_on_me is not None:
            a = math.atan2(laser_on_me["pos"][1] - y, laser_on_me["pos"][0] - x) + self.strafe * math.pi / 2
            if t > self.strafe_until:
                self.strafe = -self.strafe
                self.strafe_until = t + self.rng.uniform(0.3, 0.7)
            move = (math.cos(a), math.sin(a))
            boost = me["energy"] > 15

        # avoid NPC drones' aggro radius unless we are fighting one
        if npc_target is None and not farm:
            for n in npcs:
                d = math.dist(n["pos"][:2], (x, y))
                if d < 55:
                    ax, ay = x - n["pos"][0], y - n["pos"][1]
                    w = (55 - d) / 55
                    mx, my = move if move is not None else (0.0, 0.0)
                    move = (mx + ax / max(d, 1) * w * 1.5, my + ay / max(d, 1) * w * 1.5)

        # grab cubes and good weapons close by when not under pressure
        if not fighting and not zone_urgent:
            for c in obs.get("cubes", []):
                if math.dist(c["pos"][:2], (x, y)) < 30 and abs(c["pos"][2] - z) < 6:
                    ang = math.atan2(c["pos"][1] - y, c["pos"][0] - x)
                    move = (math.cos(ang), math.sin(ang))
                    break
        held = [w["kind"] for w in me["weapons"] if w]
        if me["weapons"][1] is None or not fighting:
            best = None
            for p in obs.get("pickups", []):
                if p["kind"] in held or WEAPON_PREF.get(p["kind"], 0) <= 0:
                    continue
                d = math.dist(p["pos"][:2], (x, y))
                if d < 4 and abs(p["pos"][2] - z) < 4:
                    if me["weapons"][1] is None:
                        act["take"] = p["id"]
                    elif WEAPON_PREF.get(p["kind"], 0) > WEAPON_PREF.get(me["weapons"][1]["kind"], 0) and not (me["weapons"][1].get("charge")):
                        act["take"] = p["id"]
                        act["drop"] = 1
                elif d < 60 and abs(p["pos"][2] - z) < 6 and not fighting and not zone_urgent:
                    if me["weapons"][1] is None or WEAPON_PREF.get(p["kind"], 0) > WEAPON_PREF.get(me["weapons"][1]["kind"], 0):
                        if best is None or d < best[0]:
                            best = (d, p)
            if best is not None and "take" not in act:
                ang = math.atan2(best[1]["pos"][1] - y, best[1]["pos"][0] - x)
                move = (math.cos(ang), math.sin(ang))

        if zone_critical:
            z = obs["zone"]
            zgoal = goal if goal is not None else (z["next_center"][0], z["next_center"][1], 0)
            self.nav.set_goal(zgoal[0], zgoal[1])
            d = self.nav.direction(x, y)
            if d is None:
                ang = math.atan2(zgoal[1] - y, zgoal[0] - x)
                d = (math.cos(ang), math.sin(ang))
            move = d
            boost = me["energy"] > 5

        # ---- stuck handling: jump and jet over what's in the way ----
        jump = False
        sp = math.hypot(me["vel"][0], me["vel"][1])
        if move is not None and math.hypot(*move) > 0.3 and sp < 2.0:
            self.stuck += 1
        else:
            self.stuck = max(0, self.stuck - 1)
        if self.stuck >= 4:
            if me["grounded"]:
                jump = True
            boost = me["energy"] > 10
            if self.stuck > 25:
                # sidestep if jetting doesn't clear it
                a = math.atan2(move[1], move[0]) + (math.pi / 2 if (self.stuck // 15) % 2 else -math.pi / 2)
                move = (math.cos(a), math.sin(a))
        elif not me["grounded"] and self.stuck == 0 and me["vel"][2] < -1:
            boost = False

        if move is not None:
            n = math.hypot(*move)
            if n > 1e-6:
                move = (move[0] / n, move[1] / n)
            act["move"] = [round(move[0], 3), round(move[1], 3)]
        else:
            act["move"] = [0, 0]
        act["boost"] = bool(boost)
        act["jump"] = jump

        # ---- aiming and firing ----
        fire = [False, False]
        gun = me["weapons"][0]
        second = me["weapons"][1]
        aim_t = target if target is not None else (npc_target if npc_target is not None else crate_target)
        if aim_t is not None:
            speed = gun["stats"]["speed"] if gun and gun.get("kind") == "gun" else 252
            yaw, pitch, at = self.lead((x, y), mz, aim_t, speed)
            act["aim"] = [yaw, pitch]
            d = math.dist(aim_t["pos"][:2], (x, y))
            err = abs(wrap(me["aim"][0] - yaw)) + abs(me["aim"][1] - pitch) * 0.5
            tol = math.atan2(2.2, max(d, 1)) + 0.01
            clear = self.grid.los((x, y, mz), at)
            if gun and gun.get("kind") == "gun" and d < gun["stats"]["range"] and err < tol and clear:
                if gun.get("ammo", 1) > 0 and gun.get("reload", 0) == 0:
                    fire[0] = True
            if gun and gun.get("kind") == "gun" and gun.get("ammo", 1) == 0 and gun.get("reload", 0) == 0:
                act["reload"] = True
            if second:
                k = second["kind"]
                if k == "lightning" and d < second["stats"]["range"] * 0.95 and abs(wrap(me["aim"][0] - yaw)) < math.radians(12) and me["energy"] > 8:
                    fire[1] = True
                elif k == "laser":
                    charge = second.get("charge", 0) or 0
                    full = second["stats"].get("charge", 0.75)
                    if charge > 0:
                        # holding: release when charged and on target
                        fire[1] = not (charge >= full * 0.95 and err < tol and clear)
                        if not clear and charge >= full:
                            fire[1] = True
                    elif second.get("cooldown", 0) <= 0 and d < second["stats"]["range"] and clear and err < tol * 3 and not retreat:
                        fire[1] = True
        else:
            # no one in sight: look toward the last threat, else sweep the view
            if self.threat and t - self.threat[1] < 6:
                act["aim"] = [self.threat[0], 0.0]
            else:
                heard = [s for s in obs.get("sounds", []) if s["kind"] in ("gun", "laser", "lightning", "mortar", "step", "boost", "jet", "laser_charge") and s["dist"] != "far"]
                if heard:
                    act["aim"] = [heard[0]["bearing"], 0.0]
                else:
                    if self.scan_yaw is None:
                        self.scan_yaw = me["aim"][0]
                    self.scan_yaw = wrap(self.scan_yaw + 0.25)
                    base = math.atan2(move[1], move[0]) if move is not None and act["move"] != [0, 0] else self.scan_yaw
                    act["aim"] = [base if move is not None and act["move"] != [0, 0] and self.rng.random() < 0.5 else self.scan_yaw, 0.0]
            if second and second["kind"] == "laser" and (second.get("charge") or 0) > 0:
                fire[1] = True  # keep holding; it auto-fires after hold_s
        act["fire"] = fire

        mf.draw.label(f"hp {me['hp']:.0f} {'RET' if retreat else ''}{'ZZ' if zone_critical else ('Z' if zone_urgent else '')} t{len(mechs)}")
        return act

    def find_cover(self, x, y, z, src):
        """Nearest walkable spot within ~40 m that the threat can't see."""
        g = self.grid
        eye = (src[0], src[1], (src[2] if len(src) > 2 else z) + 8)
        best = None
        k0 = g.ccell(x, y)
        if k0 < 0:
            return None
        j0, i0 = divmod(k0, g.cnx)
        for r in range(1, 11):
            for dj in range(-r, r + 1):
                for di in (-r, r) if abs(dj) != r else range(-r, r + 1):
                    jj, ii = j0 + dj, i0 + di
                    if not (0 <= jj < g.cny and 0 <= ii < g.cnx):
                        continue
                    kk = jj * g.cnx + ii
                    if g.cblock[kk]:
                        continue
                    cx, cy = g.ccenter(kk)
                    if not g.los(eye, (cx, cy, g.ground(cx, cy) + 4.5)):
                        d = math.hypot(cx - x, cy - y)
                        if best is None or d < best[0]:
                            best = (d, (cx, cy))
            if best is not None:
                return best[1]
        return None

    def nearest_cover(self, x, y):
        g = self.grid
        k0 = g.ccell(x, y)
        if k0 < 0:
            return None
        j0, i0 = divmod(k0, g.cnx)
        for r in range(1, 8):
            for dj in range(-r, r + 1):
                for di in (-r, r) if abs(dj) != r else range(-r, r + 1):
                    jj, ii = j0 + dj, i0 + di
                    if 0 <= jj < g.cny and 0 <= ii < g.cnx and g.cover[jj * g.cnx + ii]:
                        return g.ccenter(jj * g.cnx + ii)
        return None


mf.run(Bot)
