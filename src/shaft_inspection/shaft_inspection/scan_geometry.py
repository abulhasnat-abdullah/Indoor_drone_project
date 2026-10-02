#!/usr/bin/env python3
"""
Geometry helpers for vertical-shaft perception from a single horizontal 2D lidar.

Everything here works in the LEVEL BODY frame: the vehicle's yaw-aligned frame
with roll and pitch removed (x forward, y left, z up).  De-tilting is not
cosmetic -- see detilt_points() for why skipping it breaks the controller.
"""
import math
import numpy as np


def detilt_points(ranges, angle_min, angle_increment, roll, pitch,
                  range_min, range_max):
    """Project raw lidar returns into the level body frame.

    The lidar scans a plane fixed to the airframe.  When the vehicle tilts,
    that plane tilts with it, so a circular shaft images as an ELLIPSE whose
    apparent centre is displaced from the true axis.  A centring controller fed
    that raw centre chases a phantom offset and oscillates -- it looks exactly
    like a badly tuned gain, which is why this is worth doing properly.

    Rotating each beam by Ry(pitch) @ Rx(roll) removes the tilt and leaves a
    yaw-aligned level frame, where a round shaft really is round.

    Returns an (N, 2) array of XY points, invalid returns dropped.
    """
    r = np.asarray(ranges, dtype=np.float64)
    n = r.size
    if n == 0:
        return np.empty((0, 2))

    ang = angle_min + angle_increment * np.arange(n)

    good = np.isfinite(r) & (r > range_min) & (r < range_max)
    if not np.any(good):
        return np.empty((0, 2))
    r = r[good]
    ang = ang[good]

    # Beam endpoints in the (tilted) sensor plane: x forward, y left, z up.
    p = np.stack([r * np.cos(ang), r * np.sin(ang), np.zeros_like(r)], axis=0)

    # roll/pitch arrive in PX4's FRD convention while the scan is FLU.
    # Conjugating by diag(1,-1,-1) leaves a roll rotation unchanged but
    # reverses the sense of pitch, so pitch is negated here.
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(-pitch), math.sin(-pitch)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])

    lvl = (ry @ rx) @ p          # body -> level (yaw-aligned) frame
    return lvl[:2, :].T


def fit_circle_kasa(pts):
    """Algebraic (Kasa) circle fit.  Returns (cx, cy, r) or None."""
    if pts.shape[0] < 3:
        return None
    x = pts[:, 0]
    y = pts[:, 1]
    a = np.stack([2.0 * x, 2.0 * y, np.ones_like(x)], axis=1)
    b = x * x + y * y
    try:
        sol, *_ = np.linalg.lstsq(a, b, rcond=None)
    except np.linalg.LinAlgError:
        return None
    cx, cy, c = sol
    disc = c + cx * cx + cy * cy
    if not np.isfinite(disc) or disc <= 0.0:
        return None
    return float(cx), float(cy), float(math.sqrt(disc))


def fit_circle_robust(pts, inlier_tol=0.25, iterations=3, min_inliers=40,
                      min_radius=0.3, max_radius=8.0):
    """Circle fit with iterative outlier rejection.

    A shaft wall is never a clean circle: pipes, cables, ledges and spalling
    all pull an ordinary least-squares fit off-axis.  Re-fitting on inliers
    only keeps the estimate anchored to the dominant bore.

    Returns dict(ok, cx, cy, r, n_inliers, rms) -- cx,cy is the vector from the
    vehicle TO the shaft axis, in the level body frame.
    """
    fail = dict(ok=False, cx=0.0, cy=0.0, r=0.0, n_inliers=0, rms=float('inf'))
    if pts.shape[0] < min_inliers:
        return fail

    keep = pts
    res = fit_circle_kasa(keep)
    if res is None:
        return fail

    for _ in range(iterations):
        cx, cy, r = res
        d = np.hypot(pts[:, 0] - cx, pts[:, 1] - cy)
        mask = np.abs(d - r) < inlier_tol
        if np.count_nonzero(mask) < min_inliers:
            break
        keep = pts[mask]
        nxt = fit_circle_kasa(keep)
        if nxt is None:
            break
        res = nxt

    cx, cy, r = res
    if not (min_radius < r < max_radius):
        return fail

    d = np.hypot(keep[:, 0] - cx, keep[:, 1] - cy)
    rms = float(np.sqrt(np.mean((d - r) ** 2))) if keep.size else float('inf')

    return dict(ok=True, cx=float(cx), cy=float(cy), r=float(r),
                n_inliers=int(keep.shape[0]), rms=rms)


def sector_min_ranges(pts, n_sectors=36):
    """Minimum obstacle range per angular sector, level body frame.

    Index i covers [i*360/n, (i+1)*360/n) degrees measured CCW from +x
    (forward).  Sectors with no return hold +inf.
    """
    out = np.full(n_sectors, np.inf)
    if pts.shape[0] == 0:
        return out
    d = np.hypot(pts[:, 0], pts[:, 1])
    a = np.arctan2(pts[:, 1], pts[:, 0]) % (2.0 * math.pi)
    idx = np.minimum((a / (2.0 * math.pi) * n_sectors).astype(int), n_sectors - 1)
    np.minimum.at(out, idx, d)
    return out


def repulsion_vector(pts, d_influence=0.8, d_min=0.25, gain=1.0, max_speed=0.6):
    """Potential-field push-away velocity, level body frame.

    Complements the circle fit rather than duplicating it: the fit models the
    bore as a whole and will happily ignore a single protruding ledge or a
    hanging cable as an outlier.  This term reacts to exactly those.

    Returns (vx, vy) in m/s.
    """
    if pts.shape[0] == 0:
        return 0.0, 0.0
    d = np.hypot(pts[:, 0], pts[:, 1])
    near = d < d_influence
    if not np.any(near):
        return 0.0, 0.0

    dn = np.clip(d[near], d_min, None)
    # Unit vector pointing from the obstacle back toward the vehicle.
    ux = -pts[near, 0] / dn
    uy = -pts[near, 1] / dn
    w = gain * (1.0 / dn - 1.0 / d_influence)

    vx = float(np.sum(w * ux))
    vy = float(np.sum(w * uy))

    mag = math.hypot(vx, vy)
    if mag > max_speed and mag > 1e-9:
        vx *= max_speed / mag
        vy *= max_speed / mag
    return vx, vy


def clamp_velocity_for_obstacles(vx, vy, sector_min, n_sectors=36,
                                 stop_dist=0.35, delay=0.4, decel=1.0):
    """Scale a commanded level-frame velocity so it cannot drive into a wall.

    This is the same idea as PX4's Collision Prevention, reimplemented because
    PX4 wires CollisionPrevention only into the ManualPosition flight task --
    it does NOT constrain Offboard setpoints, which is what this mission flies.

    For each sector we allow only the speed that can still be braked inside the
    free distance, accounting for control delay.
    """
    speed = math.hypot(vx, vy)
    if speed < 1e-6:
        return vx, vy

    allowed = speed
    for i in range(n_sectors):
        d = sector_min[i]
        if not np.isfinite(d):
            continue
        ang = 2.0 * math.pi * (i + 0.5) / n_sectors
        # Component of the commanded velocity toward this sector.
        comp = vx * math.cos(ang) + vy * math.sin(ang)
        if comp <= 0.0:
            continue
        free = d - stop_dist
        if free <= 0.0:
            allowed = 0.0
            break
        # v such that v*delay + v^2/(2*decel) <= free
        v_max = (-delay * decel +
                 math.sqrt((delay * decel) ** 2 + 2.0 * decel * free))
        if comp > v_max:
            scale = v_max / comp
            allowed = min(allowed, speed * scale)

    if allowed >= speed:
        return vx, vy
    if allowed <= 0.0:
        return 0.0, 0.0
    s = allowed / speed
    return vx * s, vy * s


# ---------------------------------------------------------------------------
# Shape-agnostic centring
#
# A real shaft is not a circle: bores are cut rectangular, they weather into
# irregular polygons, and the cross-section changes with depth.  A circle fit
# is only meaningful when the bore really is round, so it is kept as an
# optional descriptor while CONTROL runs off the functions below, which assume
# nothing about shape.
# ---------------------------------------------------------------------------

def polar_min_map(pts, n_bins=180):
    """Nearest return per bearing bin.  Returns (ranges, bearings).

    Taking the minimum per bin is the conservative reduction: a protruding
    ledge shortens its bin rather than being averaged away.
    """
    rng = np.full(n_bins, np.inf)
    bearings = (np.arange(n_bins) + 0.5) * (2.0 * math.pi / n_bins)
    if pts.shape[0] == 0:
        return rng, bearings
    d = np.hypot(pts[:, 0], pts[:, 1])
    a = np.arctan2(pts[:, 1], pts[:, 0]) % (2.0 * math.pi)
    idx = np.minimum((a / (2.0 * math.pi) * n_bins).astype(int), n_bins - 1)
    np.minimum.at(rng, idx, d)
    return rng, bearings


def mirror_fill(rng):
    """Fill bearing bins with no return by mirroring the opposite bearing.

    A 270-degree lidar leaves a 90-degree blind wedge.  Left empty, a
    max-clearance search happily walks the vehicle into that unknown region,
    because unknown reads as infinitely far.  Mirroring assumes the bore is
    roughly centrally symmetric over one scan -- far weaker than assuming it is
    circular, and it fails safe: it invents a wall rather than free space.
    """
    out = rng.copy()
    n = out.size
    missing = ~np.isfinite(out)
    if not np.any(missing) or np.all(missing):
        return out
    opp = (np.arange(n) + n // 2) % n
    out[missing] = rng[opp][missing]
    # Anything still unknown: fall back to the median observed range.
    still = ~np.isfinite(out)
    if np.any(still):
        med = np.median(rng[np.isfinite(rng)])
        out[still] = med
    return out


def polygon_centroid(wall):
    """Area centroid of the closed boundary polygon (shoelace).

    Unlike averaging the raw returns, this is independent of angular sampling
    density, so an off-axis vehicle does not drag the centroid toward itself.
    """
    x = wall[:, 0]
    y = wall[:, 1]
    x1 = np.roll(x, -1)
    y1 = np.roll(y, -1)
    cross = x * y1 - x1 * y
    area = 0.5 * float(np.sum(cross))
    if abs(area) < 1e-9:
        return float(np.mean(x)), float(np.mean(y))
    cx = float(np.sum((x + x1) * cross) / (6.0 * area))
    cy = float(np.sum((y + y1) * cross) / (6.0 * area))
    return cx, cy


def center_max_clearance(pts, search=6.0, resolution=0.04, n_bins=180,
                         min_coverage=0.85, tie_eps=0.05, **_unused):
    """Chebyshev-style centre: the point of maximum clearance to the wall.

    Shape-agnostic by construction -- for a circle it lands on the axis, for a
    rectangle on the middle, for an irregular bore on the safest point.  That
    is exactly the quantity a "don't hit the wall" controller wants.

    Solved GLOBALLY with a Euclidean distance transform over a raster of the
    scanned free space, not by iterative grid refinement: coarse-to-fine search
    gets trapped in local optima once pipes and ledges carve the bore into
    several near-equal pockets, and silently returns a worse centre.

    Two constraints matter:
      * the centre must lie INSIDE the free space.  Max-min distance alone does
        not know which side of a wall it is on -- a point beyond the wall is
        also far from every wall sample.  The scan is star-shaped about the
        sensor, so a cell is free iff it is nearer than the return on its own
        bearing;
      * max clearance is degenerate for elongated bores (every point on a
        rectangle's long centreline ties), so near-ties are settled by the
        cross-section's area centroid, which is unique.

    Returns dict(ok, cx, cy, clearance, coverage) where (cx, cy) is the vector
    from the vehicle TO the best centre, in the level body frame.
    """
    from scipy.ndimage import distance_transform_edt

    fail = dict(ok=False, cx=0.0, cy=0.0, clearance=0.0, coverage=0.0)
    if pts.shape[0] < 20:
        return fail

    rng, bearings = polar_min_map(pts, n_bins)

    # Refuse to guess: filling a big gap biases the answer toward the
    # vehicle's own position -- a confidently wrong "you are centred".
    coverage = float(np.count_nonzero(np.isfinite(rng))) / float(n_bins)
    if coverage < min_coverage:
        fail['coverage'] = coverage
        return fail
    rng = mirror_fill(rng)
    if not np.all(np.isfinite(rng)):
        return fail

    wall = np.stack([rng * np.cos(bearings), rng * np.sin(bearings)], axis=1)
    centroid = np.array(polygon_centroid(wall))

    # Size the raster to enclose the whole scanned polygon.  A fixed window
    # clips it: its border counts as wall, capping clearance at the distance
    # to the border and dragging the centre toward the vehicle -- from a
    # launch point 1.95 m off-axis that under-reported 2.40 m as 1.68 m.
    # `search` is only an upper bound on cost.
    extent = min(float(np.max(rng)) + 3.0 * resolution, search)
    n = int(round(2.0 * extent / resolution)) + 1
    axis = (np.arange(n) - (n - 1) / 2.0) * resolution
    gx, gy = np.meshgrid(axis, axis, indexing='xy')

    cb = np.arctan2(gy, gx) % (2.0 * math.pi)
    cbin = np.minimum((cb / (2.0 * math.pi) * n_bins).astype(int), n_bins - 1)
    free = np.hypot(gx, gy) < rng[cbin]
    if not np.any(free):
        return fail

    # Distance from each free cell to the nearest non-free cell = clearance.
    # Cells at the raster border are marked non-free so an open-ended scan
    # cannot report clearance larger than the search window.
    free[0, :] = free[-1, :] = free[:, 0] = free[:, -1] = False
    dist = distance_transform_edt(free) * resolution

    c_max = float(dist.max())
    if c_max <= 0.0:
        return fail
    near = dist >= (c_max - tie_eps)
    iy, ix = np.nonzero(near)
    cand_x = axis[ix]
    cand_y = axis[iy]
    k = int(np.argmin((cand_x - centroid[0]) ** 2 + (cand_y - centroid[1]) ** 2))
    cx, cy = float(cand_x[k]), float(cand_y[k])
    clearance = float(dist[iy[k], ix[k]])

    return dict(ok=True, cx=cx, cy=cy, clearance=clearance, coverage=coverage)


def cross_section_metrics(pts, cx, cy, n_bins=180):
    """Descriptive profile of the bore about a given centre.

    Returns dict with inscribed clearance, max reach, mean radius and a
    roundness score (0 = ragged, 1 = perfectly circular).  Roundness is what
    decides whether a circle fit is worth trusting at this depth.
    """
    if pts.shape[0] == 0:
        return dict(r_min=0.0, r_max=0.0, r_mean=0.0, roundness=0.0)
    d = np.hypot(pts[:, 0] - cx, pts[:, 1] - cy)
    r_mean = float(np.mean(d))
    if r_mean < 1e-6:
        return dict(r_min=0.0, r_max=0.0, r_mean=0.0, roundness=0.0)
    spread = float(np.std(d)) / r_mean
    return dict(r_min=float(np.min(d)), r_max=float(np.max(d)),
                r_mean=r_mean, roundness=float(max(0.0, 1.0 - spread)))
