"""
S029 Urban Logistics Scheduling
===============================
Multi-depot, multi-drone delivery with time windows (VRPTW).  3 depots,
12 customers, 6 drones (2 per depot).  Each drone may fly several sorties;
every sortie starts and ends at the drone's depot, and capacity Q and range
R_max are enforced per sortie.

Strategies
  1. Greedy NN      : nearest-depot assignment, nearest-neighbour sorties,
                      sorties given to the depot drone that is free first.
  2. Clarke-Wright  : savings merges within each nearest-depot group,
                      sorties given to the depot drone that is free first.
  3. TW insertion   : time-window-aware VRPTW heuristic (OR-Tools is not
                      installed): customers in order of latest time l_c are
                      inserted at the cheapest feasible position (any drone,
                      any depot, any sortie, or a new sortie) measured by the
                      card objective alpha*TWT + (1-alpha)*distance, then a
                      relocate local search improves the result.

Two instances, same depots / customers / demands / weights:
  card  : time windows from the card (earliest 0-120 s, width 60-180 s)
  tight : narrow windows spread over the day (earliest 0-240 s, width 20-60 s)

Usage:
    conda activate drones
    python src/02_logistics_delivery/s029_urban_logistics_scheduling.py
"""

import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.animation import FuncAnimation
from scipy.spatial.distance import cdist
from itertools import combinations

OUT_DIR = os.path.normpath(os.path.join(
    os.path.dirname(__file__), "..", "..",
    "outputs", "02_logistics_delivery", "s029_urban_logistics_scheduling"))
os.makedirs(OUT_DIR, exist_ok=True)

# ─── Constants ────────────────────────────────────────────────────────────────
N_DEPOTS    = 3
N_CUSTOMERS = 12
N_DRONES    = 6          # 2 per depot
V_DRONE     = 15.0       # m/s
Q_MAX       = 3.0        # kg per sortie
R_MAX       = 4000.0     # m per sortie
SVC_TIME    = 10.0       # s per customer
ALPHA       = 0.7        # tardiness vs distance weight
SEED        = 42         # card instance
SEED_TIGHT  = 2029       # tight-window time windows

DEPOT_POS = np.array([[100., 100.], [500., 900.], [900., 200.]])
DEPOT_OF_DRONE = [k // 2 for k in range(N_DRONES)]

_rng = np.random.default_rng(SEED)
CUST_POS = _rng.uniform(50, 950, (N_CUSTOMERS, 2))
EARLY    = _rng.uniform(0, 120, N_CUSTOMERS)
LATE     = EARLY + _rng.uniform(60, 180, N_CUSTOMERS)
DEMAND   = _rng.uniform(0.3, 1.5, N_CUSTOMERS)
WEIGHT   = _rng.uniform(0.5, 2.0, N_CUSTOMERS)

_rng_t = np.random.default_rng(SEED_TIGHT)
EARLY_T = _rng_t.uniform(0, 240, N_CUSTOMERS)
LATE_T  = EARLY_T + _rng_t.uniform(20, 60, N_CUSTOMERS)

INSTANCES = {"card": (EARLY, LATE), "tight": (EARLY_T, LATE_T)}
STRATS = ["Greedy NN", "Clarke-Wright", "TW insertion"]
COLORS = plt.cm.tab10(np.arange(N_DRONES))


def dist(a, b):
    return float(np.linalg.norm(np.asarray(a) - np.asarray(b)))


def sortie_length(sortie, dep):
    pts = [dep] + [CUST_POS[c] for c in sortie] + [dep]
    return sum(dist(pts[i], pts[i + 1]) for i in range(len(pts) - 1))


def feasible_sortie(sortie, dep):
    return (sum(DEMAND[c] for c in sortie) <= Q_MAX + 1e-9
            and sortie_length(sortie, dep) <= R_MAX + 1e-9)


# ─── Schedule evaluation ─────────────────────────────────────────────────────

def drone_schedule(plan, k, early, late):
    """Fly drone k's sorties back to back (depot return between sorties).

    Returns dict with distance, weighted tardiness, events and deliveries.
    Events: (kind, t0, t1, p0, p1, customer)."""
    dep = DEPOT_POS[DEPOT_OF_DRONE[k]]
    t, total_d, twt = 0.0, 0.0, 0.0
    events, deliv = [], {}
    for sortie in plan:
        pos = dep
        for c in sortie:
            d = dist(pos, CUST_POS[c])
            arrive = t + d / V_DRONE
            events.append(("fly", t, arrive, pos, CUST_POS[c], c))
            start = max(arrive, early[c])
            if start > arrive:
                events.append(("wait", arrive, start, CUST_POS[c],
                               CUST_POS[c], c))
            events.append(("svc", start, start + SVC_TIME, CUST_POS[c],
                           CUST_POS[c], c))
            twt += WEIGHT[c] * max(0.0, start - late[c])
            deliv[c] = start
            total_d += d
            t, pos = start + SVC_TIME, CUST_POS[c]
        d = dist(pos, dep)
        events.append(("fly", t, t + d / V_DRONE, pos, dep, None))
        total_d += d
        t += d / V_DRONE
    return dict(dist=total_d, twt=twt, events=events, deliv=deliv, end=t,
                obj=ALPHA * twt + (1 - ALPHA) * total_d)


def evaluate(plans, inst):
    early, late = INSTANCES[inst]
    sch = [drone_schedule(p, k, early, late) for k, p in enumerate(plans)]
    deliv = {}
    for s in sch:
        deliv.update(s["deliv"])
    sorties = [(k, s) for k, p in enumerate(plans) for s in p]
    tard = {c: max(0.0, deliv[c] - late[c]) for c in deliv}
    return dict(
        sched=sch, deliv=deliv, plans=plans,
        total_dist=sum(s["dist"] for s in sch),
        total_twt=sum(s["twt"] for s in sch),
        objective=sum(s["obj"] for s in sch),
        makespan=max(s["end"] for s in sch),
        n_sorties=len(sorties),
        n_drones_used=sum(1 for p in plans if p),
        n_late=sum(1 for c in tard if tard[c] > 1e-9),
        max_load=max(sum(DEMAND[c] for c in s) for _, s in sorties),
        max_len=max(sortie_length(s, DEPOT_POS[DEPOT_OF_DRONE[k]])
                    for k, s in sorties),
        served=len(deliv))


# ─── Sortie-to-drone assignment (strategies 1 and 2) ─────────────────────────

def assign_sorties(sorties_by_depot, inst):
    """Give each depot's sorties, in order, to the depot drone free first."""
    early, late = INSTANCES[inst]
    plans = [[] for _ in range(N_DRONES)]
    for d, sorties in sorties_by_depot.items():
        drones = [k for k in range(N_DRONES) if DEPOT_OF_DRONE[k] == d]
        for s in sorties:
            k = min(drones, key=lambda kk: (
                drone_schedule(plans[kk], kk, early, late)["end"], kk))
            plans[k].append(s)
    return plans


def nearest_depot_groups():
    nd = np.argmin(cdist(CUST_POS, DEPOT_POS), axis=1)
    return {d: [c for c in range(N_CUSTOMERS) if nd[c] == d]
            for d in range(N_DEPOTS)}


def greedy_nn(inst):
    sorties_by_depot = {}
    for d, custs in nearest_depot_groups().items():
        dep, todo, sorties = DEPOT_POS[d], set(custs), []
        while todo:
            sortie, pos = [], dep
            while True:
                cands = [(dist(pos, CUST_POS[c]), c) for c in sorted(todo)
                         if feasible_sortie(sortie + [c], dep)]
                if not cands:
                    break
                _, c = min(cands)
                sortie.append(c); todo.discard(c); pos = CUST_POS[c]
            sorties.append(sortie)
        sorties_by_depot[d] = sorties
    return assign_sorties(sorties_by_depot, inst)


def clarke_wright(inst):
    sorties_by_depot = {}
    for d, custs in nearest_depot_groups().items():
        dep = DEPOT_POS[d]
        routes = [[c] for c in custs]
        sav = sorted(((dist(dep, CUST_POS[i]) + dist(dep, CUST_POS[j])
                       - dist(CUST_POS[i], CUST_POS[j]), i, j)
                      for i, j in combinations(custs, 2)), reverse=True)
        for _, i, j in sav:
            ri = next(r for r in routes if i in r)
            rj = next(r for r in routes if j in r)
            if ri is rj or i not in (ri[0], ri[-1]) or j not in (rj[0], rj[-1]):
                continue
            a = ri if ri[-1] == i else ri[::-1]
            b = rj if rj[0] == j else rj[::-1]
            if feasible_sortie(a + b, dep):
                routes.remove(ri); routes.remove(rj); routes.append(a + b)
        sorties_by_depot[d] = routes
    return assign_sorties(sorties_by_depot, inst)


# ─── Strategy 3: time-window-aware insertion + relocate local search ────────

def _best_insertion(plans, c, early, late, base):
    """Cheapest feasible insertion of customer c; returns (delta, k, plan)."""
    best = None
    for k in range(N_DRONES):
        dep = DEPOT_POS[DEPOT_OF_DRONE[k]]
        cands = []
        for si, s in enumerate(plans[k]):
            for pos in range(len(s) + 1):
                ns = s[:pos] + [c] + s[pos:]
                if feasible_sortie(ns, dep):
                    cands.append(plans[k][:si] + [ns] + plans[k][si + 1:])
        for si in range(len(plans[k]) + 1):          # new sortie
            cands.append(plans[k][:si] + [[c]] + plans[k][si:])
        for p in cands:
            delta = drone_schedule(p, k, early, late)["obj"] - base[k]
            if best is None or delta < best[0] - 1e-9:
                best = (delta, k, p)
    return best


def tw_insertion(inst):
    early, late = INSTANCES[inst]
    plans = [[] for _ in range(N_DRONES)]
    base = [0.0] * N_DRONES
    for c in sorted(range(N_CUSTOMERS), key=lambda c: (late[c], c)):
        _, k, p = _best_insertion(plans, c, early, late, base)
        plans[k] = p
        base[k] = drone_schedule(p, k, early, late)["obj"]
    history = [sum(base)]
    improved = True
    while improved:                                   # relocate local search
        improved = False
        for c in range(N_CUSTOMERS):
            k0 = next(k for k in range(N_DRONES)
                      if any(c in s for s in plans[k]))
            reduced = [[x for x in s if x != c] for s in plans[k0]]
            reduced = [s for s in reduced if s]
            trial = [list(map(list, p)) for p in plans]
            trial[k0] = reduced
            tbase = list(base)
            tbase[k0] = drone_schedule(reduced, k0, early, late)["obj"]
            _, k, p = _best_insertion(trial, c, early, late, tbase)
            new_k = drone_schedule(p, k, early, late)["obj"]
            new_total = sum(tbase) - tbase[k] + new_k
            if new_total < sum(base) - 1e-6:
                trial[k] = p
                tbase[k] = new_k
                plans, base = trial, tbase
                history.append(sum(base))
                improved = True
    return plans, history


# ─── Simulation driver ───────────────────────────────────────────────────────

def run_simulation():
    """Run all strategies on both instances; returns results and LS history."""
    res, hist = {}, {}
    for inst in INSTANCES:
        res[inst] = {}
        res[inst]["Greedy NN"] = evaluate(greedy_nn(inst), inst)
        res[inst]["Clarke-Wright"] = evaluate(clarke_wright(inst), inst)
        plans, h = tw_insertion(inst)
        res[inst]["TW insertion"] = evaluate(plans, inst)
        hist[inst] = h
    return res, hist


# ─── Plots ────────────────────────────────────────────────────────────────────

def _map(ax, r, title):
    ax.set_xlim(0, 1000); ax.set_ylim(0, 1000); ax.set_aspect("equal")
    ax.scatter(CUST_POS[:, 0], CUST_POS[:, 1], s=60, c="steelblue", zorder=4)
    for c in range(N_CUSTOMERS):
        ax.text(CUST_POS[c, 0] + 12, CUST_POS[c, 1] + 12, f"C{c}", fontsize=7)
    for d, dp in enumerate(DEPOT_POS):
        ax.plot(*dp, "^", ms=13, color="crimson", zorder=5)
        ax.text(dp[0] + 15, dp[1] - 35, f"D{d}", color="crimson",
                fontweight="bold")
    handles = []
    for k, plan in enumerate(r["plans"]):
        dep = DEPOT_POS[DEPOT_OF_DRONE[k]]
        for si, s in enumerate(plan):
            pts = np.array([dep] + [CUST_POS[c] for c in s] + [dep])
            ax.plot(pts[:, 0], pts[:, 1], color=COLORS[k], lw=1.6,
                    ls="-" if si == 0 else "--", zorder=3)
        if plan:
            handles.append(mpatches.Patch(
                color=COLORS[k], label=f"Drone {k} (D{DEPOT_OF_DRONE[k]}), "
                f"{len(plan)} sortie{'s' if len(plan) > 1 else ''}"))
    ax.legend(handles=handles, fontsize=6, loc="lower right")
    ax.set_title(f"{title}\n{r['total_dist']:.0f} m, TWT {r['total_twt']:.1f}",
                 fontsize=9)
    ax.grid(True, alpha=0.3)


def plot_route_maps(res):
    fig, axes = plt.subplots(2, 3, figsize=(16, 11))
    for row, inst in enumerate(INSTANCES):
        for col, s in enumerate(STRATS):
            _map(axes[row, col], res[inst][s], f"{inst} windows — {s}")
    plt.suptitle("S029 — Routes (dashed: second or later sortie of a drone)")
    plt.tight_layout()
    _save(fig, "route_maps.png")


def plot_gantt(res, inst):
    early, late = INSTANCES[inst]
    fig, axes = plt.subplots(3, 1, figsize=(13, 10), sharex=True)
    cols = {"fly": "steelblue", "wait": "gold", "svc": "forestgreen"}
    for ax, s in zip(axes, STRATS):
        r = res[inst][s]
        for k, sch in enumerate(r["sched"]):
            for kind, t0, t1, _, _, c in sch["events"]:
                ax.barh(k, t1 - t0, left=t0, height=0.45, color=cols[kind],
                        edgecolor="white", lw=0.4)
                if kind == "svc":
                    late_c = t0 > late[c] + 1e-9
                    ax.plot([early[c], late[c]], [k + 0.32, k + 0.32], "-",
                            color="red" if late_c else "k", lw=2)
                    ax.text(t0, k - 0.38, f"C{c}", fontsize=6,
                            color="red" if late_c else "k")
        ax.set_yticks(range(N_DRONES))
        ax.set_yticklabels([f"Drone {k} (D{DEPOT_OF_DRONE[k]})"
                            for k in range(N_DRONES)], fontsize=8)
        ax.set_title(f"{s}: TWT {r['total_twt']:.1f}, late customers "
                     f"{r['n_late']}, makespan {r['makespan']:.0f} s",
                     fontsize=10)
        ax.grid(True, axis="x", alpha=0.3)
    axes[-1].set_xlabel("Time (s)")
    axes[0].legend(handles=[
        mpatches.Patch(color="steelblue", label="Travel"),
        mpatches.Patch(color="gold", label="Wait for window"),
        mpatches.Patch(color="forestgreen", label="Service"),
        plt.Line2D([], [], color="k", lw=2, label="Time window [e, l]"),
        plt.Line2D([], [], color="red", lw=2, label="Window missed")],
        fontsize=7, loc="upper right")
    plt.suptitle(f"S029 — Drone timelines, {inst} time windows")
    plt.tight_layout()
    _save(fig, f"gantt_{inst}.png")


def plot_comparison(res):
    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    x = np.arange(len(STRATS))
    for i, inst in enumerate(INSTANCES):
        for ax, key, ttl in zip(axes, ["total_twt", "total_dist", "objective"],
                                ["Weighted tardiness (s)", "Total distance (m)",
                                 f"Objective (alpha = {ALPHA})"]):
            vals = [res[inst][s][key] for s in STRATS]
            b = ax.bar(x + (i - 0.5) * 0.38, vals, 0.38,
                       label=f"{inst} windows",
                       color=["steelblue", "darkorange"][i])
            for bb, v in zip(b, vals):
                ax.text(bb.get_x() + bb.get_width() / 2, v, f"{v:.0f}",
                        ha="center", va="bottom", fontsize=8)
            ax.set_title(ttl); ax.set_xticks(x); ax.set_xticklabels(STRATS)
            ax.grid(axis="y", alpha=0.3)
    axes[0].legend()
    plt.suptitle("S029 — Strategy comparison")
    plt.tight_layout()
    _save(fig, "comparison_metrics.png")


def plot_delivery_table(r, inst, name):
    early, late = INSTANCES[inst]
    rows = []
    for c in range(N_CUSTOMERS):
        s = r["deliv"][c]
        rows.append([f"C{c}", f"{early[c]:.1f}", f"{late[c]:.1f}",
                     f"{s:.1f}", f"{max(0, s - late[c]):.1f}",
                     f"{WEIGHT[c] * max(0, s - late[c]):.1f}"])
    fig, ax = plt.subplots(figsize=(10, 4.8))
    ax.axis("off")
    ax.set_title(f"S029 — Per-customer deliveries, {name}, {inst} windows")
    tb = ax.table(cellText=rows, cellLoc="center", loc="center",
                  colLabels=["Customer", "Earliest (s)", "Latest (s)",
                             "Service start (s)", "Late by (s)",
                             "Weighted tardiness"])
    tb.auto_set_font_size(False); tb.set_fontsize(9); tb.scale(1.1, 1.35)
    plt.tight_layout()
    _save(fig, "delivery_table.png")


def plot_convergence(hist):
    fig, ax = plt.subplots(figsize=(8, 4))
    for inst, h in hist.items():
        ax.plot(range(len(h)), h, "o-", label=f"{inst} windows")
    ax.set_xlabel("Accepted relocate moves (0 = after insertion)")
    ax.set_ylabel("Objective")
    ax.set_title("S029 — TW insertion: relocate local search")
    ax.grid(True, alpha=0.3); ax.legend()
    plt.tight_layout()
    _save(fig, "ls_convergence.png")


def _save(fig, name):
    fig.savefig(os.path.join(OUT_DIR, name), dpi=120)
    plt.close(fig)
    print(f"  Saved: {name}")


def save_animation(r, inst, name, n_frames=120, fps=10):
    """Top-down GIF of one solution (<= 150 frames)."""
    sched = r["sched"]
    t_end = max(s["end"] for s in sched)
    times = np.linspace(0, t_end, n_frames)

    def pos_at(k, t):
        for kind, t0, t1, p0, p1, _ in sched[k]["events"]:
            if t0 <= t <= t1:
                f = (t - t0) / (t1 - t0) if t1 > t0 else 1.0
                return np.asarray(p0) + f * (np.asarray(p1) - np.asarray(p0))
        return DEPOT_POS[DEPOT_OF_DRONE[k]]

    P = np.array([[pos_at(k, t) for k in range(N_DRONES)] for t in times])
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.set_xlim(0, 1000); ax.set_ylim(0, 1000); ax.set_aspect("equal")
    ax.grid(True, alpha=0.25)
    for d, dp in enumerate(DEPOT_POS):
        ax.plot(*dp, "^", ms=13, color="crimson")
    circ = [ax.plot(*CUST_POS[c], "o", ms=9, color="steelblue")[0]
            for c in range(N_CUSTOMERS)]
    for c in range(N_CUSTOMERS):
        ax.text(CUST_POS[c, 0] + 11, CUST_POS[c, 1] + 11, f"C{c}", fontsize=7)
    dots = [ax.plot([], [], "o", color=COLORS[k], ms=8,
                    label=f"Drone {k}")[0] for k in range(N_DRONES)]
    trails = [ax.plot([], [], "-", color=COLORS[k], lw=1.4, alpha=0.5)[0]
              for k in range(N_DRONES)]
    ax.legend(fontsize=7, loc="lower right")
    ttl = ax.set_title("")
    early, late = INSTANCES[inst]

    def update(f):
        t = times[f]
        for c in range(N_CUSTOMERS):
            s = r["deliv"][c]
            if t >= s:
                circ[c].set_color("red" if s > late[c] + 1e-9 else "limegreen")
        for k in range(N_DRONES):
            dots[k].set_data([P[f, k, 0]], [P[f, k, 1]])
            lo = max(0, f - 8)
            trails[k].set_data(P[lo:f + 1, k, 0], P[lo:f + 1, k, 1])
        ttl.set_text(f"{name}, {inst} windows — t = {t:.0f} s\n"
                     "(green: delivered in window, red: late)")
        return dots + trails + circ + [ttl]

    anim = FuncAnimation(fig, update, frames=n_frames, blit=False)
    anim.save(os.path.join(OUT_DIR, "animation.gif"), writer="pillow", fps=fps)
    plt.close(fig)
    print(f"  Saved: animation.gif ({n_frames} frames, {fps} fps, "
          f"{t_end:.0f} s of flight)")


# ─── Main ─────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    print("=" * 66)
    print("S029  Urban Logistics Scheduling")
    print("=" * 66)
    res, hist = run_simulation()
    for inst in INSTANCES:
        early, late = INSTANCES[inst]
        print(f"\n### {inst} time windows: earliest {early.min():.1f}-"
              f"{early.max():.1f} s, width {(late - early).min():.1f}-"
              f"{(late - early).max():.1f} s")
        for s in STRATS:
            m = res[inst][s]
            print(f"  {s:14s}: dist {m['total_dist']:7.1f} m | TWT "
                  f"{m['total_twt']:7.2f} | late {m['n_late']:2d}/12 | obj "
                  f"{m['objective']:8.2f} | makespan {m['makespan']:6.1f} s | "
                  f"sorties {m['n_sorties']:2d} on {m['n_drones_used']} drones"
                  f" | max load {m['max_load']:.2f} kg, max sortie "
                  f"{m['max_len']:.0f} m | served {m['served']}/12")
            for k, p in enumerate(m["plans"]):
                if p:
                    print(f"      drone {k} (D{DEPOT_OF_DRONE[k]}): {p}")
        h = hist[inst]
        print(f"  TW insertion local search: {h[0]:.2f} -> {h[-1]:.2f} "
              f"({len(h) - 1} relocate moves)")
        m = res[inst]["TW insertion"]
        for c in sorted(m["deliv"]):
            late_by = m["deliv"][c] - late[c]
            if late_by > 1e-9:
                d = min(dist(dp, CUST_POS[c]) for dp in DEPOT_POS)
                print(f"    late C{c}: {late_by:.1f} s late, weighted "
                      f"{WEIGHT[c] * late_by:.1f} -> objective cost "
                      f"{ALPHA * WEIGHT[c] * late_by:.1f}; a dedicated sortie"
                      f" from the nearest depot ({d:.1f} m away) would cost "
                      f"{(1 - ALPHA) * 2 * d:.1f} in distance")
    print("\nGenerating plots ...")
    plot_route_maps(res)
    plot_gantt(res, "card")
    plot_gantt(res, "tight")
    plot_comparison(res)
    best = min(STRATS, key=lambda s: res["tight"][s]["objective"])
    print(f"  Best on tight windows: {best}")
    plot_delivery_table(res["tight"][best], "tight", best)
    plot_convergence(hist)
    save_animation(res["tight"]["TW insertion"], "tight", "TW insertion")
