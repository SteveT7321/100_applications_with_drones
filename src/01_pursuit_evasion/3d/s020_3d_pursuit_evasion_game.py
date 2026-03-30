"""
S020 3D Upgrade — Pursuit-Evasion Game (3D RL/PPO)
Usage:
    conda activate drones
    python src/01_pursuit_evasion/3d/s020_3d_pursuit_evasion_game.py
"""
import sys, os, numpy as np, matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa
from matplotlib.animation import FuncAnimation, PillowWriter
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..'))

OUTPUT_DIR = os.path.join(os.path.dirname(__file__), '..', '..', '..', 'outputs', '01_pursuit_evasion', '3d', 's020_3d_pursuit_evasion_game')
os.makedirs(OUTPUT_DIR, exist_ok=True)

# ── Parameters ─────────────────────────────────
V_PURSUER  = 5.0
V_EVADER   = 3.5
CAPTURE_R  = 0.15
DT         = 1.0 / 48.0
T_MAX      = 30.0
ARENA      = 10.0
Z_MIN      = 0.3
Z_MAX      = 8.0
T_LEAD     = 0.3
EPS        = 1e-8

RNG = np.random.default_rng(0)

# Initial positions
P0_START = np.array([-5., 0., 2.])
E0_START = np.array([ 5., 0., 3.])

MAX_STEPS = int(T_MAX / DT)

PURSUER_STRATEGIES = ['pure_pursuit_3d', 'altitude_advantage', 'predictive_3d', 'rl_proxy']
EVADER_STRATEGIES  = ['straight_3d', 'helix_evader', 'altitude_jump']
PURSUER_COLORS_MAP = {
    'pure_pursuit_3d':   'crimson',
    'altitude_advantage': 'darkorange',
    'predictive_3d':     'seagreen',
    'rl_proxy':          'mediumpurple',
}
EVADER_COLORS_MAP = {
    'straight_3d':   'royalblue',
    'helix_evader':  'steelblue',
    'altitude_jump': 'navy',
}


# ── State normalization ─────────────────────────────────
def normalize_state(p_pos, p_vel, e_pos, e_vel, arena=10.0, v_max=5.0):
    dist = np.linalg.norm(e_pos - p_pos) + EPS
    state = np.concatenate([
        p_pos / arena, p_vel / v_max,
        e_pos / arena, e_vel / v_max,
        [dist / arena]
    ])
    return state.astype(np.float32)


# ── Pursuer strategies ─────────────────────────────────
def pursuer_pure_pursuit_3d(p_pos, p_vel, e_pos, e_vel, t):
    """Pure Pursuit: fly directly toward evader."""
    d = e_pos - p_pos
    n = np.linalg.norm(d) + EPS
    return V_PURSUER * d / n


def pursuer_altitude_advantage(p_pos, p_vel, e_pos, e_vel, t):
    """Altitude Advantage: approach from above if alt difference > 1m."""
    d = e_pos - p_pos
    dalt = p_pos[2] - e_pos[2]
    # Base: pure pursuit
    v = V_PURSUER * d / (np.linalg.norm(d) + EPS)
    # If pursuer is NOT above evader by >1m, add upward component
    if dalt < 1.0:
        v[2] += min(2.0, (1.0 - dalt) * 1.5)  # boost upward
    # Renormalize
    n = np.linalg.norm(v)
    if n > V_PURSUER:
        v = V_PURSUER * v / n
    return v


def pursuer_predictive_3d(p_pos, p_vel, e_pos, e_vel, t):
    """Predictive 3D: lead the evader by T_LEAD seconds."""
    e_pred = e_pos + e_vel * T_LEAD
    d = e_pred - p_pos
    n = np.linalg.norm(d) + EPS
    return V_PURSUER * d / n


def pursuer_rl_proxy(p_pos, p_vel, e_pos, e_vel, t):
    """RL Proxy: rule-based policy mimicking RL (pure pursuit + altitude bias + small noise)."""
    d = e_pos - p_pos
    n = np.linalg.norm(d) + EPS
    v = V_PURSUER * d / n
    # Altitude bias: approach from above
    dalt = p_pos[2] - e_pos[2]
    if dalt < 0.5:
        v[2] += 1.5
    # Lead component
    v += 0.3 * e_vel
    # Small noise to mimic stochastic exploration
    rng_noise = np.random.default_rng(int(t * 100) % (2**31))
    v += rng_noise.normal(0, 0.1, 3)
    n2 = np.linalg.norm(v)
    if n2 > V_PURSUER:
        v = V_PURSUER * v / n2
    return v


PURSUER_FUNCS = {
    'pure_pursuit_3d':    pursuer_pure_pursuit_3d,
    'altitude_advantage': pursuer_altitude_advantage,
    'predictive_3d':      pursuer_predictive_3d,
    'rl_proxy':           pursuer_rl_proxy,
}


# ── Evader strategies ─────────────────────────────────
class EvaderStraight3D:
    def __init__(self, rng=None):
        self.rng = rng or np.random.default_rng(1)

    def step(self, p_pos, e_pos, e_vel, t, dt):
        d = e_pos - p_pos
        n = np.linalg.norm(d) + EPS
        v = V_EVADER * d / n
        new_pos = e_pos + v * dt
        new_pos = np.clip(new_pos, -ARENA, ARENA)
        new_pos[2] = np.clip(new_pos[2], Z_MIN, Z_MAX)
        return new_pos, v


class EvaderHelix:
    """Helical motion around z-axis: horizontal circle + altitude oscillation."""
    def __init__(self, omega=0.8, Az=1.5, rng=None):
        self.omega = omega
        self.Az = Az
        self.rng = rng or np.random.default_rng(2)

    def step(self, p_pos, e_pos, e_vel, t, dt):
        # Escape from pursuer in xy, but with helical motion
        d_xy = e_pos[:2] - p_pos[:2]
        n_xy = np.linalg.norm(d_xy) + EPS
        # Escape direction in xy
        escape_xy = d_xy / n_xy
        # Perpendicular in xy for helical component
        perp_xy = np.array([-escape_xy[1], escape_xy[0]])
        # Blend: mostly escape, some helix
        v_xy = 0.6 * escape_xy + 0.4 * np.cos(self.omega * t) * perp_xy
        v_xy = v_xy / (np.linalg.norm(v_xy) + EPS)
        v_z = self.Az * np.sin(self.omega * t * 1.5)
        v = np.array([v_xy[0], v_xy[1], v_z])
        n = np.linalg.norm(v) + EPS
        v = V_EVADER * v / n
        new_pos = e_pos + v * dt
        new_pos = np.clip(new_pos, -ARENA, ARENA)
        new_pos[2] = np.clip(new_pos[2], Z_MIN, Z_MAX)
        return new_pos, v


class EvaderAltitudeJump:
    """Randomly change altitude every 2s while escaping horizontally."""
    def __init__(self, jump_interval=2.0, rng=None):
        self.jump_interval = jump_interval
        self.last_jump = -jump_interval
        self.target_alt = 3.0
        self.rng = rng or np.random.default_rng(3)

    def step(self, p_pos, e_pos, e_vel, t, dt):
        # Altitude jump logic
        if t - self.last_jump >= self.jump_interval:
            self.target_alt = self.rng.uniform(Z_MIN + 0.5, Z_MAX - 0.5)
            self.last_jump = t
        # Escape in xy from pursuer
        d_xy = e_pos[:2] - p_pos[:2]
        n_xy = np.linalg.norm(d_xy) + EPS
        v_xy = V_EVADER * d_xy / n_xy
        # Altitude control
        dz = self.target_alt - e_pos[2]
        v_z = np.clip(dz / (dt + EPS), -V_EVADER, V_EVADER)
        v = np.array([v_xy[0] * 0.8, v_xy[1] * 0.8, v_z * 0.6])
        n = np.linalg.norm(v) + EPS
        if n > V_EVADER:
            v = V_EVADER * v / n
        new_pos = e_pos + v * dt
        new_pos = np.clip(new_pos, -ARENA, ARENA)
        new_pos[2] = np.clip(new_pos[2], Z_MIN, Z_MAX)
        return new_pos, v


# ── Single episode simulation ─────────────────────────────────
def run_episode(pursuer_strategy, evader_strategy, p_start=None, e_start=None, seed=0):
    """Run one episode, return trajectory dict."""
    rng = np.random.default_rng(seed)
    p_pos = (p_start if p_start is not None else P0_START).copy().astype(float)
    e_pos = (e_start if e_start is not None else E0_START).copy().astype(float)
    p_vel = np.zeros(3)
    e_vel = np.zeros(3)

    # Init evader
    if evader_strategy == 'straight_3d':
        evader_obj = EvaderStraight3D(rng=np.random.default_rng(seed+1))
    elif evader_strategy == 'helix_evader':
        evader_obj = EvaderHelix(rng=np.random.default_rng(seed+2))
    elif evader_strategy == 'altitude_jump':
        evader_obj = EvaderAltitudeJump(rng=np.random.default_rng(seed+3))
    else:
        evader_obj = EvaderStraight3D(rng=np.random.default_rng(seed+1))

    pursuer_fn = PURSUER_FUNCS[pursuer_strategy]

    p_traj = [p_pos.copy()]
    e_traj = [e_pos.copy()]
    p_vel_hist = [p_vel.copy()]
    e_vel_hist = [e_vel.copy()]
    dist_hist = [np.linalg.norm(p_pos - e_pos)]
    capture_time = None
    captured = False

    for step in range(MAX_STEPS):
        t = step * DT
        # Pursuer moves
        p_vel = pursuer_fn(p_pos, p_vel, e_pos, e_vel, t)
        p_pos = p_pos + p_vel * DT
        p_pos = np.clip(p_pos, -ARENA, ARENA)
        p_pos[2] = np.clip(p_pos[2], Z_MIN, Z_MAX)

        # Evader moves
        e_pos_new, e_vel = evader_obj.step(p_pos, e_pos, e_vel, t, DT)
        e_pos = e_pos_new

        dist = np.linalg.norm(p_pos - e_pos)
        p_traj.append(p_pos.copy())
        e_traj.append(e_pos.copy())
        p_vel_hist.append(p_vel.copy())
        e_vel_hist.append(e_vel.copy())
        dist_hist.append(dist)

        if dist <= CAPTURE_R and not captured:
            capture_time = (step + 1) * DT
            captured = True
            break

    return {
        'p_traj': np.array(p_traj),
        'e_traj': np.array(e_traj),
        'p_vel_hist': np.array(p_vel_hist),
        'e_vel_hist': np.array(e_vel_hist),
        'dist_hist': np.array(dist_hist),
        'capture_time': capture_time,
        'captured': captured,
        'pursuer_strategy': pursuer_strategy,
        'evader_strategy': evader_strategy,
    }


# ── All combination results ─────────────────────────────────
def run_all():
    results = {}
    for ps in PURSUER_STRATEGIES:
        for es in EVADER_STRATEGIES:
            key = (ps, es)
            r = run_episode(ps, es, seed=42)
            results[key] = r
            ct = f'{r["capture_time"]:.2f}s' if r['captured'] else 'timeout'
            print(f'  {ps:20s} vs {es:16s} -> {ct}')
    return results


# ── Plots ─────────────────────────────────────────────────────

def plot_trajectories_3d(results):
    """2×2 grid: 4 pursuer strategies vs straight evader."""
    fig = plt.figure(figsize=(14, 12))
    for idx, ps in enumerate(PURSUER_STRATEGIES):
        r = results[(ps, 'straight_3d')]
        ax = fig.add_subplot(2, 2, idx + 1, projection='3d')
        ax.plot(r['p_traj'][:, 0], r['p_traj'][:, 1], r['p_traj'][:, 2],
                color=PURSUER_COLORS_MAP[ps], lw=2, label='Pursuer')
        ax.plot(r['e_traj'][:, 0], r['e_traj'][:, 1], r['e_traj'][:, 2],
                color=EVADER_COLORS_MAP['straight_3d'], lw=2, linestyle='--', label='Evader')
        ax.scatter(*r['p_traj'][0], color=PURSUER_COLORS_MAP[ps], s=60, marker='D')
        ax.scatter(*r['e_traj'][0], color=EVADER_COLORS_MAP['straight_3d'], s=60, marker='^')
        if r['captured']:
            ax.scatter(*r['p_traj'][-1], color='gold', s=150, marker='*', zorder=6, label='Capture')
        ct_str = f'{r["capture_time"]:.2f}s' if r['captured'] else 'timeout'
        ax.set_title(f'{ps}\nvs straight_3d | {ct_str}', fontsize=8)
        ax.set_xlabel('X (m)', fontsize=7)
        ax.set_ylabel('Y (m)', fontsize=7)
        ax.set_zlabel('Z (m)', fontsize=7)
        ax.set_zlim(Z_MIN, Z_MAX)
        ax.tick_params(labelsize=6)
        ax.legend(fontsize=7)

    fig.suptitle('S020 3D Pursuit-Evasion — Pursuer Strategies vs Straight Evader', fontsize=11)
    plt.tight_layout()
    path = os.path.join(OUTPUT_DIR, 'trajectories_3d.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_altitude_profiles(results):
    """Z vs time for best pursuer (altitude_advantage) vs all evader types."""
    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    best_pursuer = 'altitude_advantage'
    for ax, es in zip(axes, EVADER_STRATEGIES):
        r = results[(best_pursuer, es)]
        n = len(r['p_traj'])
        times = np.arange(n) * DT
        ax.plot(times, r['p_traj'][:, 2], color=PURSUER_COLORS_MAP[best_pursuer],
                lw=2, label='Pursuer z')
        ax.plot(times, r['e_traj'][:, 2], color=EVADER_COLORS_MAP[es],
                lw=2, linestyle='--', label='Evader z')
        ax.axhline(Z_MIN, color='gray', lw=0.8, linestyle=':', alpha=0.7)
        ax.axhline(Z_MAX, color='gray', lw=0.8, linestyle=':', alpha=0.7)
        if r['captured'] and r['capture_time']:
            ax.axvline(r['capture_time'], color='gold', lw=1.5, linestyle='--', label='Capture')
        ax.set_title(f'{best_pursuer}\nvs {es}', fontsize=9)
        ax.set_xlabel('Time (s)', fontsize=8)
        ax.set_ylabel('Altitude z (m)', fontsize=8)
        ax.set_ylim(Z_MIN - 0.5, Z_MAX + 0.5)
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3)

    fig.suptitle('S020 3D — Altitude Profiles: altitude_advantage Pursuer vs All Evaders', fontsize=11)
    plt.tight_layout()
    path = os.path.join(OUTPUT_DIR, 'altitude_profiles.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_capture_matrix(results):
    """Heatmap: 4 pursuers × 3 evaders, capture time or NaN for timeout."""
    capture_mat = np.full((len(PURSUER_STRATEGIES), len(EVADER_STRATEGIES)), np.nan)
    for i, ps in enumerate(PURSUER_STRATEGIES):
        for j, es in enumerate(EVADER_STRATEGIES):
            r = results[(ps, es)]
            if r['captured']:
                capture_mat[i, j] = r['capture_time']

    fig, ax = plt.subplots(figsize=(8, 5))
    # Mask NaN for display
    masked = np.ma.masked_invalid(capture_mat)
    cmap = plt.cm.RdYlGn_r
    cmap.set_bad('lightgray')
    im = ax.imshow(masked, aspect='auto', cmap=cmap, vmin=0, vmax=T_MAX)

    for i in range(len(PURSUER_STRATEGIES)):
        for j in range(len(EVADER_STRATEGIES)):
            val = capture_mat[i, j]
            text = f'{val:.1f}s' if not np.isnan(val) else 'timeout'
            ax.text(j, i, text, ha='center', va='center', fontsize=10, fontweight='bold',
                    color='white' if not np.isnan(val) and val < T_MAX * 0.6 else 'black')

    ax.set_xticks(range(len(EVADER_STRATEGIES)))
    ax.set_xticklabels(EVADER_STRATEGIES, rotation=15, ha='right', fontsize=9)
    ax.set_yticks(range(len(PURSUER_STRATEGIES)))
    ax.set_yticklabels(PURSUER_STRATEGIES, fontsize=9)
    ax.set_xlabel('Evader Strategy', fontsize=10)
    ax.set_ylabel('Pursuer Strategy', fontsize=10)
    ax.set_title('S020 3D — Capture Time Matrix (gray=timeout)', fontsize=11)
    plt.colorbar(im, ax=ax, label='Capture Time (s)')
    plt.tight_layout()
    path = os.path.join(OUTPUT_DIR, 'capture_matrix.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_policy_quiver_3d():
    """3D quiver: action directions for altitude_advantage policy at z=0,2,4 slices (evader at origin)."""
    fig = plt.figure(figsize=(12, 5))
    z_slices = [1.0, 3.0, 5.5]
    e_pos_fixed = np.array([0., 0., 3.])  # evader at origin-ish

    for idx, z_slice in enumerate(z_slices):
        ax = fig.add_subplot(1, 3, idx + 1, projection='3d')
        # Grid in xy at fixed z
        grid_range = np.linspace(-8, 8, 6)
        X, Y = np.meshgrid(grid_range, grid_range)
        U = np.zeros_like(X)
        V = np.zeros_like(Y)
        W = np.zeros_like(X)

        for i in range(X.shape[0]):
            for j in range(X.shape[1]):
                p_pos = np.array([X[i, j], Y[i, j], z_slice])
                p_vel = np.zeros(3)
                e_vel = np.zeros(3)
                # Skip if too close to evader
                if np.linalg.norm(p_pos - e_pos_fixed) < 1.0:
                    continue
                v = pursuer_altitude_advantage(p_pos, p_vel, e_pos_fixed, e_vel, 0.0)
                n = np.linalg.norm(v) + EPS
                U[i, j] = v[0] / n
                V[i, j] = v[1] / n
                W[i, j] = v[2] / n

        ax.quiver(X, Y, np.full_like(X, z_slice), U, V, W,
                  length=1.2, normalize=True, color='steelblue', alpha=0.7)
        # Mark evader
        ax.scatter(*e_pos_fixed, color='royalblue', s=150, marker='^', zorder=6, label='Evader')
        ax.set_xlim(-9, 9)
        ax.set_ylim(-9, 9)
        ax.set_zlim(0, 8)
        ax.set_title(f'altitude_advantage policy\nz_pursuer = {z_slice:.1f} m (evader at z=3)', fontsize=8)
        ax.set_xlabel('X (m)', fontsize=7)
        ax.set_ylabel('Y (m)', fontsize=7)
        ax.set_zlabel('Z (m)', fontsize=7)
        ax.tick_params(labelsize=6)
        ax.legend(fontsize=7)

    fig.suptitle('S020 3D — Policy Quiver: altitude_advantage at z slices', fontsize=11)
    plt.tight_layout()
    path = os.path.join(OUTPUT_DIR, 'policy_quiver_3d.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def plot_learning_curves_conceptual():
    """Synthetic learning curves — clearly labeled as Conceptual Illustration."""
    fig, axes = plt.subplots(1, 2, figsize=(13, 5))

    episodes = np.arange(1, 20001)
    rng = np.random.default_rng(7)

    # Phase 1: pursuer trains vs random (0-5000 eps): reward improves from ~-20 to ~+30
    # Phase 2: evader trains vs frozen pursuer (5000-10000): pursuer reward dips
    # Phase 3: both self-play (10000-20000): both improve
    def smooth(arr, w=200):
        kernel = np.ones(w) / w
        return np.convolve(arr, kernel, mode='same')

    # Pursuer reward
    p_raw = np.where(episodes <= 5000,
                     -20 + 50 * (episodes / 5000) + rng.normal(0, 5, len(episodes)),
                     np.where(episodes <= 10000,
                               30 - 15 * ((episodes - 5000) / 5000) + rng.normal(0, 5, len(episodes)),
                               15 + 30 * ((episodes - 10000) / 10000) + rng.normal(0, 6, len(episodes))))
    p_smooth = smooth(p_raw)

    # Evader reward (inverse of pursuer in phase 1; improves in phase 2)
    e_raw = np.where(episodes <= 5000,
                     10 - 20 * (episodes / 5000) + rng.normal(0, 4, len(episodes)),
                     np.where(episodes <= 10000,
                               -10 + 25 * ((episodes - 5000) / 5000) + rng.normal(0, 4, len(episodes)),
                               15 + 10 * ((episodes - 10000) / 10000) + rng.normal(0, 5, len(episodes))))
    e_smooth = smooth(e_raw)

    # Distance metric: average capture time
    dist_raw = np.where(episodes <= 5000,
                        T_MAX - (T_MAX - 15) * (episodes / 5000) + rng.normal(0, 1.5, len(episodes)),
                        np.where(episodes <= 10000,
                                  15 + 8 * ((episodes - 5000) / 5000) + rng.normal(0, 1.5, len(episodes)),
                                  23 - 10 * ((episodes - 10000) / 10000) + rng.normal(0, 1.5, len(episodes))))
    dist_smooth = smooth(dist_raw)

    # Plot 1: Rewards
    ax = axes[0]
    ax.fill_betweenx([-50, 100], 0, 5000, color='lightyellow', alpha=0.5, label='Phase 1 (pursuer)')
    ax.fill_betweenx([-50, 100], 5000, 10000, color='lightcyan', alpha=0.5, label='Phase 2 (evader)')
    ax.fill_betweenx([-50, 100], 10000, 20000, color='lightgreen', alpha=0.5, label='Phase 3 (self-play)')
    ax.plot(episodes, p_smooth, color='crimson', lw=2, label='Pursuer reward')
    ax.plot(episodes, e_smooth, color='royalblue', lw=2, label='Evader reward')
    ax.axvline(5000, color='gray', lw=1.5, linestyle='--')
    ax.axvline(10000, color='gray', lw=1.5, linestyle='--')
    ax.set_xlabel('Episode', fontsize=10)
    ax.set_ylabel('Episode Reward', fontsize=10)
    ax.set_title('Training Reward Curves\n[Conceptual Illustration — Not Real RL Training]', fontsize=9)
    ax.legend(fontsize=8)
    ax.set_xlim(0, 20000)
    ax.set_ylim(-50, 100)
    ax.grid(True, alpha=0.3)
    ax.text(2500, -45, 'Phase 1', ha='center', fontsize=8, color='darkgoldenrod')
    ax.text(7500, -45, 'Phase 2', ha='center', fontsize=8, color='steelblue')
    ax.text(15000, -45, 'Phase 3', ha='center', fontsize=8, color='seagreen')

    # Plot 2: Capture time
    ax = axes[1]
    ax.fill_betweenx([0, T_MAX + 2], 0, 5000, color='lightyellow', alpha=0.5)
    ax.fill_betweenx([0, T_MAX + 2], 5000, 10000, color='lightcyan', alpha=0.5)
    ax.fill_betweenx([0, T_MAX + 2], 10000, 20000, color='lightgreen', alpha=0.5)
    ax.plot(episodes, np.clip(dist_smooth, 0, T_MAX), color='darkorange', lw=2, label='Avg capture time')
    ax.axhline(T_MAX, color='black', lw=1.0, linestyle=':', alpha=0.7, label=f'Timeout ({T_MAX}s)')
    ax.axvline(5000, color='gray', lw=1.5, linestyle='--')
    ax.axvline(10000, color='gray', lw=1.5, linestyle='--')
    ax.set_xlabel('Episode', fontsize=10)
    ax.set_ylabel('Capture Time (s)', fontsize=10)
    ax.set_title('Average Capture Time vs Training\n[Conceptual Illustration — Not Real RL Training]', fontsize=9)
    ax.legend(fontsize=8)
    ax.set_xlim(0, 20000)
    ax.set_ylim(0, T_MAX + 2)
    ax.grid(True, alpha=0.3)

    # Big watermark
    for ax in axes:
        ax.text(0.5, 0.5, 'CONCEPTUAL\nILLUSTRATION', transform=ax.transAxes,
                fontsize=20, alpha=0.07, ha='center', va='center', rotation=30,
                fontweight='bold', color='gray')

    fig.suptitle('S020 3D — PPO Training Curriculum (Conceptual Illustration)\n'
                 'Phase 1: Pursuer vs Random | Phase 2: Evader vs Frozen Pursuer | Phase 3: Self-Play',
                 fontsize=10)
    plt.tight_layout()
    path = os.path.join(OUTPUT_DIR, 'learning_curves_conceptual.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {path}')


def save_animation(results):
    """Animate predictive_3d vs helix_evader."""
    r = results[('predictive_3d', 'helix_evader')]
    p_traj = r['p_traj']
    e_traj = r['e_traj']
    n_steps = len(p_traj)
    step_dec = max(1, n_steps // 80)

    fig = plt.figure(figsize=(8, 7))
    ax = fig.add_subplot(111, projection='3d')

    all_pts = np.concatenate([p_traj, e_traj], axis=0)
    margin = 1.5
    ax.set_xlim(all_pts[:, 0].min() - margin, all_pts[:, 0].max() + margin)
    ax.set_ylim(all_pts[:, 1].min() - margin, all_pts[:, 1].max() + margin)
    ax.set_zlim(Z_MIN, Z_MAX)
    ax.set_xlabel('X (m)', fontsize=7)
    ax.set_ylabel('Y (m)', fontsize=7)
    ax.set_zlabel('Z (m)', fontsize=7)

    p_line, = ax.plot([], [], [], color='crimson', lw=2, label='Pursuer (predictive_3d)')
    e_line, = ax.plot([], [], [], color='royalblue', lw=2, linestyle='--', label='Evader (helix)')
    p_dot,  = ax.plot([], [], [], 'D', color='crimson', ms=8)
    e_dot,  = ax.plot([], [], [], '^', color='royalblue', ms=8)
    time_text = ax.text2D(0.02, 0.95, '', transform=ax.transAxes, fontsize=9)
    ax.legend(loc='upper right', fontsize=8)

    sub_p = p_traj[::step_dec]
    sub_e = e_traj[::step_dec]
    n_frames = len(sub_p)

    def init():
        for art in [p_line, e_line, p_dot, e_dot]:
            art.set_data([], [])
            art.set_3d_properties([])
        time_text.set_text('')
        return p_line, e_line, p_dot, e_dot, time_text

    def update(frame):
        fp = min(frame, n_frames - 1)
        t_cur = fp * step_dec * DT
        p_line.set_data(sub_p[:fp+1, 0], sub_p[:fp+1, 1])
        p_line.set_3d_properties(sub_p[:fp+1, 2])
        e_line.set_data(sub_e[:fp+1, 0], sub_e[:fp+1, 1])
        e_line.set_3d_properties(sub_e[:fp+1, 2])
        p_dot.set_data([sub_p[fp, 0]], [sub_p[fp, 1]])
        p_dot.set_3d_properties([sub_p[fp, 2]])
        e_dot.set_data([sub_e[fp, 0]], [sub_e[fp, 1]])
        e_dot.set_3d_properties([sub_e[fp, 2]])
        captured_str = ' [CAPTURED]' if r['captured'] and t_cur >= (r['capture_time'] or T_MAX) else ''
        time_text.set_text(f't = {t_cur:.1f}s{captured_str}')
        ax.set_title(f'S020 3D predictive_3d vs helix_evader (t={t_cur:.1f}s)', fontsize=9)
        return p_line, e_line, p_dot, e_dot, time_text

    ani = FuncAnimation(fig, update, frames=n_frames, init_func=init, blit=False, interval=80)
    plt.tight_layout()
    path = os.path.join(OUTPUT_DIR, 'animation.gif')
    ani.save(path, writer=PillowWriter(fps=12), dpi=100)
    plt.close()
    print(f'Saved: {path}')


# ── Main ─────────────────────────────────
if __name__ == '__main__':
    print('Running S020 3D Pursuit-Evasion Game simulations...')
    print('Strategy combinations (4 pursuers × 3 evaders = 12 total):')
    results = run_all()

    print('\n' + '='*65)
    print('S020 3D Pursuit-Evasion — Capture Matrix')
    print('='*65)
    header = f'{"Pursuer":22s}' + ''.join(f'{es:18s}' for es in EVADER_STRATEGIES)
    print(header)
    print('-'*65)
    for ps in PURSUER_STRATEGIES:
        row = f'{ps:22s}'
        for es in EVADER_STRATEGIES:
            r = results[(ps, es)]
            ct = f'{r["capture_time"]:.2f}s' if r['captured'] else 'timeout'
            row += f'{ct:18s}'
        print(row)
    print('='*65)

    print('\nGenerating plots...')
    plot_trajectories_3d(results)
    plot_altitude_profiles(results)
    plot_capture_matrix(results)
    plot_policy_quiver_3d()
    plot_learning_curves_conceptual()
    save_animation(results)
    print('\nAll outputs saved to:', OUTPUT_DIR)
