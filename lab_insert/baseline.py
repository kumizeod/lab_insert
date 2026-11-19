"""Aligned insertion oracle for workcell validation; uses known fixture depth."""
import argparse
import numpy as np
import mujoco
from .labware_family import FAMILIES, ROUND_TUBES
from .workcell import Workcell, GAP


def aligned_trial(env):
    m, d = env.model, env.data
    site = m.site("grasp").id
    origin = d.site_xpos[site].copy()
    jac = np.zeros((6, m.nv))
    depth = env.params["depth"] + GAP
    started_held = env.seat_errors()[1] > depth - .001
    for k in range(max(200, int(np.ceil(depth/.00025))+80)):
        desired = origin.copy()
        desired[2] -= min((k+1)*.00025, depth)
        mujoco.mj_jacSite(m, d, jac[:3], jac[3:], site)
        J = jac[:, :7]
        error = np.r_[desired - d.site_xpos[site], np.zeros(3)]
        env.step(J.T @ np.linalg.solve(J @ J.T + .0001*np.eye(6), error))
    before_release = env.seated()
    env.closed = False
    for _ in range(15):
        env.step(np.zeros(7))
    for _ in range(55):
        mujoco.mj_jacSite(m,d,jac[:3],jac[3:],site)
        env.step(np.linalg.pinv(jac[:,:7]) @ [0,0,.0007,0,0,0])
    env._hold(d.qpos[:7].copy(),100)
    return started_held, before_release, env.seated()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--family", choices=FAMILIES, default="cuvette")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--sku", choices=tuple(ROUND_TUBES))
    parser.add_argument("--cap-state", choices=("open","closed"))
    args = parser.parse_args()
    env = Workcell(args.family, args.seed, sku=args.sku, cap_state=args.cap_state)
    checks = aligned_trial(env)
    xy,z,tilt = env.seat_errors()
    print(f"held / seated / released and seated = {checks}; "
          f"xy={1000*xy:.3f} mm, z={1000*z:.3f} mm, tilt={np.rad2deg(tilt):.2f} deg")
    return 0 if all(checks) else 1


if __name__ == "__main__":
    raise SystemExit(main())

