#!/usr/bin/env python3
"""M4: recover a 3-D wire line from two independent 2-D image lines."""
import argparse, json, math
from pathlib import Path
import cv2
import numpy as np


def normalize(v):
    n = np.linalg.norm(v)
    if n < 1e-12:
        raise ValueError('degenerate vector')
    return v / n


def load_obs(root, name, results):
    d = results[name]
    if not d.get('annotation_independent', False):
        raise RuntimeError(f'{name} is not an independent M3 result')
    cam = json.loads((root / name / 'camerainfo.json').read_text())
    tf = json.loads((root / name / 'tf_samples.json').read_text())[7]
    K = np.asarray(cam['k'], dtype=float).reshape(3, 3)
    D = np.asarray(cam['d'], dtype=float)
    x1, y1, x2, y2 = d['visible_segment_px']
    pts = np.asarray([[[x1, y1]], [[x2, y2]]], dtype=np.float64)
    und = cv2.undistortPoints(pts, K, D).reshape(2, 2)
    r1 = np.array([und[0, 0], und[0, 1], 1.0])
    r2 = np.array([und[1, 0], und[1, 1], 1.0])
    l = normalize(np.cross(r1, r2))
    T = np.asarray(tf['base_T_camera'], dtype=float)
    R, t = T[:3, :3], T[:3, 3]
    n = normalize(R @ l)
    plane_d = -float(n @ t)
    return {'name': name, 'K': K, 'D': D, 'T_BC': T, 'pixels': np.array([[x1,y1],[x2,y2]], float),
            'undistorted': und, 'line_C': l, 'n_B': n, 'd_B': plane_d}


def project_line(obs, P0, v):
    T = obs['T_BC']; T_CB = np.linalg.inv(T)
    # A long finite portion is sufficient for an image overlay. Keep only points in front.
    ts = np.linspace(-2.0, 2.0, 801)
    P = P0[None, :] + ts[:, None] * v[None, :]
    Pc = (T_CB[:3, :3] @ P.T + T_CB[:3, 3:4]).T
    good = Pc[:, 2] > 0.03
    if good.sum() < 2:
        raise RuntimeError('recovered line is not visible in camera')
    q = Pc[good]
    pix = (obs['K'] @ (q / q[:, 2:3]).T).T[:, :2]
    # Choose the widest in-frame-looking projected span.
    i, j = 0, len(pix)-1
    return pix[i], pix[j]


def line_distance(p, a, b):
    e = b-a
    return abs(float(np.cross(e, p-a))) / max(np.linalg.norm(e), 1e-12)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--session', type=Path, required=True)
    ap.add_argument('--out', type=Path, required=True)
    ap.add_argument('--bad-angle-deg', type=float, default=5.0)
    args = ap.parse_args()
    root = args.session
    args.out.mkdir(parents=True, exist_ok=True)
    results = json.loads((root / 'm3_wire_results_independent.json').read_text())
    A, B = load_obs(root, 'wire_a', results), load_obs(root, 'wire_b', results)
    cross = np.cross(A['n_B'], B['n_B'])
    cross_norm = np.linalg.norm(cross)
    angle = math.degrees(math.acos(float(np.clip(abs(A['n_B'] @ B['n_B']), -1, 1))))
    if cross_norm < 1e-8:
        raise RuntimeError('degenerate observation planes')
    v = normalize(cross)
    # Closest-to-origin point on the common line: solve plane A, plane B, v^T P=0.
    M = np.vstack([A['n_B'], B['n_B'], v])
    rhs = -np.array([A['d_B'], B['d_B'], 0.0])
    P0 = np.linalg.lstsq(M, rhs, rcond=None)[0]
    # Unsigned convention: deterministic sign only; no semantic outside/entry decision in M4.
    if v[np.argmax(np.abs(v))] < 0: v = -v

    overlays = {}
    reproj = {}
    for obs in (A, B):
        pa, pb = project_line(obs, P0, v)
        measured = obs['pixels']
        # The recovered 3-D line should project onto the measured 2-D line.
        rms = math.sqrt(np.mean([line_distance(p, pa, pb)**2 for p in measured]))
        reproj[obs['name']] = {'projected_segment_px': [*pa.tolist(), *pb.tolist()], 'rms_px': rms}
        img = cv2.imread(str(root / obs['name'] / 'rgb.png'), cv2.IMREAD_COLOR)
        if img is None: raise RuntimeError(f'missing RGB for {obs["name"]}')
        cv2.line(img, tuple(np.round(pa).astype(int)), tuple(np.round(pb).astype(int)), (0,255,0), 2, cv2.LINE_AA)
        cv2.line(img, tuple(np.round(measured[0]).astype(int)), tuple(np.round(measured[1]).astype(int)), (255,0,0), 1, cv2.LINE_AA)
        cv2.putText(img, f'M4 reproj RMS {rms:.2f}px', (8,22), cv2.FONT_HERSHEY_SIMPLEX, .55, (0,255,0), 2)
        out_img = args.out / f'{obs["name"]}_reprojection.png'
        cv2.imwrite(str(out_img), img); overlays[obs['name']] = str(out_img)

    status = 'BAD_VIEW_GEOMETRY' if angle < args.bad_angle_deg else 'WAITING_CP2'
    output = {
        'algorithm': 'undistortPoints -> normalized homogeneous image line -> backprojection planes -> SVD plane intersection',
        'input_result': str(root / 'm3_wire_results_independent.json'),
        'tf_sample_index': 7, 'plane_angle_deg': angle,
        'P0_m': P0.tolist(), 'wire_direction_unsigned': v.tolist(),
        'planes': {'A_normal_base': A['n_B'].tolist(), 'A_d': A['d_B'], 'B_normal_base': B['n_B'].tolist(), 'B_d': B['d_B']},
        'reprojection': reproj, 'overlay_images': overlays, 'status': status,
        'rviz_marker_topic': '/m4/wire_3d'
    }
    (args.out / 'm4_wire_3d.json').write_text(json.dumps(output, indent=2), encoding='utf-8')
    report = f'''===== M4 3D WIRE =====\n\nplane_angle_deg = {angle:.6f}\nP0 = {P0.tolist()}\nwire_direction_unsigned = {v.tolist()}\nreprojection_A_px = {reproj["wire_a"]["rms_px"]:.6f}\nreprojection_B_px = {reproj["wire_b"]["rms_px"]:.6f}\nRViz Marker topic = /m4/wire_3d\nalgorithm = undistortPoints + normalized line + backprojection planes + SVD intersection\ndebug = {args.out}\nstatus = {status}\n'''
    (args.out / 'stage_report.txt').write_text(report, encoding='utf-8')
    print(report)

if __name__ == '__main__': main()
