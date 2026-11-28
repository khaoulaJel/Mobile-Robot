"""Turns the logbook of robot_test.py into report.html: a comparison table per session, what looks wrong, and one plot per run.
Everything in it comes from logbook.csv and runs/runNNN_trace.csv; it can be rebuilt at any time (robot_test.py report)."""
import base64
import csv
import html
import io
import os

import robot_test as rt


LIMIT = {'turn': 1.0, 'drive': 2.0}       # deg: under 1 deg is the MINIMUM requirement for headings; cm for drives


def pi2_results(rows):
    """Every turn to the heading 90 deg that finished: the errors, per session, controller and settings."""
    groups = {}
    for r in rows:
        if r['test'] == 'heading' and number(r['asked']) == 90.0 and r.get('note') != 'setup' and rt.completed(r['status']):
            key = (r.get('session') or 'single runs', r['controller'], r.get('profile', ''), r['changed'], r['frame'])
            groups.setdefault(key, []).append(r)
    out = []
    for (session, controller, profile, changed, frame), items in groups.items():
        errors = [number(r['error']) for r in items]
        out.append({'session': session, 'controller': controller, 'settings': ' '.join(x for x in (profile, changed) if x) or 'defaults',
                    'frame': frame, 'runs': len(errors), 'errors': ', '.join(f'{e:+.2f}' for e in errors),
                    'mean': mean([abs(e) for e in errors]), 'worst': max(abs(e) for e in errors),
                    'under1': sum(abs(e) < 1.0 for e in errors),
                    'time': mean([number(r['time_s']) for r in items if number(r['time_s']) is not None])})
    return out


def number(text):
    try:
        return float(text)
    except (TypeError, ValueError):
        return None


def mean(values):
    return sum(values) / len(values) if values else None


def summarize(rows):
    """-> list of dicts, one per (session, test kind, controller, changed coefficients). Aborted runs are counted, not averaged."""
    groups = {}
    for r in rows:
        kind = 'drive' if r['test'] == 'drive' else 'turn'
        groups.setdefault((r.get('session') or 'single runs', kind, r['controller'], r['changed']), []).append(r)
    out = []
    for (session, kind, controller, changed), items in groups.items():
        good = [r for r in items if rt.completed(r['status'])]
        errors = [abs(number(r['error'])) for r in good if number(r['error']) is not None]
        within = LIMIT[kind]
        times = [number(r['time_s']) for r in good if number(r['time_s']) is not None]
        overs = [number(r['overshoot']) for r in good if number(r['overshoot']) is not None]
        out.append({'session': session, 'test': kind, 'unit': 'deg' if kind == 'turn' else 'cm', 'controller': controller,
                    'changed': changed, 'runs': len(items), 'aborted': len(items) - len(good),
                    'stuck': sum(r['status'] == 'STUCK' for r in items), 'timeout': sum(r['status'] == 'TIMEOUT' for r in items),
                    'mean_error': mean(errors), 'worst_error': max(errors, default=None),
                    'within': sum(e <= within for e in errors), 'judged': len(errors), 'mean_time': mean(times),
                    'mean_overshoot': mean(overs)})
    return out


def diagnose(row):
    """One plain sentence on what went wrong in a run, or '' if nothing stands out. It describes; it does not prove a cause."""
    error, over = number(row['error']), number(row['overshoot'])
    if not rt.completed(row['status']):
        return row['status']
    if error is None:
        return ''
    limit = LIMIT['drive' if row['test'] == 'drive' else 'turn']
    asked = number(row['asked'])
    direction = 1.0 if (asked or 0) >= 0 else -1.0
    unit = 'cm' if row['test'] == 'drive' else 'deg'
    if row['status'] == 'STUCK':
        return f'stopped {abs(error):.1f} {unit} away while still commanding: command probably below what moves the motors'
    if row['status'] == 'TIMEOUT':
        return f'not finished in time, {abs(error):.1f} {unit} away'
    if abs(error) > limit:
        short = error * direction > 0 if row['test'] != 'heading' else None
        side = 'ended short of the target' if short else ('ended past the target' if short is False else 'ended away from the target')
        return f'{side} by {abs(error):.1f} {unit}'
    if over is not None and over > 2 * limit:
        return f'arrived, but first overshot by {over:.1f} {unit}'
    if row['test'] == 'drive' and abs(number(row['sideways_cm']) or 0) > 3.0:
        return f'ended {abs(number(row["sideways_cm"])):.1f} cm to the side'
    return ''


def plot(folder, row):
    """-> PNG bytes of error and command against time, or None."""
    path = os.path.join(folder, 'runs', f'run{int(row["run"]):03d}_trace.csv')
    if not os.path.exists(path):
        return None
    with open(path, newline='') as f:
        data = list(csv.DictReader(f))
    if len(data) < 2:
        return None
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    t = [float(d['t']) for d in data]
    drive = row['test'] == 'drive'
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(7.2, 3.6), sharex=True)
    a1.plot(t, [float(d['error']) for d in data])
    a1.axhline(0, color='black', linewidth=0.6)
    band = LIMIT['drive' if drive else 'turn']
    a1.axhspan(-band, band, alpha=0.12, color='green')
    a1.set_ylabel('remaining ' + ('(cm)' if drive else '(deg)'))
    a1.set_title(f'run {row["run"]}: {row["controller"]}, {row["test"]} {row["asked"]} {row["unit"]} -> {row["status"].split(":")[0]}',
                 fontsize=9)
    a2.plot(t, [float(d['cmd_v' if drive else 'cmd_w']) for d in data], label='command')
    a2.plot(t, [float(d['v' if drive else 'w']) for d in data], label='measured')
    a2.set_ylabel('m/s' if drive else 'rad/s')
    a2.set_xlabel('time (s)')
    a2.legend(fontsize=7)
    for a in (a1, a2):
        a.grid(alpha=0.3)
        a.tick_params(labelsize=7)
    fig.tight_layout()
    buffer = io.BytesIO()
    fig.savefig(buffer, format='png', dpi=110)
    plt.close(fig)
    return buffer.getvalue()


COLORS = {'stop_turn': '#1f77b4', 'polar': '#ff7f0e', 'pure_pursuit': '#2ca02c', 'p_servo': '#d62728', 'symbolic': '#9467bd'}


def path_trace(folder, row):
    path = os.path.join(folder, 'runs', f'path{int(row["run"]):03d}_trace.csv')
    if not os.path.exists(path):
        return []
    with open(path, newline='') as f:
        return list(csv.DictReader(f))


def figure_bytes(fig):
    import matplotlib.pyplot as plt
    buffer = io.BytesIO()
    fig.savefig(buffer, format='png', dpi=110)
    plt.close(fig)
    return buffer.getvalue()


def path_plot(folder, row):
    """Left: the floor seen from above (ideal, odometry, LiDAR map). Right: distance from the path against time."""
    data = path_trace(folder, row)
    if len(data) < 2:
        return None
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import paths
    ideal = paths.dense(paths.shape(row['shape'])[1])
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(9.6, 4.0), gridspec_kw={'width_ratios': [1.1, 1]})
    a1.plot([q[0] for q in ideal], [q[1] for q in ideal], 'k--', linewidth=1, label='ideal path')
    a1.plot([float(d['x']) for d in data], [float(d['y']) for d in data], color=COLORS.get(row['controller']), label='wheel odometry')
    mapped = [d for d in data if d['map_x']]
    if mapped:
        a1.plot([float(d['map_x']) for d in mapped], [float(d['map_y']) for d in mapped], color='gray', linewidth=1, label='LiDAR map')
    a1.plot(0, 0, 'ko', markersize=5)
    a1.set_aspect('equal', adjustable='datalim')
    a1.set_xlabel('ahead of start (m)')
    a1.set_ylabel('left of start (m)')
    a1.legend(fontsize=7)
    a2.plot([float(d['t']) for d in data], [float(d['off_path_cm']) for d in data], color=COLORS.get(row['controller']))
    a2.set_xlabel('time (s)')
    a2.set_ylabel('distance from path (cm), position used')
    for a in (a1, a2):
        a.grid(alpha=0.3)
        a.tick_params(labelsize=7)
    fig.suptitle(f'path run {row["run"]}: {row["shape"]} / {row["controller"]} / steering by {row["state"]} -> '
                 f'{row["status"].split(":")[0]}   rms {row["rms_cm"]} cm, max {row["max_cm"]} cm', fontsize=9)
    fig.tight_layout()
    return figure_bytes(fig)


def overlay(folder, rows, shape, state):
    """The newest completed run of every controller on one shape, in one picture."""
    newest = {}
    for r in rows:
        if r['shape'] == shape and r['state'] == state and r['status'] == 'COMPLETE':
            newest[r['controller']] = r
    if len(newest) < 2:
        return None
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import paths
    ideal = paths.dense(paths.shape(shape)[1])
    fig, a = plt.subplots(figsize=(5.6, 5.0))
    a.plot([q[0] for q in ideal], [q[1] for q in ideal], 'k--', linewidth=1.2, label='ideal')
    for name, r in sorted(newest.items()):
        data = path_trace(folder, r)
        a.plot([float(d['x']) for d in data], [float(d['y']) for d in data], color=COLORS.get(name), linewidth=1.1,
               label=f'{name} (rms {r["rms_cm"]} cm)')
    a.set_aspect('equal', adjustable='datalim')
    a.set_title(f'{shape}: all controllers, steering by {state} (wheel-odometry position)', fontsize=9)
    a.grid(alpha=0.3)
    a.tick_params(labelsize=7)
    a.legend(fontsize=7)
    fig.tight_layout()
    return figure_bytes(fig)


def path_sections(folder, rows):
    """-> (html pieces, text lines for the terminal) for the path runs."""
    def img(png):
        return f'<img src="data:image/png;base64,{base64.b64encode(png).decode()}" alt="path plot">'
    parts = ['<h1>Paths</h1><p>Distances are from the IDEAL path (cutting a corner counts). rms/max/end = by the position the '
             'controller steered by. "map" = by the LiDAR map position; when the controller steered by odometry it is an '
             'independent check (the map has its own noise of about a centimetre).</p>']
    keys = ('run', 'session', 'shape', 'controller', 'state', 'status', 'rms_cm', 'max_cm', 'end_cm', 'end_deg', 'time_s',
            'map_rms_cm', 'map_max_cm', 'map_end_cm', 'estimates_apart_cm', 'fallbacks', 'note')
    parts.append(table(['run', 'session', 'shape', 'controller', 'steering by', 'status', 'rms cm', 'max cm', 'end cm', 'end deg',
                        'time s', 'map rms cm', 'map max cm', 'map end cm', 'odom-map apart cm', 'fallbacks', 'note'],
                       [[r.get(k, '') for k in keys] for r in reversed(rows)]))
    parts.append('<h2>Controllers compared on each shape (newest completed run of each)</h2>')
    for shape, state in sorted({(r['shape'], r['state']) for r in rows}):
        png = overlay(folder, rows, shape, state)
        if png:
            parts.append(img(png))
    parts.append('<h2>Every path run (newest first)</h2>')
    for r in reversed(rows):
        png = path_plot(folder, r)
        if png:
            parts.append(img(png))
    newest = rows[-1]['session'] or ''
    lines = [f'\nPATHS of session {newest or "(single runs)"}',
             f'{"shape":11} {"controller":13} {"by":6} {"status":9} {"rms cm":>7} {"max cm":>7} {"end cm":>7} {"end deg":>8} '
             f'{"time s":>7} {"map rms":>8}']
    for r in rows:
        if (r['session'] or '') == newest:
            lines.append(f'{r["shape"]:11} {r["controller"]:13} {r["state"]:6} {r["status"].split(":")[0]:9} {r["rms_cm"]:>7} '
                         f'{r["max_cm"]:>7} {r["end_cm"]:>7} {r["end_deg"]:>8} {r["time_s"]:>7} {r["map_rms_cm"] or "-":>8}')
    return parts, lines


def table(header, lines):
    head = ''.join(f'<th>{html.escape(h)}</th>' for h in header)
    body = ''.join('<tr>' + ''.join(f'<td>{html.escape(str(c))}</td>' for c in line) + '</tr>' for line in lines)
    return f'<table><tr>{head}</tr>{body}</table>'


def fmt(value, digits=2):
    return '-' if value is None else f'{value:.{digits}f}'


def make_report(folder):
    import robot_path as rp
    rows = rt.read_book(os.path.join(folder, 'logbook.csv'))
    path_rows = rp.read_rows(folder)
    if not rows and not path_rows:
        print('the logbook is empty: no report')
        return None
    summary = summarize(rows)
    parts = ['<meta charset="utf-8"><title>Robot test report</title><style>body{font-family:Segoe UI,Arial,sans-serif;margin:24px;'
             'max-width:1100px}table{border-collapse:collapse;margin:8px 0 20px;font-size:13px}td,th{border:1px solid #ccc;'
             'padding:4px 8px;text-align:left}th{background:#f0f0f0}img{max-width:100%;border:1px solid #ddd;margin:6px 0}</style>',
             '<h1>Robot test report</h1>',
             '<p><b>How to read it.</b> Errors come from the robot\'s own sensors (wheel odometry, or the map pose when the frame is '
             '"map"). With wheel odometry alone they show how well the controller reaches what the robot <i>measures</i>; they do '
             'not show wheel slip or the true distance on the floor. Turns are in degrees, drives in centimetres. '
             '"Within limit" counts final errors of at most 1 deg (the minimum required) or 2 cm. Aborted runs are counted but not '
             'averaged.</p>']
    target = pi2_results(rows)
    if target:
        parts.append('<h2>Heading pi/2 (90 deg)</h2><p>Final error of every turn to 90 deg. Required: every error under 1 deg.</p>')
        parts.append(table(['session', 'controller', 'settings', 'frame', 'runs', 'each error (deg)', 'mean |error|', 'worst', 'under 1 deg',
                            'verdict', 'mean time (s)'],
                           [[t['session'], t['controller'], t['settings'], t['frame'], t['runs'], t['errors'], fmt(t['mean']), fmt(t['worst']),
                             f'{t["under1"]}/{t["runs"]}', 'PASS' if t['under1'] == t['runs'] else 'FAIL', fmt(t['time'], 1)]
                            for t in reversed(target)]))
    for session in sorted({s['session'] for s in summary}, reverse=True):
        parts.append(f'<h2>Session {html.escape(session)}</h2>')
        parts.append(table(['test', 'controller', 'changed coefficients', 'runs', 'mean error', 'worst error', 'unit', 'within limit',
                            'mean time (s)', 'mean overshoot', 'stuck', 'timeout', 'aborted'],
                           [[s['test'], s['controller'], s['changed'] or 'defaults', s['runs'], fmt(s['mean_error']),
                             fmt(s['worst_error']), s['unit'], f'{s["within"]}/{s["judged"]}', fmt(s['mean_time'], 1),
                             fmt(s['mean_overshoot']), s['stuck'], s['timeout'], s['aborted']]
                            for s in sorted(summary, key=lambda x: (x['test'], x['controller'])) if s['session'] == session]))
    problems = [[r['run'], r['controller'], f'{r["test"]} {r["asked"]} {r["unit"]}', diagnose(r)] for r in rows if diagnose(r)]
    parts.append('<h2>Where the errors are</h2>')
    parts.append(table(['run', 'controller', 'test', 'what happened'], problems) if problems else '<p>No run stands out.</p>')
    parts.append('<h2>All runs</h2>')
    cols = ['run', 'session', 'controller', 'test', 'asked', 'unit', 'topic', 'frame', 'status', 'error', 'overshoot',
            'sideways_cm', 'time_s', 'hand_value', 'hand_error', 'changed', 'profile', 'controller_version', 'learned_u_min', 'learned_lead', 'learned_gain', 'note']
    parts.append(table(cols, [[r.get(c, '') for c in cols] for r in rows]))
    parts.append('<h2>Plots (newest first)</h2><p>Top: what is left to the target (the green band is within the limit). '
                 'Bottom: command sent and speed measured.</p>')
    for r in reversed(rows):
        png = plot(folder, r)
        if png:
            parts.append(f'<img src="data:image/png;base64,{base64.b64encode(png).decode()}" alt="run {r["run"]}">')
    path_lines = []
    if path_rows:
        extra, path_lines = path_sections(folder, path_rows)
        parts[3:3] = extra                                  # after the title and the reading note, before turns and drives
    path = os.path.join(folder, 'report.html')
    with open(path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(parts))
    if path_lines:
        print('\n'.join(path_lines))
    if not rows:
        print(f'report: {path}')
        return path
    newest = rows[-1].get('session') or 'single runs'
    print(f'\nRESULTS of {newest} (error: deg for turns, cm for drives)')
    print(f'{"test":6} {"controller":13} {"runs":>4} {"mean err":>9} {"worst":>7} {"in limit":>9} {"time s":>7}  changed')
    for s in sorted(summary, key=lambda x: (x['test'], x['controller'])):
        if s['session'] == newest:
            print(f'{s["test"]:6} {s["controller"]:13} {s["runs"]:>4} {fmt(s["mean_error"]):>9} {fmt(s["worst_error"]):>7} '
                  f'{str(s["within"]) + "/" + str(s["judged"]):>9} {fmt(s["mean_time"], 1):>7}  {s["changed"] or "defaults"}')
    for t in pi2_results(rows):
        if t['session'] == newest:
            print(f'PI/2  {t["controller"]}: {t["under1"]}/{t["runs"]} under 1 deg, mean {t["mean"]:.2f}, worst {t["worst"]:.2f} deg'
                  f'  [{t["errors"]}]  {"PASS" if t["under1"] == t["runs"] else "FAIL"}')
    print(f'report: {path}')
    return path
