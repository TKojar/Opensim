from pathlib import Path
import numpy as np


def load_trc(fname):
    """
    Load an OpenSim/Vicon-style .trc file.

    Returns
    -------
    data : np.ndarray
        Numeric table of shape (n_frames, n_cols)
    headers : list[str]
        Base header row, e.g. ['Frame#', 'Time', 'marker1', 'marker2', ...]
    headers_xyz : list[str]
        Expanded headers with XYZ suffixes, e.g.
        ['Frame#', 'Time', 'marker1_X', 'marker1_Y', 'marker1_Z', ...]
    """
    fname = Path(fname)

    with fname.open("r", encoding="utf-8", errors="ignore") as f:
        lines = f.readlines()

    if len(lines) < 5:
        raise ValueError(f"TRC file too short: {fname}")

    # MATLAB comments:
    # line 1: PathFileType...
    # line 2: DataRate CameraRate ...
    # line 3: numeric metadata
    # line 4: marker header names
    # line 5: X1 Y1 Z1 ...
    header_line = lines[3].rstrip("\n")
    headers = header_line.split()

    headers_xyz = list(headers)
    j = 2  # Python index for MATLAB j=3

    for i in range(2, len(headers)):  # MATLAB i=3:length(headers)
        marker = headers[i]
        if j < len(headers_xyz):
            headers_xyz[j] = f"{marker}_X"
        else:
            headers_xyz.append(f"{marker}_X")

        headers_xyz.append(f"{marker}_Y")
        headers_xyz.append(f"{marker}_Z")
        j += 3

    data_rows = []
    for line in lines[5:]:
        s = line.strip()
        if not s:
            continue
        row = [float(x) for x in s.split()]
        data_rows.append(row)

    data = np.array(data_rows, dtype=float)

    return data, headers, headers_xyz