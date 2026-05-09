from pathlib import Path
import numpy as np


def load_sto(fname):
    """
    Load an OpenSim .sto file.

    Returns
    -------
    data : np.ndarray
        Numeric data, shape (n_rows, n_cols)
    headers : list[str]
        Column names
    """
    fname = Path(fname)

    with fname.open("r", encoding="utf-8", errors="ignore") as f:
        lines = f.readlines()

    # Find endheader
    endheader_idx = None
    for i, line in enumerate(lines):
        if line.strip().lower() == "endheader":
            endheader_idx = i
            break

    if endheader_idx is None:
        raise ValueError(f"'endheader' not found in STO file: {fname}")

    # Next line after endheader is column names
    if endheader_idx + 1 >= len(lines):
        raise ValueError(f"No header row found after 'endheader' in: {fname}")

    headers = lines[endheader_idx + 1].strip().split()

    # Remaining lines are numeric data
    data_rows = []
    for line in lines[endheader_idx + 2:]:
        stripped = line.strip()
        if not stripped:
            continue
        values = [float(x) for x in stripped.split()]
        data_rows.append(values)

    data = np.array(data_rows, dtype=float)

    return data, headers