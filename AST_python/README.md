# AST Python

Python/OpenSim translation of the AST scaling workflow. The main entry point is `Main_AST_v1.py`.

## Requirements

- Windows with OpenSim Python available.
- Recommended environment: `opensim45_py`.
- Python packages used by the scripts: `numpy` and `opensim`.

Example:

```powershell
conda activate opensim45_py
python Main_AST_v1.py path\to\config.json
```

## Main workflow

`Main_AST_v1.py` runs an iterative static-trial scaling workflow:

1. Loads an unscaled OpenSim model, static `.trc` file, and OpenSim scaling setup XML.
2. Reads subject and generic model height/weight from the config.
3. Creates initial scale and static IK setup files.
4. Runs OpenSim `ScaleTool` and `InverseKinematicsTool`.
5. Compares marker locations from the static TRC against model marker locations.
6. Iteratively adjusts scale factors for bodies with marker errors above the configured threshold.
7. Runs a final marker placement step.
8. Writes an adjusted scaled model and unlocks its coordinates.

The script writes timestamped output files beside the input model, including setup XML files, marker sets, static-pose motions, error arrays, and the final adjusted `.osim`.

## Configuration

The script has built-in defaults, but normally you should pass a JSON config file as the first argument:

```powershell
python Main_AST_v1.py Main_AST_v1_Tom_config.json
```

Typical config fields:

```json
{
  "base_model_path": "C:\\path\\to\\model.osim",
  "trc_path": "C:\\path\\to\\static_trial.trc",
  "setup_path": "C:\\path\\to\\setup_markers_scale.xml",
  "subject_height": 182.88,
  "subject_weight": 75.0,
  "generic_model_height": 170.0,
  "generic_model_weight": 75.0,
  "pose": 0,
  "km": 60,
  "end_err": 0.004,
  "manual_scale_err": 0.025,
  "rep": 4,
  "rep2": 8
}
```

Important fields:

- `base_model_path`: unscaled or starting `.osim` model.
- `trc_path`: static calibration trial in TRC format.
- `setup_path`: OpenSim scale setup XML.
- `pose`: set to `1` to estimate an average static pose before iterative scaling; set to `0` to skip that step.
- `km`: maximum number of AST iterations.
- `end_err`: target RMS marker error threshold.
- `manual_scale_err`: marker error threshold for assigning manual body-specific scale adjustments.
- `rep` and `rep2`: repetition controls used by the iterative scale adjustment logic.

## Included helper files

- `inputAST.py`: interactive-style input helper translated from MATLAB.
- `load_trc.py`, `load_sto.py`, `load_mot.py`: file readers for OpenSim/TRC tables.
- `UnlockModel.py`: unlocks coordinates in an OpenSim model.
- `setup_markers_scale.xml`: example scale setup XML.
- `ModelScaledMarkerAdj.osim`: example adjusted/scaled model output.
- `Main_AST_v1.m`: original MATLAB reference version.

## Notes

- Paths in JSON configs should be updated for the machine/session being processed.
- The script writes outputs to the folder containing `base_model_path`.
- Run from an OpenSim-capable Python environment; a standard Python install without the `opensim` package will fail.
