from datetime import datetime
from pathlib import Path
import json
import re
import sys
import time
import numpy as np
import opensim as osim
import os


# ============================================================
# IO HELPERS
# ============================================================

def load_sto(fname):
    fname = Path(fname)
    with fname.open("r", encoding="utf-8", errors="ignore") as f:
        lines = f.readlines()

    endheader_idx = None
    for i, line in enumerate(lines):
        if line.strip().lower() == "endheader":
            endheader_idx = i
            break

    if endheader_idx is None:
        raise ValueError(f"'endheader' not found in STO file: {fname}")

    headers = lines[endheader_idx + 1].strip().split()

    data_rows = []
    for line in lines[endheader_idx + 2:]:
        s = line.strip()
        if not s:
            continue
        data_rows.append([float(x) for x in s.split()])

    data = np.array(data_rows, dtype=float)
    return data, headers


def load_mot(fname):
    fname = Path(fname)
    with fname.open("r", encoding="utf-8", errors="ignore") as f:
        lines = f.readlines()

    endheader_idx = None
    for i, line in enumerate(lines):
        if line.strip().lower() == "endheader":
            endheader_idx = i
            break

    if endheader_idx is None:
        raise ValueError(f"'endheader' not found in MOT file: {fname}")

    headers = lines[endheader_idx + 1].strip().split()

    data_rows = []
    for line in lines[endheader_idx + 2:]:
        s = line.strip()
        if not s:
            continue
        data_rows.append([float(x) for x in s.split()])

    data = np.array(data_rows, dtype=float)
    return data, headers


def load_trc(fname):
    fname = Path(fname)
    with fname.open("r", encoding="utf-8", errors="ignore") as f:
        lines = f.readlines()

    if len(lines) < 6:
        raise ValueError(f"TRC file too short: {fname}")

    raw_headers = lines[3].rstrip("\n\r").split("\t")
    if len(raw_headers) <= 2:
        raw_headers = lines[3].split()

    headers = [h.strip() for h in raw_headers if h.strip()]
    if len(headers) < 2:
        raise ValueError(f"Could not parse TRC headers from: {fname}")

    headers_xyz = headers[:2]
    for marker in headers[2:]:
        headers_xyz.extend([f"{marker}_X", f"{marker}_Y", f"{marker}_Z"])

    expected_cols = 2 + 3 * max(0, len(headers) - 2)
    data_rows = []
    for line in lines[5:]:
        s = line.rstrip("\n\r")
        if not s.strip():
            continue
        parts = s.split("\t")
        if len(parts) <= 2:
            parts = s.split()
        if len(parts) < expected_cols:
            parts.extend([""] * (expected_cols - len(parts)))
        elif len(parts) > expected_cols:
            parts = parts[:expected_cols]
        data_rows.append([float(x) if str(x).strip() else np.nan for x in parts])

    data = np.array(data_rows, dtype=float)
    return data, headers, headers_xyz


# ============================================================
# OPENSIM HELPERS
# ============================================================

def unlock_model(model_path, output_path=None):
    model_path = Path(model_path)
    output_path = Path(output_path) if output_path is not None else model_path
    base_model = osim.Model(str(model_path))

    joint_set = base_model.getJointSet()
    for u in range(joint_set.getSize()):
        joint = joint_set.get(u)
        for v in range(joint.numCoordinates()):
            coord = joint.get_coordinates(v)
            coord.set_locked(False)

    base_model.printToXML(str(output_path))
    return base_model


def get_marker_by_name(markerset, name):
    try:
        return markerset.get(name)
    except Exception:
        idx = markerset.getIndex(name)
        return markerset.get(idx)


def safe_set_ik_task_set(ik_tool, ik_set):
    for method_name in ["set_IKTaskSet", "setIKTaskSet"]:
        if hasattr(ik_tool, method_name):
            getattr(ik_tool, method_name)(ik_set)
            return
    raise AttributeError("Could not find IK task set setter on InverseKinematicsTool.")


def safe_set_marker_file(ik_tool, trc_file):
    for method_name in ["set_marker_file", "setMarkerDataFileName"]:
        if hasattr(ik_tool, method_name):
            getattr(ik_tool, method_name)(trc_file)
            return
    raise AttributeError("Could not find marker file setter on InverseKinematicsTool.")


def safe_set_report_marker_locations(ik_tool, flag):
    for method_name in ["set_report_marker_locations", "setReportMarkerLocations"]:
        if hasattr(ik_tool, method_name):
            getattr(ik_tool, method_name)(flag)
            return
    raise AttributeError("Could not find report marker locations setter on InverseKinematicsTool.")


def safe_set_output_motion_file(ik_tool, outpath):
    for method_name in ["set_output_motion_file", "setOutputMotionFileName"]:
        if hasattr(ik_tool, method_name):
            getattr(ik_tool, method_name)(outpath)
            return
    raise AttributeError("Could not find output motion setter on InverseKinematicsTool.")


# ============================================================
# INPUT / CONFIG
# ============================================================

def input_ast_from_config(config):
    base_model_path = Path(config["base_model_path"])
    trc_path = Path(config["trc_path"])
    setup_path = Path(config["setup_path"])

    model_folder = str(base_model_path.parent)
    base_model_file = base_model_path.name

    model = osim.Model(str(base_model_path))

    model_file = re.sub(r"\.osim$", "_itr.osim", base_model_file)
    model.printToXML(str(base_model_path.parent / model_file))

    stat_trc, head_trc, head_trc_xyz = load_trc(str(trc_path))
    print("head_trc:", head_trc[:10])
    print("head_trc_xyz:", head_trc_xyz[:15])
    print("stat_trc shape:", stat_trc.shape)
    print("first row:", stat_trc[0, :12])

    subject_height = float(config["subject_height"])
    subject_weight = float(config["subject_weight"])
    generic_model_height = float(config["generic_model_height"])
    generic_model_weight = float(config["generic_model_weight"])
    pose = int(config["pose"])

    mean_scale_fact_y = subject_height / generic_model_height
    mean_scale_fact = osim.Vec3(mean_scale_fact_y, mean_scale_fact_y, mean_scale_fact_y)

    print("------------------------------")
    print(f"Model unscaled : {re.sub(r'.osim$', '', base_model_file)}")
    print("------------------------------")
    print(f"Generic model height : {generic_model_height} cm")
    print(f"Generic model weight : {generic_model_weight} kg")
    print(f"Subject height : {subject_height} cm")
    print(f"Subject weight : {subject_weight} kg")
    print("------------------------------")
    print(f"Average scale factor (ASF): {mean_scale_fact_y}")
    print("------------------------------")

    return {
        "base_model_file": base_model_file,
        "model_folder": model_folder,
        "model": model,
        "model_file": str(base_model_path.parent / model_file),
        "trc_file": str(trc_path),
        "trc_folder": str(trc_path.parent),
        "stat_trc": stat_trc,
        "head_trc": head_trc,
        "head_trc_xyz": head_trc_xyz,
        "setup_path": str(setup_path),
        "starting_setup_folder": str(setup_path.parent),
        "subject_height": subject_height,
        "subject_weight": subject_weight,
        "generic_model_height": generic_model_height,
        "generic_model_weight": generic_model_weight,
        "pose": pose,
        "mean_scale_fact_y": mean_scale_fact_y,
        "mean_scale_fact": mean_scale_fact,
    }


# ============================================================
# CREATE TOOLS
# ============================================================

def create_tools(
    setup_path,
    model_folder,
    model_file,
    trc_file,
    model,
    mean_scale_fact,
    subject_weight,
    subject_height,
    label,
):
    setup_path = Path(setup_path)
    model_folder = Path(model_folder)

    trc_file = str(Path(trc_file))
    try:
        trc_rel_to_setup = os.path.relpath(trc_file, start=str(setup_path.parent))
    except ValueError:
        trc_rel_to_setup = trc_file

    scaler = osim.ScaleTool(str(setup_path))

    array2 = osim.ArrayStr()
    array2.append("measurements")
    array2.append("manualScale")
    scaler.getModelScaler().setScalingOrder(array2)

    scaled_file_name = str(model_folder / f"{label}_ModelScaled_API.osim")
    scaler.getModelScaler().setOutputModelFileName(scaled_file_name)
    scaler.getGenericModelMaker().setModelFileName(model_file)

    scaler.getModelScaler().setMarkerFileName(trc_rel_to_setup)
    scaler.getMarkerPlacer().setMarkerFileName(trc_rel_to_setup)

    time_range = scaler.getModelScaler().getTimeRange()
    preserve_mass_dist = scaler.getModelScaler().getPreserveMassDist()

    scaler.getMarkerPlacer().setTimeRange(time_range)
    scaler.getMarkerPlacer().setApply(False)
    scaler.printToXML(str(setup_path))

    ik_set = scaler.getMarkerPlacer().getIKTaskSet()

    ik_tool = osim.InverseKinematicsTool()
    safe_set_ik_task_set(ik_tool, ik_set)
    safe_set_marker_file(ik_tool, trc_file)
    safe_set_report_marker_locations(ik_tool, True)

    coord_file_name = str(model_folder / f"{label}_Coord_Static.mot")
    safe_set_output_motion_file(ik_tool, coord_file_name)
    ik_tool.setStartTime(time_range.get(0))
    ik_tool.setEndTime(time_range.get(1))

    path_ik_static = model_folder / f"{label}_IkSetup_static_trial.xml"
    ik_tool.printToXML(str(path_ik_static))

    scaler_manual = scaler.clone()
    scaler_manual.setSubjectMass(subject_weight)
    scaler_manual.setSubjectHeight(subject_height)

    num_bodies = model.getBodySet().getSize()
    for m in range(num_bodies):
        scale = osim.Scale()
        scale.setScaleFactors(mean_scale_fact)
        scale.setSegmentName(model.getBodySet().get(m).getName())
        scale.setApply(True)
        scaler_manual.getModelScaler().getScaleSet().cloneAndAppend(scale)

    scaler_manual.getModelScaler().setOutputModelFileName(scaled_file_name)
    scaler_manual.getModelScaler().setPreserveMassDist(preserve_mass_dist)
    scaler_manual.getModelScaler().setMarkerFileName(trc_rel_to_setup)
    scaler_manual.getMarkerPlacer().setApply(False)
    scaler_manual.getGenericModelMaker().setModelFileName(model_file)

    array_manual = osim.ArrayStr()
    array_manual.append("manualScale")
    scaler_manual.getModelScaler().setScalingOrder(array_manual)
    scaler_manual.getModelScaler().setTimeRange(time_range)

    path_manual_scale = model_folder / f"{label}_ManualScaleSetup.xml"
    scaler_manual.printToXML(str(path_manual_scale))

    return {
        "scaler": scaler,
        "scaler_manual": scaler_manual,
        "array2": array2,
        "scaled_file_name": scaled_file_name,
        "coord_file_name": coord_file_name,
        "path_ik_static": str(path_ik_static),
        "path_manual_scale": str(path_manual_scale),
        "time_range": time_range,
        "preserve_mass_dist": preserve_mass_dist,
        "num_bodies": num_bodies,
    }


# ============================================================
# AST CORE
# ============================================================

def run_ast_core(
    model,
    model_folder,
    model_file,
    setup_path,
    scaled_file_name,
    path_ik_static,
    path_manual_scale,
    scaler,
    markerset,
    time_range,
    stat_trc,
    head_trc,
    end_err,
    km,
    manual_scale_err,
    rep,
    rep2,
    mean_scale_fact_y,
    mean_scale_fact,
    num_bodies,
    array2,
    label,
):
    s = -1
    k = 1
    flag = 0
    ind = 0
    flag2 = 0
    ind2 = 0

    err = []
    rms_err = []
    direction = []
    merr = []
    step = []

    manual_bodies = 0
    path_scaler_mix = None
    names_body_manual = []

    scale_factors_range = None
    model_folder = Path(model_folder)

    while True:
        if k > 1 and flag == 0 and flag2 == 0:
            tolerance = 0.08
            if rms_err[k - 2] >= manual_scale_err:
                tolerance = 0.0

            if mean_scale_fact_y > 1:
                scale_factors_range = [
                    mean_scale_fact_y - tolerance / 4,
                    mean_scale_fact_y + tolerance * 3 / 4,
                ]
            elif mean_scale_fact_y < 1:
                scale_factors_range = [
                    mean_scale_fact_y - tolerance * 3 / 4,
                    mean_scale_fact_y + tolerance / 4,
                ]
            else:
                scale_factors_range = [
                    mean_scale_fact_y - tolerance / 2,
                    mean_scale_fact_y + tolerance / 2,
                ]

        if k == 1:
            osim.ScaleTool(str(setup_path)).run()

        elif k > 1:
            if ((rms_err[k - 2] > manual_scale_err and flag == 0) or flag2 == 1):
                ind2 += 1
                flag2 = 1
                if ind2 >= rep2:
                    flag2 = 0
                    ind2 = 0

                osim.ScaleTool(path_manual_scale).run()
                print("Manual scaling for all bodies")

            elif manual_bodies > 1 or flag == 1:
                ind += 1
                flag = 1
                if ind >= rep:
                    flag = 0
                    ind = 0

                if path_scaler_mix is None:
                    raise RuntimeError("path_scaler_mix is None when mixed scaling was requested.")

                osim.ScaleTool(path_scaler_mix).run()

                body_names = [str(item) for item in names_body_manual]
                if body_names:
                    print("Manual scale for body: " + ", ".join(sorted(set(body_names))))
                names_body_manual = []

            else:
                osim.ScaleTool(str(setup_path)).run()

        ik_tool = osim.InverseKinematicsTool(path_ik_static)

        scaled_model = osim.Model(scaled_file_name)
        ik_tool.setModel(scaled_model)
        ik_tool.set_accuracy(1e-5)
        ik_tool.run()

        marker_location, head_sto = load_sto(str(model_folder / "_ik_model_marker_locations.sto"))

        selected_marker = []
        for i in range(1, len(head_sto)):
            mk = head_sto[i].replace("_tx", "").replace("_ty", "").replace("_tz", "")
            selected_marker.append(mk)

        selected_marker_list = list(dict.fromkeys(selected_marker))

        t1 = time_range.get(0)
        t2 = time_range.get(1)

        trc_start = next(i for i in range(len(stat_trc)) if round(stat_trc[i, 1], 4) > t1)
        trc_end = max(i for i in range(len(stat_trc)) if round(stat_trc[i, 1], 4) <= t2)

        length_mark_err = min(
            marker_location.shape[0],
            stat_trc[trc_start:trc_end + 1, :].shape[0]
        )

        mark_err_blocks = []
        module_err = []

        for i, marker_name in enumerate(selected_marker_list):
            idxs = [j for j, h in enumerate(head_trc) if h.strip() == marker_name.strip()]
            if not idxs:
                print(f"Skipping marker not found in TRC header: {marker_name}")
                continue

            marker_idx = idxs[0] - 2
            pos_trc = 2 + 3 * marker_idx
            pos_sto_start = i * 3 + 1
            pos_sto_end = i * 3 + 4

            if pos_trc < 2 or pos_trc + 3 > stat_trc.shape[1]:
                print(f"Skipping marker with invalid TRC slice: {marker_name}, pos_trc={pos_trc}, stat_trc.shape={stat_trc.shape}")
                continue

            trc_xyz = stat_trc[trc_start:trc_start + length_mark_err, pos_trc:pos_trc + 3] / 1000.0
            sto_xyz = marker_location[:length_mark_err, pos_sto_start:pos_sto_end]

            if trc_xyz.shape[1] != 3 or sto_xyz.shape[1] != 3:
                print(f"Skipping marker due to bad slice: {marker_name}, trc_xyz.shape={trc_xyz.shape}, sto_xyz.shape={sto_xyz.shape}")
                continue

            diff = trc_xyz - sto_xyz
            mark_err_blocks.append(diff)
            module_err.append(np.sqrt(np.sum(np.mean(diff, axis=0) ** 2)))

        if not mark_err_blocks:
            raise RuntimeError("No marker error blocks were computed. Check TRC/STO marker names.")

        mark_err = np.hstack(mark_err_blocks)
        module_err = np.array(module_err)

        mean_mark_err = np.mean(mark_err, axis=0)
        pos_max_err = int(np.argmax(np.abs(mean_mark_err)))
        max_err = float(np.max(np.abs(mean_mark_err)))
        rms_err_k = float(np.sqrt(np.mean(module_err ** 2)))
        rms_err.append(rms_err_k)

        to_change = head_sto[pos_max_err + 1]

        if "_tx" in to_change:
            dir_to_change = 1
            marker_to_change = to_change.replace("_tx", "")
        elif "_ty" in to_change:
            dir_to_change = 2
            marker_to_change = to_change.replace("_ty", "")
        else:
            dir_to_change = 3
            marker_to_change = to_change.replace("_tz", "")

        direction.append(dir_to_change)
        err.append(max_err)
        merr.append(marker_to_change)

        manual_bodies = 0
        names_body_manual = []

        if k != 1:
            measurement_set = scaler.getModelScaler().getMeasurementSet()
            for n in range(measurement_set.getSize()):
                dist_mod = []
                dist_exp = []
                marker_pair_set = measurement_set.get(n).getMarkerPairSet()

                for z in range(marker_pair_set.getSize()):
                    first_marker = marker_pair_set.get(z).getMarkerName(0)
                    second_marker = marker_pair_set.get(z).getMarkerName(1)

                    pos_a_exp = None
                    pos_b_exp = None
                    for p in range(len(head_trc)):
                        if head_trc[p].strip() == first_marker.strip():
                            pos_a_exp = 2 + 3 * (p - 2)
                        elif head_trc[p].strip() == second_marker.strip():
                            pos_b_exp = 2 + 3 * (p - 2)

                    if pos_a_exp is None or pos_b_exp is None:
                        continue

                    state = model.initSystem()

                    loc_a = get_marker_by_name(markerset, first_marker).getLocationInGround(state)
                    loc_b = get_marker_by_name(markerset, second_marker).getLocationInGround(state)

                    loc_a_np = np.array([loc_a.get(0), loc_a.get(1), loc_a.get(2)])
                    loc_b_np = np.array([loc_b.get(0), loc_b.get(1), loc_b.get(2)])
                    dist_mod.append(np.linalg.norm(loc_a_np - loc_b_np))

                    exp_a = np.mean(stat_trc[:, pos_a_exp:pos_a_exp + 3], axis=0)
                    exp_b = np.mean(stat_trc[:, pos_b_exp:pos_b_exp + 3], axis=0)
                    dist_exp.append(np.linalg.norm(exp_a - exp_b) / 1000.0)

                if not dist_mod or not dist_exp:
                    continue

                dist_mod = np.array(dist_mod)
                dist_exp = np.array(dist_exp)
                scale_fact_n = float(np.mean(dist_exp / dist_mod))

                if (
                    scale_factors_range is not None and
                    (scale_fact_n < scale_factors_range[0] or scale_fact_n > scale_factors_range[1])
                ):
                    if measurement_set.get(n).getApply() == 1:
                        names_body_manual.append(
                            measurement_set.get(n).getBodyScaleSet().get(0).getName()
                        )
                        manual_bodies += 1

        if manual_bodies > 0:
            scaler_mix = scaler.clone()
            scaler_mix.getModelScaler().setScalingOrder(array2)

            for m in range(num_bodies):
                scale = osim.Scale()
                scale.setSegmentName(model.getBodySet().get(m).getName())
                scale.setApply(False)
                scaler_mix.getModelScaler().getScaleSet().cloneAndAppend(scale)

            for m in range(num_bodies):
                body_name = model.getBodySet().get(m).getName()
                for l in range(manual_bodies):
                    manual_name = str(names_body_manual[l])
                    scale = osim.Scale()
                    scale.setScaleFactors(mean_scale_fact)
                    scale.setSegmentName(body_name)
                    if body_name == manual_name:
                        scale.setApply(True)
                        scaler_mix.getModelScaler().getScaleSet().cloneAndAppend(scale)

            path_scaler_mix = str(model_folder / f"{label}_ScalerMix.xml")
            scaler_mix.printToXML(path_scaler_mix)

        if flag == 1:
            step_k = max_err
        elif flag2 == 1:
            step_k = max_err / 2.0
        else:
            step_k = max_err / 6.0

        state = model.initSystem()
        generic_marker = get_marker_by_name(markerset, merr[k - 1])

        sock_name = generic_marker.getParentFrame()
        ground_frame = model.getGround()

        generic_marker.changeFramePreserveLocation(state, ground_frame)
        current_location = generic_marker.get_location()
        current_coord_abs = np.array([
            current_location.get(0),
            current_location.get(1),
            current_location.get(2),
        ])

        if k != 1:
            if (
                err[k - 1] >= err[k - 2]
                and merr[k - 1] == merr[k - 2]
                and direction[k - 1] == direction[k - 2]
            ):
                s = -s
                print("sign changed")
                step_k = 2 * step[k - 2]

        if dir_to_change == 1:
            new_coord = current_coord_abs + np.array([s * step_k, 0.0, 0.0])
        elif dir_to_change == 2:
            new_coord = current_coord_abs + np.array([0.0, s * step_k, 0.0])
        else:
            new_coord = current_coord_abs + np.array([0.0, 0.0, s * step_k])

        new_location = osim.Vec3(float(new_coord[0]), float(new_coord[1]), float(new_coord[2]))
        generic_marker.set_location(new_location)
        generic_marker.changeFramePreserveLocation(state, sock_name)

        model.finalizeConnections()
        markerset.connectToModel(model)
        model.printToXML(model_file)

        step.append(step_k)

        print(
            f"cycle #{k} "
            f"RMS Error: {rms_err[k - 1]} "
            f"Max Error: {err[k - 1]} "
            f"Coord: {to_change} "
            f"increment: {s * step_k}"
        )

        if (
            rms_err[k - 1] < end_err
            or k >= km
            or (
                k > 1
                and rms_err[k - 1] > rms_err[k - 2]
                and flag == 0
                and flag2 == 0
            )
        ):
            break

        k += 1
        np.save(str(model_folder / f"{label}_err.npy"), np.array(err))
        np.save(str(model_folder / f"{label}_RMSErr.npy"), np.array(rms_err))

    return {
        "model": model,
        "err": np.array(err),
        "RMSErr": np.array(rms_err),
        "direction": direction,
        "merr": merr,
        "step": step,
        "manual_bodies": manual_bodies,
        "path_scaler_mix": path_scaler_mix,
    }


# ============================================================
# MAIN
# ============================================================

def main():
    t_start = time.time()
    run_tag = datetime.now().strftime("%Y%m%d_%H%M%S")

    CONFIG = {
        "base_model_path": r"C:\Users\tomas\Documents\OpenSim\4.5\Models\Rajagopal_OpenSense\Russell_model_athletes\Russell_markers_March3.osim",
        "trc_path": r"G:\My Drive\Documents\AI_and_math_physics\Sessions_vicon\March3\vicon_data_1_3-5-2026\X01.trc",
        "setup_path": r"G:\My Drive\Documents\AI_and_math_physics\AST_python\setup_markers_scale_march3.xml",
        "subject_height": 162.0,
        "subject_weight": 62.0,
        "generic_model_height": 170.0,
        "generic_model_weight": 75.0,
        "pose": 0,
    }

    if len(sys.argv) > 1:
        config_path = Path(sys.argv[1])
        with config_path.open("r", encoding="utf-8-sig") as f:
            CONFIG.update(json.load(f))
        print(f"Loaded config: {config_path}")

    inp = input_ast_from_config(CONFIG)
    base_model_stem = Path(inp["base_model_file"]).stem
    label = f"{base_model_stem}_{run_tag}"

    name_model_scaled_adj = str(Path(inp["model_folder"]) / f"{label}.osim")
    name_model_scaled_adj_locked = str(Path(inp["model_folder"]) / f"{label}_locked_tmp.osim")
    km = int(CONFIG.get("km", 60))
    end_err = float(CONFIG.get("end_err", 0.004))
    manual_scale_err = float(CONFIG.get("manual_scale_err", 0.025))
    rep = int(CONFIG.get("rep", 4))
    rep2 = int(CONFIG.get("rep2", 8))

    tools = create_tools(
        setup_path=inp["setup_path"],
        model_folder=inp["model_folder"],
        model_file=inp["model_file"],
        trc_file=inp["trc_file"],
        model=inp["model"],
        mean_scale_fact=inp["mean_scale_fact"],
        subject_weight=inp["subject_weight"],
        subject_height=inp["subject_height"],
        label=label,
    )

    if inp["pose"] == 1:
        osim.ScaleTool(tools["path_manual_scale"]).run()
        scaled_model_first = osim.Model(tools["scaled_file_name"])
        ik_coord = osim.InverseKinematicsTool(tools["path_ik_static"])
        ik_coord.setModel(scaled_model_first)
        ik_coord.run()

        coord_data, coord_head = load_mot(tools["coord_file_name"])
        coord_data = coord_data[:, 1:]
        avg_coord_data = np.deg2rad(np.mean(coord_data, axis=0))
        avg_coord_data[3:6] = 0.0

        d = 0
        joint_set = inp["model"].getJointSet()
        for u in range(joint_set.getSize()):
            joint = joint_set.get(u)
            for v in range(joint.numCoordinates()):
                coord = joint.get_coordinates(v)
                coord.set_clamped(False)
                coord.set_default_value(float(avg_coord_data[d]))
                d += 1

        inp["model"].printToXML(inp["model_file"])

    markerset = inp["model"].getMarkerSet()
    markerset.printToXML(str(Path(inp["model_folder"]) / f"{label}_MarkerSet.xml"))
    n_markers = markerset.getSize()
    print(f"Marker count: {n_markers}")

    result = run_ast_core(
        model=inp["model"],
        model_folder=inp["model_folder"],
        model_file=inp["model_file"],
        setup_path=inp["setup_path"],
        scaled_file_name=tools["scaled_file_name"],
        path_ik_static=tools["path_ik_static"],
        path_manual_scale=tools["path_manual_scale"],
        scaler=tools["scaler"],
        markerset=markerset,
        time_range=tools["time_range"],
        stat_trc=inp["stat_trc"],
        head_trc=inp["head_trc"],
        end_err=end_err,
        km=km,
        manual_scale_err=manual_scale_err,
        rep=rep,
        rep2=rep2,
        mean_scale_fact_y=inp["mean_scale_fact_y"],
        mean_scale_fact=inp["mean_scale_fact"],
        num_bodies=tools["num_bodies"],
        array2=tools["array2"],
        label=label,
    )

    if result["manual_bodies"] != 0 and result["path_scaler_mix"] is not None:
        adj_scaler = osim.ScaleTool(result["path_scaler_mix"])
    else:
        adj_scaler = osim.ScaleTool(inp["setup_path"])

    adj_scaler.getGenericModelMaker().setModelFileName(inp["model_file"])
    adj_scaler.getMarkerPlacer().setApply(True)
    adj_scaler.getMarkerPlacer().setOutputModelFileName(Path(name_model_scaled_adj_locked).name)
    adj_scaler.getMarkerPlacer().setOutputMarkerFileName(f"{label}_MarkerSetAdj.xml")
    adj_scaler.getMarkerPlacer().setOutputMotionFileName(f"{label}_StaticPoseAdj.mot")

    path_setup_scale_adj = str(Path(inp["model_folder"]) / f"{label}_ScalingSetupMarkerAdj.xml")
    adj_scaler.printToXML(path_setup_scale_adj)

    os.chdir(inp["model_folder"])
    adj_scaler.run()

    adj_model_name = Path(name_model_scaled_adj_locked).name

    candidates = [
        Path(inp["model_folder"]) / adj_model_name,
        Path(path_setup_scale_adj).parent / adj_model_name,
        Path.cwd() / adj_model_name,
        Path(inp["setup_path"]).parent / adj_model_name,
    ]

    existing = next((p for p in candidates if p.exists()), None)
    if existing is None:
        raise FileNotFoundError(
            "Could not find adjusted model in any expected location:\n" +
            "\n".join(str(p) for p in candidates)
        )

    print("Unlocking model from:", existing)
    unlock_model(existing, name_model_scaled_adj)
    if Path(existing).resolve() != Path(name_model_scaled_adj).resolve():
        Path(existing).unlink(missing_ok=True)

    elapsed_sec = time.time() - t_start
    print(f"Elapsed time: {elapsed_sec:.2f} s")
    print(f"Elapsed time: {elapsed_sec / 60:.2f} min")


if __name__ == "__main__":
    main()
