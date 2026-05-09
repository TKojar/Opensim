from pathlib import Path
import re
import opensim as osim


def input_ast(select_file_func, ask_value_func, ask_yes_no_func, load_trc_func):
    """
    Python translation of inputAST.m

    Parameters
    ----------
    select_file_func : callable
        Function like:
            path = select_file_func(prompt, pattern)
        returning a full file path as string.

    ask_value_func : callable
        Function like:
            values = ask_value_func(fields, defaults)
        returning a list of strings.

    ask_yes_no_func : callable
        Function like:
            answer = ask_yes_no_func(prompt)
        returning True/False.

    load_trc_func : callable
        Function that loads a TRC file and returns:
            stat_trc, head_trc, head_trc_xyz

    Returns
    -------
    dict
        Config/state values used by the AST pipeline.
    """

    # ------------------------------------------------------------
    # Importing data files to run operations
    # ------------------------------------------------------------
    base_model_path = Path(select_file_func("Pick the base model file to use", "*.osim"))
    model_folder = str(base_model_path.parent)
    base_model_file = base_model_path.name

    model = osim.Model(str(base_model_path))

    model_file = re.sub(r"\.osim$", "_itr.osim", base_model_file)
    model.printToXML(str(base_model_path.parent / model_file))

    # preserve current lock state explicitly, like MATLAB code
    joint_set = model.getJointSet()
    for u in range(joint_set.getSize()):
        joint = joint_set.get(u)
        for v in range(joint.numCoordinates()):
            coord = joint.get_coordinates(v)
            coord.set_locked(coord.get_locked())

    trc_path = Path(select_file_func("Pick Static Trial file to use", "*.trc"))
    trc_file = trc_path.name
    trc_folder = str(trc_path.parent)

    stat_trc, head_trc, head_trc_xyz = load_trc_func(str(trc_path))

    setup_path = Path(select_file_func("Pick the Scaling setup file to use", "*.xml"))
    setup_file = setup_path.name
    starting_setup_folder = str(setup_path.parent)

    # ------------------------------------------------------------
    # parameters to set
    # ------------------------------------------------------------
    info = ask_value_func(
        fields=[
            "Subject height (cm):",
            "Subject weight (kg):",
            "Model height (cm):",
            "Model weight (kg):",
        ],
        defaults=["156.5", "53.8", "170", "75.3"],
    )

    subject_height = float(info[0])
    subject_weight = float(info[1])
    generic_model_height = float(info[2])
    generic_model_weight = float(info[3])

    pose = 1 if ask_yes_no_func("Do you want to estimate the subject pose?") else 0

    # ------------------------------------------------------------
    # defining the manual scale factors
    # ------------------------------------------------------------
    mean_scale_fact_y = subject_height / generic_model_height
    mean_scale_fact = osim.Vec3(mean_scale_fact_y, mean_scale_fact_y, mean_scale_fact_y)

    # ------------------------------------------------------------
    # Display useful info
    # ------------------------------------------------------------
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
        "model_file": model_file,
        "trc_file": trc_file,
        "trc_folder": trc_folder,
        "stat_trc": stat_trc,
        "head_trc": head_trc,
        "head_trc_xyz": head_trc_xyz,
        "setup_file": setup_file,
        "starting_setup_folder": starting_setup_folder,
        "subject_height": subject_height,
        "subject_weight": subject_weight,
        "generic_model_height": generic_model_height,
        "generic_model_weight": generic_model_weight,
        "pose": pose,
        "mean_scale_fact_y": mean_scale_fact_y,
        "mean_scale_fact": mean_scale_fact,
    }