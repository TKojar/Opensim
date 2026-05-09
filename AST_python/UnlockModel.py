from pathlib import Path
import opensim as osim


def unlock_model(model_folder, model_file):
    """
    Unlock all coordinates of an OpenSim model and save a new file.

    Parameters
    ----------
    model_folder : str or Path
    model_file : str

    Returns
    -------
    model_unlocked : osim.Model
    """
    model_folder = Path(model_folder)

    model_path = model_folder / model_file
    base_model = osim.Model(str(model_path))

    num = 0

    joint_set = base_model.getJointSet()
    for u in range(joint_set.getSize()):
        joint = joint_set.get(u)
        for v in range(joint.numCoordinates()):
            coord = joint.get_coordinates(v)
            coord.set_locked(False)
            num += 1

    model_unlocked = base_model

    output_name = model_file.replace(".osim", "_Unlocked.osim")
    output_path = model_folder / output_name

    model_unlocked.printToXML(str(output_path))

    return model_unlocked