from importlib.resources import files
from pathlib import Path

def asset_path(relative: str) -> str:
    """
    Return an absolute filesystem path to an asset inside this package.
    Example: asset_path("openarm_bimanual.xml")
    """
    p = files("openarm_mujoco_v1").joinpath(relative)
    return str(Path(p))


def openarm_bimanual_paths() -> list[str]:
    """
    Returns the list of the absolute path to bimanual file and
    the other required files/directories.
    """
    return [
        asset_path("openarm_bimanual.xml"),
        asset_path("meshes"),
    ]
