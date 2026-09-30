"""Find an installed SUMO on Windows, macOS, Linux, or in a Python environment."""
import importlib.util
import os
from pathlib import Path
import shutil
import sys


def configure():
    candidates = []
    if os.environ.get("SUMO_HOME"):
        candidates.append(Path(os.environ["SUMO_HOME"]))
    if importlib.util.find_spec("sumo"):
        import sumo
        candidates.append(Path(sumo.SUMO_HOME))
    executable = shutil.which("sumo")
    if executable:
        candidates.append(Path(executable).resolve().parent.parent)
    candidates.append(Path("/Library/Frameworks/EclipseSUMO.framework/Versions/Current/EclipseSUMO/share/sumo"))
    for variable in ("PROGRAMFILES", "PROGRAMFILES(X86)", "LOCALAPPDATA"):
        if os.environ.get(variable):
            candidates.extend(Path(os.environ[variable]) / p for p in ("Eclipse/Sumo", "SUMO"))
    candidates.extend((Path("/usr/share/sumo"), Path("/usr/local/share/sumo")))
    suffix = ".exe" if os.name == "nt" else ""
    for home in candidates:
        binary = home / "bin" / ("sumo" + suffix)
        if not binary.is_file() and executable and (home / "tools").is_dir():
            binary = Path(executable)
        if not binary.is_file():
            continue
        os.environ["SUMO_HOME"] = str(home)
        if (home / "tools").is_dir():
            sys.path.insert(0, str(home / "tools"))
        # The official macOS package stores PROJ data in its nested framework.
        projections = [home.parent / "proj"]
        projections.extend(home.parent.parent.glob("framework/EclipseSUMO.framework/Versions/*/EclipseSUMO/share/proj"))
        for projection in projections:
            if (projection / "proj.db").is_file():
                os.environ.setdefault("PROJ_DATA", str(projection))
                break
        if not importlib.util.find_spec("traci") or not importlib.util.find_spec("sumolib"):
            raise RuntimeError("SUMO was found, but its Python tools are missing. Run: python -m pip install -r requirements.txt")
        return home, binary
    raise RuntimeError("SUMO was not found. Install SUMO 1.27.1, or run: python -m pip install -r requirements.txt")
