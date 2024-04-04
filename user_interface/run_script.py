# Module to open a terminal and run a script within it.

import subprocess


def run_script(script_path: str) -> int:
    """
    Function to open a new terminal and run a script within it.

    Args:
        script_path: Path to the script to be run.

    Return:
        Integer representing the code terminal returns when executed.
    """
    try:
        # Open a new terminal and run the script
        result = subprocess.check_call(["gnome-terminal", "--", "bash", "-c", f"bash {script_path}; exec bash"])
    except subprocess.CalledProcessError as e:
        print("Command exited with non-zero code %s" % e)
        result = -1
    return result

