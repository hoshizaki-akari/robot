"""Compatibility name that blocks accidental use of the FR5-only launch."""


def generate_launch_description():
    raise RuntimeError(
        "The copied FR5 traction launch is disabled in the AUBO i5 checkout. "
        "Use aubo_i5_status.launch.py for read-only bring-up."
    )
