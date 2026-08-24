"""Runtime path setup for the self-contained release package."""

from pathlib import Path
import sys

PROJECT_ROOT = Path(__file__).resolve().parent
FAIRINO_SDK_ROOT = PROJECT_ROOT / "vendor" / "fairino-python-sdk" / "linux"

if FAIRINO_SDK_ROOT.is_dir() and str(FAIRINO_SDK_ROOT) not in sys.path:
    sys.path.insert(0, str(FAIRINO_SDK_ROOT))

