from pathlib import Path

# Define the root directory of the project
ROOT = Path(__file__).resolve().parents[1]

def abspath(p):
    """make absolute path, relative to ROOT if not already absolute"""
    p = Path(p)
    return p if p.is_absolute() else ROOT / p