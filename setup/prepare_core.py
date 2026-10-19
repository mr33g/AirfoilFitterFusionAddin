"""Bundle the sibling shared core package without copying numerical source into Git."""
from pathlib import Path
import argparse
import shutil


ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--core', type=Path, default=ROOT.parent / 'airfoil-splines-core')
args = parser.parse_args()
core = args.core.resolve()
source = core / 'src' / 'airfoil_splines_core'
if not (core / 'pyproject.toml').is_file() or not (source / '__init__.py').is_file():
    parser.error(f'Local airfoil-splines-core project not found: {core}')

target = ROOT / '_vendor'
if target.resolve().parent != ROOT or target.is_symlink():
    raise RuntimeError('Unexpected generated bundle path.')
if target.exists():
    shutil.rmtree(target)
target.mkdir()
shutil.copytree(source, target / 'airfoil_splines_core')
