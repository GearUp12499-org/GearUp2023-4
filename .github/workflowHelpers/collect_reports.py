import os
import shutil
import sys
from pathlib import Path

target = Path(sys.argv[1])
if target.exists():
    shutil.rmtree(target)
target.mkdir()

for path, dirs, files in os.walk('.'):
    path2 = Path(path)
    if tuple(path2.parts[-3:]) == ('build', 'reports', 'tests'):
        print(f'collecting {str(path2)}')
        rel = path2.relative_to('.')
        beforeIdentifier = '_'.join(rel.parts[:-3])
        beforeDir = (target / beforeIdentifier)
        beforeDir.mkdir(exist_ok=True)
        for dir_ in dirs:
            print(f'adding {dir_}')
            copyTarget = beforeDir / dir_
            shutil.copytree(path2 / dir_, copyTarget)
