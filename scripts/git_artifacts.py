#!/usr/bin/env python3
"""Maintain the explicit Git allowlist for inputs/reports inside run directories.

Does not delete files, stage changes, access devices, or rewrite history.
"""
import argparse
import json
from pathlib import Path, PurePosixPath
import subprocess

ROOT = Path(__file__).resolve().parents[1]
MANIFEST = ROOT / 'config/git_retained_artifacts.json'
BEGIN = '# BEGIN curated generated artifacts (scripts/git_artifacts.py)'
END = '# END curated generated artifacts'


def policy(path=MANIFEST):
    data = json.loads(Path(path).read_text())
    if data.get('schema') != 'x5-git-artifact-policy-v1':
        raise ValueError('Unsupported artifact policy')
    roots = data['generated_roots']
    if len(set(roots)) != len(roots) or any('/' in r or r in ('', '.', '..') for r in roots):
        raise ValueError('Generated roots must be unique directory names')
    paths = []
    for row in data['artifacts']:
        name = row['path']
        p = PurePosixPath(name)
        if (p.is_absolute() or '..' in p.parts or p.as_posix() != name
                or len(p.parts) < 2 or p.parts[0] not in roots
                or any(c in name for c in '*?[]\\\n\r') or not row.get('reason')):
            raise ValueError('Invalid curated artifact: ' + name)
        paths.append(name)
    if len(set(paths)) != len(paths):
        raise ValueError('Duplicate curated artifact')
    return data, sorted(paths)


def ignore_block(data, paths):
    parents = set()
    for name in paths:
        parents.update(p.as_posix() for p in PurePosixPath(name).parents
                       if p.as_posix() != '.' and len(p.parts) > 1)
    return '\n'.join([
        BEGIN,
        '# Routine outputs stay local. Explicit exceptions preserve runtime inputs and reports.',
        *('/' + r + '/**' for r in data['generated_roots']),
        *('!/' + p + '/' for p in sorted(parents, key=lambda x: (x.count('/'), x))),
        *('!/' + p for p in paths),
        END,
    ]) + '\n'


def update_ignore(root=ROOT, manifest=MANIFEST, *, write=False):
    data, paths = policy(manifest)
    file = root / '.gitignore'
    text = file.read_text()
    block = ignore_block(data, paths)
    if BEGIN in text:
        before, remainder = text.split(BEGIN, 1)
        if END not in remainder:
            raise ValueError('Unterminated generated ignore block')
        _, after = remainder.split(END, 1)
        expected = before + block + after.lstrip('\n')
    else:
        expected = text.rstrip() + '\n\n' + block
    if write:
        file.write_text(expected)
    elif text != expected:
        raise ValueError('Ignore rules differ from manifest; run scripts/git_artifacts.py --write-ignore')
    return data, paths


def check(root=ROOT, manifest=MANIFEST, *, index=False):
    data, paths = update_ignore(root, manifest)
    for name in paths:
        file = root / name
        if not file.is_file() or file.is_symlink() or not file.resolve().is_relative_to(root.resolve()):
            raise ValueError('Missing or nonlocal curated artifact: ' + name)
    if index:
        tracked = set(subprocess.check_output(
            ['git', '-C', str(root), 'ls-files', '-z']).decode().rstrip('\0').split('\0'))
        missing = set(paths) - tracked
        unwanted = {p for p in tracked if p.split('/')[0] in data['generated_roots']} - set(paths)
        if missing or unwanted:
            raise ValueError(f'Index policy mismatch: {len(missing)} missing, {len(unwanted)} uncurated files')
    return dict(curated_files=len(paths), curated_bytes=sum((root / p).stat().st_size for p in paths))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--write-ignore', action='store_true', help='Update only the generated .gitignore block')
    parser.add_argument('--check-index', action='store_true', help='Also check staged/tracked artifact coverage')
    args = parser.parse_args()
    if args.write_ignore:
        update_ignore(write=True)
    print(json.dumps(check(index=args.check_index), indent=2))


if __name__ == '__main__':
    main()
