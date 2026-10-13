#!/usr/bin/env python3
"""Enforce a small, explicit BARR-C subset; human review is still required."""

from pathlib import Path
import re

ROOT = Path(__file__).resolve().parent
problems = []
files = sorted((ROOT / "src").glob("*.[ch]"))
files += sorted((ROOT / "tests").glob("*.[ch]"))
for path in files:
    content = path.read_bytes()
    text = content.decode("ascii")
    for number, line in enumerate(text.splitlines(), 1):
        if len(line) > 80:
            problems.append(f"{path.name}:{number}: exceeds 80 columns")
        if "\t" in line or "\r" in line or line.rstrip() != line:
            problems.append(f"{path.name}:{number}: whitespace violation")
    # Remove comments and strings before scanning C tokens.
    tokens = re.sub(r'/\*.*?\*/|//[^\n]*|"(?:\\.|[^"\\])*"', "", text,
                    flags=re.S)
    if re.search(r"\b(auto|register|goto|continue|short|long)\b", tokens):
        problems.append(f"{path.name}: forbidden keyword")
    if re.search(r"\b(abort|exit|setjmp|longjmp)\s*\(", tokens):
        problems.append(f"{path.name}: forbidden jump function")
    for match in re.finditer(r"\b(if|for|while|switch)\s*\(", tokens):
        depth = 1
        cursor = match.end()
        while cursor < len(tokens) and depth:
            depth += (tokens[cursor] == "(") - (tokens[cursor] == ")")
            cursor += 1
        if not tokens[cursor:].lstrip().startswith("{"):
            problems.append(f"{path.name}: control block without braces")
    for match in re.finditer(r"\)\s*\{", tokens):
        brace = match.end() - 1
        prefix = tokens[tokens.rfind("\n", 0, brace) + 1:brace]
        if prefix.strip():
            problems.append(f"{path.name}: opening brace must be on own line")
    if not text.endswith("/* End of file. */\n\n"):
        problems.append(f"{path.name}: missing end marker/blank line")
    if path.suffix == ".c":
        if f'#include "{path.stem}.h"' not in text:
            problems.append(f"{path.name}: missing matching header")
    else:
        guard = path.stem.upper() + "_H"
        if f"#ifndef {guard}\n#define {guard}\n" not in text:
            problems.append(f"{path.name}: missing header guard")

if problems:
    raise SystemExit("\n".join(problems))
print(f"BARR-C subset checks passed ({len(files)} C/header files).")
