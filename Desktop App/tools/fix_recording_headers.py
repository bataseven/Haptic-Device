"""
Fixes the header line of CSV files recorded by older versions of the data recorder.

Older versions had two problems in the header line:
  - The "Wave Amplitude" and "Wave Frequency" labels were swapped. The values were always written in the
    order type, frequency, amplitude, offset, value, but the header said type, amplitude, frequency, ...
  - Some versions did not end the header with a newline, so the first data row was on the header line.
  - Some versions wrote the PID gains in the rows but did not have the gain columns in the header.

Only the header line is changed, the data rows are left as they are. Files that are already correct are skipped,
so it is safe to run this more than once. A backup with the ".bak" extension is kept next to each changed file.

Usage: python fix_recording_headers.py <file or folder> [<file or folder> ...]
"""
import os
import shutil
import sys

LAST_HEADER_FIELD = "Index Outer Force"
GAIN_COLUMNS = ["PID Position P", "PID Position I", "PID Position D",
                "PID Velocity P", "PID Velocity I", "PID Velocity D",
                "PID Force P", "PID Force I", "PID Force D"]


def addMissingGainColumns(header: str, firstRow: str) -> str:
    names = header.split(", ")
    if any("PID" in name for name in names):
        return header
    if len(firstRow.split(",")) != len(names) + 2 * len(GAIN_COLUMNS):
        return header
    for motor in ("Finger", "Palm"):
        index = names.index(f"{motor} Control Mode") + 1
        names[index:index] = [f"{motor} {name}" for name in GAIN_COLUMNS]
    return ", ".join(names)


def fixText(text: str) -> str:
    firstLine, _, rest = text.partition("\n")
    end = firstLine.find(LAST_HEADER_FIELD)
    if end == -1:
        return text  # Not a recording made by the data recorder

    end += len(LAST_HEADER_FIELD)
    header = firstLine[:end]
    mergedRow = firstLine[end:]  # The first data row if the header line was not terminated

    for motor in ("Finger", "Palm"):
        header = header.replace(f"{motor} Wave Amplitude, {motor} Wave Frequency",
                                f"{motor} Wave Frequency, {motor} Wave Amplitude")

    firstRow = mergedRow if mergedRow.strip() else rest.partition("\n")[0]
    header = addMissingGainColumns(header, firstRow)

    lines = [header]
    if mergedRow.strip():
        lines.append(mergedRow)
    return "\n".join(lines) + "\n" + rest


def fixFile(path: str) -> bool:
    with open(path) as file:
        text = file.read()
    fixed = fixText(text)
    if fixed == text:
        return False

    backupPath = path + ".bak"
    if not os.path.exists(backupPath):
        shutil.copy2(path, backupPath)
    with open(path, "w") as file:
        file.write(fixed)
    return True


def csvFiles(paths):
    for path in paths:
        if os.path.isdir(path):
            for name in sorted(os.listdir(path)):
                if name.lower().endswith(".csv"):
                    yield os.path.join(path, name)
        else:
            yield path


def main(paths) -> None:
    if not paths:
        print(__doc__)
        sys.exit(1)
    for path in csvFiles(paths):
        print(("Fixed:   " if fixFile(path) else "Skipped: ") + path)


if __name__ == "__main__":
    main(sys.argv[1:])
