# Flowcharts Documentation

This directory contains the flowcharts that document the operation of the Stage 1 cutting machine.

## Folder Structure

- **dot_files/**: Contains the source DOT files used to generate the flowcharts
- **png_files/**: Contains the rendered PNG images of the flowcharts
- **regenerate_charts.sh**: Script to regenerate PNG files from DOT sources

## Available Flowcharts

1. **cutting_machine_flowchart**: The main state machine diagram showing all states and transitions
2. **cutting_process**: Detailed flowchart of the cutting operation (states 5-7)
3. **homing_process**: Detailed flowchart of the homing operation (state 2)

## How to Regenerate the Charts

If you make changes to any DOT file in the `dot_files` folder, you can regenerate all the PNG files by running:

```bash
./regenerate_charts.sh
```

This will update all PNG files in the `png_files` folder based on the current DOT file sources.

## Requirements

- GraphViz must be installed to generate the charts (`dot` command)
- The script must be run from the flowcharts directory 