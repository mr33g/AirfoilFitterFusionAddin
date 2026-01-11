# Airfoil Fitter Add-In for Fusion 360

A Fusion 360 add-in that imports airfoil coordinate data from `.dat` files and fits optimized B-spline curves to it. The generated splines are aligned to the selected sketch line.  The result maintains smooth curvature and geometric continuity at the leading edge and can be used immediately for lofts, sweeps, and other CAD operations.

## Installation

### Option 1: MSI Installer (Windows)

Two MSI installer variants are available:

#### Clean Version
- **File**: `AirfoilFitterFusion.msi`
- **Includes**: Only the add-in code, no bundled dependencies
- **Size**: Smaller (~1 MB)
- **Requirements**: The Add-in will attempt to install the required dependencies in the add-in folder on first run. Fusion needs to be restarted after installation is complete.

#### Bundled Version
- **File**: `AirfoilFitterFusion-bundled.msi`
- **Includes**: All required Python dependencies (numpy, scipy, ezdxf) bundled in the installer
- **Use when**: Local dependency installation fails
- **Size**: Larger (~50+ MB) due to bundled libraries


### Option 2: Manual Installation

1. Download or clone this repository
2. Copy the `AirfoilFitterFusionAddIn` folder to your Fusion 360 add-ins directory:
   - **Windows**: `%APPDATA%\Autodesk\Autodesk Fusion 360\API\AddIns\`
   - **macOS**: `~/Library/Application Support/Autodesk/Autodesk Fusion 360/API/AddIns/`
3. Restart Fusion 360
4. Go to **Utilities → Add-Ins → Scripts and Add-Ins** and enable **AirfoilFitterFusionAddIn**

## Usage

1. **Create a sketch** with a line representing your desired chord position and length
2. **Launch the command** "Insert fitted airfoil" from the INSERT panel in the Solid or Surface workspace toolbar
3. **Select the chord line** in your sketch
4. **Select an airfoil file** (`.dat` format)
5. **Adjust fitting parameters**:
   - **Control Point Count**: More points = higher accuracy but might introduce oscillations, fewer = smoother curves
   - **Smoothness Penalty**: Higher values produce smoother control polygons at the cost of accuracy
   - **G2/G3**: Enable for curvature continuity at the leading edge
   - **Enforce TE Tangency**: Match trailing edge tangent direction from original data
6. **Position the airfoil**:
   - Use **Rotate 90°** to orient the airfoil plane relative to the chord line
   - Use **Flip** to reverse nose/tail direction
   - Use **TE Thickness** manipulator to add trailing edge thickness
   - Preview is automatically displayed when both chord line and file are selected
7. **Enable Editable** if you need to modify control points after insertion
8. **Click OK** to insert the final geometry

## Features

### Airfoil Data Import
- **Automatic format detection**: The loader analyzes coordinate patterns to determine the format. Currently Selig and Lednicer are supported.
- **Automatic normalization**: Coordinates are translated, rotated, and scaled so the leading edge is at origin and chord lies along the X-axis

### B-Spline Fitting
- **Single-span Bézier curves**: Uses degree = (control points - 1), creating true Bézier curves without internal knots to ensure compability with Fusion 360. Based on Dev Rajnarayan et al. 2019 (https://arc.aiaa.org/doi/10.2514/6.2018-3949).
- **Adjustable control point count**: 4 to 19 control points per surface (upper/lower fitted independently)
- **G1 continuity**: Tangent continuity is always enforced at the leading edge between upper and lower surfaces
- **G2 continuity** (optional): Curvature continuity at the leading edge via constrained optimization
- **G3 continuity** (optional): Curvature derivative continuity at the leading edge
- **Trailing edge tangency** (optional): Enforces tangent direction at the trailing edge based on the original airfoil data
- **Smoothness penalty**: Adjustable regularization to balance accuracy vs. smoothness of the control polygon

### Geometry Placement
- **Chord line selection**: Select any sketch line to define the chord position and length
- **Automatic scaling**: Airfoil is scaled to match the selected chord line length
- **Rotation**: Rotate the airfoil plane in 90° increments around the chord line (0°, 90°, 180°, 270°)
- **Flip orientation**: Reverse the nose-to-tail direction along the chord line
- **Trailing edge thickness**: Add symmetric trailing edge thickness with minimal distortion of the airfoil.

### Output Options
- **Editable splines** (experimental): Create geometry via DXF import to create control-point splines that can be edited in Fusion
- **Fixed splines**: Create read-only NURBS curves directly through the API
- **Live preview**: Preview is automatically shown when both chord line and airfoil file are selected
- **Show input data**: Optionally display the original airfoil coordinate points for comparison

### Error Reporting
- **Max deviation display**: Shows the maximum fitting error for upper and lower surfaces in document units

### Dependencies

The add-in requires the following Python packages:
- `numpy`
- `scipy`  
- `ezdxf`

**Bundled installer**: These dependencies are included and automatically installed.

**Automatic or manual installation**: If the dependencies are missing from the `lib/` folder, the add-in will prompt to install them automatically on first run. You can also install them manually (see Troubleshooting section).

## File Format Support

### Selig Format
```
NACA 2412
1.000000  0.001260
0.950000  0.011480
...
0.000000  0.000000   <- Leading edge (minimum x)
...
0.950000 -0.009330
1.000000 -0.001260
```

### Lednicer Format
```
NACA 2412
35.  35.              <- Optional: point counts (upper, lower)
0.000000  0.000000    <- Upper surface starts at LE
0.012500  0.012500
...
1.000000  0.001260    <- Upper surface ends at TE
0.000000  0.000000    <- Lower surface starts at LE
0.012500 -0.010000
...
1.000000 -0.001260    <- Lower surface ends at TE
```

## Troubleshooting

### "Dependencies Missing" on startup
The add-in will offer to install numpy, scipy, and ezdxf automatically. If this fails:
1. Locate Fusion 360's Python: typically in the Fusion 360 installation directory
2. Run: `python -m pip install --target "<addin-path>/lib" numpy scipy ezdxf`
3. Restart Fusion 360

## License

MIT License - see [LICENSE](LICENSE) for details.

## Author

Michael Reeg

## Version History

- **1.0.0**: Initial release

