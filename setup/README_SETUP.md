# MSI Installer Setup (WiX v4+)

This folder contains the files necessary to build a Windows MSI installer for the Airfoil Fitter Fusion Add-in, following Autodesk's recommended `.bundle` structure and the latest **WiX v4+** standards.

## Prerequisites

1.  **WiX Toolset**: Installed via `dotnet tool`:
    ```bash
    dotnet tool install --global wix
    ```
2.  **WiX UI Extension**: Since we use the standard UI, you'll need the extension:
    ```bash
    wix extension add -g WixToolset.UI.wixext
    ```
3.  **WiX Util Extension**: Since we use utils, you'll need the extension:
    ```bash
    wix extension add -g WixToolset.Util.wixext
    ```
4.  **Python 3**: To run the file harvesting script.

## Building the Installer

**IMPORTANT**: If you have a previous version installed, please uninstall it via Windows Settings -> Apps before installing the new one.

Two installer variants can be built:

### Option 1: Bundled Installer (with libraries)

The bundled installer includes all Python dependencies (numpy, scipy, ezdxf) in the package.

**Quick build:**
```bash
build-bundled.bat
```

**Manual build:**
```bash
# 1. Generate WiX fragment (includes lib folder)
python generate_wxs_fragment.py --output Files.wxs

# 2. Build the MSI
wix build AirfoilFitter.wxs Files.wxs -ext WixToolset.UI.wixext -ext WixToolset.Util.wixext -o AirfoilFitterAddin-bundled.msi
```

### Option 2: Standalone Installer (without libraries)

The standalone installer excludes the `lib` folder, resulting in a much smaller package. Users will need to install dependencies manually or let the add-in prompt for installation on first run.

**Quick build:**
```bash
build-standalone.bat
```

**Manual build:**
```bash
# 1. Generate WiX fragment (excludes lib folder)
python generate_wxs_fragment.py --exclude-lib --output Files-Standalone.wxs

# 2. Build the MSI
wix build AirfoilFitter-Standalone.wxs Files-Standalone.wxs -ext WixToolset.UI.wixext -ext WixToolset.Util.wixext -o AirfoilFitterAddin.msi
```

### Building Both

To build both installers:

```bash
build-bundled.bat
build-standalone.bat
```
The GitHub Actions workflow automatically builds both variants and uploads them to releases.

### Uninstall

To uninstall the add-in run the msi-installer again, or unistall through "Installed Apps". Make sure Fusion is not running, so the installer is able to remove all files! 

## Autodesk Bundle Structure

The installer is configured to install the add-in to:
`%AppData%\Autodesk\ApplicationPlugins\AirfoilFitter.bundle\`

*   **PackageContents.xml**: Located at the root of the `.bundle` folder. It tells Fusion how to load the add-in.
*   **Contents/**: Subfolder containing all the actual add-in files and folders (`core`, `lib`, `ui`, etc.).

This structure ensures compatibility with the Autodesk App Store and provides a "clean" installation that Fusion automatically discovers on startup.

## Troubleshooting

If the add-in doesn't show up in Fusion:
1.  Verify the files are at `%AppData%\Autodesk\ApplicationPlugins\AirfoilFitter.bundle\`.
2.  Check that `PackageContents.xml` exists in that folder.
3.  Ensure `Contents/AirfoilFitter.py` exists relative to that folder.
4.  Check Fusion's "Add-ins" dialog (Shift+S) and look for "AirfoilFitter" in the list. Verify it is running.
5.  Check the Text output for error messages.
