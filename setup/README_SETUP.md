# MSI Installer Setup (WiX v4+)

This folder contains the files necessary to build a Windows MSI installer for the Airfoil Fitter Fusion 360 Add-in, following Autodesk's recommended `.bundle` structure and the latest **WiX v4+** standards.

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

### 1. Harvest Files
Run the Python script to generate the WiX file fragment (`Files.wxs`) for all files in the `AirfoilFitterFusionAddIn` directory:

```bash
python generate_wxs_fragment.py
```

### 2. Build the MSI
Open a terminal in this `setup` directory and run the build command (the UI extension is required for the new installer dialogs):

```bash
wix build AirfoilFitterFusionAddIn.wxs Files.wxs -ext WixToolset.UI.wixext -ext WixToolset.Util.wixext -o AirfoilFitterFusionAddInSetup.msi
```

## Autodesk Bundle Structure

The installer is configured to install the add-in to:
`%AppData%\Autodesk\ApplicationPlugins\AirfoilFitterFusionAddIn.bundle\`

*   **PackageContents.xml**: Located at the root of the `.bundle` folder. It tells Fusion 360 how to load the add-in.
*   **Contents/**: Subfolder containing all the actual add-in files and folders (`core`, `lib`, `ui`, etc.).

This structure ensures compatibility with the Autodesk App Store and provides a "clean" installation that Fusion 360 automatically discovers on startup.

## Troubleshooting

If the add-in doesn't show up in Fusion 360:
1.  Verify the files are at `%AppData%\Autodesk\ApplicationPlugins\AirfoilFitterFusionAddIn.bundle\`.
2.  Check that `PackageContents.xml` exists in that folder.
3.  Ensure `Contents/AirfoilFitterFusionAddIn.py` exists relative to that folder.
4.  Check Fusion 360's "Add-ins" dialog (Shift+S) and look for "AirfoilFitterFusionAddIn" in the list. Verify it is running.
5.  Check the Text output for error messages.
