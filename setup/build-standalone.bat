@echo off
REM Build installer without bundled libraries (requires local Python dependencies)
echo Building standalone installer (without libraries)...
python generate_wxs_fragment.py --exclude-lib --output Files-Standalone.wxs
if errorlevel 1 exit /b 1

wix build AirfoilFitterFusionAddIn-Standalone.wxs Files-Standalone.wxs -ext WixToolset.UI.wixext -ext WixToolset.Util.wixext -o AirfoilFitterFusionAddInSetup-Standalone.msi
if errorlevel 1 exit /b 1

echo.
echo Standalone installer built successfully: AirfoilFitterFusionAddInSetup-Standalone.msi
