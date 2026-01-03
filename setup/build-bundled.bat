@echo off
REM Build installer with bundled libraries
echo Building bundled installer (with libraries)...
python generate_wxs_fragment.py --output Files.wxs
if errorlevel 1 exit /b 1

wix build AirfoilFitterFusionAddIn.wxs Files.wxs -ext WixToolset.UI.wixext -ext WixToolset.Util.wixext -o AirfoilFitterFusionAddInSetup-Bundled.msi
if errorlevel 1 exit /b 1

echo.
echo Bundled installer built successfully: AirfoilFitterFusionAddInSetup-Bundled.msi
