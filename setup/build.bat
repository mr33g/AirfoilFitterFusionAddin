@echo off
REM Build installer without bundled libraries (requires local Python dependencies)
echo Building clean installer (without libraries)...
python generate_wxs_fragment.py --exclude-lib --output Files.wxs
if errorlevel 1 exit /b 1

wix build AirfoilFitterFusion.wxs Files.wxs -ext WixToolset.UI.wixext -ext WixToolset.Util.wixext -o AirfoilFitterFusion.msi
if errorlevel 1 exit /b 1

echo.
echo Installer built successfully: AirfoilFitterFusion.msi
