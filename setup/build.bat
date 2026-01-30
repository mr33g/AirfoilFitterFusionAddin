@echo off
REM Build installer without bundled libraries (requires local Python dependencies)
REM Usage: build.bat [version]
REM   version: Optional version number (e.g., 1.2.3). Defaults to 1.1.0 if not specified.

set VERSION=%~1
if "%VERSION%"=="" (
    echo Building clean installer with default version...
    set VERSION_ARG=
) else (
    echo Building clean installer version %VERSION%...
    set VERSION_ARG=-d Version=%VERSION%
)

python generate_wxs_fragment.py --exclude-lib --output Files.wxs
if errorlevel 1 exit /b 1

wix build AirfoilFitterAddin.wxs Files.wxs -ext WixToolset.UI.wixext -ext WixToolset.Util.wixext %VERSION_ARG% -o AirfoilFitterAddin.msi
if errorlevel 1 exit /b 1

echo.
echo Installer built successfully: AirfoilFitterAddin.msi
