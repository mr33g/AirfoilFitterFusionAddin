@echo off
REM Build installer with bundled libraries
REM This script creates a staging directory, installs dependencies there, and builds from staging
echo Building bundled installer (with libraries)...
setlocal enabledelayedexpansion

REM Get the directory where this script is located (setup folder)
set "SETUP_DIR=%~dp0"
set "SETUP_DIR=%SETUP_DIR:~0,-1%"

REM Get the parent directory (project root)
set "PROJECT_ROOT=%SETUP_DIR%\.."
cd /d "%PROJECT_ROOT%"

REM Create temporary staging directory
set "STAGING_DIR=%TEMP%\AirfoilFitterAddin-staging-%RANDOM%"
echo Creating staging directory: %STAGING_DIR%
if exist "%STAGING_DIR%" rmdir /s /q "%STAGING_DIR%"
mkdir "%STAGING_DIR%"

REM Copy source files to staging (excluding lib, setup, .git, etc.)
echo Copying source files to staging...
python "%SETUP_DIR%\stage_for_build.py" "%PROJECT_ROOT%" "%STAGING_DIR%"
if errorlevel 1 (
    echo ERROR: Failed to copy source files to staging
    rmdir /s /q "%STAGING_DIR%"
    exit /b 1
)

REM Create lib directory in staging
mkdir "%STAGING_DIR%\lib" 2>nul

REM Install dependencies to staging/lib
echo Installing Python dependencies to staging/lib...
python -m pip install --target "%STAGING_DIR%\lib" -r "%PROJECT_ROOT%\requirements.txt" --quiet --no-warn-script-location
if errorlevel 1 (
    echo ERROR: Failed to install dependencies
    rmdir /s /q "%STAGING_DIR%"
    exit /b 1
)

REM Generate WiX fragment from staging directory
echo Generating WiX fragment from staging directory...
cd /d "%SETUP_DIR%"
python generate_wxs_fragment.py --source-dir "%STAGING_DIR%" --output Files-bundled.wxs
if errorlevel 1 (
    echo ERROR: Failed to generate WiX fragment
    rmdir /s /q "%STAGING_DIR%"
    exit /b 1
)

REM Build MSI installer
echo Building MSI installer...
wix build AirfoilFitterAddin.wxs Files-bundled.wxs -ext WixToolset.UI.wixext -ext WixToolset.Util.wixext -o AirfoilFitterAddin-bundled.msi
set BUILD_RESULT=!errorlevel!

REM Clean up staging directory
echo Cleaning up staging directory...
rmdir /s /q "%STAGING_DIR%"

if !BUILD_RESULT! neq 0 (
    echo ERROR: Failed to build MSI installer
    exit /b 1
)

echo.
echo Bundled installer built successfully: AirfoilFitterAddin-bundled.msi
