@echo off
REM Build installer without bundled libraries (requires local Python dependencies)
REM Usage: build.bat [version]
REM   version: Optional version number (e.g., 1.2.3). Defaults to 1.1.0 if not specified.

setlocal
set DEFAULT_VERSION=1.1.0
set BUILD_RESULT=0
set VERSION=%~1
if "%VERSION%"=="" (
    echo Building clean installer with default version...
    set EFFECTIVE_VERSION=%DEFAULT_VERSION%
    set VERSION_ARG=
) else (
    echo Building clean installer version %VERSION%...
    set EFFECTIVE_VERSION=%VERSION%
    set VERSION_ARG=-d Version=%VERSION%
)

set GENERATED_PACKAGE_XML=PackageContents.generated.xml
set PACKAGE_XML_ARG=-d PackageContentsSource=%GENERATED_PACKAGE_XML%

python update_packagecontents_version.py --input PackageContents.xml --output "%GENERATED_PACKAGE_XML%" --version "%EFFECTIVE_VERSION%"
if errorlevel 1 (
    set BUILD_RESULT=1
    goto :cleanup
)

python generate_wxs_fragment.py --exclude-lib --output Files.wxs
if errorlevel 1 (
    set BUILD_RESULT=1
    goto :cleanup
)

wix build AirfoilFitterAddin.wxs Files.wxs -ext WixToolset.UI.wixext -ext WixToolset.Util.wixext %VERSION_ARG% %PACKAGE_XML_ARG% -o AirfoilFitterAddin.msi
if errorlevel 1 (
    set BUILD_RESULT=1
    goto :cleanup
)

echo.
echo Installer built successfully: AirfoilFitterAddin.msi

:cleanup
if exist "%GENERATED_PACKAGE_XML%" del /q "%GENERATED_PACKAGE_XML%"
exit /b %BUILD_RESULT%
