@echo on
rem AssettoServer v0.0.54 - win-x86 build (Steam disabled, 32-bit csp_xxhash3.dll)
cd /d "%~dp0"

echo [1/3] Checking .NET SDK (8.0 or newer needed)...
dotnet --list-sdks
if errorlevel 1 (
  echo .NET SDK not found. Install the .NET 8 SDK from https://dotnet.microsoft.com/download/dotnet/8.0
  goto end
)

echo [2/3] Publishing for win-x86 (Release, self-contained)...
dotnet publish --runtime win-x86 -c Release
if errorlevel 1 (
  echo BUILD FAILED - see the messages above.
  goto end
)

echo [3/3] Done. Output folder:
echo %~dp0out-win-x86
dir "%~dp0out-win-x86"

:end
pause
