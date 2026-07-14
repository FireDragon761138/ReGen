@echo off
REM Build ReGen.dll (64-bit VST2) with MinGW-w64 g++.
REM Statically links libgcc/libstdc++ so the DLL has no external runtime deps.
REM
REM   build_mingw.bat        -> ReGen.dll       (generic x86-64, runs anywhere)
REM   build_mingw.bat avx2   -> ReGen-AVX2.dll  (x86-64-v3: AVX2+FMA, ~2013+ CPUs;
REM                             measured ~25%% faster on the filter chains)
setlocal
where g++ >nul 2>&1
if errorlevel 1 (
  echo g++ not on PATH. Add the WinLibs mingw64\bin folder to PATH, or run
  echo   set PATH=%%LOCALAPPDATA%%\Microsoft\WinGet\Packages\BrechtSanders.WinLibs.POSIX.UCRT_*\mingw64\bin;%%PATH%%
  exit /b 1
)
set OUT=ReGen.dll
set ARCH=
if /i "%~1"=="avx2" (
  set OUT=ReGen-AVX2.dll
  set ARCH=-march=x86-64-v3
)
g++ -O2 %ARCH% -shared -static -static-libgcc -static-libstdc++ ^
    -o %OUT% ReGen.cpp -lcomctl32 -lgdi32 -luser32
if errorlevel 1 (echo BUILD FAILED & exit /b 1)
echo Built %OUT%
g++ -O2 -o test_host.exe test_host.cpp
if errorlevel 1 (echo TEST HOST BUILD FAILED & exit /b 1)
echo Built test_host.exe
endlocal
