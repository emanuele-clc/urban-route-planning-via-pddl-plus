@echo off
echo ============================================
echo   Setup — Map Construction in PDDL+
echo ============================================
echo.

:: Controlla Python
python --version >nul 2>&1
if errorlevel 1 (
    echo [ERRORE] Python non trovato.
    echo Scaricalo da: https://www.python.org/downloads/
    pause
    exit /b 1
)

:: Controlla Java
java -version >nul 2>&1
if errorlevel 1 (
    echo [ERRORE] Java non trovato.
    echo Scaricalo da: https://www.java.com/it/download/
    pause
    exit /b 1
)

echo [OK] Python e Java trovati.
echo.
echo Installazione dipendenze Python...
python -m pip install --upgrade pip setuptools >nul 2>&1
python -m pip install -r requirements.txt
echo.
echo ============================================
echo   Setup completato!
echo.
echo   Per risolvere il problema PDDL+:
echo     cd files\pddl_files
echo     python run.py piccola
echo     python run.py media
echo     python run.py grande
echo ============================================
pause
