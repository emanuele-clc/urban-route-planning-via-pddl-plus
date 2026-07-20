#!/usr/bin/env python3
"""
Setup automatico per macOS — crea il virtualenv Python, installa le
dipendenze Python (requirements.txt) e i programmi esterni
necessari (Homebrew, Java 17+, SUMO) per
urban-route-planning-via-pddl-plus.

Uso:
    python3 setup_macos.py
"""
import os
import shutil
import subprocess
import sys
import venv
from pathlib import Path

BASE = Path(__file__).resolve().parent
VENV_DIR = BASE / ".venv"
REQUIREMENTS = BASE / "requirements.txt"
LOG_FILE = BASE / "setup.log"

MIN_PYTHON = (3, 9)


def log(msg):
    print(msg)
    with open(LOG_FILE, "a", encoding="utf-8") as f:
        f.write(msg + "\n")


def run(cmd, **kwargs):
    log(f"$ {' '.join(str(c) for c in cmd)}")
    return subprocess.run(cmd, check=True, **kwargs)


def check_macos():
    if sys.platform != "darwin":
        sys.exit("[ERRORE] Questo script è pensato per macOS (Darwin).")
    log(f"[OK] Piattaforma: {sys.platform} (kernel {os.uname().release})")


def check_python_version():
    if sys.version_info < MIN_PYTHON:
        sys.exit(f"[ERRORE] Serve Python >= {'.'.join(map(str, MIN_PYTHON))}, "
                  f"trovato {sys.version.split()[0]}")
    log(f"[OK] Python {sys.version.split()[0]}")


def ensure_homebrew():
    brew = shutil.which("brew")
    if brew:
        log(f"[OK] Homebrew trovato: {brew}")
        return brew

    log("[INFO] Homebrew non trovato, installazione in corso...")
    install_cmd = (
        '/bin/bash -c "$(curl -fsSL '
        'https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"'
    )
    subprocess.run(install_cmd, shell=True, check=True)

    # Homebrew si installa in /opt/homebrew su Apple Silicon, /usr/local su Intel.
    for candidate in ("/opt/homebrew/bin/brew", "/usr/local/bin/brew"):
        if os.path.exists(candidate):
            log(f"[OK] Homebrew installato: {candidate}")
            return candidate

    sys.exit("[ERRORE] Installazione di Homebrew fallita. Installalo manualmente: https://brew.sh")


def brew_prefix(brew, formula):
    out = subprocess.run([brew, "--prefix", formula], capture_output=True, text=True)
    return out.stdout.strip() if out.returncode == 0 else ""


def ensure_java(brew):
    java = shutil.which("java")
    if java:
        out = subprocess.run([java, "-version"], capture_output=True, text=True).stderr
        log(f"[OK] Java trovato: {out.splitlines()[0] if out else java}")
        return

    log("[INFO] Java non trovato, installo OpenJDK 17 via Homebrew...")
    run([brew, "install", "openjdk@17"])

    prefix = brew_prefix(brew, "openjdk@17")
    if prefix:
        target = Path("/Library/Java/JavaVirtualMachines/openjdk-17.jdk")
        if not target.exists():
            log(f"[INFO] Registro la JVM per il launcher di sistema (richiede password sudo)...")
            subprocess.run(
                ["sudo", "ln", "-sfn", f"{prefix}/libexec/openjdk.jdk", str(target)],
                check=False,
            )
        log("[OK] OpenJDK 17 installato.")
        log(f"[AZIONE CONSIGLIATA] Aggiungi al tuo ~/.zshrc:\n"
            f'    export PATH="{prefix}/bin:$PATH"')


def ensure_sumo(brew):
    sumo_bin = shutil.which("sumo") or shutil.which("sumo-gui")
    sumo_home = os.environ.get("SUMO_HOME")
    if sumo_bin and sumo_home:
        log(f"[OK] SUMO trovato: {sumo_bin} (SUMO_HOME={sumo_home})")
        return

    log("[INFO] SUMO non trovato (o SUMO_HOME non impostata), installo via Homebrew...")
    run([brew, "install", "sumo"])

    prefix = brew_prefix(brew, "sumo")
    if prefix:
        share_sumo = Path(prefix) / "share" / "sumo"
        sumo_home_val = str(share_sumo) if share_sumo.exists() else prefix
        log(f"[OK] SUMO installato in {prefix}.")
        log(f"[AZIONE RICHIESTA] Aggiungi al tuo ~/.zshrc:\n"
            f'    export SUMO_HOME="{sumo_home_val}"\n'
            f'    export PATH="{prefix}/bin:$PATH"')
    else:
        log("[ATTENZIONE] Impossibile determinare il prefix di installazione di SUMO.")


def create_venv():
    if VENV_DIR.exists():
        log(f"[OK] Virtualenv già presente: {VENV_DIR}")
        return
    log(f"[INFO] Creo virtualenv in {VENV_DIR}...")
    venv.EnvBuilder(with_pip=True, clear=False).create(str(VENV_DIR))
    log("[OK] Virtualenv creato.")


def venv_python():
    return str(VENV_DIR / "bin" / "python")


def install_python_deps():
    py = venv_python()
    log("[INFO] Aggiorno pip/setuptools/wheel nel virtualenv...")
    run([py, "-m", "pip", "install", "--upgrade", "pip", "setuptools", "wheel"])

    if REQUIREMENTS.exists():
        log(f"[INFO] Installo dipendenze da {REQUIREMENTS.name}...")
        run([py, "-m", "pip", "install", "-r", str(REQUIREMENTS)])
    else:
        log(f"[ATTENZIONE] {REQUIREMENTS} non trovato, salto.")


def main():
    open(LOG_FILE, "w", encoding="utf-8").close()
    log("==================================================")
    log("  Setup — Urban Route Planning via PDDL+ (macOS)")
    log("==================================================")

    check_macos()
    check_python_version()

    brew = ensure_homebrew()
    ensure_java(brew)
    ensure_sumo(brew)

    create_venv()
    install_python_deps()

    log("")
    log("==================================================")
    log("  Setup completato!")
    log("==================================================")
    log(f"Attiva il virtualenv con:\n    source {VENV_DIR}/bin/activate")
    log("")
    log("Per risolvere un problema PDDL+:")
    log("    cd pddl_files && python run.py piccola   # oppure media / grande")
    log("")
    log("Se compaiono le note [AZIONE RICHIESTA]/[AZIONE CONSIGLIATA] sopra,")
    log("aggiungile al tuo ~/.zshrc e apri un nuovo terminale prima di continuare.")


if __name__ == "__main__":
    try:
        main()
    except subprocess.CalledProcessError as e:
        sys.exit(f"[ERRORE] Comando fallito: {e}")
