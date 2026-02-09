from __future__ import annotations

import adsk.core

_LANG_CACHE = None

STRINGS = {
    "en": {
        "deps_missing_msg": (
            "AirfoilFitter requires external libraries (numpy, scipy, ezdxf).\n\n"
            "They were not found in the 'lib' folder.\n"
            "Would you like to attempt a local installation into the add-in folder?"
        ),
        "deps_missing_title": "Dependencies Missing",
        "deps_install_started": (
            "Installation has started in a separate window.\n\n"
            "Please wait for it to complete, then restart Fusion."
        ),
        "deps_install_complete": "Installation complete. Please restart Fusion.",
        "deps_install_failed": (
            "Resilient installation failed: {error}\n\n"
            "Please manually install dependencies."
        ),
        "cmd_button_name": "Insert fitted Airfoil",
        "cmd_button_desc": "Fit a spline to an airfoil .dat file in Selig or Lednicer format",
        "failed_to_start": "Failed to start:\n{error}",
        "select_chord_line": "Chord Line",
        "select_chord_line_help": "Select a construction line",
        "select_airfoil": "Input File",
        "selected_file": "Selected File",
        "fitter_settings": "Fitter Settings",
        "cp_count_upper": "Points Upper",
        "cp_count_lower": "Points Lower",
        "reset": "Reset",
        "smoothness_penalty": "Smoothness",
        "le_continuity": "Continuity",
        "enforce_te_tangency": "Trailing Edge Tangency",
        "import_settings": "Import Settings",
        "turn_90": "Turn 90\u00b0",
        "flip": "Flip",
        "te_thickness": "TE Thickness",
        "keep_adjustable": "Keep adjustable",
        "curvature_comb": "Curvature comb",
        "comb_scale": "Comb Scale",
        "comb_density": "Comb Density",
        "show_input_data": "Show Input Data",
        "ui_creation_failed": "UI Creation Failed:\n{error}",
        "command_created_failed": "Command Created Failed:\n{error}",
        "execution_error": "Execution Error:\n{error}",
        "file_filter": "Airfoil (*.dat);;All Files (*.*)",
        "failed_load_airfoil_data": "Failed to load airfoil data, please check the file path and try again.",
        "failed_fit_airfoil": "Failed to fit airfoil, please check the input parameters and try again.",
        "failed_insert_knot": "Failed to insert knot on {surface} surface.",
        "failed_refit_surface": "Failed to re-fit {surface} surface with reduced control points.",
        "generic_error": "An error occurred, please check the log for more details.",
    },
    "de": {
        "deps_missing_msg": (
            "AirfoilFitter ben\u00f6tigt externe Bibliotheken (numpy, scipy, ezdxf).\n\n"
            "Diese wurden im Ordner 'lib' nicht gefunden.\n"
            "M\u00f6chten Sie eine lokale Installation im Add-in-Ordner versuchen?"
        ),
        "deps_missing_title": "Abh\u00e4ngigkeiten fehlen",
        "deps_install_started": (
            "Die Installation wurde in einem separaten Fenster gestartet.\n\n"
            "Bitte warten Sie, bis sie abgeschlossen ist, und starten Sie Fusion dann neu."
        ),
        "deps_install_complete": "Installation abgeschlossen. Bitte Fusion neu starten.",
        "deps_install_failed": (
            "Fehlgeschlagene Installation: {error}\n\n"
            "Bitte installieren Sie die Abh\u00e4ngigkeiten manuell."
        ),
        "cmd_button_name": "Angepasstes Profil einf\u00fcgen",
        "cmd_button_desc": "Spline an eine .dat-Profildatei im Selig- oder Lednicer-Format anpassen",
        "failed_to_start": "Start fehlgeschlagen:\n{error}",
        "select_chord_line": "Sehnenlinie ausw\u00e4hlen",
        "select_chord_line_help": "W\u00e4hlen Sie eine Linie aus",
        "select_airfoil": "Datei ausw\u00e4hlen",
        "selected_file": "Ausgew\u00e4hlte Datei",
        "fitter_settings": "Fitter Einstellungen",
        "cp_count_upper": "Punkte Oberseite",
        "cp_count_lower": "Punkte Unterseite",
        "reset": "Zur\u00fccksetzen",
        "smoothness_penalty": "Gl\u00e4ttung",
        "le_continuity": "Kontinuit\u00e4t",
        "enforce_te_tangency": "Tangentialit\u00e4t Endleiste",
        "import_settings": "Import Einstellungen",
        "turn_90": "Um 90\u00b0 drehen",
        "flip": "Spiegeln",
        "te_thickness": "Dicke Endleiste",
        "keep_adjustable": "Bearbeitbar",
        "curvature_comb": "Kurvenkamm",
        "comb_scale": "Kamm-Skalierung",
        "comb_density": "Kamm-Dichte",
        "show_input_data": "Rohdaten anzeigen",
        "ui_creation_failed": "UI-Erstellung fehlgeschlagen:\n{error}",
        "command_created_failed": "Befehlserstellung fehlgeschlagen:\n{error}",
        "execution_error": "Ausf\u00fchrungsfehler:\n{error}",
        "file_filter": "Profil (*.dat);;Alle Dateien (*.*)",
        "failed_load_airfoil_data": "Profildaten konnten nicht geladen werden. Bitte Dateipfad pr\u00fcfen und erneut versuchen.",
        "failed_fit_airfoil": "Profilanpassung fehlgeschlagen. Bitte Eingabeparameter pr\u00fcfen und erneut versuchen.",
        "failed_insert_knot": "Knoten konnte auf der {surface}-Fl\u00e4che nicht eingef\u00fcgt werden.",
        "failed_refit_surface": "Neu-Anpassung der {surface}-Fl\u00e4che mit reduzierter Kontrollpunktzahl fehlgeschlagen.",
        "generic_error": "Ein Fehler ist aufgetreten. Bitte pr\u00fcfen Sie das Protokoll f\u00fcr Details.",
    },
}

def _enum_name_for_value(enum_type, value):
    try:
        for name in dir(enum_type):
            if name.startswith("_"):
                continue
            try:
                if getattr(enum_type, name) == value:
                    return name
            except Exception:
                continue
    except Exception:
        pass
    return None

def _extract_language_code(value) -> str | None:
    if value is None:
        return None
    # Handle numeric language enums.
    if isinstance(value, int):
        try:
            enum_type = getattr(adsk.core, "UserLanguages", None)
            if enum_type:
                enum_name = _enum_name_for_value(enum_type, value)
                if enum_name:
                    value = enum_name
        except Exception:
            pass
    text = str(value).strip().lower()
    if not text:
        return None
    if text.startswith("de") or "german" in text or "deutsch" in text:
        return "de"
    if text.startswith("en") or "english" in text:
        return "en"
    return None

def _get_fusion_language_code() -> str | None:
    try:
        app = adsk.core.Application.get()
    except Exception:
        return None

    # Prefer the documented preferences.generalPreferences.userLanguage.
    candidates = []
    try:
        prefs = getattr(app, "preferences", None)
        gp = getattr(prefs, "generalPreferences", None) if prefs else None
        if gp:
            candidates.append(getattr(gp, "userLanguage", None))
            candidates.extend([
                getattr(gp, "language", None),
                getattr(gp, "defaultLanguage", None),
                getattr(gp, "locale", None),
            ])
    except Exception:
        pass

    # Some builds expose a direct language on the app
    candidates.append(getattr(app, "language", None))

    for value in candidates:
        code = _extract_language_code(value)
        if code:
            return code
    return None

def get_language_code() -> str:
    global _LANG_CACHE
    if _LANG_CACHE:
        return _LANG_CACHE
    code = _get_fusion_language_code()
    _LANG_CACHE = code if code in STRINGS else "en"
    return _LANG_CACHE

def t(key: str, **kwargs) -> str:
    lang = get_language_code()
    table = STRINGS.get(lang, STRINGS["en"])
    text = table.get(key, STRINGS["en"].get(key, key))
    try:
        return text.format(**kwargs)
    except Exception:
        return text
