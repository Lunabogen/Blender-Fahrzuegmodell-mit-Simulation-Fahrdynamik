#=============================================================
# STEP 2 Solver
#=============================================================

from pathlib import Path
import pathlib
import pandas as pd
import json
import math

RAD_KEYS = ["VL", "VR", "HL", "HR"]

DEFAULT_PARAMETER = {
    "FAHRZEUGMASSE_KG": 1600.0,
    "GEFEDERTE_MASSE_ANTEIL": 0.85,
    "UNGEFEDERTEMASSE_PRO_RAD_KG": 45.0,

    "FEDERSTEIFIGKEIT_N_PRO_M": 30000.0,
    "DAEMPFERKONSTANTE_N_S_PRO_M": 3000.0,
    "REIFENSTEIFIGKEIT_N_PRO_M": 200000.0,

    "MAX_EINFEDERUNG_M": 0.12,
    "MAX_AUSFEDERUNG_M": 0.12,

    "LASTVERLAGERUNG_AKTIV": True,
    "LASTVERLAGERUNG_LAENGS_AKTIV": True,
    "LASTVERLAGERUNG_QUER_AKTIV": True,

    "SCHWERPUNKT_HOEHE_M": 0.55,
    "LASTVERLAGERUNG_LAENGS_VORZEICHEN": 1.0,
    "LASTVERLAGERUNG_QUER_VORZEICHEN": 1.0,
}



#json aufladen==============================================================
def lade_parameter(parameter_pfad : pathlib.Path) -> dict:
    parameter = DEFAULT_PARAMETER.copy()

    if parameter_pfad.exists():
        with parameter_pfad.open("r", encoding="utf-8") as json_file:
            geladene_parameter = json.load(json_file)
        
        parameter.update(geladene_parameter)
    
    else:
        print("vehicle_parameter.json not found. Using default parameters.")

    parameter["GEFEDERTEMASSE_PRO_RAD_KG"] = (
        parameter["FAHRZEUGMASSE_KG"]
        * parameter["GEFEDERTE_MASSE_ANTEIL"]
        / 4.0
    )

    return parameter

#CSV die Erreichbarkeit der Daten pruefen==============================================================
def pruefe_road_input_spalten(road_input_df : pd.DataFrame) -> None:
    basis_spalten = ["frame", "t_s", "dt_s"]

    for spalte in basis_spalten:
        if spalte not in road_input_df.columns:
            raise ValueError(
                "Fehlende Pflichtspalte in road_input.csv: " + spalte
            )
        
    geometrie_spalten = [
        "car_spurweite_mean_m", "car_radstand_mean_m",
    ]

    for rad_key in RAD_KEYS:
        geometrie_spalten.append("rad_offset_x_local_" + rad_key + "_m")
        geometrie_spalten.append("rad_offset_y_local_" + rad_key + "_m")

    for spalte in geometrie_spalten:
        if spalte not in road_input_df.columns:
            raise ValueError(
                "Fehlende Geometriespalte in road_input.csv: " + spalte
            )        
    
    for rad_key in RAD_KEYS:
        envelope_spalte = ("rad_envelope_required_max_z_rel_local_" + rad_key + "_m")
        center_spalte = ("rad_center_fahrbahn_z_rel_local_" + rad_key + "_m")

        if envelope_spalte not in road_input_df.columns and center_spalte not in road_input_df.columns:
            raise ValueError(
                "Keine Strassenanregung fuer " + rad_key + " in road_input.csv. Erwartet: "
                + envelope_spalte + " oder " + center_spalte
            )
        
#Strassenanregung aus CSV laden==============================================================
def lese_strassenanregung_z_rel_local_m(row: pd.Series, rad_key: str) -> float:
    envelope_spalte = ("rad_envelope_required_max_z_rel_local_" + rad_key + "_m")
    center_spalte = ("rad_center_fahrbahn_z_rel_local_" + rad_key + "_m")

    if envelope_spalte in row.index:
        return float(row[envelope_spalte])
    elif center_spalte in row.index:
        return float(row[center_spalte])
    else:
        return 0.0
    
#initialisierung=================================================================================    
def initialisiere_zustand() -> dict:
    zustand = {}

    for rad_key in RAD_KEYS:
        zustand[rad_key] = {
            "gefedertemasse_z_local_m": 0.0,
            "gefedertemasse_geschwindigkeit_local_mps": 0.0,
            "gefedertemasse_beschleunigung_local_mps2": 0.0,

            "ungefedertemasse_z_local_m": 0.0,
            "ungefedertemasse_geschwindigkeit_local_mps": 0.0,
            "ungefedertemasse_beschleunigung_local_mps2": 0.0,
        }

    return zustand

#Kraftberechnung_ein_rad=================================================================================
def berechne_kraefte_einer_rad_ecke(
        zustand_rad: dict,
        strassenanregung_z_rel_local_m: float,
        parameter: dict, zusatzkraft_lastverlagerung_N: float,
    ) -> dict:
    
    reifen_eindruckung_m = (
        strassenanregung_z_rel_local_m-zustand_rad["ungefedertemasse_z_local_m"])
    reifen_kraft_N = parameter["REIFENSTEIFIGKEIT_N_PRO_M"] * reifen_eindruckung_m
    
    feder_kraft_N = parameter["FEDERSTEIFIGKEIT_N_PRO_M"] * (zustand_rad["ungefedertemasse_z_local_m"] - 
                                                             zustand_rad["gefedertemasse_z_local_m"])
    
    daempfer_kraft_N = parameter["DAEMPFERKONSTANTE_N_S_PRO_M"] * (zustand_rad["ungefedertemasse_geschwindigkeit_local_mps"] - 
                                                                   zustand_rad["gefedertemasse_geschwindigkeit_local_mps"])
    feder_daempfer_kraft_N = feder_kraft_N + daempfer_kraft_N

    gefedertemasse_beschleunigung_mps2 = (feder_daempfer_kraft_N + zusatzkraft_lastverlagerung_N) / (
    parameter["GEFEDERTEMASSE_PRO_RAD_KG"])
    ungefederte_masse_beschleunigung_mps2 = (reifen_kraft_N - feder_daempfer_kraft_N) / (
    parameter["UNGEFEDERTEMASSE_PRO_RAD_KG"])

    return {
        "reifen_eindrueckung_m": reifen_eindruckung_m,
        "reifen_kraft_N": reifen_kraft_N,
        "feder_kraft_N": feder_kraft_N,
        "daempfer_kraft_N": daempfer_kraft_N,
        "feder_daempfer_kraft_N": feder_daempfer_kraft_N,
        "gefedertemasse_beschleunigung_mps2": gefedertemasse_beschleunigung_mps2,
        "ungefedertemasse_beschleunigung_mps2": ungefederte_masse_beschleunigung_mps2,}

#kraft(Beschleunigung)_zur_Geschiwindigkeit=================================================================================
def integriere_eine_rad_ecke(
        zustand_rad: dict,
        kraefte: dict,
        dt_s: float,) -> None:
    zustand_rad["gefedertemasse_beschleunigung_local_mps2"] = kraefte["gefedertemasse_beschleunigung_mps2"]
    zustand_rad["ungefedertemasse_beschleunigung_local_mps2"] = kraefte["ungefedertemasse_beschleunigung_mps2"]

    zustand_rad["gefedertemasse_geschwindigkeit_local_mps"] += (
        zustand_rad["gefedertemasse_beschleunigung_local_mps2"] * dt_s
    )
    zustand_rad["ungefedertemasse_geschwindigkeit_local_mps"] += (
        zustand_rad["ungefedertemasse_beschleunigung_local_mps2"] * dt_s
    )

    zustand_rad["gefedertemasse_z_local_m"] += (
        zustand_rad["gefedertemasse_geschwindigkeit_local_mps"] * dt_s
    )
    zustand_rad["ungefedertemasse_z_local_m"] += (
        zustand_rad["ungefedertemasse_geschwindigkeit_local_mps"] * dt_s
    )

#Federwegebegrenzung=================================================================================
def clamp(wert: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(wert, maximum))

#Karo_rekonstruktion=================================================================================
def rekonstruiere_karosserie(
        zustand: dict,
        car_spurweite_mean_m: float,
        car_radstand_mean_m: float,
) -> tuple:
    geferderte_masse_z_local_VL = zustand["VL"]["gefedertemasse_z_local_m"]
    geferderte_masse_z_local_VR = zustand["VR"]["gefedertemasse_z_local_m"]
    geferderte_masse_z_local_HL = zustand["HL"]["gefedertemasse_z_local_m"]
    geferderte_masse_z_local_HR = zustand["HR"]["gefedertemasse_z_local_m"]

    karosserie_hub_local_m = (
        geferderte_masse_z_local_VL
        + geferderte_masse_z_local_VR
        + geferderte_masse_z_local_HL
        + geferderte_masse_z_local_HR
    ) / 4.0

    vorderachse_geferderte_masse_z_local_m = (
        geferderte_masse_z_local_VL + geferderte_masse_z_local_VR
    ) / 2.0

    hinterachse_geferderte_masse_z_local_m = (
        geferderte_masse_z_local_HL + geferderte_masse_z_local_HR
    ) / 2.0 

    karosserie_nicken_rad = math.atan2(
        vorderachse_geferderte_masse_z_local_m - hinterachse_geferderte_masse_z_local_m,
        car_radstand_mean_m,
    )

    linke_seite_geferderte_masse_z_local_m = (
        geferderte_masse_z_local_VL + geferderte_masse_z_local_HL
    ) / 2.0

    rechte_seite_geferderte_masse_z_local_m = (
        geferderte_masse_z_local_VR + geferderte_masse_z_local_HR
    ) / 2.0

    karosserie_rollen_rad = math.atan2(
        linke_seite_geferderte_masse_z_local_m - rechte_seite_geferderte_masse_z_local_m,
        car_spurweite_mean_m,
    )

    return (karosserie_hub_local_m, karosserie_nicken_rad, karosserie_rollen_rad)




#CSV_Schreiben=================================================================================
def schreibe_vehicle_response_csv(
        output_pfad: Path,
        output_rows: list,
) -> None:
    vehicle_response_df = pd.DataFrame(output_rows)
    vehicle_response_df.to_csv(output_pfad, index=False, encoding="utf-8")

    print("============================================================")
    print("vehicle_response.csv geschrieben:")
    print(output_pfad)
    print("Anzahl Output-Zeilen:", len(vehicle_response_df))
    print("Anzahl Output-Spalten:", len(vehicle_response_df.columns))
    print("============================================================")


#Durchgehen_aller_Frames=================================================================================
def berechne_vehicle_response_rows(
    road_input_df: pd.DataFrame,
    parameter: dict,
) -> list:
    zustand = initialisiere_zustand()

    output_rows = []

    print("============================================================")
    print("Viertelfahrzeugmodell Solver - Berechnung ueber alle Frames")
    print("============================================================")

    for index, row in road_input_df.iterrows():
        frame = int(row["frame"])
        t_s = float(row["t_s"])
        dt_s = float(row["dt_s"])

        if pd.isna(dt_s) or dt_s < 0.0:
            dt_s = 0.0

        output_row = {
            "frame": frame,
            "t_s": t_s,
            "dt_s": dt_s,
        }

        # ------------------------------------------------------------
        # 1. Vier Rad-Ecken berechnen und Zustand integrieren
        # ------------------------------------------------------------

        for rad_key in RAD_KEYS:
            strassenanregung_z_rel_local_m = (
                lese_strassenanregung_z_rel_local_m(
                    row=row,
                    rad_key=rad_key,
                )
            )

            kraefte = berechne_kraefte_einer_rad_ecke(
                zustand_rad=zustand[rad_key],
                strassenanregung_z_rel_local_m=strassenanregung_z_rel_local_m,
                parameter=parameter,
                zusatzkraft_lastverlagerung_N=0.0,
            )

            integriere_eine_rad_ecke(
                zustand_rad=zustand[rad_key],
                kraefte=kraefte,
                dt_s=dt_s,
            )

            output_row[
                "strassenanregung_z_rel_local_" + rad_key + "_m"
            ] = strassenanregung_z_rel_local_m

            output_row[
                "gefedertemasse_z_local_" + rad_key + "_m"
            ] = zustand[rad_key]["gefedertemasse_z_local_m"]

            output_row[
                "gefedertemasse_geschwindigkeit_local_" + rad_key + "_mps"
            ] = zustand[rad_key]["gefedertemasse_geschwindigkeit_local_mps"]

            output_row[
                "gefedertemasse_beschleunigung_local_" + rad_key + "_mps2"
            ] = zustand[rad_key]["gefedertemasse_beschleunigung_local_mps2"]

            output_row[
                "ungefedertemasse_z_local_" + rad_key + "_m"
            ] = zustand[rad_key]["ungefedertemasse_z_local_m"]

            output_row[
                "ungefedertemasse_geschwindigkeit_local_" + rad_key + "_mps"
            ] = zustand[rad_key]["ungefedertemasse_geschwindigkeit_local_mps"]

            output_row[
                "ungefedertemasse_beschleunigung_local_" + rad_key + "_mps2"
            ] = zustand[rad_key]["ungefedertemasse_beschleunigung_local_mps2"]

            output_row[
                "reifen_eindrueckung_" + rad_key + "_m"
            ] = kraefte["reifen_eindrueckung_m"]

            output_row[
                "reifen_kraft_" + rad_key + "_N"
            ] = kraefte["reifen_kraft_N"]

            output_row[
                "feder_kraft_" + rad_key + "_N"
            ] = kraefte["feder_kraft_N"]

            output_row[
                "daempfer_kraft_" + rad_key + "_N"
            ] = kraefte["daempfer_kraft_N"]

            output_row[
                "feder_daempfer_kraft_" + rad_key + "_N"
            ] = kraefte["feder_daempfer_kraft_N"]

        # ------------------------------------------------------------
        # 2. Karosserie aus vier gefedertemasse-Zustaenden rekonstruieren
        # ------------------------------------------------------------

        car_radstand_mean_m = float(row["car_radstand_mean_m"])
        car_spurweite_mean_m = float(row["car_spurweite_mean_m"])

        (
            karosserie_hub_local_m,
            karosserie_nicken_rad,
            karosserie_rollen_rad,
        ) = rekonstruiere_karosserie(
            zustand=zustand,
            car_radstand_mean_m=car_radstand_mean_m,
            car_spurweite_mean_m=car_spurweite_mean_m,
        )

        output_row["karosserie_hub_local_m"] = karosserie_hub_local_m
        output_row["karosserie_nicken_rad"] = karosserie_nicken_rad
        output_row["karosserie_rollen_rad"] = karosserie_rollen_rad

        # ------------------------------------------------------------
        # 3. Karosserie-Hoehe am Rad und Federweg fuer Baker berechnen
        # ------------------------------------------------------------

        for rad_key in RAD_KEYS:
            rad_offset_x_local_m = float(
                row["rad_offset_x_local_" + rad_key + "_m"]
            )

            rad_offset_y_local_m = float(
                row["rad_offset_y_local_" + rad_key + "_m"]
            )

            karosserie_z_am_rad_m = (
                karosserie_hub_local_m
                + karosserie_nicken_rad * rad_offset_y_local_m
                - karosserie_rollen_rad * rad_offset_x_local_m
            )

            rad_federweg_raw_z_local_m = (
                zustand[rad_key]["ungefedertemasse_z_local_m"]
                - karosserie_z_am_rad_m
            )

            rad_federweg_z_local_m = clamp(
                rad_federweg_raw_z_local_m,
                -parameter["MAX_AUSFEDERUNG_M"],
                parameter["MAX_EINFEDERUNG_M"],
            )

            output_row[
                "karosserie_z_am_rad_" + rad_key + "_m"
            ] = karosserie_z_am_rad_m

            output_row[
                "rad_federweg_raw_z_local_" + rad_key + "_m"
            ] = rad_federweg_raw_z_local_m

            output_row[
                "rad_federweg_z_local_" + rad_key + "_m"
            ] = rad_federweg_z_local_m

        output_rows.append(output_row)

        if index % 50 == 0:
            print("Frame berechnet:", frame)

    print("============================================================")
    print("FERTIG: Berechnung ueber alle Frames abgeschlossen.")
    print("============================================================")

    return output_rows




    
#Hauptpruefung===============================================================================
def main() -> None:
    projekt_ordner = Path(__file__).resolve().parent

    road_input_pfad = projekt_ordner / "road_input.csv"
    parameter_pfad = projekt_ordner / "vehicle_parameter.json"

    print("============================================================")
    print("Viertelfahrzeugmodell Solver - Eingangsdaten Check")
    print("============================================================")

    print("Projektordner:")
    print(projekt_ordner)

    print("road_input.csv:")
    print(road_input_pfad)

    print("vehicle_parameter.json:")
    print(parameter_pfad)

    parameter = lade_parameter(parameter_pfad)
    zustand = initialisiere_zustand()

    print("------------------------------------------------------------")
    print("Geladene Parameter:")
    print("FAHRZEUGMASSE_KG:", parameter["FAHRZEUGMASSE_KG"])
    print("GEFEDERTE_MASSE_ANTEIL:", parameter["GEFEDERTE_MASSE_ANTEIL"])
    print("GEFEDERTEMASSE_PRO_RAD_KG:", parameter["GEFEDERTEMASSE_PRO_RAD_KG"])
    print("UNGEFEDERTEMASSE_PRO_RAD_KG:", parameter["UNGEFEDERTEMASSE_PRO_RAD_KG"])
    print("FEDERSTEIFIGKEIT_N_PRO_M:", parameter["FEDERSTEIFIGKEIT_N_PRO_M"])
    print("DAEMPFERKONSTANTE_N_S_PRO_M:", parameter["DAEMPFERKONSTANTE_N_S_PRO_M"])
    print("REIFENSTEIFIGKEIT_N_PRO_M:", parameter["REIFENSTEIFIGKEIT_N_PRO_M"])
    print("------------------------------------------------------------")
    print("Initialer Solver-Zustand:")

    for rad_key in RAD_KEYS:
        print(
            rad_key,
            "gefedertemasse_z_local_m =",
            zustand[rad_key]["gefedertemasse_z_local_m"],
            "| ungefederte Masse z =",
            zustand[rad_key]["ungefedertemasse_z_local_m"],
        )

    if not road_input_pfad.exists():
        raise FileNotFoundError(
            "road_input.csv nicht gefunden: "
            + str(road_input_pfad)
        )

    road_input_df = pd.read_csv(road_input_pfad)

    pruefe_road_input_spalten(road_input_df)

    print("------------------------------------------------------------")
    print("road_input.csv gelesen.")
    print("Anzahl Frames:", len(road_input_df))
    print("Anzahl Spalten:", len(road_input_df.columns))

    erste_row = road_input_df.iloc[0]

    print("------------------------------------------------------------")
    print("Erste Frame Kontrolle:")
    print("frame:", int(erste_row["frame"]))
    print("t_s:", float(erste_row["t_s"]))
    print("dt_s:", float(erste_row["dt_s"]))

    print("car_spurweite_mean_m:", float(erste_row["car_spurweite_mean_m"]))
    print("car_radstand_mean_m:", float(erste_row["car_radstand_mean_m"]))

    for rad_key in RAD_KEYS:
        strassenanregung_z_rel_local_m = (
            lese_strassenanregung_z_rel_local_m(
                row=erste_row,
                rad_key=rad_key,
            )
        )

        print(
            "strassenanregung_z_rel_local_"
            + rad_key
            + "_m:",
            strassenanregung_z_rel_local_m,
        )

        kraefte = berechne_kraefte_einer_rad_ecke(
        zustand_rad=zustand[rad_key],
        strassenanregung_z_rel_local_m=strassenanregung_z_rel_local_m,
        parameter=parameter,
        zusatzkraft_lastverlagerung_N=0.0,
        )

        integriere_eine_rad_ecke(
            zustand_rad=zustand[rad_key],
            kraefte=kraefte,
            dt_s=float(erste_row["dt_s"]),
        )


        print(
            "  reifen_eindrueckung_m =",
            kraefte["reifen_eindrueckung_m"],
            "| reifen_kraft_N =",
            kraefte["reifen_kraft_N"],
            "| feder_daempfer_kraft_N =",
            kraefte["feder_daempfer_kraft_N"],
            "nach Integration: gefedertemasse_z_local_m =",
            zustand[rad_key]["gefedertemasse_z_local_m"],
            "| ungefederte Masse z =",
            zustand[rad_key]["ungefedertemasse_z_local_m"],
        )

    print("============================================================")
    print("FERTIG: Eingangsdaten sehen grundsaetzlich OK aus.")
    print("============================================================")

    output_pfad = projekt_ordner / "vehicle_response.csv"

    output_rows = berechne_vehicle_response_rows(
        road_input_df=road_input_df,
        parameter=parameter,
    )

    schreibe_vehicle_response_csv(
        output_rows=output_rows,
        output_pfad=output_pfad,
    )

if __name__ == "__main__":
    main()



