# ============================================================
# ============================================================

# Road Input Exporter v1

# Export der Fahrtrajektorie und Strassenanregung aus Blender

# ============================================================

#

# Dieses Skript liest die vorhandene Blender-Animation aus und

# exportiert pro Frame strukturierte Simulationsdaten als CSV.
# Die Parameter werden als JSON exportiert.

#

# Referenzsystem:

# - "local" bedeutet in diesem Skript immer:

# Fahrzeug_Pfadsteuerung local.

#

# Systemaufteilung:

# - Fahrzeug_Pfadsteuerung:

# Input-Ebene / Fahrtrajektorie / Spur / Timing.

#

# - Fahrzeug_Hauptsteuerung:

# Output-Ebene / spaeteres Bake-Ziel.

#

# - Karosserie_Steuerung:

# visuelle Fahrzeugbewegung, z.B. Hub, Nicken, Rollen.

#

# Dieses Skript verwendet ausschliesslich Fahrzeug_Pfadsteuerung

# als Referenz fuer die exportierten lokalen Groessen.

#

# Exportierte Groessen:

#

# 1. Zeitbasis:

# frame

# t_s

# dt_s

#

# 2. Fahrtrajektorie:

# car_pos_x_world_m

# car_pos_y_world_m

# car_pos_z_world_m

# car_yaw_rad

# car_yaw_unwrapped_rad

#

# 3. Fahrzeug-Kinematik:

# car_vel_x_world_mps

# car_vel_y_world_mps

# car_vel_z_world_mps

# car_vel_abs_mps

#

# car_vel_forward_local_mps

# car_vel_lateral_local_mps

# car_vel_vertical_local_mps

#

# car_acc_x_world_mps2

# car_acc_y_world_mps2

# car_acc_z_world_mps2

#

# car_acc_forward_local_mps2

# car_acc_lateral_local_mps2

# car_acc_vertical_local_mps2

#

# car_yaw_rate_radps

# car_yaw_acc_radps2

#

# 4. Rad-center Road Input:

# rad_center_fahrbahn_z_local_*_m

# rad_center_fahrbahn_z0_local_*_m      (Z Wert des Hitpunktes von Rad Zentrum auf der Fahrbahn in frame 0, ohne Rad-Radius)

# rad_center_fahrbahn_z_rel_local_*_m

#

# * = VL, VR, HL, HR

#

# 5. Rad-envelope Road Input:

# rad_envelope_required_max_z_local_*_m

# rad_envelope_required_max_z0_local_*_m

# rad_envelope_required_max_z_rel_local_*_m

# rad_envelope_contact_offset_y_local_*_m

#

# 6. Raycast-Diagnose:

# rad_center_hit_*_bool

# rad_envelope_hit_count_*

#

# Wichtig:!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
# - Nach der aenderung der Fahrspur oder der Fahrbahn MUSS die Animation neu gespielt werden,
#  und MUSS diese Skript erneut ausgefuehrt werden, um die Road Input CSV zu aktualisieren.
# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

# - Dieses Skript bewegt keine Objekte dauerhaft.

# - Dieses Skript schreibt keine Keyframes.

# - Dieses Skript backt keine Simulation.

# - Dieses Skript veraendert keine Federung.

# - Waehrend des Exports wird die Szene frameweise ausgewertet.

# - Nach dem Export wird der urspruengliche Frame wiederhergestellt.

#

# Ergebnis:

# - road_input.csv im Ordner der .blend-Datei.

# ============================================================

# ============================================================

import bpy
import csv
import json
import math
from mathutils import Vector, Matrix
from pathlib import Path


# ============================================================
# 0. Object names
# ============================================================

CAR_OBJECT_NAME = "Fahrzeug_Pfadsteuerung"
FAHRBAHN_OBJECT_NAME = "Fahrbahn_Kollider"

RAD_OBJECT_NAMES = {
    "VL": "Rad_VorneLinks",
    "VR": "Rad_VorneRechts",
    "HL": "Rad_HintenLinks",
    "HR": "Rad_HintenRechts",
}

RAY_START_OFFSET_M = 2.0
RAY_DISTANCE_M = 5.0


RAD_ENVELOPE_OFFSET_RATIO_LIST = [
    -0.8,
    -0.4,
    0.0,
    0.4,
    0.8,
]

AUTO_RAD_RADIUS = True
RAD_RADIUS_MANUAL_M = 0.32

RAD_RADIUS_MIN_PLAUSIBLE_M = 0.10
RAD_RADIUS_MAX_PLAUSIBLE_M = 1.00

# ============================================================
# 1. Basic helper
# ============================================================

def get_obj(object_name):
    obj = bpy.data.objects.get(object_name)

    if obj is None:
        raise Exception("Objekt nicht gefunden: " + object_name)

    return obj


def collect_mesh_children(obj):
    mesh_obj_list = []

    if obj.type == "MESH":
        mesh_obj_list.append(obj)

    for child_obj in obj.children:
        mesh_obj_list.extend(
            collect_mesh_children(child_obj)
        )

    return mesh_obj_list


def raycast_fahrbahn_world(
    fahrbahn_eval,
    ray_start_world_vec,
    ray_dir_world_vec,
    ray_distance_m,
):
    fahrbahn_to_world_mat = fahrbahn_eval.matrix_world.copy()
    world_to_fahrbahn_mat = fahrbahn_eval.matrix_world.inverted()

    if ray_dir_world_vec.length == 0.0:
        raise Exception("Raycast-Richtung darf kein Nullvektor sein.")

    ray_dir_world_vec = ray_dir_world_vec.normalized()

    ray_end_world_vec = (
        ray_start_world_vec
        + ray_dir_world_vec * ray_distance_m
    )

    ray_start_fahrbahn_vec = (
        world_to_fahrbahn_mat @ ray_start_world_vec
    )

    ray_end_fahrbahn_vec = (
        world_to_fahrbahn_mat @ ray_end_world_vec
    )

    ray_vec_fahrbahn = (
        ray_end_fahrbahn_vec - ray_start_fahrbahn_vec
    )

    ray_distance_fahrbahn = ray_vec_fahrbahn.length

    if ray_distance_fahrbahn == 0.0:
        raise Exception(
            "Raycast-Laenge im Fahrbahn-Koordinatensystem ist null."
        )

    ray_dir_fahrbahn_vec = (
        ray_vec_fahrbahn / ray_distance_fahrbahn
    )

    hit_bool, hit_pos_fahrbahn_vec, hit_normal_fahrbahn_vec, hit_face_index = (
        fahrbahn_eval.ray_cast(
            ray_start_fahrbahn_vec,
            ray_dir_fahrbahn_vec,
            distance=ray_distance_fahrbahn,
        )
    )

    if hit_bool:
        hit_world_vec = fahrbahn_to_world_mat @ hit_pos_fahrbahn_vec

        hit_normal_world_vec = (
            world_to_fahrbahn_mat.to_3x3().transposed()
            @ hit_normal_fahrbahn_vec
        ).normalized()
    else:
        hit_world_vec = None
        hit_normal_world_vec = None

    return (
        hit_bool,
        hit_world_vec,
        hit_normal_world_vec,
        hit_face_index,
    )


# ============================================================
# 2. Remove old Raycast handlers
# ============================================================

def remove_old_raycast_handlers():
    old_handler_names = [
        "raycast_federweg_handler",
        "raycast_federweg_v2_handler",
        "raycast_federweg_v3_handler",
    ]

    removed_count = 0

    for handler in list(bpy.app.handlers.frame_change_post):
        handler_name = getattr(handler, "__name__", "")

        if handler_name in old_handler_names:
            bpy.app.handlers.frame_change_post.remove(handler)
            removed_count += 1
            print("Removed old handler:", handler_name)

    if removed_count == 0:
        print("Keine alten Raycast-Federweg-Handler gefunden.")
    else:
        print("Removed old handler count:", removed_count)


# ============================================================
# 3. Test Basic setup test
# ============================================================

def test_basic_setup():
    print("============================================================")
    print("Road Input Exporter - Basic Setup Check")
    print("============================================================")

    remove_old_raycast_handlers()

    car_obj = get_obj(CAR_OBJECT_NAME)
    fahrbahn_obj = get_obj(FAHRBAHN_OBJECT_NAME)

    print("car_obj gefunden:", car_obj.name)
    print("fahrbahn_obj gefunden:", fahrbahn_obj.name)

    if fahrbahn_obj.type != "MESH":
        raise Exception(
            "Fahrbahn_Kollider muss ein Mesh sein. Aktueller Typ: "
            + fahrbahn_obj.type
        )

    for rad_key, rad_object_name in RAD_OBJECT_NAMES.items():
        rad_obj = get_obj(rad_object_name)
        print("rad_obj gefunden:", rad_key, "=", rad_obj.name)

    print("============================================================")
    print("FERTIG: Basic Setup funktioniert.")
    print("============================================================")


# ============================================================
# 4. Test current frame car state
# ============================================================

def print_vec(vec_name, vec_value):
    print(
        vec_name + ":",
        "x =", round(vec_value.x, 4),
        "y =", round(vec_value.y, 4),
        "z =", round(vec_value.z, 4),
    )


def test_car_state_current_frame():
    print("============================================================")
    print("Road Input Exporter - Car State current Frame Check")
    print("============================================================")

    depsgraph = bpy.context.evaluated_depsgraph_get()

    car_obj = get_obj(CAR_OBJECT_NAME)
    fahrbahn_obj = get_obj(FAHRBAHN_OBJECT_NAME)

    car_eval = car_obj.evaluated_get(depsgraph)
    fahrbahn_eval = fahrbahn_obj.evaluated_get(depsgraph)

    car_local_to_world_mat = car_eval.matrix_world.copy()
    car_world_to_local_mat = car_local_to_world_mat.inverted()

    car_pos_world_vec = car_local_to_world_mat.translation

    car_forward_world_vec = (
        car_local_to_world_mat.to_3x3() @ Vector((0.0, 1.0, 0.0))
    ).normalized()

    car_right_world_vec = (
        car_local_to_world_mat.to_3x3() @ Vector((1.0, 0.0, 0.0))
    ).normalized()

    car_up_world_vec = (
        car_local_to_world_mat.to_3x3() @ Vector((0.0, 0.0, 1.0))
    ).normalized()

    car_yaw_rad = math.atan2(
        car_forward_world_vec.x,
        car_forward_world_vec.y,
    )

    print("Aktueller Frame:", bpy.context.scene.frame_current)
    print("car_eval:", car_eval.name)
    print("fahrbahn_eval:", fahrbahn_eval.name)

    print_vec("car_pos_world_vec", car_pos_world_vec)
    print_vec("car_forward_world_vec", car_forward_world_vec)
    print_vec("car_right_world_vec", car_right_world_vec)
    print_vec("car_up_world_vec", car_up_world_vec)

    print("car_yaw_rad:", round(car_yaw_rad, 4))

    print("============================================================")
    print("FERTIG: Current Frame Check funktioniert.")
    print("============================================================")


# ============================================================
# 5. Test current frame Rad center positions
# ============================================================

def test_rad_center_positions_current_frame():
    print("============================================================")
    print("Road Input Exporter - Rad Center Position Check")
    print("============================================================")

    depsgraph = bpy.context.evaluated_depsgraph_get()

    car_obj = get_obj(CAR_OBJECT_NAME)
    car_eval = car_obj.evaluated_get(depsgraph)

    car_local_to_world_mat = car_eval.matrix_world.copy()
    car_world_to_local_mat = car_local_to_world_mat.inverted()

    for rad_key, rad_object_name in RAD_OBJECT_NAMES.items():
        rad_obj = get_obj(rad_object_name)
        rad_eval = rad_obj.evaluated_get(depsgraph)

        rad_center_pos_world_vec = rad_eval.matrix_world.translation
        rad_center_pos_local_vec = car_world_to_local_mat @ rad_center_pos_world_vec

        print("------------------------------------------------------------")
        print("Rad:", rad_key, "=", rad_obj.name)
        print_vec("rad_center_pos_world_vec", rad_center_pos_world_vec)
        print_vec("rad_center_pos_local_vec", rad_center_pos_local_vec)

    print("============================================================")
    print("FERTIG: Rad Center Position Check funktioniert.")
    print("============================================================")


# ============================================================
# 6. Test all Rad center Raycast current frame
# ============================================================

def test_raycast_all_rad_center_current_frame():
    print("============================================================")
    print("Road Input Exporter - All Rad Center Raycast Test")
    print("============================================================")

    depsgraph = bpy.context.evaluated_depsgraph_get()

    car_obj = get_obj(CAR_OBJECT_NAME)
    fahrbahn_obj = get_obj(FAHRBAHN_OBJECT_NAME)

    car_eval = car_obj.evaluated_get(depsgraph)
    fahrbahn_eval = fahrbahn_obj.evaluated_get(depsgraph)

    car_local_to_world_mat = car_eval.matrix_world.copy()
    car_world_to_local_mat = car_local_to_world_mat.inverted()

    car_up_world_vec = (
        car_local_to_world_mat.to_3x3() @ Vector((0.0, 0.0, 1.0))
    ).normalized()

    for rad_key, rad_object_name in RAD_OBJECT_NAMES.items():
        rad_obj = get_obj(rad_object_name)
        rad_eval = rad_obj.evaluated_get(depsgraph)

        rad_center_pos_world_vec = rad_eval.matrix_world.translation.copy()
        rad_center_pos_local_vec = car_world_to_local_mat @ rad_center_pos_world_vec

        rad_center_ray_start_world_vec = (
            rad_center_pos_world_vec
            + car_up_world_vec * RAY_START_OFFSET_M
        )

        rad_center_ray_dir_world_vec = -car_up_world_vec

        (
            rad_center_hit_bool,
            rad_center_hit_world_vec,
            rad_center_hit_normal_world_vec,
            rad_center_hit_face_index,
        ) = raycast_fahrbahn_world(
            fahrbahn_eval,
            rad_center_ray_start_world_vec,
            rad_center_ray_dir_world_vec,
            RAY_DISTANCE_M,
        )

        print("------------------------------------------------------------")
        print("Rad:", rad_key, "=", rad_obj.name)

        print_vec("rad_center_pos_world_vec", rad_center_pos_world_vec)
        print_vec("rad_center_pos_local_vec", rad_center_pos_local_vec)

        print("rad_center_hit_bool:", rad_center_hit_bool)

        if rad_center_hit_bool:
            rad_center_hit_local_vec = (
                car_world_to_local_mat @ rad_center_hit_world_vec
            )

            rad_center_fahrbahn_z_local_m = rad_center_hit_local_vec.z

            print_vec("rad_center_hit_world_vec", rad_center_hit_world_vec)
            print_vec("rad_center_hit_local_vec", rad_center_hit_local_vec)

            print(
                "rad_center_fahrbahn_z_local_m:",
                round(rad_center_fahrbahn_z_local_m, 4),
            )

            print("rad_center_hit_face_index:", rad_center_hit_face_index)
        else:
            print("WARNUNG: Kein Raycast-Hit fuer Rad:", rad_key)

    print("============================================================")
    print("FERTIG: All Rad Center Raycast Test funktioniert.")
    print("============================================================")


# ============================================================
# 7. Read Rad center Fahrbahn data for current frame
# ============================================================

def read_rad_center_fahrbahn_current_frame(
    car_eval,
    fahrbahn_eval,
    depsgraph,
):
    car_local_to_world_mat = car_eval.matrix_world.copy()
    car_world_to_local_mat = car_local_to_world_mat.inverted()

    car_up_world_vec = (
        car_local_to_world_mat.to_3x3() @ Vector((0.0, 0.0, 1.0))
    ).normalized()

    rad_center_fahrbahn_data = {}

    for rad_key, rad_object_name in RAD_OBJECT_NAMES.items():
        rad_obj = get_obj(rad_object_name)
        rad_eval = rad_obj.evaluated_get(depsgraph)

        rad_center_pos_world_vec = rad_eval.matrix_world.translation.copy()
        rad_center_pos_local_vec = car_world_to_local_mat @ rad_center_pos_world_vec

        rad_center_ray_start_world_vec = (
            rad_center_pos_world_vec
            + car_up_world_vec * RAY_START_OFFSET_M
        )

        rad_center_ray_dir_world_vec = -car_up_world_vec

        (
            rad_center_hit_bool,
            rad_center_hit_world_vec,
            rad_center_hit_normal_world_vec,
            rad_center_hit_face_index,
        ) = raycast_fahrbahn_world(
            fahrbahn_eval,
            rad_center_ray_start_world_vec,
            rad_center_ray_dir_world_vec,
            RAY_DISTANCE_M,
        )

        if rad_center_hit_bool:
            rad_center_hit_local_vec = (
                car_world_to_local_mat @ rad_center_hit_world_vec
            )

            rad_center_fahrbahn_z_local_m = rad_center_hit_local_vec.z
        else:
            rad_center_hit_local_vec = None
            rad_center_fahrbahn_z_local_m = None

        rad_center_fahrbahn_data[rad_key] = {
            "rad_object_name": rad_object_name,

            "rad_center_pos_world_vec": rad_center_pos_world_vec,
            "rad_center_pos_local_vec": rad_center_pos_local_vec,

            "rad_center_ray_start_world_vec": rad_center_ray_start_world_vec,
            "rad_center_ray_dir_world_vec": rad_center_ray_dir_world_vec,

            "rad_center_hit_bool": rad_center_hit_bool,
            "rad_center_hit_world_vec": rad_center_hit_world_vec,
            "rad_center_hit_local_vec": rad_center_hit_local_vec,
            "rad_center_hit_normal_world_vec": rad_center_hit_normal_world_vec,
            "rad_center_hit_face_index": rad_center_hit_face_index,

            "rad_center_fahrbahn_z_local_m": rad_center_fahrbahn_z_local_m,
        }

    return rad_center_fahrbahn_data

# ============================================================
# 8. Read Rad envelope Fahrbahn data for current frame
# ============================================================

def read_rad_envelope_fahrbahn_current_frame(
    car_eval,
    fahrbahn_eval,
    depsgraph,
    static_geometry_data,
):
    car_local_to_world_mat = car_eval.matrix_world.copy()
    car_world_to_local_mat = car_local_to_world_mat.inverted()

    car_up_world_vec = (
        car_local_to_world_mat.to_3x3() @ Vector((0.0, 0.0, 1.0))
    ).normalized()

    rad_envelope_fahrbahn_data = {}

    for rad_key, rad_object_name in RAD_OBJECT_NAMES.items():
        rad_radius_used_m = static_geometry_data[
            "rad_radius_used_" + rad_key + "_m"
        ]

        if rad_radius_used_m is None or rad_radius_used_m <= 0.0:
            raise Exception(
                "Ungueltiger Rad-Radius fuer "
                + rad_key
                + ": "
                + str(rad_radius_used_m)
            )

        rad_obj = get_obj(rad_object_name)
        rad_eval = rad_obj.evaluated_get(depsgraph)

        rad_center_pos_world_vec = rad_eval.matrix_world.translation.copy()
        rad_center_pos_local_vec = car_world_to_local_mat @ rad_center_pos_world_vec

        rad_envelope_sample_data_list = []

        rad_envelope_hit_count = 0
        rad_envelope_required_max_z_local_m = None
        rad_envelope_contact_offset_y_local_m = None

        for rad_envelope_offset_ratio in RAD_ENVELOPE_OFFSET_RATIO_LIST:
            rad_envelope_offset_y_local_m = (
                rad_envelope_offset_ratio * rad_radius_used_m
            )

            rad_envelope_vertical_radius_m = math.sqrt(
                max(
                    0.0,
                    rad_radius_used_m * rad_radius_used_m
                    - rad_envelope_offset_y_local_m
                    * rad_envelope_offset_y_local_m,
                )
            )

            rad_envelope_sample_pos_local_vec = (
                rad_center_pos_local_vec
                + Vector((0.0, rad_envelope_offset_y_local_m, 0.0))
            )

            rad_envelope_sample_pos_world_vec = (
                car_local_to_world_mat @ rad_envelope_sample_pos_local_vec
            )

            rad_envelope_ray_start_world_vec = (
                rad_envelope_sample_pos_world_vec
                + car_up_world_vec * RAY_START_OFFSET_M
            )

            rad_envelope_ray_dir_world_vec = -car_up_world_vec

            (
                rad_envelope_hit_bool,
                rad_envelope_hit_world_vec,
                rad_envelope_hit_normal_world_vec,
                rad_envelope_hit_face_index,
            ) = raycast_fahrbahn_world(
                fahrbahn_eval,
                rad_envelope_ray_start_world_vec,
                rad_envelope_ray_dir_world_vec,
                RAY_DISTANCE_M,
            )

            if rad_envelope_hit_bool:
                rad_envelope_hit_count += 1

                rad_envelope_hit_local_vec = (
                    car_world_to_local_mat @ rad_envelope_hit_world_vec
                )

                rad_envelope_fahrbahn_z_local_m = (
                    rad_envelope_hit_local_vec.z
                )

                rad_envelope_required_z_local_m = (
                    rad_envelope_fahrbahn_z_local_m
                    + rad_envelope_vertical_radius_m
                )

                if (
                    rad_envelope_required_max_z_local_m is None
                    or rad_envelope_required_z_local_m
                    > rad_envelope_required_max_z_local_m
                ):
                    rad_envelope_required_max_z_local_m = (
                        rad_envelope_required_z_local_m
                    )

                    rad_envelope_contact_offset_y_local_m = (
                        rad_envelope_offset_y_local_m
                    )

            else:
                rad_envelope_hit_local_vec = None
                rad_envelope_fahrbahn_z_local_m = None
                rad_envelope_required_z_local_m = None

            rad_envelope_sample_data_list.append(
                {
                    "rad_envelope_offset_ratio": rad_envelope_offset_ratio,
                    "rad_envelope_offset_y_local_m": rad_envelope_offset_y_local_m,
                    "rad_envelope_vertical_radius_m": rad_envelope_vertical_radius_m,

                    "rad_envelope_sample_pos_local_vec": rad_envelope_sample_pos_local_vec,
                    "rad_envelope_sample_pos_world_vec": rad_envelope_sample_pos_world_vec,

                    "rad_envelope_ray_start_world_vec": rad_envelope_ray_start_world_vec,
                    "rad_envelope_ray_dir_world_vec": rad_envelope_ray_dir_world_vec,

                    "rad_envelope_hit_bool": rad_envelope_hit_bool,
                    "rad_envelope_hit_world_vec": rad_envelope_hit_world_vec,
                    "rad_envelope_hit_local_vec": rad_envelope_hit_local_vec,
                    "rad_envelope_hit_normal_world_vec": rad_envelope_hit_normal_world_vec,
                    "rad_envelope_hit_face_index": rad_envelope_hit_face_index,

                    "rad_envelope_fahrbahn_z_local_m": rad_envelope_fahrbahn_z_local_m,
                    "rad_envelope_required_z_local_m": rad_envelope_required_z_local_m,
                }
            )

        rad_envelope_fahrbahn_data[rad_key] = {
            "rad_object_name": rad_object_name,

            "rad_center_pos_world_vec": rad_center_pos_world_vec,
            "rad_center_pos_local_vec": rad_center_pos_local_vec,

            "rad_envelope_hit_count": rad_envelope_hit_count,
            "rad_envelope_required_max_z_local_m": rad_envelope_required_max_z_local_m,
            "rad_envelope_contact_offset_y_local_m": rad_envelope_contact_offset_y_local_m,

            "rad_envelope_sample_data_list": rad_envelope_sample_data_list,
        }

    return rad_envelope_fahrbahn_data


# ============================================================
# 9. Read static Fahrzeuggeometrie
# ============================================================

def estimate_rad_radius_from_mesh(rad_key, rad_object_name, depsgraph):
    rad_drehung_obj = get_obj(rad_object_name)
    rad_drehung_eval = rad_drehung_obj.evaluated_get(depsgraph)

    # Wichtig:
    # Fuer die Radius-Messung darf die Skala des Rad-Objekts
    # NICHT aus dem Koordinatensystem herausgerechnet werden.
    #
    # Grund:
    # Wenn rad_drehung_obj selbst ein Mesh ist und skaliert wurde,
    # wuerde matrix_world.inverted() diese Skala wieder herauskuerzen.
    #
    # Deshalb wird fuer das Mess-Koordinatensystem nur Translation
    # und Rotation des Rad-Objekts verwendet, aber keine Skala.
    rad_drehung_pos_world_vec = (
        rad_drehung_eval.matrix_world.translation.copy()
    )

    rad_drehung_rot_world_mat = (
        rad_drehung_eval.matrix_world
        .to_quaternion()
        .to_matrix()
        .to_4x4()
    )

    rad_drehung_pos_world_mat = Matrix.Translation(
        rad_drehung_pos_world_vec
    )

    rad_drehung_to_world_ohne_skala_mat = (
        rad_drehung_pos_world_mat
        @ rad_drehung_rot_world_mat
    )

    world_to_rad_drehung_mat = (
        rad_drehung_to_world_ohne_skala_mat.inverted()
    )

    wheel_mesh_obj_list = collect_mesh_children(rad_drehung_obj)

    if len(wheel_mesh_obj_list) == 0:
        print(
            "WARNUNG: Kein Mesh unter Rad gefunden:",
            rad_key,
            "=",
            rad_object_name,
        )
        return None

    rad_radius_auto_m = None

    for wheel_mesh_obj in wheel_mesh_obj_list:
        wheel_mesh_eval = wheel_mesh_obj.evaluated_get(depsgraph)

        if wheel_mesh_eval.type != "MESH":
            continue

        wheel_mesh_data = wheel_mesh_eval.data

        for vertex in wheel_mesh_data.vertices:
            vertex_pos_world_vec = (
                wheel_mesh_eval.matrix_world @ vertex.co
            )

            vertex_pos_rad_drehung_vec = (
                world_to_rad_drehung_mat @ vertex_pos_world_vec
            )

            radius_candidate_m = math.sqrt(
                vertex_pos_rad_drehung_vec.y
                * vertex_pos_rad_drehung_vec.y
                + vertex_pos_rad_drehung_vec.z
                * vertex_pos_rad_drehung_vec.z
            )

            if (
                rad_radius_auto_m is None
                or radius_candidate_m > rad_radius_auto_m
            ):
                rad_radius_auto_m = radius_candidate_m

    if rad_radius_auto_m is None:
        print(
            "WARNUNG: Rad Radius konnte nicht berechnet werden:",
            rad_key,
            "=",
            rad_object_name,
        )
        return None

    if (
        rad_radius_auto_m < RAD_RADIUS_MIN_PLAUSIBLE_M
        or rad_radius_auto_m > RAD_RADIUS_MAX_PLAUSIBLE_M
    ):
        print(
            "WARNUNG: Rad Radius wirkt unplausibel:",
            rad_key,
            "=",
            round(rad_radius_auto_m, 4),
            "m",
        )

    print(
        "rad_radius_auto_" + rad_key + "_m:",
        round(rad_radius_auto_m, 6),
    )

    return rad_radius_auto_m


def read_static_vehicle_geometry(static_geometry_frame):
    scene = bpy.context.scene
    scene.frame_set(static_geometry_frame)

    depsgraph = bpy.context.evaluated_depsgraph_get()

    car_obj = get_obj(CAR_OBJECT_NAME)
    car_eval = car_obj.evaluated_get(depsgraph)

    car_local_to_world_mat = car_eval.matrix_world.copy()
    car_world_to_local_mat = car_local_to_world_mat.inverted()

    rad_pos_local = {}

    for rad_key, rad_object_name in RAD_OBJECT_NAMES.items():
        rad_obj = get_obj(rad_object_name)
        rad_eval = rad_obj.evaluated_get(depsgraph)

        rad_pos_world_vec = rad_eval.matrix_world.translation.copy()
        rad_pos_local_vec = car_world_to_local_mat @ rad_pos_world_vec

        rad_pos_local[rad_key] = rad_pos_local_vec

    vl = rad_pos_local["VL"]
    vr = rad_pos_local["VR"]
    hl = rad_pos_local["HL"]
    hr = rad_pos_local["HR"]

    car_rad_center_x_local_m = (
        vl.x + vr.x + hl.x + hr.x
    ) / 4.0

    car_rad_center_y_local_m = (
        vl.y + vr.y + hl.y + hr.y
    ) / 4.0

    car_spurweite_front_m = abs(vr.x - vl.x)
    car_spurweite_rear_m = abs(hr.x - hl.x)

    car_spurweite_mean_m = (
        car_spurweite_front_m
        + car_spurweite_rear_m
    ) / 2.0

    car_radstand_left_m = abs(vl.y - hl.y)
    car_radstand_right_m = abs(vr.y - hr.y)

    car_radstand_mean_m = (
        car_radstand_left_m
        + car_radstand_right_m
    ) / 2.0

    static_geometry_data = {}

    static_geometry_data["static_geometry_frame"] = static_geometry_frame

    static_geometry_data["car_spurweite_front_m"] = car_spurweite_front_m
    static_geometry_data["car_spurweite_rear_m"] = car_spurweite_rear_m
    static_geometry_data["car_spurweite_mean_m"] = car_spurweite_mean_m

    static_geometry_data["car_radstand_left_m"] = car_radstand_left_m
    static_geometry_data["car_radstand_right_m"] = car_radstand_right_m
    static_geometry_data["car_radstand_mean_m"] = car_radstand_mean_m

    for rad_key, rad_object_name in RAD_OBJECT_NAMES.items():
        rad_radius_auto_m = estimate_rad_radius_from_mesh(
            rad_key,
            rad_object_name,
            depsgraph,
        )

        if AUTO_RAD_RADIUS and rad_radius_auto_m is not None:
            rad_radius_used_m = rad_radius_auto_m
        else:
            rad_radius_used_m = RAD_RADIUS_MANUAL_M

        static_geometry_data[
            "rad_radius_auto_" + rad_key + "_m"
        ] = rad_radius_auto_m

        static_geometry_data[
            "rad_radius_used_" + rad_key + "_m"
        ] = rad_radius_used_m

    for rad_key in RAD_OBJECT_NAMES.keys():
        rad_pos_local_vec = rad_pos_local[rad_key]

        static_geometry_data[
            "rad_offset_x_local_" + rad_key + "_m"
        ] = rad_pos_local_vec.x - car_rad_center_x_local_m

        static_geometry_data[
            "rad_offset_y_local_" + rad_key + "_m"
        ] = rad_pos_local_vec.y - car_rad_center_y_local_m

    print("============================================================")
    print("Road Input Exporter - Static Fahrzeuggeometrie")
    print("============================================================")
    print("static_geometry_frame:", static_geometry_frame)
    print("car_spurweite_front_m:", round(car_spurweite_front_m, 4))
    print("car_spurweite_rear_m:", round(car_spurweite_rear_m, 4))
    print("car_radstand_left_m:", round(car_radstand_left_m, 4))
    print("car_radstand_right_m:", round(car_radstand_right_m, 4))

    for rad_key in RAD_OBJECT_NAMES.keys():
        print(
            "rad_radius_auto_" + rad_key + "_m:",
            static_geometry_data["rad_radius_auto_" + rad_key + "_m"],
        )
        print(
            "rad_radius_used_" + rad_key + "_m:",
            static_geometry_data["rad_radius_used_" + rad_key + "_m"],
        )

    print("============================================================")

    return static_geometry_data


# ============================================================
# 10. Read all Groessen for one frame
# ============================================================

def read_groessen_current_frame(
    frame,
    static_geometry_data,
):
    bpy.context.scene.frame_set(frame)

    depsgraph = bpy.context.evaluated_depsgraph_get()

    car_obj = get_obj(CAR_OBJECT_NAME)
    fahrbahn_obj = get_obj(FAHRBAHN_OBJECT_NAME)

    car_eval = car_obj.evaluated_get(depsgraph)
    fahrbahn_eval = fahrbahn_obj.evaluated_get(depsgraph)

    car_local_to_world_mat = car_eval.matrix_world.copy()

    car_pos_world_vec = car_local_to_world_mat.translation.copy()

    car_forward_world_vec = (
        car_local_to_world_mat.to_3x3() @ Vector((0.0, 1.0, 0.0))
    ).normalized()

    car_right_world_vec = (
        car_local_to_world_mat.to_3x3() @ Vector((1.0, 0.0, 0.0))
    ).normalized()

    car_up_world_vec = (
        car_local_to_world_mat.to_3x3() @ Vector((0.0, 0.0, 1.0))
    ).normalized()

    car_yaw_rad = math.atan2(
        car_forward_world_vec.x,
        car_forward_world_vec.y,
    )

    scene = bpy.context.scene
    fps = scene.render.fps / scene.render.fps_base

    if fps <= 0.0:
        raise Exception("Die Blender-Framerate muss groesser als 0 sein.")

    t_s = (frame - scene.frame_start) / fps

    if frame == scene.frame_start:
        dt_s = 0.0
    else:
        dt_s = 1.0 / fps

    rad_center_fahrbahn_data = read_rad_center_fahrbahn_current_frame(
        car_eval,
        fahrbahn_eval,
        depsgraph,
    )

    rad_envelope_fahrbahn_data = read_rad_envelope_fahrbahn_current_frame(
        car_eval,
        fahrbahn_eval,
        depsgraph,
        static_geometry_data,
    )

    frame_data = {}

    frame_data["frame"] = frame
    frame_data["t_s"] = t_s
    frame_data["dt_s"] = dt_s

    frame_data["car_pos_x_world_m"] = car_pos_world_vec.x
    frame_data["car_pos_y_world_m"] = car_pos_world_vec.y
    frame_data["car_pos_z_world_m"] = car_pos_world_vec.z

    frame_data["car_yaw_rad"] = car_yaw_rad
    frame_data["car_forward_x_world"] = car_forward_world_vec.x
    frame_data["car_forward_y_world"] = car_forward_world_vec.y
    frame_data["car_forward_z_world"] = car_forward_world_vec.z

    frame_data["car_right_x_world"] = car_right_world_vec.x
    frame_data["car_right_y_world"] = car_right_world_vec.y
    frame_data["car_right_z_world"] = car_right_world_vec.z

    frame_data["car_up_x_world"] = car_up_world_vec.x
    frame_data["car_up_y_world"] = car_up_world_vec.y
    frame_data["car_up_z_world"] = car_up_world_vec.z


    for rad_key in RAD_OBJECT_NAMES.keys():
        rad_center_data = rad_center_fahrbahn_data[rad_key]
        rad_envelope_data = rad_envelope_fahrbahn_data[rad_key]

        frame_data[
            "rad_center_hit_" + rad_key + "_bool"
        ] = rad_center_data["rad_center_hit_bool"]

        frame_data[
            "rad_center_fahrbahn_z_local_" + rad_key + "_m"
        ] = rad_center_data["rad_center_fahrbahn_z_local_m"]

        frame_data[
            "rad_envelope_hit_count_" + rad_key
        ] = rad_envelope_data["rad_envelope_hit_count"]

        frame_data[
            "rad_envelope_required_max_z_local_" + rad_key + "_m"
        ] = rad_envelope_data["rad_envelope_required_max_z_local_m"]

        frame_data[
            "rad_envelope_contact_offset_y_local_" + rad_key + "_m"
        ] = rad_envelope_data["rad_envelope_contact_offset_y_local_m"]

    return frame_data

# ============================================================
# 11. Read Groessen for all frames
# ============================================================

def read_groessen_all_frames():
    scene = bpy.context.scene

    frame_start = scene.frame_start
    frame_end = scene.frame_end
    frame_current_before_export = scene.frame_current

    frame_data_list = []

    try:
        static_geometry_data = read_static_vehicle_geometry(frame_start)

        print("============================================================")
        print("Road Input Exporter - Read Groessen All Frames")
        print("============================================================")
        print("frame_start:", frame_start)
        print("frame_end:", frame_end)

        for frame in range(frame_start, frame_end + 1):
            frame_data = read_groessen_current_frame(
                frame,
                static_geometry_data,
            )

            frame_data.update(static_geometry_data)

            frame_data_list.append(frame_data)

            print(
                "Frame gelesen:",
                frame,
                "/",
                frame_end,
            )

    finally:
        scene.frame_set(frame_current_before_export)

    print("============================================================")
    print("FERTIG: Read Groessen All Frames funktioniert.")
    print("Gelesene Frames:", len(frame_data_list))
    print("============================================================")

    return frame_data_list

# ============================================================
# 12. Compute car kinematics from all frames
# ============================================================

def unwrap_yaw_angle_rad(yaw_rad, previous_yaw_unwrapped_rad):
    yaw_unwrapped_rad = yaw_rad

    while yaw_unwrapped_rad - previous_yaw_unwrapped_rad > math.pi:
        yaw_unwrapped_rad -= 2.0 * math.pi

    while yaw_unwrapped_rad - previous_yaw_unwrapped_rad < -math.pi:
        yaw_unwrapped_rad += 2.0 * math.pi

    return yaw_unwrapped_rad


def compute_scalar_derivative(frame_data_list, index, value_key):
    frame_count = len(frame_data_list)

    if frame_count <= 1:
        return 0.0

    if index == 0:
        previous_data = frame_data_list[index]
        next_data = frame_data_list[index + 1]

    elif index == frame_count - 1:
        previous_data = frame_data_list[index - 1]
        next_data = frame_data_list[index]

    else:
        previous_data = frame_data_list[index - 1]
        next_data = frame_data_list[index + 1]

    previous_value = previous_data[value_key]
    next_value = next_data[value_key]

    previous_t_s = previous_data["t_s"]
    next_t_s = next_data["t_s"]

    dt_s = next_t_s - previous_t_s

    if dt_s == 0.0:
        return 0.0

    derivative_value = (next_value - previous_value) / dt_s

    return derivative_value


def compute_car_kinematics(frame_data_list):
    print("============================================================")
    print("Road Input Exporter - Compute Car Kinematics")
    print("============================================================")

    previous_yaw_unwrapped_rad = None

    for frame_data in frame_data_list:
        car_yaw_rad = frame_data["car_yaw_rad"]

        if previous_yaw_unwrapped_rad is None:
            car_yaw_unwrapped_rad = car_yaw_rad
        else:
            car_yaw_unwrapped_rad = unwrap_yaw_angle_rad(
                car_yaw_rad,
                previous_yaw_unwrapped_rad,
            )

        frame_data["car_yaw_unwrapped_rad"] = car_yaw_unwrapped_rad
        previous_yaw_unwrapped_rad = car_yaw_unwrapped_rad

    for index, frame_data in enumerate(frame_data_list):
        car_vel_x_world_mps = compute_scalar_derivative(
            frame_data_list,
            index,
            "car_pos_x_world_m",
        )

        car_vel_y_world_mps = compute_scalar_derivative(
            frame_data_list,
            index,
            "car_pos_y_world_m",
        )

        car_vel_z_world_mps = compute_scalar_derivative(
            frame_data_list,
            index,
            "car_pos_z_world_m",
        )

        car_vel_world_vec = Vector(
            (
                car_vel_x_world_mps,
                car_vel_y_world_mps,
                car_vel_z_world_mps,
            )
        )

        car_vel_abs_mps = car_vel_world_vec.length

        car_forward_world_vec = Vector(
            (
                frame_data["car_forward_x_world"],
                frame_data["car_forward_y_world"],
                frame_data["car_forward_z_world"],
            )
        ).normalized()

        car_right_world_vec = Vector(
            (
                frame_data["car_right_x_world"],
                frame_data["car_right_y_world"],
                frame_data["car_right_z_world"],
            )
        ).normalized()

        car_up_world_vec = Vector(
            (
                frame_data["car_up_x_world"],
                frame_data["car_up_y_world"],
                frame_data["car_up_z_world"],
            )
        ).normalized()

        car_vel_forward_local_mps = car_vel_world_vec.dot(
            car_forward_world_vec
        )

        car_vel_lateral_local_mps = car_vel_world_vec.dot(
            car_right_world_vec
        )

        car_vel_vertical_local_mps = car_vel_world_vec.dot(
            car_up_world_vec
        )

        frame_data["car_vel_x_world_mps"] = car_vel_x_world_mps
        frame_data["car_vel_y_world_mps"] = car_vel_y_world_mps
        frame_data["car_vel_z_world_mps"] = car_vel_z_world_mps

        frame_data["car_vel_abs_mps"] = car_vel_abs_mps

        frame_data["car_vel_forward_local_mps"] = car_vel_forward_local_mps
        frame_data["car_vel_lateral_local_mps"] = car_vel_lateral_local_mps
        frame_data["car_vel_vertical_local_mps"] = car_vel_vertical_local_mps

        car_yaw_rate_radps = compute_scalar_derivative(
            frame_data_list,
            index,
            "car_yaw_unwrapped_rad",
        )

        frame_data["car_yaw_rate_radps"] = car_yaw_rate_radps

    for index, frame_data in enumerate(frame_data_list):
        car_acc_x_world_mps2 = compute_scalar_derivative(
            frame_data_list,
            index,
            "car_vel_x_world_mps",
        )

        car_acc_y_world_mps2 = compute_scalar_derivative(
            frame_data_list,
            index,
            "car_vel_y_world_mps",
        )

        car_acc_z_world_mps2 = compute_scalar_derivative(
            frame_data_list,
            index,
            "car_vel_z_world_mps",
        )

        car_acc_world_vec = Vector(
            (
                car_acc_x_world_mps2,
                car_acc_y_world_mps2,
                car_acc_z_world_mps2,
            )
        )

        car_forward_world_vec = Vector(
            (
                frame_data["car_forward_x_world"],
                frame_data["car_forward_y_world"],
                frame_data["car_forward_z_world"],
            )
        ).normalized()

        car_right_world_vec = Vector(
            (
                frame_data["car_right_x_world"],
                frame_data["car_right_y_world"],
                frame_data["car_right_z_world"],
            )
        ).normalized()

        car_up_world_vec = Vector(
            (
                frame_data["car_up_x_world"],
                frame_data["car_up_y_world"],
                frame_data["car_up_z_world"],
            )
        ).normalized()

        car_acc_forward_local_mps2 = car_acc_world_vec.dot(
            car_forward_world_vec
        )

        car_acc_lateral_local_mps2 = car_acc_world_vec.dot(
            car_right_world_vec
        )

        car_acc_vertical_local_mps2 = car_acc_world_vec.dot(
            car_up_world_vec
        )

        frame_data["car_acc_x_world_mps2"] = car_acc_x_world_mps2
        frame_data["car_acc_y_world_mps2"] = car_acc_y_world_mps2
        frame_data["car_acc_z_world_mps2"] = car_acc_z_world_mps2

        frame_data["car_acc_forward_local_mps2"] = car_acc_forward_local_mps2
        frame_data["car_acc_lateral_local_mps2"] = car_acc_lateral_local_mps2
        frame_data["car_acc_vertical_local_mps2"] = car_acc_vertical_local_mps2

        car_yaw_acc_radps2 = compute_scalar_derivative(
            frame_data_list,
            index,
            "car_yaw_rate_radps",
        )

        frame_data["car_yaw_acc_radps2"] = car_yaw_acc_radps2

    print("FERTIG: Compute Car Kinematics funktioniert.")
    print("============================================================")

    return frame_data_list

# ============================================================
# 13. Compute relative road input
# ============================================================

def compute_relative_road_input(frame_data_list):
    print("============================================================")
    print("Road Input Exporter - Compute Relative Road Input")
    print("============================================================")

    if len(frame_data_list) == 0:
        raise Exception("frame_data_list ist leer.")

    first_frame_data = frame_data_list[0]

    for rad_key in RAD_OBJECT_NAMES.keys():
        rad_center_z_key = (
            "rad_center_fahrbahn_z_local_" + rad_key + "_m"
        )

        rad_center_z0_key = (
            "rad_center_fahrbahn_z0_local_" + rad_key + "_m"
        )

        rad_center_z_rel_key = (
            "rad_center_fahrbahn_z_rel_local_" + rad_key + "_m"
        )

        rad_envelope_z_key = (
            "rad_envelope_required_max_z_local_" + rad_key + "_m"
        )

        rad_envelope_z0_key = (
            "rad_envelope_required_max_z0_local_" + rad_key + "_m"
        )

        rad_envelope_z_rel_key = (
            "rad_envelope_required_max_z_rel_local_" + rad_key + "_m"
        )

        rad_center_z0_local_m = first_frame_data[rad_center_z_key]
        rad_envelope_z0_local_m = first_frame_data[rad_envelope_z_key]

        for frame_data in frame_data_list:
            rad_center_z_local_m = frame_data[rad_center_z_key]
            rad_envelope_z_local_m = frame_data[rad_envelope_z_key]

            if (
                rad_center_z_local_m is None
                and rad_envelope_z_local_m is None
            ):
                raise Exception(
                    "Kein Fahrbahn-Ray-Hit fuer Rad "
                    + rad_key
                    + " in Frame "
                    + str(frame_data["frame"])
                    + "."
                )

            frame_data[rad_center_z0_key] = rad_center_z0_local_m
            frame_data[rad_envelope_z0_key] = rad_envelope_z0_local_m

            if (
                rad_center_z_local_m is not None
                and rad_center_z0_local_m is not None
            ):
                frame_data[rad_center_z_rel_key] = (
                    rad_center_z_local_m
                    - rad_center_z0_local_m
                )
            else:
                frame_data[rad_center_z_rel_key] = None

            if (
                rad_envelope_z_local_m is not None
                and rad_envelope_z0_local_m is not None
            ):
                frame_data[rad_envelope_z_rel_key] = (
                    rad_envelope_z_local_m
                    - rad_envelope_z0_local_m
                )
            else:
                frame_data[rad_envelope_z_rel_key] = None

    print("FERTIG: Compute Relative Road Input funktioniert.")
    print("============================================================")

    return frame_data_list

# ============================================================
# 14. Export road input CSV
# ============================================================

def build_csv_fieldnames():
    fieldnames = [
        "frame",
        "t_s",
        "dt_s",

        "car_pos_x_world_m",
        "car_pos_y_world_m",
        "car_pos_z_world_m",

        "car_yaw_rad",
        "car_yaw_unwrapped_rad",
        "car_yaw_rate_radps",
        "car_yaw_acc_radps2",

        "car_forward_x_world",
        "car_forward_y_world",
        "car_forward_z_world",

        "car_right_x_world",
        "car_right_y_world",
        "car_right_z_world",

        "car_up_x_world",
        "car_up_y_world",
        "car_up_z_world",

        "static_geometry_frame",

        "car_spurweite_front_m",
        "car_spurweite_rear_m",
        "car_spurweite_mean_m",

        "car_radstand_left_m",
        "car_radstand_right_m",
        "car_radstand_mean_m",

        "car_vel_x_world_mps",
        "car_vel_y_world_mps",
        "car_vel_z_world_mps",
        "car_vel_abs_mps",

        "car_vel_forward_local_mps",
        "car_vel_lateral_local_mps",
        "car_vel_vertical_local_mps",

        "car_acc_x_world_mps2",
        "car_acc_y_world_mps2",
        "car_acc_z_world_mps2",

        "car_acc_forward_local_mps2",
        "car_acc_lateral_local_mps2",
        "car_acc_vertical_local_mps2",
    ]

    for rad_key in RAD_OBJECT_NAMES.keys():
        fieldnames.extend(
            [
                "rad_offset_x_local_" + rad_key + "_m",
                "rad_offset_y_local_" + rad_key + "_m",

                "rad_radius_auto_" + rad_key + "_m",
                "rad_radius_used_" + rad_key + "_m",

                "rad_center_hit_" + rad_key + "_bool",

                "rad_center_fahrbahn_z_local_" + rad_key + "_m",
                "rad_center_fahrbahn_z0_local_" + rad_key + "_m",
                "rad_center_fahrbahn_z_rel_local_" + rad_key + "_m",

                "rad_envelope_hit_count_" + rad_key,

                "rad_envelope_required_max_z_local_" + rad_key + "_m",
                "rad_envelope_required_max_z0_local_" + rad_key + "_m",
                "rad_envelope_required_max_z_rel_local_" + rad_key + "_m",

                "rad_envelope_contact_offset_y_local_" + rad_key + "_m",
            ]
        )

    return fieldnames

def write_road_input_csv(frame_data_list):
    if len(frame_data_list) == 0:
        raise Exception("frame_data_list ist leer. CSV kann nicht geschrieben werden.")

    if bpy.data.filepath == "":
        raise Exception(
            "Die .blend-Datei ist noch nicht gespeichert. "
            "Bitte zuerst die Blender-Datei speichern."
        )

    csv_path = Path(bpy.path.abspath("//road_input.csv"))

    fieldnames = build_csv_fieldnames()

    with open(csv_path, mode="w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(
            csv_file,
            fieldnames=fieldnames,
            extrasaction="ignore",
        )

        writer.writeheader()

        for frame_data in frame_data_list:
            writer.writerow(frame_data)

    print("============================================================")
    print("Road Input Exporter - CSV Export")
    print("============================================================")
    print("CSV geschrieben:")
    print(csv_path)
    print("Anzahl Frames:", len(frame_data_list))
    print("Anzahl Spalten:", len(fieldnames))
    print("============================================================")

    return csv_path

# ============================================================
# 15. Full export pipeline
# ============================================================

def export_road_input_csv():
    print("============================================================")
    print("Road Input Exporter - Full Export Pipeline")
    print("============================================================")

    frame_data_list = read_groessen_all_frames()

    frame_data_list = compute_car_kinematics(frame_data_list)

    frame_data_list = compute_relative_road_input(frame_data_list)

    csv_path = write_road_input_csv(frame_data_list)

    print("============================================================")
    print("FERTIG: Road Input Export abgeschlossen.")
    print("CSV:", csv_path)
    print("============================================================")

    return csv_path


# ============================================================
# 16. Fahrzeugparameter Json
# Diese Parameter gehoeren zum Solver Modell und werden nicht in den CSV sondern als Json exportiert.
#

class ROADINPUT_VehicleParameter(bpy.types.PropertyGroup): 
    fahrzeugmasse_kg: bpy.props.FloatProperty(
        name="Fahrzeugmasse [kg]",
        default=1800.0,
        min=100.0,
        max=4000.0,
    )

    gefedertemasse_anteil: bpy.props.FloatProperty(
        name="Gefederte-Masse-Anteil [-]",
        default=0.85,
        min=0.5,
        max=0.98,
    )

    ungefedertemasse_pro_rad_kg: bpy.props.FloatProperty(
        name="Ungefederte Masse pro Rad [kg]",
        default=45.0,
        min=10.0,
        max=120.0,
    )
    federsteifigkeit_n_pro_m: bpy.props.FloatProperty(
        name="Federsteifigkeit [N/m]",
        default=30000.0,
        min=5000.0,
        max=100000.0,
    )

    daempferkonstante_n_s_pro_m: bpy.props.FloatProperty(
        name="Daempferkonstante [N*s/m]",
        default=3000.0,
        min=100.0,
        max=15000.0,
    )

    reifensteifigkeit_n_pro_m: bpy.props.FloatProperty(
        name="Reifensteifigkeit [N/m]",
        default=200000.0,
        min=50000.0,
        max=500000.0,
    )

    max_einfederung_m: bpy.props.FloatProperty(
        name="Max. Einfederung [m]",
        default=0.035,
        min=0.01,
        max=0.50,
    )

    max_ausfederung_m: bpy.props.FloatProperty(
        name="Max. Ausfederung [m]",
        default=0.035,
        min=0.01,
        max=0.50,
    )

    lastverlagerung_aktiv: bpy.props.BoolProperty(
        name="Lastverlagerung aktiv",
        default=True,
    )

    lastverlagerung_laengs_aktiv: bpy.props.BoolProperty(
        name="Laengs-Lastverlagerung aktiv",
        default=True,
    )

    lastverlagerung_quer_aktiv: bpy.props.BoolProperty(
        name="Quer-Lastverlagerung aktiv",
        default=True,
    )

    schwerpunkt_hoehe_m: bpy.props.FloatProperty(
        name="Schwerpunkt-Hoehe [m]",
        default=0.55,
        min=0.10,
        max=1.50,
    )

    lastverlagerung_laengs_vorzeichen: bpy.props.FloatProperty(
        name="Laengs-Vorzeichen",
        default=1.0,
        min=-1.0,
        max=1.0,
    )

    lastverlagerung_quer_vorzeichen: bpy.props.FloatProperty(
        name="Quer-Vorzeichen",
        default=1.0,
        min=-1.0,
        max=1.0,
    )

def build_vehicle_parameter_dict(context):
    parameter = context.scene.road_input_vehicle_parameter

    vehicle_parameter_data = {
        "modell": "vier_unabhaengige_viertelfahrzeugmodelle",

        "FAHRZEUGMASSE_KG": parameter.fahrzeugmasse_kg,
        "GEFEDERTE_MASSE_ANTEIL": parameter.gefedertemasse_anteil,
        "UNGEFEDERTEMASSE_PRO_RAD_KG": parameter.ungefedertemasse_pro_rad_kg,

        "FEDERSTEIFIGKEIT_N_PRO_M": parameter.federsteifigkeit_n_pro_m,
        "DAEMPFERKONSTANTE_N_S_PRO_M": parameter.daempferkonstante_n_s_pro_m,
        "REIFENSTEIFIGKEIT_N_PRO_M": parameter.reifensteifigkeit_n_pro_m,

        "MAX_EINFEDERUNG_M": parameter.max_einfederung_m,
        "MAX_AUSFEDERUNG_M": parameter.max_ausfederung_m,

        "LASTVERLAGERUNG_AKTIV": parameter.lastverlagerung_aktiv,
        "LASTVERLAGERUNG_LAENGS_AKTIV": parameter.lastverlagerung_laengs_aktiv,
        "LASTVERLAGERUNG_QUER_AKTIV": parameter.lastverlagerung_quer_aktiv,

        "SCHWERPUNKT_HOEHE_M": parameter.schwerpunkt_hoehe_m,
        "LASTVERLAGERUNG_LAENGS_VORZEICHEN": parameter.lastverlagerung_laengs_vorzeichen,
        "LASTVERLAGERUNG_QUER_VORZEICHEN": parameter.lastverlagerung_quer_vorzeichen,
    }

    return vehicle_parameter_data

def write_vehicle_parameter_json(context):
    vehicle_parameter_data = build_vehicle_parameter_dict(context)

    if bpy.data.filepath == "":
        raise Exception(
            "Die .blend-Datei ist noch nicht gespeichert. "
            "Bitte zuerst die Blender-Datei speichern."
        )

    json_path = Path(bpy.path.abspath("//vehicle_parameter.json"))

    with open(json_path, mode="w", encoding="utf-8") as json_file:
        json.dump(
            vehicle_parameter_data,
            json_file,
            indent=4,
            ensure_ascii=False,
        )

    print("============================================================")
    print("Road Input Exporter - Vehicle Parameter JSON Export")
    print("============================================================")
    print("JSON geschrieben:")
    print(json_path)
    print("============================================================")

    return json_path

# ============================================================
# 17. Blender UI v0.1
# ============================================================
#
# Wirkung:
# - Beim Ausfuehren des Skripts wird nur die UI registriert.
# - Der Export startet NICHT automatisch.
# - Der Benutzer startet Tests oder Export ueber Buttons im Blender N-Panel.
#
# In Blender:
# 3D Viewport -> Taste N -> Tab "Road Input"
#

def run_all_diagnostic_tests():
    print("============================================================")
    print("Road Input Exporter - Run All Diagnostic Tests")
    print("============================================================")

    test_basic_setup()
    test_car_state_current_frame()
    test_rad_center_positions_current_frame()
    test_raycast_all_rad_center_current_frame()

    print("============================================================")
    print("FERTIG: Alle Diagnostic Tests abgeschlossen.")
    print("============================================================")


class ROADINPUT_OT_run_diagnostic_tests(bpy.types.Operator):
    bl_idname = "road_input.run_diagnostic_tests"
    bl_label = "Run Diagnostic Tests"
    bl_description = "Fuehrt alle Diagnose-Tests aus"

    def execute(self, context):
        try:
            run_all_diagnostic_tests()
            self.report({"INFO"}, "Diagnostic Tests abgeschlossen.")
            return {"FINISHED"}

        except Exception as error:
            self.report({"ERROR"}, str(error))
            print("FEHLER in Diagnostic Tests:", error)
            return {"CANCELLED"}


class ROADINPUT_OT_export_csv(bpy.types.Operator):
    bl_idname = "road_input.export_csv"
    bl_label = "Export road_input.csv"
    bl_description = "Exportiert die Road-Input-CSV"

    def execute(self, context):
        try:
            csv_path = export_road_input_csv()
            self.report({"INFO"}, "CSV exportiert: " + str(csv_path))
            return {"FINISHED"}

        except Exception as error:
            self.report({"ERROR"}, str(error))
            print("FEHLER beim CSV Export:", error)
            return {"CANCELLED"}


class ROADINPUT_OT_export_vehicle_parameter_json(bpy.types.Operator):
    bl_idname = "road_input.export_vehicle_parameter_json"
    bl_label = "Export vehicle_parameter.json"
    bl_description = "Exportiert die Viertelfahrzeugmodell-Parameter als JSON"

    def execute(self, context):
        try:
            json_path = write_vehicle_parameter_json(context)
            self.report({"INFO"}, "JSON exportiert: " + str(json_path))
            return {"FINISHED"}

        except Exception as error:
            self.report({"ERROR"}, str(error))
            print("FEHLER beim JSON Export:", error)
            return {"CANCELLED"}


class ROADINPUT_OT_test_and_export(bpy.types.Operator):
    bl_idname = "road_input.test_and_export"
    bl_label = "Test + Export CSV + JSON"
    bl_description = (
        "Fuehrt zuerst alle Tests aus und exportiert danach "
        "road_input.csv und vehicle_parameter.json"
    )

    def execute(self, context):
        try:
            run_all_diagnostic_tests()

            csv_path = export_road_input_csv()
            json_path = write_vehicle_parameter_json(context)

            self.report(
                {"INFO"},
                "Tests abgeschlossen, CSV und JSON exportiert.",
            )

            print("CSV:", csv_path)
            print("JSON:", json_path)

            return {"FINISHED"}

        except Exception as error:
            self.report({"ERROR"}, str(error))
            print("FEHLER bei Test + Export CSV + JSON:", error)
            return {"CANCELLED"}


class ROADINPUT_PT_panel(bpy.types.Panel):
    bl_label = "Road Input Exporter"
    bl_idname = "ROADINPUT_PT_panel"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Road Input"

    def draw(self, context):
        layout = self.layout

        layout.label(text="Road Input Exporter v1")

        layout.separator()

        layout.operator(
            "road_input.run_diagnostic_tests",
            text="Run Diagnostic Tests",
            icon="CHECKMARK",
        )

        layout.operator(
            "road_input.export_csv",
            text="Export road_input.csv",
            icon="EXPORT",
        )

        layout.operator(
            "road_input.export_vehicle_parameter_json",
            text="Export vehicle_parameter.json",
            icon="FILE_TICK",
        )

        layout.separator()

        layout.operator(
            "road_input.test_and_export",
            text="Test + Export CSV + JSON",
            icon="PLAY",
        )

        layout.separator()
        layout.label(text="Viertelfahrzeugmodell Parameter")

        parameter = context.scene.road_input_vehicle_parameter

        layout.prop(parameter, "fahrzeugmasse_kg")
        layout.prop(parameter, "gefedertemasse_anteil")
        layout.prop(parameter, "ungefedertemasse_pro_rad_kg")

        layout.separator()
        layout.label(text="Feder / Daempfer / Reifen")

        layout.prop(parameter, "federsteifigkeit_n_pro_m")
        layout.prop(parameter, "daempferkonstante_n_s_pro_m")
        layout.prop(parameter, "reifensteifigkeit_n_pro_m")

        layout.separator()
        layout.label(text="Federweg Grenzen")

        layout.prop(parameter, "max_einfederung_m")
        layout.prop(parameter, "max_ausfederung_m")

        layout.separator()
        layout.label(text="Lastverlagerung")

        layout.prop(parameter, "lastverlagerung_aktiv")
        layout.prop(parameter, "lastverlagerung_laengs_aktiv")
        layout.prop(parameter, "lastverlagerung_quer_aktiv")
        layout.prop(parameter, "schwerpunkt_hoehe_m")
        layout.prop(parameter, "lastverlagerung_laengs_vorzeichen")
        layout.prop(parameter, "lastverlagerung_quer_vorzeichen")


classes = [
    ROADINPUT_VehicleParameter,

    ROADINPUT_OT_run_diagnostic_tests,
    ROADINPUT_OT_export_csv,
    ROADINPUT_OT_export_vehicle_parameter_json,
    ROADINPUT_OT_test_and_export,

    ROADINPUT_PT_panel,
]


def unregister():
    if hasattr(bpy.types.Scene, "road_input_vehicle_parameter"):
        del bpy.types.Scene.road_input_vehicle_parameter

    for cls in reversed(classes):
        try:
            bpy.utils.unregister_class(cls)
        except RuntimeError:
            pass
        except ValueError:
            pass


def register():
    unregister()

    for cls in classes:
        bpy.utils.register_class(cls)

    bpy.types.Scene.road_input_vehicle_parameter = bpy.props.PointerProperty(
        type=ROADINPUT_VehicleParameter
    )


if __name__ == "__main__":
    register()
