# OpenRoad Dynamics - proving-ground level generator (Unreal Editor Python).
# SPDX-License-Identifier: MIT
#
# NOT RUN in this repository (no Unreal Engine in CI). Written against the UE
# 5.4 editor Python API; see docs/PROVING_GROUND.md for how to run it and for
# troubleshooting.
#
# Builds /Game/OpenRoad/ProvingGround (a new level) from engine basic shapes:
#   * ground slab, 1.6 km x 0.6 km, top at Z = 0
#   * 1.2 km acceleration / braking strip (start line at X = 0) with 100 m boards
#   * skidpad, R = 40 m (ring of marker posts), as in ord::scenarios::skidpad()
#   * slalom, 10 cones at 18 m spacing (physics-simulated, ignored by the
#     ground traces so the tires do not ride over them)
#   * curb lane: 8 cm and 12 cm curbs (0.5 m long, 0.6 m wide, under the left
#     or right wheels, 2 cm "chamfer" steps) as in ord::scenarios::curbStrike(),
#     an 8 cm x 0.5 m sin^2 speed bump (sliced) as in speedBump(), a rumble
#     sequence and two long raised kerb strips (8 cm and 12 cm)
#   * wet braking lane (2 mm proud slab) with PM_OpenRoad_Wet, mapped by the
#     surface table to frictionScale 0.75, wetSpeedFalloff 0.004 (the
#     ordbench wet case)
#   * lights, PlayerStart, an auto-possessed AOpenRoadDemoVehicle, World
#     Settings (game mode, gravity -980.665 cm/s^2 for parity runs)
#
# Units: Unreal centimetres, X forward, Y right, Z up.
#
# Run (Output Log, "Python" mode):   import build_proving_ground; build_proving_ground.build()
# or (Output Log, "Cmd" mode):        py "<Project>/Plugins/OpenRoadDynamics/Content/Python/build_proving_ground.py"

import math

import unreal

LEVEL_PATH = "/Game/OpenRoad/ProvingGround"
ASSET_DIR = "/Game/OpenRoad"
CUBE = "/Engine/BasicShapes/Cube.Cube"          # 100 cm cube, pivot at the centre
CYLINDER = "/Engine/BasicShapes/Cylinder.Cylinder"  # 100 cm tall, 50 cm radius, pivot at the centre

# Lane centre lines (Y, cm) and section origins (X, cm).
STRIP_Y = 0.0
WET_Y = 4000.0
SLALOM_Y = 10000.0
CURB_Y = 20000.0
SKIDPAD_CENTER = (30000.0, -15000.0)
SKIDPAD_RADIUS = 4000.0
HALF_TRACK = 80.0   # demo car: wheel centres at Y = -80 (left) and +80 (right) cm


# ----------------------------------------------------------------------------- helpers

def _actors():
    return unreal.get_editor_subsystem(unreal.EditorActorSubsystem)


def _cosmetic(what, fn):
    """Runs a purely cosmetic step; an API difference there must not abort the build."""
    try:
        fn()
    except Exception as error:  # noqa: BLE001 - editor API names differ between versions
        unreal.log_warning("OpenRoad: skipped %s (%s)" % (what, error))


def _load(path):
    asset = unreal.EditorAssetLibrary.load_asset(path)
    if asset is None:
        raise RuntimeError("could not load " + path)
    return asset


class Builder:
    def __init__(self):
        self.cube = _load(CUBE)
        self.cylinder = _load(CYLINDER)
        self.count = 0

    def box(self, label, folder, center, size, collision=True, phys_mat=None, yaw=0.0):
        """Static cube; `center` and `size` in cm (size = full extents)."""
        actor = _actors().spawn_actor_from_class(
            unreal.StaticMeshActor, unreal.Vector(*center), unreal.Rotator(0.0, 0.0, yaw))
        actor.set_actor_label(label)
        actor.set_folder_path(folder)
        actor.set_actor_scale3d(unreal.Vector(size[0] / 100.0, size[1] / 100.0, size[2] / 100.0))
        comp = actor.static_mesh_component
        comp.set_static_mesh(self.cube)
        comp.set_mobility(unreal.ComponentMobility.STATIC)
        if not collision:
            comp.set_collision_enabled(unreal.CollisionEnabled.NO_COLLISION)
        if phys_mat is not None:
            comp.set_phys_material_override(phys_mat)
        self.count += 1
        return actor

    def slab(self, label, folder, x0, x1, y0, y1, top, thickness=100.0, phys_mat=None):
        """Box whose top face is at Z = `top`, spanning [x0, x1] x [y0, y1]."""
        return self.box(label, folder, ((x0 + x1) / 2, (y0 + y1) / 2, top - thickness / 2),
                        (x1 - x0, y1 - y0, thickness), phys_mat=phys_mat)

    def post(self, label, folder, x, y, height=60.0, width=20.0):
        """Visual-only marker (no collision, invisible to the ground traces)."""
        return self.box(label, folder, (x, y, height / 2), (width, width, height), collision=False)

    def cone(self, label, folder, x, y):
        actor = _actors().spawn_actor_from_class(
            unreal.StaticMeshActor, unreal.Vector(x, y, 36.0), unreal.Rotator(0.0, 0.0, 0.0))
        actor.set_actor_label(label)
        actor.set_folder_path(folder)
        actor.set_actor_scale3d(unreal.Vector(0.3, 0.3, 0.7))
        comp = actor.static_mesh_component
        comp.set_static_mesh(self.cylinder)
        comp.set_mobility(unreal.ComponentMobility.MOVABLE)
        comp.set_collision_profile_name("PhysicsActor")
        comp.set_simulate_physics(True)
        comp.set_mass_override_in_kg("", 4.0, True)
        # The vehicle's ground traces use ECC_Visibility: cones must not count as road.
        comp.set_collision_response_to_channel(unreal.CollisionChannel.ECC_VISIBILITY,
                                               unreal.CollisionResponseType.ECR_IGNORE)
        self.count += 1
        return actor

    def label(self, text, x, y, z=250.0, yaw=180.0):
        actor = _actors().spawn_actor_from_class(
            unreal.TextRenderActor, unreal.Vector(x, y, z), unreal.Rotator(0.0, 0.0, yaw))
        actor.set_actor_label("Label_" + text.replace(" ", "_"))
        actor.set_folder_path("ProvingGround/Labels")
        comp = actor.text_render
        _cosmetic("label text", lambda: comp.set_editor_property("text", text))
        _cosmetic("label size", lambda: comp.set_editor_property("world_size", 120.0))
        _cosmetic("label alignment",
                  lambda: comp.set_editor_property("horizontal_alignment", unreal.HorizTextAligment.EHTA_CENTER))
        self.count += 1
        return actor


# ----------------------------------------------------------------------------- assets

def _create_or_load(name, asset_class, factory):
    path = ASSET_DIR + "/" + name
    if unreal.EditorAssetLibrary.does_asset_exist(path):
        return unreal.EditorAssetLibrary.load_asset(path)
    tools = unreal.AssetToolsHelpers.get_asset_tools()
    asset = tools.create_asset(name, ASSET_DIR, asset_class, factory)
    if asset is None:
        raise RuntimeError("could not create " + path)
    return asset


def build_assets():
    """Physical materials + the OpenRoad surface table that maps them."""
    wet = _create_or_load("PM_OpenRoad_Wet", unreal.PhysicalMaterial, unreal.PhysicalMaterialFactoryNew())
    # Chaos body contacts only (e.g. the chassis box sliding); tire grip comes from the surface table.
    _cosmetic("wet physical material friction", lambda: wet.set_editor_property("friction", 0.5))
    curb = _create_or_load("PM_OpenRoad_Curb", unreal.PhysicalMaterial, unreal.PhysicalMaterialFactoryNew())

    factory = unreal.DataAssetFactory()
    factory.set_editor_property("data_asset_class", unreal.OpenRoadSurfaceTable)
    table = _create_or_load("DA_OpenRoad_ProvingGroundSurfaces", unreal.OpenRoadSurfaceTable, factory)

    wet_surface = unreal.OpenRoadSurface()
    wet_surface.set_editor_property("friction_scale", 0.75)
    wet_surface.set_editor_property("wet_speed_falloff", 0.004)
    wet_surface.set_editor_property("surface_id", 1)
    curb_surface = unreal.OpenRoadSurface()
    curb_surface.set_editor_property("roughness", 0.6)  # dense strip sampling, as in curbStrike()
    curb_surface.set_editor_property("surface_id", 2)
    surfaces = unreal.Map(unreal.PhysicalMaterial, unreal.OpenRoadSurface)
    surfaces[wet] = wet_surface
    surfaces[curb] = curb_surface
    table.set_editor_property("surfaces", surfaces)

    for asset in (wet, curb, table):
        unreal.EditorAssetLibrary.save_loaded_asset(asset)
    return wet, curb, table


# ----------------------------------------------------------------------------- sections

def build_environment(b):
    les = _actors()
    sun = les.spawn_actor_from_class(unreal.DirectionalLight, unreal.Vector(0, 0, 1000), unreal.Rotator(0, -40, 30))
    sun.set_actor_label("Sun")
    sun.set_folder_path("ProvingGround/Lighting")
    _cosmetic("sun as atmosphere light", lambda: sun.light_component.set_editor_property("atmosphere_sun_light", True))
    sky = les.spawn_actor_from_class(unreal.SkyLight, unreal.Vector(0, 0, 1000), unreal.Rotator(0, 0, 0))
    sky.set_actor_label("SkyLight")
    sky.set_folder_path("ProvingGround/Lighting")
    _cosmetic("sky light real-time capture", lambda: sky.light_component.set_editor_property("real_time_capture", True))
    atmosphere = les.spawn_actor_from_class(unreal.SkyAtmosphere, unreal.Vector(0, 0, 0), unreal.Rotator(0, 0, 0))
    atmosphere.set_folder_path("ProvingGround/Lighting")

    # Ground: one slab, top at Z = 0.
    b.slab("Ground", "ProvingGround/Ground", -30000.0, 130000.0, -30000.0, 30000.0, top=0.0)


def build_strip(b):
    """1.2 km straight: 0-100 / 100-0 / top speed. Start line at X = 0, heading +X."""
    folder = "ProvingGround/AccelerationStrip"
    for x in range(0, 120001, 10000):
        for side in (-1, 1):
            b.post("Board_%dm_%s" % (x // 100, "L" if side < 0 else "R"), folder, float(x), STRIP_Y + side * 800.0,
                   height=150.0 if x % 50000 == 0 else 80.0)
    # Lane edges (visual-only, flush with the road).
    for side in (-1, 1):
        b.box("LaneEdge_%s" % ("L" if side < 0 else "R"), folder, (60000.0, STRIP_Y + side * 600.0, 0.5),
              (120000.0, 15.0, 1.0), collision=False)
    b.box("StartLine", folder, (0.0, STRIP_Y, 0.5), (30.0, 1200.0, 1.0), collision=False)
    b.box("FinishLine", folder, (120000.0, STRIP_Y, 0.5), (30.0, 1200.0, 1.0), collision=False)
    b.label("ACCEL / BRAKE 1.2 km", 0.0, STRIP_Y - 1000.0)


def build_skidpad(b):
    """R = 40 m circle (ord::scenarios::skidpad default), counter-clockwise from its south point."""
    folder = "ProvingGround/Skidpad"
    cx, cy = SKIDPAD_CENTER
    for ring, radius, n in (("Inner", SKIDPAD_RADIUS - 300.0, 48), ("Path", SKIDPAD_RADIUS, 24), ("Outer", SKIDPAD_RADIUS + 300.0, 48)):
        for k in range(n):
            a = 2.0 * math.pi * k / n
            b.post("Skidpad_%s_%02d" % (ring, k), folder, cx + radius * math.cos(a), cy + radius * math.sin(a),
                   height=30.0 if ring == "Path" else 60.0, width=15.0 if ring == "Path" else 25.0)
    b.post("Skidpad_Center", folder, cx, cy, height=200.0, width=40.0)
    b.label("SKIDPAD R40", cx, cy - SKIDPAD_RADIUS - 600.0)


def build_slalom(b):
    """10 cones, 18 m apart (first at X = 100 m)."""
    folder = "ProvingGround/Slalom"
    for k in range(10):
        b.cone("Cone_%02d" % k, folder, 10000.0 + k * 1800.0, SLALOM_Y)
    b.label("SLALOM 18 m", 7000.0, SLALOM_Y)


def _curb(b, name, folder, x0, y_center, height, phys_mat, length=50.0, width=60.0, chamfer=2.0):
    """Raised block `length` cm along X with `chamfer` cm half-height steps at both ends
    (approximates the 2 cm edge ramps of ord::scenarios::curbStrike)."""
    core_len = length - 2.0 * chamfer
    b.slab(name, folder, x0 + chamfer, x0 + chamfer + core_len, y_center - width / 2, y_center + width / 2,
           top=height, thickness=height, phys_mat=phys_mat)
    for end, xa in (("In", x0), ("Out", x0 + length - chamfer)):
        b.slab(name + "_Chamfer" + end, folder, xa, xa + chamfer, y_center - width / 2, y_center + width / 2,
               top=height / 2, thickness=height / 2, phys_mat=phys_mat)


def build_curb_lane(b, curb_pm):
    """Curb strikes, speed bump, rumble sequence and long kerb strips along Y = CURB_Y."""
    folder = "ProvingGround/Curbs"
    left = CURB_Y - HALF_TRACK
    right = CURB_Y + HALF_TRACK
    _curb(b, "Curb08_Left", folder, 10000.0, left, 8.0, curb_pm)
    _curb(b, "Curb12_Left", folder, 15000.0, left, 12.0, curb_pm)
    _curb(b, "Curb12_Right", folder, 25000.0, right, 12.0, curb_pm)

    # Speed bump: h(x) = 8 cm * sin^2(pi x / 50 cm), full width, as 10 slices.
    n = 10
    for k in range(n):
        xm = (k + 0.5) / n
        h = 8.0 * math.sin(math.pi * xm) ** 2
        x0 = 20000.0 + k * 50.0 / n
        b.slab("Bump08_%02d" % k, folder, x0, x0 + 50.0 / n, CURB_Y - 400.0, CURB_Y + 400.0,
               top=h, thickness=max(h, 0.5), phys_mat=curb_pm)

    # Rumble: 8 cm curbs every 1 m under the left wheels, 20 m.
    for k in range(20):
        _curb(b, "Rumble08_%02d" % k, folder, 30000.0 + k * 100.0, left, 8.0, curb_pm, length=40.0)

    # Long kerb strips (0.5 m wide, 40 m long) on the left edge: drive onto them at an angle.
    for name, x0, height in (("Kerb08", 40000.0, 8.0), ("Kerb12", 46000.0, 12.0)):
        y_outer = CURB_Y - 250.0
        b.slab(name, folder, x0, x0 + 4000.0, y_outer - 50.0 + 2.0, y_outer, top=height, thickness=height, phys_mat=curb_pm)
        b.slab(name + "_Chamfer", folder, x0, x0 + 4000.0, y_outer - 50.0, y_outer - 50.0 + 2.0,
               top=height / 2, thickness=height / 2, phys_mat=curb_pm)

    for x, text in ((10000.0, "CURB 8"), (15000.0, "CURB 12"), (20000.0, "BUMP 8"), (25000.0, "CURB 12 R"),
                    (30000.0, "RUMBLE"), (40000.0, "KERB 8"), (46000.0, "KERB 12")):
        b.label(text, x - 1500.0, CURB_Y + 600.0)


def build_wet(b, wet_pm):
    """Wet braking lane: 400 m x 12 m, 2 mm proud of the ground so traces hit it first."""
    b.slab("WetLane", "ProvingGround/Wet", 0.0, 40000.0, WET_Y - 600.0, WET_Y + 600.0, top=0.2, thickness=10.0, phys_mat=wet_pm)
    b.label("WET mu x0.75", 0.0, WET_Y)


def build_player(table):
    les = _actors()
    start_location = unreal.Vector(-3000.0, STRIP_Y, 100.0)
    start = les.spawn_actor_from_class(unreal.PlayerStart, start_location, unreal.Rotator(0, 0, 0))
    start.set_folder_path("ProvingGround/Player")

    pawn = les.spawn_actor_from_class(unreal.OpenRoadDemoVehicle, unreal.Vector(-3000.0, STRIP_Y, 75.0), unreal.Rotator(0, 0, 0))
    pawn.set_actor_label("OpenRoadDemoVehicle")
    pawn.set_folder_path("ProvingGround/Player")
    pawn.set_editor_property("auto_possess_player", unreal.AutoReceiveInput.PLAYER0)
    vehicle = pawn.get_component_by_class(unreal.OpenRoadVehicleComponent)
    vehicle.set_editor_property("surface_table", table)
    return pawn


def configure_world():
    world = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).get_editor_world()
    settings = world.get_world_settings()
    settings.set_editor_property("default_game_mode", unreal.OpenRoadDemoGameMode)
    # The core uses g = 9.80665 m/s^2; UE's default is 980 cm/s^2.
    settings.set_editor_property("global_gravity_set", True)
    settings.set_editor_property("global_gravity_z", -980.665)


# ----------------------------------------------------------------------------- entry point

def build(level_path=LEVEL_PATH):
    level_editor = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
    if unreal.EditorAssetLibrary.does_asset_exist(level_path):
        unreal.log_warning("OpenRoad: %s exists; opening it and adding to it is not supported. "
                           "Delete it (or pass another path) and run again." % level_path)
        return False
    if not level_editor.new_level(level_path):
        raise RuntimeError("could not create level " + level_path)

    wet_pm, curb_pm, table = build_assets()
    b = Builder()
    with unreal.ScopedSlowTask(7, "Building OpenRoad proving ground") as task:
        task.make_dialog(True)
        for step in (lambda: build_environment(b), lambda: build_strip(b), lambda: build_skidpad(b),
                     lambda: build_slalom(b), lambda: build_curb_lane(b, curb_pm), lambda: build_wet(b, wet_pm),
                     lambda: build_player(table)):
            task.enter_progress_frame(1)
            step()
    configure_world()
    level_editor.save_current_level()
    unreal.log("OpenRoad: proving ground built at %s (%d actors). Press Play." % (level_path, b.count))
    return True


if __name__ == "__main__":
    build()
