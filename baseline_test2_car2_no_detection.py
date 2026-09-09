#!/usr/bin/env python

# Copyright (c) 2018 Intel Labs.
# authors: German Ros (german.ros@intel.com)
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Example of automatic vehicle control from client side."""

from __future__ import print_function

import argparse
import collections
import datetime
import glob
import logging
import math
import os
import numpy.random as random
import re
import sys
import weakref

try:
    import pygame
    from pygame.locals import KMOD_CTRL
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_q
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError(
        'cannot import numpy, make sure numpy package is installed')

# ==============================================================================
# -- Find CARLA module ---------------------------------------------------------
# ==============================================================================
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

# ==============================================================================
# -- Add PythonAPI for release mode --------------------------------------------
# ==============================================================================
try:
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))) + '/carla')
except IndexError:
    pass

import carla
from carla import ColorConverter as cc

from agents.navigation.behavior_agent import BehaviorAgent  # pylint: disable=import-error
from agents.navigation.basic_agent import BasicAgent  # pylint: disable=import-error
from agents.navigation.constant_velocity_agent import ConstantVelocityAgent  # pylint: disable=import-error


# ==============================================================================
# -- My Imports ----------------------------------------------------------
# ==============================================================================
from CommonRoadSceneGenerator import CommonRoadSceneGenerator
from PyQt6.QtWidgets import QApplication
from mp_visualizer.CommonRoadVisualizer import CommonRoadVisualizer
from PyQt6.QtCore import QTimer
from VisualizationThread import VisualizationThread
from synchroniser.synchroniser6 import Subscriber
import time
from occupation_grid.occupation_grid_with_grid_generator.occupation_grid import OccupationGrid
from hybid_a_star_agent.MotionPlanning.HybridAstarPlanner import hybrid_astar
from agents.navigation.controller import VehiclePIDController
import copy
from scipy.interpolate import CubicSpline
import numpy as np
import os
from scipy.ndimage import gaussian_filter1d
import csv
import threading
# ==============================================================================
# -- Global functions ----------------------------------------------------------
# ==============================================================================


def find_weather_presets():
    """Method to find weather presets"""
    rgx = re.compile('.+?(?:(?<=[a-z])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])|$)')
    def name(x): return ' '.join(m.group(0) for m in rgx.finditer(x))
    presets = [x for x in dir(carla.WeatherParameters) if re.match('[A-Z].+', x)]
    return [(getattr(carla.WeatherParameters, x), name(x)) for x in presets]


def get_actor_display_name(actor, truncate=250):
    """Method to get actor display name"""
    name = ' '.join(actor.type_id.replace('_', '.').title().split('.')[1:])
    return (name[:truncate - 1] + u'\u2026') if len(name) > truncate else name

def get_actor_blueprints(world, filter, generation):
    bps = world.get_blueprint_library().filter(filter)

    if generation.lower() == "all":
        return bps

    # If the filter returns only one bp, we assume that this one needed
    # and therefore, we ignore the generation
    if len(bps) == 1:
        return bps

    try:
        int_generation = int(generation)
        # Check if generation is in available generations
        if int_generation in [1, 2, 3]:
            bps = [x for x in bps if int(x.get_attribute('generation')) == int_generation]
            return bps
        else:
            print("   Warning! Actor Generation is not valid. No actor will be spawned.")
            return []
    except:
        print("   Warning! Actor Generation is not valid. No actor will be spawned.")
        return []

# ==============================================================================
# -- World ---------------------------------------------------------------
# ==============================================================================

class World(object):
    """ Class representing the surrounding environment """

    def __init__(self, carla_world, hud, args):
        """Constructor method"""
        self._args = args
        self.world = carla_world
        try:
            self.map = self.world.get_map()
        except RuntimeError as error:
            print('RuntimeError: {}'.format(error))
            print('  The server could not send the OpenDRIVE (.xodr) file:')
            print('  Make sure it exists, has the same name of your town, and is correct.')
            sys.exit(1)
        self.hud = hud
        self.player = None
        self.collision_sensor = None
        self.lane_invasion_sensor = None
        self.gnss_sensor = None
        self.camera_manager = None
        self._weather_presets = find_weather_presets()
        self._weather_index = 0
        self._actor_filter = args.filter
        self._actor_generation = args.generation
        self.restart(args)
        self.world.on_tick(hud.on_world_tick)
        self.recording_enabled = False
        self.recording_start = 0

    def restart(self, args):
        """Restart the world"""
        # Keep same camera config if the camera manager exists.
        cam_index = self.camera_manager.index if self.camera_manager is not None else 0
        cam_pos_id = self.camera_manager.transform_index if self.camera_manager is not None else 0

        # Get a random blueprint.
        blueprint_list = get_actor_blueprints(self.world, self._actor_filter, self._actor_generation)
        if not blueprint_list:
            raise ValueError("Couldn't find any blueprints with the specified filters")
        blueprint = random.choice(blueprint_list)
        blueprint.set_attribute('role_name', 'hero')
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)

        # Spawn the player.
        if self.player is not None:
            spawn_point = self.player.get_transform()
            spawn_point.location.z += 2.0
            spawn_point.rotation.roll = 0.0
            spawn_point.rotation.pitch = 0.0
            self.destroy()
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)
            self.modify_vehicle_physics(self.player)
        while self.player is None:
            if not self.map.get_spawn_points():
                print('There are no spawn points available in your map/town.')
                print('Please add some Vehicle Spawn Point to your UE4 scene.')
                sys.exit(1)
            spawn_points = self.map.get_spawn_points()
            spawn_point = random.choice(spawn_points) if spawn_points else carla.Transform()
            spawn_point = spawn_points[5]
            custom_location = carla.Location(x=24.5, y=40, z=0.5)
            custom_rotation = carla.Rotation(pitch=0, yaw=90, roll=0)
            spawn_point = carla.Transform(custom_location, custom_rotation)
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)
            self.modify_vehicle_physics(self.player)

        if self._args.sync:
            self.world.tick()
        else:
            self.world.wait_for_tick()

        # Set up the sensors.
        self.collision_sensor = CollisionSensor(self.player, self.hud)
        self.lane_invasion_sensor = LaneInvasionSensor(self.player, self.hud)
        self.gnss_sensor = GnssSensor(self.player)
        self.camera_manager = CameraManager(self.player, self.hud)
        self.camera_manager.transform_index = cam_pos_id
        self.camera_manager.set_sensor(cam_index, notify=False)
        actor_type = get_actor_display_name(self.player)
        self.hud.notification(actor_type)

    def next_weather(self, reverse=False):
        """Get next weather setting"""
        self._weather_index += -1 if reverse else 1
        self._weather_index %= len(self._weather_presets)
        preset = self._weather_presets[self._weather_index]
        self.hud.notification('Weather: %s' % preset[1])
        self.player.get_world().set_weather(preset[0])

    def modify_vehicle_physics(self, actor):
        #If actor is not a vehicle, we cannot use the physics control
        try:
            physics_control = actor.get_physics_control()
            physics_control.use_sweep_wheel_collision = True
            actor.apply_physics_control(physics_control)
        except Exception:
            pass

    def tick(self, clock):
        """Method for every tick"""
        self.hud.tick(self, clock)

    def render(self, display):
        """Render world"""
        self.camera_manager.render(display)
        self.hud.render(display)

    def destroy_sensors(self):
        """Destroy sensors"""
        self.camera_manager.sensor.destroy()
        self.camera_manager.sensor = None
        self.camera_manager.index = None

    def destroy(self):
        """Destroys all actors"""
        actors = [
            self.camera_manager.sensor,
            self.collision_sensor.sensor,
            self.lane_invasion_sensor.sensor,
            self.gnss_sensor.sensor,
            self.player]
        for actor in actors:
            if actor is not None:
                actor.destroy()


# ==============================================================================
# -- KeyboardControl -----------------------------------------------------------
# ==============================================================================


class KeyboardControl(object):
    def __init__(self, world):
        world.hud.notification("Press 'H' or '?' for help.", seconds=4.0)

    def parse_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
            if event.type == pygame.KEYUP:
                if self._is_quit_shortcut(event.key):
                    return True

    @staticmethod
    def _is_quit_shortcut(key):
        """Shortcut for quitting"""
        return (key == K_ESCAPE) or (key == K_q and pygame.key.get_mods() & KMOD_CTRL)

# ==============================================================================
# -- HUD -----------------------------------------------------------------------
# ==============================================================================


class HUD(object):
    """Class for HUD text"""

    def __init__(self, width, height):
        """Constructor method"""
        self.dim = (width, height)
        font = pygame.font.Font(pygame.font.get_default_font(), 20)
        font_name = 'courier' if os.name == 'nt' else 'mono'
        fonts = [x for x in pygame.font.get_fonts() if font_name in x]
        default_font = 'ubuntumono'
        mono = default_font if default_font in fonts else fonts[0]
        mono = pygame.font.match_font(mono)
        self._font_mono = pygame.font.Font(mono, 12 if os.name == 'nt' else 14)
        self._notifications = FadingText(font, (width, 40), (0, height - 40))
        self.help = HelpText(pygame.font.Font(mono, 24), width, height)
        self.server_fps = 0
        self.frame = 0
        self.simulation_time = 0
        self._show_info = True
        self._info_text = []
        self._server_clock = pygame.time.Clock()

    def on_world_tick(self, timestamp):
        """Gets informations from the world at every tick"""
        self._server_clock.tick()
        self.server_fps = self._server_clock.get_fps()
        self.frame = timestamp.frame_count
        self.simulation_time = timestamp.elapsed_seconds

    def tick(self, world, clock):
        """HUD method for every tick"""
        self._notifications.tick(world, clock)
        if not self._show_info:
            return
        transform = world.player.get_transform()
        vel = world.player.get_velocity()
        control = world.player.get_control()
        heading = 'N' if abs(transform.rotation.yaw) < 89.5 else ''
        heading += 'S' if abs(transform.rotation.yaw) > 90.5 else ''
        heading += 'E' if 179.5 > transform.rotation.yaw > 0.5 else ''
        heading += 'W' if -0.5 > transform.rotation.yaw > -179.5 else ''
        colhist = world.collision_sensor.get_collision_history()
        collision = [colhist[x + self.frame - 200] for x in range(0, 200)]
        max_col = max(1.0, max(collision))
        collision = [x / max_col for x in collision]
        vehicles = world.world.get_actors().filter('vehicle.*')

        self._info_text = [
            'Server:  % 16.0f FPS' % self.server_fps,
            'Client:  % 16.0f FPS' % clock.get_fps(),
            '',
            'Vehicle: % 20s' % get_actor_display_name(world.player, truncate=20),
            'Map:     % 20s' % world.map.name.split('/')[-1],
            'Simulation time: % 12s' % datetime.timedelta(seconds=int(self.simulation_time)),
            '',
            'Speed:   % 15.0f km/h' % (3.6 * math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)),
            u'Heading:% 16.0f\N{DEGREE SIGN} % 2s' % (transform.rotation.yaw, heading),
            'Location:% 20s' % ('(% 5.1f, % 5.1f)' % (transform.location.x, transform.location.y)),
            'GNSS:% 24s' % ('(% 2.6f, % 3.6f)' % (world.gnss_sensor.lat, world.gnss_sensor.lon)),
            'Height:  % 18.0f m' % transform.location.z,
            '']
        if isinstance(control, carla.VehicleControl):
            self._info_text += [
                ('Throttle:', control.throttle, 0.0, 1.0),
                ('Steer:', control.steer, -1.0, 1.0),
                ('Brake:', control.brake, 0.0, 1.0),
                ('Reverse:', control.reverse),
                ('Hand brake:', control.hand_brake),
                ('Manual:', control.manual_gear_shift),
                'Gear:        %s' % {-1: 'R', 0: 'N'}.get(control.gear, control.gear)]
        elif isinstance(control, carla.WalkerControl):
            self._info_text += [
                ('Speed:', control.speed, 0.0, 5.556),
                ('Jump:', control.jump)]
        self._info_text += [
            '',
            'Collision:',
            collision,
            '',
            'Number of vehicles: % 8d' % len(vehicles)]

        if len(vehicles) > 1:
            self._info_text += ['Nearby vehicles:']

        def dist(l):
            return math.sqrt((l.x - transform.location.x)**2 + (l.y - transform.location.y)
                             ** 2 + (l.z - transform.location.z)**2)
        vehicles = [(dist(x.get_location()), x) for x in vehicles if x.id != world.player.id]

        for dist, vehicle in sorted(vehicles):
            if dist > 200.0:
                break
            vehicle_type = get_actor_display_name(vehicle, truncate=22)
            self._info_text.append('% 4dm %s' % (dist, vehicle_type))

    def toggle_info(self):
        """Toggle info on or off"""
        self._show_info = not self._show_info

    def notification(self, text, seconds=2.0):
        """Notification text"""
        self._notifications.set_text(text, seconds=seconds)

    def error(self, text):
        """Error text"""
        self._notifications.set_text('Error: %s' % text, (255, 0, 0))

    def render(self, display):
        """Render for HUD class"""
        if self._show_info:
            info_surface = pygame.Surface((220, self.dim[1]))
            info_surface.set_alpha(100)
            display.blit(info_surface, (0, 0))
            v_offset = 4
            bar_h_offset = 100
            bar_width = 106
            for item in self._info_text:
                if v_offset + 18 > self.dim[1]:
                    break
                if isinstance(item, list):
                    if len(item) > 1:
                        points = [(x + 8, v_offset + 8 + (1 - y) * 30) for x, y in enumerate(item)]
                        pygame.draw.lines(display, (255, 136, 0), False, points, 2)
                    item = None
                    v_offset += 18
                elif isinstance(item, tuple):
                    if isinstance(item[1], bool):
                        rect = pygame.Rect((bar_h_offset, v_offset + 8), (6, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect, 0 if item[1] else 1)
                    else:
                        rect_border = pygame.Rect((bar_h_offset, v_offset + 8), (bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect_border, 1)
                        fig = (item[1] - item[2]) / (item[3] - item[2])
                        if item[2] < 0.0:
                            rect = pygame.Rect(
                                (bar_h_offset + fig * (bar_width - 6), v_offset + 8), (6, 6))
                        else:
                            rect = pygame.Rect((bar_h_offset, v_offset + 8), (fig * bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect)
                    item = item[0]
                if item:  # At this point has to be a str.
                    surface = self._font_mono.render(item, True, (255, 255, 255))
                    display.blit(surface, (8, v_offset))
                v_offset += 18
        self._notifications.render(display)
        self.help.render(display)

# ==============================================================================
# -- FadingText ----------------------------------------------------------------
# ==============================================================================


class FadingText(object):
    """ Class for fading text """

    def __init__(self, font, dim, pos):
        """Constructor method"""
        self.font = font
        self.dim = dim
        self.pos = pos
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)

    def set_text(self, text, color=(255, 255, 255), seconds=2.0):
        """Set fading text"""
        text_texture = self.font.render(text, True, color)
        self.surface = pygame.Surface(self.dim)
        self.seconds_left = seconds
        self.surface.fill((0, 0, 0, 0))
        self.surface.blit(text_texture, (10, 11))

    def tick(self, _, clock):
        """Fading text method for every tick"""
        delta_seconds = 1e-3 * clock.get_time()
        self.seconds_left = max(0.0, self.seconds_left - delta_seconds)
        self.surface.set_alpha(500.0 * self.seconds_left)

    def render(self, display):
        """Render fading text method"""
        display.blit(self.surface, self.pos)

# ==============================================================================
# -- HelpText ------------------------------------------------------------------
# ==============================================================================


class HelpText(object):
    """ Helper class for text render"""

    def __init__(self, font, width, height):
        """Constructor method"""
        lines = __doc__.split('\n')
        self.font = font
        self.dim = (680, len(lines) * 22 + 12)
        self.pos = (0.5 * width - 0.5 * self.dim[0], 0.5 * height - 0.5 * self.dim[1])
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)
        self.surface.fill((0, 0, 0, 0))
        for i, line in enumerate(lines):
            text_texture = self.font.render(line, True, (255, 255, 255))
            self.surface.blit(text_texture, (22, i * 22))
            self._render = False
        self.surface.set_alpha(220)

    def toggle(self):
        """Toggle on or off the render help"""
        self._render = not self._render

    def render(self, display):
        """Render help text method"""
        if self._render:
            display.blit(self.surface, self.pos)

# ==============================================================================
# -- CollisionSensor -----------------------------------------------------------
# ==============================================================================


class CollisionSensor(object):
    """ Class for collision sensors"""

    def __init__(self, parent_actor, hud):
        """Constructor method"""
        self.sensor = None
        self.history = []
        self._parent = parent_actor
        self.hud = hud
        world = self._parent.get_world()
        blueprint = world.get_blueprint_library().find('sensor.other.collision')
        self.sensor = world.spawn_actor(blueprint, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to
        # self to avoid circular reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: CollisionSensor._on_collision(weak_self, event))

    def get_collision_history(self):
        """Gets the history of collisions"""
        history = collections.defaultdict(int)
        for frame, intensity in self.history:
            history[frame] += intensity
        return history

    @staticmethod
    def _on_collision(weak_self, event):
        """On collision method"""
        self = weak_self()
        if not self:
            return
        actor_type = get_actor_display_name(event.other_actor)
        self.hud.notification('Collision with %r' % actor_type)
        impulse = event.normal_impulse
        intensity = math.sqrt(impulse.x ** 2 + impulse.y ** 2 + impulse.z ** 2)
        self.history.append((event.frame, intensity))
        if len(self.history) > 4000:
            self.history.pop(0)

# ==============================================================================
# -- LaneInvasionSensor --------------------------------------------------------
# ==============================================================================


class LaneInvasionSensor(object):
    """Class for lane invasion sensors"""

    def __init__(self, parent_actor, hud):
        """Constructor method"""
        self.sensor = None
        self._parent = parent_actor
        self.hud = hud
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.lane_invasion')
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: LaneInvasionSensor._on_invasion(weak_self, event))

    @staticmethod
    def _on_invasion(weak_self, event):
        """On invasion method"""
        self = weak_self()
        if not self:
            return
        lane_types = set(x.type for x in event.crossed_lane_markings)
        text = ['%r' % str(x).split()[-1] for x in lane_types]
        self.hud.notification('Crossed line %s' % ' and '.join(text))

# ==============================================================================
# -- GnssSensor --------------------------------------------------------
# ==============================================================================


class GnssSensor(object):
    """ Class for GNSS sensors"""

    def __init__(self, parent_actor):
        """Constructor method"""
        self.sensor = None
        self._parent = parent_actor
        self.lat = 0.0
        self.lon = 0.0
        world = self._parent.get_world()
        blueprint = world.get_blueprint_library().find('sensor.other.gnss')
        self.sensor = world.spawn_actor(blueprint, carla.Transform(carla.Location(x=1.0, z=2.8)),
                                        attach_to=self._parent)
        # We need to pass the lambda a weak reference to
        # self to avoid circular reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: GnssSensor._on_gnss_event(weak_self, event))

    @staticmethod
    def _on_gnss_event(weak_self, event):
        """GNSS method"""
        self = weak_self()
        if not self:
            return
        self.lat = event.latitude
        self.lon = event.longitude

# ==============================================================================
# -- CameraManager -------------------------------------------------------------
# ==============================================================================


class CameraManager(object):
    """ Class for camera management"""

    def __init__(self, parent_actor, hud):
        """Constructor method"""
        self.sensor = None
        self.surface = None
        self._parent = parent_actor
        self.hud = hud
        self.recording = False
        bound_x = 0.5 + self._parent.bounding_box.extent.x
        bound_y = 0.5 + self._parent.bounding_box.extent.y
        bound_z = 0.5 + self._parent.bounding_box.extent.z
        attachment = carla.AttachmentType
        self._camera_transforms = [
            (carla.Transform(carla.Location(x=-2.0*bound_x, y=+0.0*bound_y, z=2.0*bound_z), carla.Rotation(pitch=8.0)), attachment.SpringArmGhost),
            (carla.Transform(carla.Location(x=+0.8*bound_x, y=+0.0*bound_y, z=1.3*bound_z)), attachment.Rigid),
            (carla.Transform(carla.Location(x=+1.9*bound_x, y=+1.0*bound_y, z=1.2*bound_z)), attachment.SpringArmGhost),
            (carla.Transform(carla.Location(x=-2.8*bound_x, y=+0.0*bound_y, z=4.6*bound_z), carla.Rotation(pitch=6.0)), attachment.SpringArmGhost),
            (carla.Transform(carla.Location(x=-1.0, y=-1.0*bound_y, z=0.4*bound_z)), attachment.Rigid)]

        self.transform_index = 1
        self.sensors = [
            ['sensor.camera.rgb', cc.Raw, 'Camera RGB'],
            ['sensor.camera.depth', cc.Raw, 'Camera Depth (Raw)'],
            ['sensor.camera.depth', cc.Depth, 'Camera Depth (Gray Scale)'],
            ['sensor.camera.depth', cc.LogarithmicDepth, 'Camera Depth (Logarithmic Gray Scale)'],
            ['sensor.camera.semantic_segmentation', cc.Raw, 'Camera Semantic Segmentation (Raw)'],
            ['sensor.camera.semantic_segmentation', cc.CityScapesPalette,
             'Camera Semantic Segmentation (CityScapes Palette)'],
            ['sensor.lidar.ray_cast', None, 'Lidar (Ray-Cast)']]
        world = self._parent.get_world()
        bp_library = world.get_blueprint_library()
        for item in self.sensors:
            blp = bp_library.find(item[0])
            if item[0].startswith('sensor.camera'):
                blp.set_attribute('image_size_x', str(hud.dim[0]))
                blp.set_attribute('image_size_y', str(hud.dim[1]))
            elif item[0].startswith('sensor.lidar'):
                blp.set_attribute('range', '50')
            item.append(blp)
        self.index = None

    def toggle_camera(self):
        """Activate a camera"""
        self.transform_index = (self.transform_index + 1) % len(self._camera_transforms)
        self.set_sensor(self.index, notify=False, force_respawn=True)

    def set_sensor(self, index, notify=True, force_respawn=False):
        """Set a sensor"""
        index = index % len(self.sensors)
        needs_respawn = True if self.index is None else (
            force_respawn or (self.sensors[index][0] != self.sensors[self.index][0]))
        if needs_respawn:
            if self.sensor is not None:
                self.sensor.destroy()
                self.surface = None
            self.sensor = self._parent.get_world().spawn_actor(
                self.sensors[index][-1],
                self._camera_transforms[self.transform_index][0],
                attach_to=self._parent,
                attachment_type=self._camera_transforms[self.transform_index][1])

            # We need to pass the lambda a weak reference to
            # self to avoid circular reference.
            weak_self = weakref.ref(self)
            self.sensor.listen(lambda image: CameraManager._parse_image(weak_self, image))
        if notify:
            self.hud.notification(self.sensors[index][2])
        self.index = index

    def next_sensor(self):
        """Get the next sensor"""
        self.set_sensor(self.index + 1)

    def toggle_recording(self):
        """Toggle recording on or off"""
        self.recording = not self.recording
        self.hud.notification('Recording %s' % ('On' if self.recording else 'Off'))

    def render(self, display):
        """Render method"""
        if self.surface is not None:
            display.blit(self.surface, (0, 0))

    @staticmethod
    def _parse_image(weak_self, image):
        self = weak_self()
        if not self:
            return
        if self.sensors[self.index][0].startswith('sensor.lidar'):
            points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
            points = np.reshape(points, (int(points.shape[0] / 4), 4))
            lidar_data = np.array(points[:, :2])
            lidar_data *= min(self.hud.dim) / 100.0
            lidar_data += (0.5 * self.hud.dim[0], 0.5 * self.hud.dim[1])
            lidar_data = np.fabs(lidar_data)  # pylint: disable=assignment-from-no-return
            lidar_data = lidar_data.astype(np.int32)
            lidar_data = np.reshape(lidar_data, (-1, 2))
            lidar_img_size = (self.hud.dim[0], self.hud.dim[1], 3)
            lidar_img = np.zeros(lidar_img_size)
            lidar_img[tuple(lidar_data.T)] = (255, 255, 255)
            self.surface = pygame.surfarray.make_surface(lidar_img)
        else:
            image.convert(self.sensors[self.index][1])
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        if self.recording:
            image.save_to_disk('_out/%08d' % image.frame)


# ==============================================================================
# -- Vehicle PID Controller ---------------------------------------------------------
# ==============================================================================

class FakeWaypoint:
    def __init__(self, transform):
        self.transform = transform

# Convert from grid map to world coordinates
def grid_to_world(gx, gy, center, cell_size):
    x = (gx - center) * cell_size
    y = (gy - center) * cell_size
    return x, y

def follow_path_with_pid(vehicle, path, speed=20):
    """
    Generator to follow a custom path using PID controller.
    Each iteration yields a control command to apply to the vehicle.
    """
    controller = VehiclePIDController(
        vehicle,
        args_lateral={'K_P': 1.0, 'K_D': 0.0, 'K_I': 0.0},
        args_longitudinal={'K_P': 1.0, 'K_D': 0.0, 'K_I': 0.0}
    )

    index = 0
    num_points = len(path.x)


    

    grid_size = 500  # Assuming a grid size of 500x500
    center = grid_size // 2
    #center = getattr(path, 'center', 100)
    cell_size = 0.5  # Assuming each cell in the grid is 0.5x0.5 meters

    while index < num_points:
        # Generate world coordinate
        world_x, world_y = grid_to_world(path.x[index], path.y[index], center, cell_size)
        target_location = carla.Location(x=world_x, y=world_y, z=vehicle.get_location().z)
        target_yaw = path.yaw[index]
        target_rotation = carla.Rotation(yaw=target_yaw)
        target_transform = carla.Transform(target_location, target_rotation)

        # Wrap in a fake waypoint object so run_step doesn't crash
        fake_wp = FakeWaypoint(target_transform)

        # Get control from PID
        control = controller.run_step(speed, fake_wp)
        yield control

        # Advance to next point if close enough
        if vehicle.get_location().distance(target_location) < 5.0:
            index += 1


# ==============================================================================
# -- Update Path Dynamically ---------------------------------------------------
# ==============================================================================

def closest_point_on_path(path, new_path_point):
    """
    Find the closest point on the path to the new point.
    Returns the index of the closest point and its coordinates.
    """
    min_distance = float('inf')
    closest_index = -1
    closest_point = None

    for i, (x, y) in enumerate(zip(path.x, path.y)):
        distance = math.sqrt((x - new_path_point[0]) ** 2 + (y - new_path_point[1]) ** 2)
        if distance < min_distance:
            min_distance = distance
            closest_index = i
            closest_point = (x, y)

    return closest_index, closest_point

def conflict_area_to_real_grid(data, conflict_area_bounds, grid_size=500, cell_size=0.5):
    '""Convert conflict area bounds and data to real grid coordinates."""'
    print('conversion: data:', data)
    def to_grid(subgrid_row, subgrid_col):
        """
        Convert row and column indices to real grid coordinates.
        """
        x = min_col + subgrid_col
        y = min_row + subgrid_row
        return x, y
    min_row = conflict_area_bounds['min_row']
    max_row = conflict_area_bounds['max_row']
    min_col = conflict_area_bounds['min_col']
    max_col = conflict_area_bounds['max_col']

    # If data is a list of coordinates (row, col)
    if isinstance(data, list) and len(data) > 0 and isinstance(data[0], (list, tuple)):
        real_coords = [to_grid(row, col) for row, col in data]
    # If data is a single coordinate (row, col)
    elif isinstance(data, (list, tuple)) and len(data) == 2:
        real_coords = to_grid(data[0], data[1])
    else:
        real_coords = None  # Unknown format

    return real_coords

def round_off_grid(conflict_area):
    #np.save("small_grids_test/path_non.npy", conflict_area)
    conflict_shape = conflict_area.shape
    max_axis = max(conflict_shape)
    # Choose the new size: round up to nearest 50 or 100
    if max_axis <= 50:
        new_size = 50
    elif max_axis <= 100:
        new_size = 100
    else:
        new_size = ((max_axis + 99) // 100) * 100  # Next multiple of 100

    # Pad conflict_area to new_size x new_size, fill new cells with 1
    pad_y = new_size - conflict_shape[0]
    pad_x = new_size - conflict_shape[1]
    pad_top = pad_y // 2
    pad_bottom = pad_y - pad_top
    pad_left = pad_x // 2
    pad_right = pad_x - pad_left

    conflict_area_expanded = np.pad(
        conflict_area,
        ((pad_top, pad_bottom), (pad_left, pad_right)),
        mode='constant',
        constant_values=1
    )
    print("Expanded conflict_area shape:", conflict_area_expanded.shape)
    #np.save("small_grids_test/path.npy", conflict_area_expanded)
    # Also increment the coordinates of points by the padding
    pad_x = pad_left
    pad_y = pad_top
    # If you have a list of points to adjust, e.g. path_points = [(y, x), ...]
    # You can adjust them like this:
    # adjusted_points = [(y + pad_y, x + pad_x) for (y, x) in path_points]
    # If you want to return the padding values for use elsewhere:
    padding = {
        'pad_x': pad_x,
        'pad_y': pad_y
    }

    return conflict_area_expanded, padding

def stitch_paths(path, updated_path, conflict_area_bounds, padding):
    """
    Stitch the updated_path into the original path using global coordinates.
    Smooth the result with a Gaussian filter.
    Returns the stitched and smoothed path object.
    """
    updated_path_x_global = [x - padding['pad_x'] + conflict_area_bounds['min_col'] for x in updated_path.x]
    updated_path_y_global = [y - padding['pad_y'] + conflict_area_bounds['min_row'] for y in updated_path.y]

    # Find closest points on original path to start and end of updated_path
    start_idx, start_pt = closest_point_on_path(path, (updated_path_x_global[0], updated_path_y_global[0]))
    end_idx, end_pt = closest_point_on_path(path, (updated_path_x_global[-1], updated_path_y_global[-1]))

    # Stitch the new path: remove everything before start_idx, add updated_path, then everything after end_idx
    stitched_x = list(updated_path_x_global) + list(path.x[end_idx+1:])
    stitched_y = list(updated_path_y_global) + list(path.y[end_idx+1:])

    # Make a copy of path before modifying
    new_path = copy.deepcopy(path)

    # Replace path.x and path.y with the stitched path
    new_path.x = stitched_x
    new_path.y = stitched_y

    # Smooth the path using a Gaussian filter
    sigma = 2  # Adjust sigma for more/less smoothing
    new_path.x = gaussian_filter1d(new_path.x, sigma)
    new_path.y = gaussian_filter1d(new_path.y, sigma)

    return new_path

# def mark_parking_lines_on_grid(grid, scenario):



# ==============================================================================
# -- Game Loop ---------------------------------------------------------
# ==============================================================================


def game_loop(args):
    """
    Main loop of the simulation. It handles updating all the HUD information,
    ticking the agent and, if needed, the world.
    """
    # app = QApplication([])
    # # Force initial GUI update
    # QApplication.processEvents()
    
    subscriber = Subscriber()

    pygame.init()
    pygame.font.init()

    world = None

    try:
        if args.seed:
            random.seed(args.seed)

        client = carla.Client(args.host, args.port)
        client.set_timeout(60.0)

        #traffic_manager = client.get_trafficmanager()
        sim_world = client.get_world()

        occupationgrid = OccupationGrid(sim_world, cell_size=0.5)
        grid_map = occupationgrid.grid
        np.save("parking_lines/grid_map.npy", grid_map)

        #occupationgrid.start_visualization()

        if args.sync:
            settings = sim_world.get_settings()
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = 0.05
            sim_world.apply_settings(settings)

            #traffic_manager.set_synchronous_mode(True)
        
        if args.visualize:
            display = pygame.display.set_mode(
                (args.width, args.height),
                pygame.HWSURFACE | pygame.DOUBLEBUF)

        hud = HUD(args.width, args.height)
        world = World(client.get_world(), hud, args)
        controller = KeyboardControl(world)

        # if args.agent == "Basic":
        #     agent = BasicAgent(world.player, 30)
        #     agent.follow_speed_limits(True)
        # elif args.agent == "Constant":
        #     agent = ConstantVelocityAgent(world.player, 30)
        #     ground_loc = world.world.ground_projection(world.player.get_location(), 5)
        #     if ground_loc:
        #         world.player.set_location(ground_loc.location + carla.Location(z=0.01))
        #     agent.follow_speed_limits(True)
        # elif args.agent == "Behavior":
        #     agent = BehaviorAgent(world.player, behavior=args.behavior)

        # Set the agent destination
        spawn_points = world.map.get_spawn_points()
        # destination = random.choice(spawn_points).location
        destination = spawn_points[10].location
        destination = carla.Location(x=24.5, y=70, z=0)
        # agent.set_destination(destination)
        # clock = pygame.time.Clock()

        # Send world.player coordinates and yaw to hybrid_astar
        player_transform = world.player.get_transform()
        player_x = player_transform.location.x
        player_y = player_transform.location.y
        player_yaw = math.radians(player_transform.rotation.yaw)
        destination_transform = carla.Transform(destination, world.player.get_transform().rotation)
        destination_x = destination_transform.location.x
        destination_y = destination_transform.location.y
        destination_yaw = math.radians(player_transform.rotation.yaw)

        path = hybrid_astar.path_finder(player_x, player_y, player_yaw, destination_x, destination_y, destination_yaw, grid_map)

        os.makedirs("path_test", exist_ok=True)
        # with open(os.path.join("path_test", "path.txt"), "w") as f:
        #     for x, y in zip(path.x, path.y):
        #         f.write(f"({x} ,{y})\n")

        # # Convert grid coordinates to world coordinates for the reference path
        # grid_size = 500  # Should match the value used in follow_path_with_pid
        # center = grid_size // 2
        # cell_size = 0.5
        # reference_path = [
        #     [wx, -wy]
        #     for wx, wy in (grid_to_world(x, y, center, cell_size) for x, y in zip(path.x, path.y))
        # ]

        #if args.visualize:
        # Initialize Qt in the main thread
        app = QApplication([])

        test = CommonRoadSceneGenerator(world)
        window = CommonRoadVisualizer(test.base_config, test.scenario, test.planning_problem, test.world, world.player, visualize=args.visualize)
        if args.visualize:
            window.setGeometry(100, 100, 800, 600)
            window.show()
            # Force initial GUI update
        QApplication.processEvents()

        scenario = test.scenario


        # Initialize path follower
        path_follower = follow_path_with_pid(world.player, path, speed=6)
        clock = pygame.time.Clock()
        conflict = False
        new_path = None

        #test = CommonRoadSceneGenerator()
        #test.run()


        # test = CommonRoadSceneGenerator()
        # vis_thread = VisualizationThread(
        #     test.base_config,
        #     test.scenario,
        #     test.planning_problem,
        #     test.world
        # )
        # vis_thread.start()
        # subscriber = Subscriber()
        start_time = time.time()



        while True:
            start_loop_time = None
            end_loop_time = None
            reachability_calculation_start = None
            reachability_calculation_end = None
            occupationgrid_generation_start = None
            occupationgrid_generation_end = None
            hybrid_astar_start = None
            hybrid_astar_end = None
            update_path_start = None
            update_path_end = None
            solution_start = None
            solution_end = None

            start_loop_time = time.time()
            if subscriber.receive_messages():

                # Process Qt events in each iteration
                # QApplication.processEvents()
                clock.tick()
                # if args.sync:
                #     world.world.tick()
                # else:
                #     world.world.wait_for_tick()
                if controller.parse_events():
                    return
                

                world.tick(clock)
                if args.visualize:
                    world.render(display)
                    pygame.display.flip()

                # if agent.done():
                #     if args.loop:
                #         agent.set_destination(random.choice(spawn_points).location)
                #         world.hud.notification("Target reached", seconds=4.0)
                #         print("The target has been reached, searching for another target")
                #     else:
                #         print("The target has been reached, stopping the simulation")
                #         break

                try:
                    control = next(path_follower)
                    control.manual_gear_shift = False
                    world.player.apply_control(control)
                except StopIteration:
                    print("Reached the end of the path.")
                    break
                #test.window.update_visualization()
                        # Update visualization
                reachability_calculation_start = time.time()
                # Get all car objects except the ego vehicle
                all_vehicles = world.world.get_actors().filter('vehicle.*')
                other_cars = [v for v in all_vehicles if v.id != world.player.id]
                # polygons,  decision_polygons = window.update_visualization(other_cars=other_cars)
                # Use threading to call update_visualization
                def update_visualization_thread(result_container):
                    result_container.append(window.update_visualization(other_cars=None))

                result_container = []
                vis_thread = threading.Thread(target=update_visualization_thread, args=(result_container,))
                vis_thread.start()
                vis_thread.join()
                polygons, decision_polygons = result_container[0]
                reachability_calculation_end = time.time()
                occupationgrid_generation_start = time.time()
                reach_occupancygrid, car_box_index = occupationgrid.generate_occupation_grid(world.player, polygons)

                decision_occupancygrid, _ = occupationgrid.generate_occupation_grid(world.player, decision_polygons)
                decision_occupancygrid = decision_occupancygrid.copy().astype(np.int8)
                test_reach_occupancygrid = reach_occupancygrid.copy().astype(np.int8)
                # Mark the path in the occupancy grid as -2
                path_copy = copy.deepcopy(path)
                if car_box_index is not None and len(path_copy.x) > 0:
                    try:
                        # Find the front-most point of the car (in the direction of the path)
                        path_head = np.array([path_copy.x[0], path_copy.y[0]])
                        car_box_array = np.array(car_box_index)
                        # Find which car box point is furthest along the direction to the path head
                        dists_to_path_head = np.linalg.norm(car_box_array - path_head, axis=1)
                        front_idx = int(np.argmin(dists_to_path_head))
                        car_gx, car_gy = car_box_index[front_idx]

                        # Find the closest path point to the front of the car
                        dists = [(gx - car_gx) ** 2 + (gy - car_gy) ** 2 for gx, gy in zip(path_copy.x, path_copy.y)]
                        start_idx = int(np.argmin(dists))

                        # Mark the path from the front of the car to the goal
                        for gx, gy in zip(path_copy.x[start_idx:], path_copy.y[start_idx:]):
                            if 0 <= gx < reach_occupancygrid.shape[0] and 0 <= gy < reach_occupancygrid.shape[1]:
                                grid_x = int(round(gx))
                                grid_y = int(round(gy))
                                if test_reach_occupancygrid[grid_y, grid_x] != 2:  # Avoid overwriting car box
                                    test_reach_occupancygrid[grid_y, grid_x] = -2
                                    decision_occupancygrid[grid_y, grid_x] = -2
                    except Exception as e:
                        print("Error marking path from car front to goal:", e)
                occupationgrid_generation_end = time.time()
                if args.visualize:
                    if new_path is not None:
                        path_copy = copy.deepcopy(new_path)
                        if car_box_index is not None and len(path_copy.x) > 0:
                            try:
                                # Find the front-most point of the car (in the direction of the path)
                                path_head = np.array([path_copy.x[0], path_copy.y[0]])
                                car_box_array = np.array(car_box_index)
                                # Find which car box point is furthest along the direction to the path head
                                dists_to_path_head = np.linalg.norm(car_box_array - path_head, axis=1)
                                front_idx = int(np.argmin(dists_to_path_head))
                                car_gx, car_gy = car_box_index[front_idx]

                                # Find the closest path point to the front of the car
                                dists = [(gx - car_gx) ** 2 + (gy - car_gy) ** 2 for gx, gy in zip(path_copy.x, path_copy.y)]
                                start_idx = int(np.argmin(dists))

                                # Mark the path from the front of the car to the goal
                                for gx, gy in zip(path_copy.x[start_idx:], path_copy.y[start_idx:]):
                                    if 0 <= gx < reach_occupancygrid.shape[0] and 0 <= gy < reach_occupancygrid.shape[1]:
                                        grid_x = int(round(gx))
                                        grid_y = int(round(gy))
                                        if test_reach_occupancygrid[grid_y, grid_x] != 2 and test_reach_occupancygrid[grid_y, grid_x] != -2:  # Avoid overwriting car box
                                            test_reach_occupancygrid[grid_y, grid_x] = 6
                            except Exception as e:
                                print("Error marking path from car front to goal:", e)
                # # Get ego vehicle's world coordinates
                # ego_location = world.player.get_location()
                # ego_x = ego_location.x
                # ego_y = ego_location.y

                # # Convert ego vehicle's world coordinates to grid coordinates
                # ego_gx, ego_gy = hybrid_astar.world_to_grid(ego_x, ego_y, 500 // 2, 0.5)

                # # Mark the ego vehicle's grid cell as -2 in the occupancy grid
                # if 0 <= ego_gy < test_reach_occupancygrid.shape[0] and 0 <= ego_gx < test_reach_occupancygrid.shape[1]:
                #     test_reach_occupancygrid[int(ego_gy), int(ego_gx)] = 3


                solution_start = time.time()
                # sent = subscriber.send_conflict([test_reach_occupancygrid, decision_occupancygrid])
                # if sent:
                #     print("Conflict sent to subscriber")
                #     final_occupancy_grid = subscriber.receive_solution()
                #     print('Received solution from subscriber:', final_occupancy_grid)
                #     if final_occupancy_grid is not None:
                if True:
                    if True:
                        conflict = True
                        # print("Received final occupancy grid from subscriber")
                        # conflict_area = final_occupancy_grid['conflict_area']
                        # new_path_point = final_occupancy_grid['new_path_point']
                        # conflict_area_bounds = final_occupancy_grid['conflict_area_bounds']
                        # real_path_point = conflict_area_to_real_grid(new_path_point, conflict_area_bounds)

                        # # Send world.player coordinates and yaw to hybrid_astar
                        # player_transform = world.player.get_transform()
                        # test_x, test_y =hybrid_astar.world_to_grid(player_transform.location.x, player_transform.location.y, 500 // 2, 0.5)
                        # player_x = player_transform.location.x
                        # player_y = player_transform.location.y
                        # player_yaw = math.radians(player_transform.rotation.yaw)
                        # #destination_transform = carla.Transform(destination, world.player.get_transform().rotation)
                        # destination_x = real_path_point[0]
                        # destination_y = real_path_point[1]
                        # #destination_yaw = math.radians(player_transform.rotation.yaw)
                        # # Expand the conflict_area to a square grid along its longest axis, fill new cells with 1
                        # small_grid, padding = round_off_grid(conflict_area)

                        # print("Control side: player_x, player_ y, destination_x, destination_y:", player_x, player_y, destination_x, destination_y)

                        # print("Control side: player_x, player_y, player_yaw:", test_x-conflict_area_bounds['min_col']+padding['pad_x'], test_y-conflict_area_bounds['min_row']+padding['pad_y'], player_yaw)
                        # print("Control side: destination_x, destination_y, destination_yaw:", destination_x+padding['pad_x'], destination_y+padding['pad_y'], destination_yaw)

                        # # # Increment destination_y by 10, but ensure it doesn't exceed the conflict area bounds
                        # # new_destination_y = destination_y
                        # # # max_y = conflict_area_bounds['max_row'] + padding['pad_y']
                        # # # min_y = conflict_area_bounds['min_row'] + padding['pad_y']
                        # # # # Clamp new_destination_y within bounds
                        # # # new_destination_y = max(min_y, min(new_destination_y, max_y))

                        # # # Compare with the final path point; if new_destination_y is beyond, use the final path point
                        # # #final_path_y = path.y[-1] if path is not None  else new_destination_y
                        # # if new_destination_y > path.y[-1]:
                        # #     new_destination_y = path.y[-1] + padding['pad_y'] - conflict_area_bounds['min_row']
                        # #     destination_x = path.x[-1] + padding['pad_x'] - conflict_area_bounds['min_col']

                        # #player_y = player_y
                        # new_destination_y = destination_y - 5
                        # #player_idx, player_point = closest_point_on_path(path, (player_x, player_y))
                        # new_destination_idx, new_destination_point= closest_point_on_path(path, (destination_x, new_destination_y))

                        # destination_x = new_destination_point[0] - conflict_area_bounds['min_col']
                        # new_destination_y = new_destination_point[1] - conflict_area_bounds['min_row']

                        # solution_end = time.time()
                        
                        # hybrid_astar_start = time.time()
                        all_vehicles = world.world.get_actors().filter('vehicle.*')
                        other_cars = [v for v in all_vehicles if v.id != world.player.id]

                        baseline_occupancygrid, _= occupationgrid.generate_occupation_grid_baseline(world.player, other_vehicles=other_cars) 
                        # updated_path = hybrid_astar.short_path_finder(
                        #     player_x, player_y+2, destination_yaw,#player_yaw,
                        #     destination_x, new_destination_y, destination_yaw,
                        #     small_grid, conflict_area_bounds['min_col'], conflict_area_bounds['min_row'],
                        #     padding['pad_x'], padding['pad_y']
                        # )
                        player_transform = world.player.get_transform()
                        player_x = player_transform.location.x
                        player_y = player_transform.location.y
                        player_yaw = math.radians(player_transform.rotation.yaw)
                        destination_transform = carla.Transform(destination, world.player.get_transform().rotation)
                        destination_x = destination_transform.location.x
                        destination_y = destination_transform.location.y
                        destination_yaw = math.radians(player_transform.rotation.yaw)
                        baseline_occupancygrid[baseline_occupancygrid == 2] = 0

                        np.save("baseline_grid/baseline_occupancygrid2.npy", baseline_occupancygrid)
                        new_path= hybrid_astar.path_finder(player_x, player_y-2, player_yaw, destination_x, destination_y, destination_yaw, baseline_occupancygrid)

                        #updated_path = hybrid_astar.short_path_finder(player_x, player_y, player_yaw, destination_x, destination_y+10, destination_yaw, small_grid, conflict_area_bounds['min_col'], conflict_area_bounds['min_row'], padding['pad_x'], padding['pad_y'])

                        # print("Control side: conflict_area : ", conflict_area.shape)
                        # print("Control side: new_path_point : ", new_path_point)
                        # print("Control side: conflict_area_bounds : ", conflict_area_bounds)
                        # print("Control side: waypoint to real grid: ", real_path_point)
                        # print("Control side: new_destination_point : ", new_destination_point)
                        # if updated_path is not None:
                        #     update_path_start = time.time()
                        #     new_path = stitch_paths(path, updated_path, conflict_area_bounds, padding)


                        path_follower = follow_path_with_pid(world.player, new_path, speed=6)
                            # print("Stitched path (x, y) combo:")
                            # for x, y in zip(path.x, path.y):
                            #     print(f"Stitched path: ({x}, {y})")
                        update_path_end = time.time()

                        hybrid_astar_end = time.time()

                        # if updated_path is not None and len(updated_path.x) > 0:

                        #     # Find closest points on original path to start and end of updated_path
                        #     start_idx, start_pt = closest_point_on_path(path, (updated_path.y[0], updated_path.x[0]))
                        #     end_idx, end_pt = closest_point_on_path(path, (updated_path.y[-1], updated_path.x[-1]))

                        #     # Stitch the new path: remove everything before start_idx, add updated_path, then everything after end_idx
                        #     stitched_x = list(updated_path.x) + list(path.x[end_idx+1:])
                        #     stitched_y = list(updated_path.y) + list(path.y[end_idx+1:])

                        #     # Replace path.x and path.y with the stitched path
                        #     path.x = stitched_x
                        #     path.y = stitched_y

                        #     path_follower = follow_path_with_pid(world.player, path, speed=20)
                



                        # Update the visualization with the final occupancy grid
                        #print("control side: ", final_occupancy_grid.shape)
                    
                    else:
                        print("Control side: No Conflict detected, proceeding with the path")
                        solution_end = time.time()


                # if sent:
                #     print("Conflict sent to subscriber")
                #     final_occupancy_grid = subscriber.receive_solution()
                #     if final_occupancy_grid is not None:
                #         if final_occupancy_grid == 'No Conflict':
                #             print("Control side : No Conflict detected, proceeding with the path")
                #         else:
                #             print('Control side:', 'Shape of final occupancy grid :', final_occupancy_grid['conflict_area'].shape)
                #             print("Control side: Conflict detected, waypoint: ", final_occupancy_grid['new_path_point'])
                #         #print("Received final occupancy grid from subscriber")
                #         # Update the visualization with the final occupancy grid
                #         #print("control side: ", final_occupancy_grid.shape)
                #     else:
                #         print("Control side: No Conflict detected, proceeding with the path")

                # if final_occupancy_grid is True:
                #     print("control side: No Conflict")
                # else:
                #     print("control side: Conflict detected, stopping the vehicle")
                #     print("control side: ", final_occupancy_grid.shape)
                #     #world.player.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))


                #occupationgrid.update_visualization(world.player, 2, 200, polygons)
                
                # Process Qt events without blocking
                QApplication.processEvents()

                if subscriber.acknowledge_message():
                    # Acknowledge the message to the subscriber
                    print("Message acknowledged")
                
                # [Exit conditions]
                if controller.parse_events():
                    break
            else:
                time.sleep(0.1)
                # Cleanup
            # Close the subscriber connection
            # subscriber.close()
            # print("Subscriber closed")
            app.quit()
            end_loop_time = time.time()
            elapsed_time = (end_loop_time - start_loop_time)
            os.makedirs("csv_time_data/conflict_car1", exist_ok=True)
            os.makedirs("csv_time_data/no_conflict_car1", exist_ok=True)
            if conflict:
                csv_file = os.path.join("csv_time_data/conflict_car1", "conflict.csv")
                with open(csv_file, "a", newline="") as f:
                    writer = csv.writer(f)
                    writer.writerow([start_loop_time, end_loop_time, elapsed_time])
                if reachability_calculation_start is not None and reachability_calculation_end is not None:
                    reachability_time = (reachability_calculation_end - reachability_calculation_start)
                    reach_csv_file = os.path.join("csv_time_data/conflict_car1", "reachability_calculation.csv")
                    with open(reach_csv_file, "a", newline="") as f:
                        writer = csv.writer(f)
                        writer.writerow([reachability_calculation_start, reachability_calculation_end, reachability_time])
                if occupationgrid_generation_start is not None and occupationgrid_generation_end is not None:
                    occupationgrid_time = (occupationgrid_generation_end - occupationgrid_generation_start)
                    occupation_csv_file = os.path.join("csv_time_data/conflict_car1", "occupancygrid_generation.csv")
                    with open(occupation_csv_file, "a", newline="") as f:
                        writer = csv.writer(f)
                        writer.writerow([occupationgrid_generation_start, occupationgrid_generation_end, occupationgrid_time])
                if solution_start is not None and solution_end is not None:
                    solution_time = (solution_end - solution_start)
                    solution_csv_file = os.path.join("csv_time_data/conflict_car1", "solution.csv")
                    with open(solution_csv_file, "a", newline="") as f:
                        writer = csv.writer(f)
                        writer.writerow([solution_start, solution_end, solution_time])
                if update_path_start is not None and update_path_end is not None:
                    update_path_time = (update_path_end - update_path_start)
                    update_csv_file = os.path.join("csv_time_data/conflict_car1", "update_path.csv")
                    with open(update_csv_file, "a", newline="") as f:
                        writer = csv.writer(f)
                        writer.writerow([update_path_start, update_path_end, update_path_time])
                if hybrid_astar_start is not None and hybrid_astar_end is not None:
                    hybrid_astar_time = (hybrid_astar_end - hybrid_astar_start)
                    hybrid_csv_file = os.path.join("csv_time_data/conflict_car1", "hybrid_astar.csv")
                    with open(hybrid_csv_file, "a", newline="") as f:
                        writer = csv.writer(f)
                        writer.writerow([hybrid_astar_start, hybrid_astar_end, hybrid_astar_time])
            else:
                csv_file = os.path.join("csv_time_data/no_conflict_car1", "no_conflict.csv")
                with open(csv_file, "a", newline="") as f:
                    writer = csv.writer(f)
                    writer.writerow([start_loop_time, end_loop_time, elapsed_time])
                if reachability_calculation_start is not None and reachability_calculation_end is not None:
                    reachability_time = (reachability_calculation_end - reachability_calculation_start)
                    reach_csv_file = os.path.join("csv_time_data/no_conflict_car1", "reachability_calculation.csv")
                    with open(reach_csv_file, "a", newline="") as f:
                        writer = csv.writer(f)
                        writer.writerow([reachability_calculation_start, reachability_calculation_end, reachability_time])
                if occupationgrid_generation_start is not None and occupationgrid_generation_end is not None:
                    occupationgrid_time = (occupationgrid_generation_end - occupationgrid_generation_start)
                    occupation_csv_file = os.path.join("csv_time_data/no_conflict_car1", "occupationgrid_generation.csv")
                    with open(occupation_csv_file, "a", newline="") as f:
                        writer = csv.writer(f)
                        writer.writerow([occupationgrid_generation_start, occupationgrid_generation_end, occupationgrid_time])
                if solution_start is not None and solution_end is not None:
                    solution_time = (solution_end - solution_start)
                    solution_csv_file = os.path.join("csv_time_data/no_conflict_car1", "solution.csv")
                    with open(solution_csv_file, "a", newline="") as f:
                        writer = csv.writer(f)
                        writer.writerow([solution_start, solution_end, solution_time])
                if update_path_start is not None and update_path_end is not None:
                    update_path_time = (update_path_end - update_path_start)
                    update_csv_file = os.path.join("csv_time_data/no_conflict_car1", "update_path.csv")
                    with open(update_csv_file, "a", newline="") as f:
                        writer = csv.writer(f)
                        writer.writerow([update_path_start, update_path_end, update_path_time])
                if hybrid_astar_start is not None and hybrid_astar_end is not None:
                    hybrid_astar_time = (hybrid_astar_end - hybrid_astar_start)
                    hybrid_csv_file = os.path.join("csv_time_data/no_conflict_car1", "hybrid_astar.csv")
                    with open(hybrid_csv_file, "a", newline="") as f:
                        writer = csv.writer(f)
                        writer.writerow([hybrid_astar_start, hybrid_astar_end, hybrid_astar_time])


    finally:

        # if world is not None:
        #     settings = world.world.get_settings()
        #     settings.synchronous_mode = False
        #     settings.fixed_delta_seconds = None
        #     world.world.apply_settings(settings)
        #     traffic_manager.set_synchronous_mode(True)

        #     world.destroy()
        subscriber.close()
        if world is not None and world.player is not None:
            world.player.destroy()
        pygame.quit()
        occupationgrid.stop_visualization()
        end_time = time.time()
        print(f"Simulation ended. Total time: {end_time - start_time}")


# ==============================================================================
# -- main() --------------------------------------------------------------
# ==============================================================================


def main():
    """Main method"""

    argparser = argparse.ArgumentParser(
        description='CARLA Automatic Control Client')
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='Print debug information')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '--res',
        metavar='WIDTHxHEIGHT',
        default='1280x720',
        help='Window resolution (default: 1280x720)')
    argparser.add_argument(
        '--sync',
        action='store_true',
        help='Synchronous mode execution')
    argparser.add_argument(
        '--visualize',
        action='store_true',
        help='Synchronous mode execution')
    argparser.add_argument(
        '--filter',
        metavar='PATTERN',
        #default='vehicle.*',
        default='vehicle.mini.cooper_s',
        help='Actor filter (default: "vehicle.*")')
    argparser.add_argument(
        '--generation',
        metavar='G',
        default='2',
        help='restrict to certain actor generation (values: "1","2","All" - default: "2")')
    argparser.add_argument(
        '-l', '--loop',
        action='store_true',
        dest='loop',
        help='Sets a new random destination upon reaching the previous one (default: False)')
    argparser.add_argument(
        "-a", "--agent", type=str,
        choices=["Behavior", "Basic", "Constant"],
        help="select which agent to run",
        default="Behavior")
    argparser.add_argument(
        '-b', '--behavior', type=str,
        choices=["cautious", "normal", "aggressive"],
        help='Choose one of the possible agent behaviors (default: normal) ',
        default='normal')
    argparser.add_argument(
        '-s', '--seed',
        help='Set seed for repeating executions (default: None)',
        default=None,
        type=int)

    args = argparser.parse_args()

    args.width, args.height = [int(x) for x in args.res.split('x')]

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('listening to server %s:%s', args.host, args.port)

    print(__doc__)

    try:
        game_loop(args)

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')


if __name__ == '__main__':
    main()
