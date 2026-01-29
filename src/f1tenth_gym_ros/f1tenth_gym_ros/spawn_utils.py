# Copyright (c) 2025 Composiv.ai
#
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License 2.0 which is available at
# http://www.eclipse.org/legal/epl-2.0.
#
# SPDX-License-Identifier: EPL-2.0

"""Map-aware spawn utilities for automatic agent positioning."""

import os
from typing import List, Tuple, Optional

import numpy as np
import yaml
from PIL import Image


class MapSpawnFinder:
    """Find valid spawn positions for agents based on map occupancy grid."""

    def __init__(self, map_yaml_path: str):
        """
        Initialize MapSpawnFinder with a map YAML file.

        Args:
            map_yaml_path: Path to the map YAML file (e.g., 'levine.yaml')
        """
        with open(map_yaml_path, 'r') as f:
            self.meta = yaml.safe_load(f)

        self.resolution = self.meta['resolution']  # meters per pixel
        self.origin = self.meta['origin']  # [x, y, theta]
        self.free_thresh = self.meta.get('free_thresh', 0.196)
        self.occupied_thresh = self.meta.get('occupied_thresh', 0.65)

        # Load occupancy grid image
        map_dir = os.path.dirname(map_yaml_path)
        img_path = os.path.join(map_dir, self.meta['image'])
        self.grid = np.array(Image.open(img_path).convert('L')) / 255.0

        # Apply negation if specified
        if self.meta.get('negate', 0):
            self.grid = 1.0 - self.grid

        # Cache grid dimensions
        self.height, self.width = self.grid.shape

        # Check for spawn hints in YAML
        self.spawn_hints = self.meta.get('spawn_region', None)

    def world_to_pixel(self, x: float, y: float) -> Tuple[int, int]:
        """
        Convert world coordinates to pixel coordinates.

        Args:
            x: World x coordinate (meters)
            y: World y coordinate (meters)

        Returns:
            Tuple of (px, py) pixel coordinates
        """
        px = int((x - self.origin[0]) / self.resolution)
        py = int((y - self.origin[1]) / self.resolution)
        # Flip Y axis (image origin is top-left, world origin is bottom-left)
        py = self.height - 1 - py
        return (px, py)

    def pixel_to_world(self, px: int, py: int) -> Tuple[float, float]:
        """
        Convert pixel coordinates to world coordinates.

        Args:
            px: Pixel x coordinate
            py: Pixel y coordinate

        Returns:
            Tuple of (x, y) world coordinates in meters
        """
        # Flip Y axis
        py_flipped = self.height - 1 - py
        x = px * self.resolution + self.origin[0]
        y = py_flipped * self.resolution + self.origin[1]
        return (x, y)

    def is_free(self, x: float, y: float, radius: float = 0.3) -> bool:
        """
        Check if a position is free with a safety margin.

        Args:
            x: World x coordinate
            y: World y coordinate
            radius: Safety radius around the position (meters)

        Returns:
            True if the position and surrounding area are free
        """
        px, py = self.world_to_pixel(x, y)
        r_px = int(np.ceil(radius / self.resolution))

        # Check circular region around the point
        for dx in range(-r_px, r_px + 1):
            for dy in range(-r_px, r_px + 1):
                # Only check points within the circular radius
                if dx * dx + dy * dy <= r_px * r_px:
                    cx, cy = px + dx, py + dy

                    # Check bounds
                    if not (0 <= cx < self.width and 0 <= cy < self.height):
                        return False

                    # Check if pixel is free (high value = free, low = occupied)
                    if self.grid[cy, cx] < self.free_thresh:
                        return False

        return True

    def _find_free_centroid(self) -> Tuple[float, float]:
        """Find the centroid of free space in the map."""
        free_mask = self.grid >= self.free_thresh
        free_coords = np.argwhere(free_mask)

        if len(free_coords) == 0:
            # Fallback to map center
            return self.pixel_to_world(self.width // 2, self.height // 2)

        centroid_py, centroid_px = free_coords.mean(axis=0).astype(int)
        return self.pixel_to_world(centroid_px, centroid_py)

    def find_spawn_positions(
        self,
        num_agents: int,
        min_distance: float = 1.5,
        vehicle_radius: float = 0.3,
        strategy: str = 'auto'
    ) -> List[Tuple[float, float, float]]:
        """
        Find valid spawn positions for N agents.

        Args:
            num_agents: Number of agents to spawn
            min_distance: Minimum distance between agents (meters)
            vehicle_radius: Safety radius for collision checking (meters)
            strategy: Spawn strategy ('auto', 'grid', 'line', 'radial')

        Returns:
            List of (x, y, theta) tuples for each agent
        """
        if strategy == 'auto' or strategy == 'radial':
            return self._find_radial_spawn(num_agents, min_distance, vehicle_radius)
        elif strategy == 'grid':
            return self._find_grid_spawn(num_agents, min_distance, vehicle_radius)
        elif strategy == 'line':
            return self._find_line_spawn(num_agents, min_distance, vehicle_radius)
        else:
            return self._find_radial_spawn(num_agents, min_distance, vehicle_radius)

    def _find_radial_spawn(
        self,
        num_agents: int,
        min_distance: float,
        vehicle_radius: float
    ) -> List[Tuple[float, float, float]]:
        """Distribute agents radially from free space centroid."""
        # Use spawn hints if available
        if self.spawn_hints:
            center_x = (self.spawn_hints.get('x_min', 0) +
                        self.spawn_hints.get('x_max', 0)) / 2
            center_y = (self.spawn_hints.get('y_min', 0) +
                        self.spawn_hints.get('y_max', 0)) / 2
            default_theta = self.spawn_hints.get('default_theta', 0.0)
        else:
            center_x, center_y = self._find_free_centroid()
            default_theta = 0.0

        spawn_positions = []
        angle_step = 2 * np.pi / max(num_agents, 1)

        for i in range(num_agents):
            angle = i * angle_step
            found = False

            # Search outward from center
            for r in np.arange(0, 30, 0.5):
                x = center_x + r * np.cos(angle)
                y = center_y + r * np.sin(angle)

                if self.is_free(x, y, vehicle_radius):
                    # Check distance from other spawn positions
                    too_close = False
                    for sx, sy, _ in spawn_positions:
                        dist = np.sqrt((x - sx) ** 2 + (y - sy) ** 2)
                        if dist < min_distance:
                            too_close = True
                            break

                    if not too_close:
                        # Orient toward center (or use default theta)
                        if self.spawn_hints and 'default_theta' in self.spawn_hints:
                            theta = default_theta
                        else:
                            theta = np.arctan2(center_y - y, center_x - x)

                        spawn_positions.append((x, y, theta))
                        found = True
                        break

            if not found:
                # Fallback: find any free spot
                spawn_positions.append(
                    self._find_fallback_spawn(spawn_positions, vehicle_radius)
                )

        return spawn_positions

    def _find_grid_spawn(
        self,
        num_agents: int,
        min_distance: float,
        vehicle_radius: float
    ) -> List[Tuple[float, float, float]]:
        """Distribute agents in a grid pattern."""
        center_x, center_y = self._find_free_centroid()

        # Calculate grid dimensions
        cols = int(np.ceil(np.sqrt(num_agents)))
        rows = int(np.ceil(num_agents / cols))

        spawn_positions = []
        grid_spacing = max(min_distance, vehicle_radius * 3)

        # Center the grid
        start_x = center_x - (cols - 1) * grid_spacing / 2
        start_y = center_y - (rows - 1) * grid_spacing / 2

        for i in range(num_agents):
            row = i // cols
            col = i % cols
            x = start_x + col * grid_spacing
            y = start_y + row * grid_spacing

            # Adjust if position is not free
            if not self.is_free(x, y, vehicle_radius):
                x, y, _ = self._find_fallback_spawn(spawn_positions, vehicle_radius)

            spawn_positions.append((x, y, 0.0))

        return spawn_positions

    def _find_line_spawn(
        self,
        num_agents: int,
        min_distance: float,
        vehicle_radius: float
    ) -> List[Tuple[float, float, float]]:
        """Distribute agents in a line."""
        center_x, center_y = self._find_free_centroid()

        spawn_positions = []
        spacing = max(min_distance, vehicle_radius * 3)

        # Place agents in a horizontal line centered at centroid
        start_x = center_x - (num_agents - 1) * spacing / 2

        for i in range(num_agents):
            x = start_x + i * spacing
            y = center_y

            # Adjust if position is not free
            if not self.is_free(x, y, vehicle_radius):
                x, y, _ = self._find_fallback_spawn(spawn_positions, vehicle_radius)

            spawn_positions.append((x, y, 0.0))

        return spawn_positions

    def _find_fallback_spawn(
        self,
        existing_positions: List[Tuple[float, float, float]],
        vehicle_radius: float
    ) -> Tuple[float, float, float]:
        """Find any valid free position as fallback."""
        free_mask = self.grid >= self.free_thresh
        free_coords = np.argwhere(free_mask)

        if len(free_coords) == 0:
            # Absolute fallback: center of map
            x, y = self.pixel_to_world(self.width // 2, self.height // 2)
            return (x, y, 0.0)

        # Randomly sample free positions until we find a valid one
        np.random.shuffle(free_coords)

        for py, px in free_coords[:1000]:  # Limit search
            x, y = self.pixel_to_world(px, py)

            if self.is_free(x, y, vehicle_radius):
                # Check it's not too close to existing
                valid = True
                for sx, sy, _ in existing_positions:
                    if np.sqrt((x - sx) ** 2 + (y - sy) ** 2) < vehicle_radius * 2:
                        valid = False
                        break

                if valid:
                    return (x, y, 0.0)

        # Absolute fallback
        x, y = self.pixel_to_world(self.width // 2, self.height // 2)
        return (x, y, 0.0)


def get_spawn_positions(
    map_yaml_path: str,
    num_agents: int,
    min_distance: float = 1.5,
    vehicle_radius: float = 0.22,
    strategy: str = 'auto'
) -> np.ndarray:
    """
    Convenience function to get spawn positions as numpy array.

    Args:
        map_yaml_path: Path to map YAML file
        num_agents: Number of agents
        min_distance: Minimum distance between agents
        vehicle_radius: Vehicle collision radius
        strategy: Spawn strategy

    Returns:
        numpy array of shape (num_agents, 3) with [x, y, theta] rows
    """
    finder = MapSpawnFinder(map_yaml_path)
    positions = finder.find_spawn_positions(
        num_agents, min_distance, vehicle_radius, strategy
    )
    return np.array(positions)
