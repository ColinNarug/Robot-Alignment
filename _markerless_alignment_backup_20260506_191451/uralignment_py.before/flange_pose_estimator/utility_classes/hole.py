import numpy as np
from typing import Optional, Dict
from ur_alignment.flange_pose_estimator.utility_classes import Ellipse  



"""
The `Hole` class represents a 3D hole detected on a flange or similar mechanical part.
It supports multiple geometric descriptions in different reference frames (e.g., 'flange', 'camera').

Each description (position and orientation) is accessed via:
    hole["camera"].center           # get the 3D center in 'camera' frame
    hole["camera"].normal           # get the normalized normal vector in 'camera' frame
    hole["camera"].normal = [0,0,1] # set and normalize the normal in 'camera' frame

Additionally, the hole may include:
    - A unique ID for tracking
    - An ellipse projection on the image plane
    - A height from the flange base (used only in the flange frame)

Usage example:
    hole = Hole()
    hole["flange"].center = [10.0, 5.0, 2.0]
    hole["flange"].normal = [0.0, 0.0, 1.0]
    hole["camera"].center = [150, -30, 500]
    hole["camera"].normal = [0.1, 0.2, 0.97]
"""



class HoleDescription:
	"""
	This class represents a description of a Hole in a specific Reference Frame -> e.g. flange/camera
	It's defined by the center and the nomal vector
	"""
	def __init__(self) -> None:
		# 3D center of the hole in a given reference frame
		self._center: Optional[np.ndarray] = None
		# Normal vector (automatically normalized if set)
		self._normal: Optional[np.ndarray] = None


	@property
	def center(self) -> Optional[np.ndarray]:
		return self._center

	@center.setter
	def center(self, value: Optional[np.ndarray]) -> None:
		self._center = np.asarray(value, dtype=float) if value is not None else None


	@property
	def normal(self) -> Optional[np.ndarray]:
		return self._normal

	@normal.setter
	def normal(self, value: Optional[np.ndarray]) -> None:
		if value is None:
			self._normal = None
		else:
			value = np.asarray(value, dtype=float)
			norm = np.linalg.norm(value)
			if norm == 0:
				raise ValueError("Normal vector cannot be zero.")
			self._normal = value / norm



class Hole:
	def __init__(self) -> None:
		# Unique ID assigned by the tracking
		self.ID: Optional[int] = None
		# Unique ID assigned by the model
		self.ID_model: Optional[int] = None

		# Ellipse projection in the image (if available)
		self.projection_edge: Optional[Ellipse] = None

		# Dictionary mapping each reference frame to a HoleDescription
		self.described_in: Dict[str, HoleDescription] = {}

		# Vertical distance from flange base (valid only in 'flange' frame)
		self.height: Optional[float] = None  # [mm]

		self.radius: float = None # [mm]


	def __getitem__(self, RF: str) -> HoleDescription:
		"""
		Access or create the HoleDescription for a specific reference frame.
		"""
		if RF not in self.described_in:
			self.described_in[RF] = HoleDescription()
			
		return self.described_in[RF]


	def __repr__(self) -> str:
		lines = [f"\nHole ID: {self.ID} // ID model: {self.ID_model}"]
		for sdr, desc in self.described_in.items():
			lines.append(f" --> [{sdr}] center={desc.center}")
		string = "\n".join(lines)
		return string


	def remove_description(self, RF: str) -> None:
		"""
		Remove the HoleDescription for a specific reference frame.
		"""
		if RF in self.described_in:
			del self.described_in[RF]