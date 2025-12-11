from typing import TypeAlias

import numpy as np

Array1D: TypeAlias = np.ndarray[tuple[int], np.dtype[np.floating]]
Array2D: TypeAlias = np.ndarray[tuple[int, int], np.dtype[np.floating]]
Array3D: TypeAlias = np.ndarray[tuple[int, int, int], np.dtype[np.floating]]
