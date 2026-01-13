import numpy as np

from core import config
from utils.data_loader import load_airfoil_data


class AirfoilProcessor:
    """
    Acts as a bridge between the Fusion plugin and the core logic.
    Modified for Fusion to remove Qt dependencies.
    """

    def __init__(self, logger_func=print):
        # Core airfoil data
        self.upper_data = None
        self.lower_data = None
        self.upper_te_tangent_vector = None
        self.lower_te_tangent_vector = None
        self._is_blunt_TE = False # True if original airfoil has thickened TE
        self.logger_func = logger_func
        self.airfoil_name = ""


    def load_airfoil_data_and_initialize_model(self, file_path):
        """
        Loads airfoil data and initializes the model.
        Resets internal flags and state.
        """
        self.upper_data = None
        self.lower_data = None
        self.upper_te_tangent_vector = None
        self.lower_te_tangent_vector = None
        self._is_blunt_TE = False

        try:
            upper, lower, airfoil_name, blunt_te = load_airfoil_data(file_path, logger_func=self.logger_func)
            self.upper_data = upper
            self.lower_data = lower
            self.airfoil_name = airfoil_name
            self._is_blunt_TE = blunt_te
            # Recalculate TE tangent vectors using configured default
            te_vector_points = config.DEFAULT_TE_VECTOR_POINTS
            self.upper_te_tangent_vector, self.lower_te_tangent_vector = self._calculate_te_tangent(
                self.upper_data, self.lower_data, te_vector_points)
            self.logger_func("Airfoil data loaded.")
            return True
        except Exception as e:
            self.logger_func(f"Failed to load or initialize airfoil data: {e}")
            return False

    def is_trailing_edge_thickened(self):
        """Returns True if the loaded airfoil has a thickened trailing edge."""
        return self._is_blunt_TE

    def recalculate_te_vectors(self, te_vector_points):
        """
        Recalculate the trailing edge tangent vectors using the specified number of points.
        """
        if self.upper_data is None or self.lower_data is None:
            self.logger_func("Error: No airfoil data loaded. Cannot recalculate TE vectors.")
            return
        
        upper_te_tangent_vector, lower_te_tangent_vector = self._calculate_te_tangent(
            self.upper_data, self.lower_data, te_vector_points
        )
        self.upper_te_tangent_vector = upper_te_tangent_vector
        self.lower_te_tangent_vector = lower_te_tangent_vector
        self.logger_func(f"Trailing edge vectors recalculated with {te_vector_points} points.")

    def _calculate_te_tangent(self, upper_data, lower_data, te_vector_points):
        """
        Calculate trailing edge tangent vectors for upper and lower surfaces using the last N points.
        Returns (upper_te_tangent_vector, lower_te_tangent_vector)
        """
        def tangent(data, n):
            # Use the last n points to estimate the tangent at the trailing edge
            if n < 2 or len(data) < n:
                n = min(3, len(data))
            pts = data[-n:]
            dx = pts[-1, 0] - pts[0, 0]
            dy = pts[-1, 1] - pts[0, 1]
            norm = np.hypot(dx, dy)
            if norm == 0:
                return np.array([1.0, 0.0])
            return np.array([dx, dy]) / norm
            
        upper_te_tangent = tangent(upper_data, te_vector_points)
        lower_te_tangent = tangent(lower_data, te_vector_points)
        return upper_te_tangent, lower_te_tangent

