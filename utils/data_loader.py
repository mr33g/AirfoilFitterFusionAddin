import numpy as np

def normalize_airfoil_data(upper_surface, lower_surface, logger_func=print):
    """
    Normalizes airfoil coordinates to have a chord length of 1, with the
    leading edge at (0,0) and the trailing edge chord-line at (1,0) if appropriate.
    """
    le_point = upper_surface[0].copy()
    te_upper = upper_surface[-1]
    te_lower = lower_surface[-1]
    te_point = (te_upper + te_lower) / 2.0
    
    logger_func(f"Normalizing airfoil. Original LE: {le_point}, Original TE (midpoint): {te_point}")

    # Translate so LE is at the origin (0,0)
    upper_translated = upper_surface - le_point
    lower_translated = lower_surface - le_point
    te_translated = te_point - le_point

    # Rotate so the TE point lies on the positive x-axis.
    if np.allclose(te_translated, [0.0, 0.0]):
        logger_func("Warning: Leading and trailing edges are coincident. Cannot determine rotation.")
        rotation_angle = 0.0
    else:
        rotation_angle = -np.arctan2(te_translated[1], te_translated[0])

    if abs(rotation_angle) > 0:
        rotation_matrix = np.array([
            [np.cos(rotation_angle), -np.sin(rotation_angle)],
            [np.sin(rotation_angle),  np.cos(rotation_angle)]
        ])
        upper_rotated = upper_translated @ rotation_matrix.T
        lower_rotated = lower_translated @ rotation_matrix.T
        te_rotated = te_translated @ rotation_matrix.T
    else:
        upper_rotated = upper_translated
        lower_rotated = lower_translated
        te_rotated = te_translated
    
    chord_length = te_rotated[0]

    if np.isclose(chord_length, 0):
        raise ValueError("Cannot normalize airfoil with zero chord length.")
    
    logger_func(f"Detected chord length: {chord_length:.6f}, rotating by {np.rad2deg(rotation_angle):.4f} degrees.")

    upper_normalized = upper_rotated / chord_length
    lower_normalized = lower_rotated / chord_length

    # --- Trailing edge y-normalization logic ---
    y_te_upper = upper_normalized[-1, 1]
    y_te_lower = lower_normalized[-1, 1]
    tol = 1e-8
    if np.isclose(y_te_upper, y_te_lower, atol=tol):
        # Not thickened, proceed as before: shift so TE is at y=0
        y_shift = (y_te_upper + y_te_lower) / 2.0
        logger_func(f"TE y-values equal (not thickened). Shifting y by {-y_shift:.8f}.")
        upper_normalized[:, 1] -= y_shift
        lower_normalized[:, 1] -= y_shift
    elif np.isclose(y_te_upper, -y_te_lower, atol=tol):
        # Thickened and symmetric, do not shift y
        logger_func("TE y-values are symmetric about y=0 (thickened trailing edge). No y-shift performed.")
    else:
        # Not symmetric, shift so TE midpoint is at y=0
        y_shift = (y_te_upper + y_te_lower) / 2.0
        logger_func(f"TE y-values not symmetric. Shifting y by {-y_shift:.8f}.")
        upper_normalized[:, 1] -= y_shift
        lower_normalized[:, 1] -= y_shift

    return upper_normalized, lower_normalized

def load_airfoil_data(filename, logger_func=print):
    """
    Loads airfoil coordinates from a file, supporting Selig and Lednicer .dat formats.
    """
    with open(filename, 'r') as f:
        lines = [line.strip() for line in f.readlines() if line.strip()]

    if not lines:
        raise ValueError(f"Airfoil data file '{filename}' is empty or contains no valid data.")

    airfoil_name = lines[0]
    coords_start_line = 1

    is_lednicer = False
    num_upper_points = 0
    num_lower_points = 0

    if len(lines) > 1:
        parts = lines[1].split()
        if len(parts) == 2:
            try:
                n1 = float(parts[0])
                n2 = float(parts[1])
                is_n1_int = abs(n1 - int(round(n1))) < 1e-6 and n1 > 1
                is_n2_int = abs(n2 - int(round(n2))) < 1e-6 and n2 > 1
                if is_n1_int and is_n2_int:
                    num_upper_points = int(round(n1))
                    num_lower_points = int(round(n2))
                    total_points = num_upper_points + num_lower_points
                    if abs(len(lines) - (2 + total_points)) <= 1:
                        is_lednicer = True
                        coords_start_line = 2
            except ValueError:
                pass

    if not is_lednicer:
        slope_start = coords_start_line
        sample_coords = []
        for i in range(slope_start, min(slope_start + 5, len(lines))):
            try:
                x, y = map(float, lines[i].split())
                sample_coords.append(x)
            except Exception:
                break
        if len(sample_coords) >= 3:
            if sample_coords[-1] > sample_coords[0]:
                is_lednicer = True
                logger_func(f"Detected Lednicer format for '{airfoil_name}' (by x slope).")
            elif sample_coords[-1] < sample_coords[0]:
                is_lednicer = False
                logger_func(f"Detected Selig-like format for '{airfoil_name}' (by x slope).")
            else:
                logger_func(f"Ambiguous x slope for '{airfoil_name}'. Defaulting to Selig-like format.")
                is_lednicer = False
        else:
            logger_func(f"Insufficient coordinate data to determine format for '{airfoil_name}'. Defaulting to Selig-like format.")
            is_lednicer = False

    if is_lednicer:
        if coords_start_line == 2:
            logger_func(f"Detected Lednicer format for '{airfoil_name}' (by count line). Expected upper points: {num_upper_points}, lower points: {num_lower_points}.")
            all_coords_raw = np.array([list(map(float, line.split())) for line in lines[coords_start_line:]])
            if len(all_coords_raw) != (num_upper_points + num_lower_points):
                logger_func(f"Warning: Declared points ({num_upper_points + num_lower_points}) do not match actual points read ({len(all_coords_raw)}). Attempting to proceed.")
            upper_surface = all_coords_raw[:num_upper_points]
            lower_surface = all_coords_raw[num_upper_points:]
        else:
            logger_func(f"Detected Lednicer format for '{airfoil_name}' (by x slope, no count line).")
            all_coords_raw = np.array([list(map(float, line.split())) for line in lines[coords_start_line:]])
            midpoint = len(all_coords_raw) // 2
            upper_surface = all_coords_raw[:midpoint]
            lower_surface = all_coords_raw[midpoint:]
    else:
        logger_func(f"Detected Selig-like format for '{airfoil_name}'.")
        coords_str = lines[coords_start_line:]
        all_coords = np.array([list(map(float, line.split())) for line in coords_str if line])
        
        le_index = int(np.argmin(all_coords[:, 0]))
        if le_index == 0 or le_index == len(all_coords) - 1:
            raise ValueError(f"Leading edge (min x) found at start or end of data, input may be malformed.")

        upper_surface_raw = all_coords[:le_index + 1]
        lower_surface = all_coords[le_index:]
        upper_surface = np.flipud(upper_surface_raw)

    upper_surface, lower_surface = normalize_airfoil_data(upper_surface, lower_surface, logger_func)

    y_te_upper = upper_surface[-1, 1]
    y_te_lower = lower_surface[-1, 1]
    tol = 1e-8
    thickened = False
    if not np.isclose(y_te_upper, y_te_lower, atol=tol) and np.isclose(y_te_upper, -y_te_lower, atol=tol):
        thickened = True
        logger_func("Trailing edge is thickened and symmetric about y=0.")

    return upper_surface, lower_surface, airfoil_name, thickened

