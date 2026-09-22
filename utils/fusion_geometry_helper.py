import adsk.core


def create_fusion_spline(sketch, control_points, knots, degree, is_closed=False):
    """
    Creates a high-accuracy NURBS spline in a Fusion sketch.
    Note: These are created as 'Fixed' (read-only) geometry.
    """
    # Use a native Python list of Point3D objects for the API
    points_list = []
    for cp in control_points:
        z = cp[2] if len(cp) > 2 else 0
        points_list.append(adsk.core.Point3D.create(cp[0], cp[1], z))
    
    try:
        # Step 1: Create the mathematical NURBS geometry
        # This handles the exact degree and knots from the B-spline fit
        nurbs_geom = adsk.core.NurbsCurve3D.createNonRational(
            points_list, 
            degree, 
            knots.tolist() if hasattr(knots, 'tolist') else knots, 
            is_closed
        )
        
        if not nurbs_geom:
            raise RuntimeError("NurbsCurve3D.createNonRational returned None")

        # Step 2: Add it to the sketch as a Fixed Spline
        spline = sketch.sketchCurves.sketchFixedSplines.addByNurbsCurve(nurbs_geom)
        
        # Show the control polygon for reference
        if hasattr(spline, 'isControlPolygonVisible'):
            spline.isControlPolygonVisible = True
            
        return spline
        
    except Exception as e:
        raise RuntimeError(f"NURBS geometry creation failed: {str(e)}")
