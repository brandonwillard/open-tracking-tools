package org.openplans.tools.tracking.impl.util;


import org.codehaus.jackson.annotate.JsonIgnore;
import org.codehaus.jackson.map.annotate.JsonSerialize;
import org.opengis.referencing.operation.MathTransform;

import com.vividsolutions.jts.geom.Coordinate;

public class ProjectedCoordinate extends Coordinate {
  
  private static final long serialVersionUID = 2905131060296578237L;
  
  final private MathTransform transform;
  final private Coordinate refLatLon;

  public ProjectedCoordinate(MathTransform mathTransform, Coordinate to, Coordinate refLatLon) {
    this.transform = mathTransform;
    this.x = to.x;
    this.y = to.y;
    this.refLatLon = refLatLon; 
  }

  @JsonIgnore
  public MathTransform getTransform() {
    return transform;
  }

  @JsonSerialize
  public Coordinate getReferenceLatLon() {
    return refLatLon;
  }

  @JsonSerialize
  public String epsgCode() {
    final String epsgCode = "EPSG:" + GeoUtils.getEPSGCodefromUTS(refLatLon);
    return epsgCode;
  }

}
