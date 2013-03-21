package org.opentrackingtools.model;

import org.codehaus.jackson.annotate.JsonIgnore;
import org.codehaus.jackson.map.annotate.JsonSerialize;
import org.opengis.referencing.operation.MathTransform;
import org.opentrackingtools.util.GeoUtils;

import com.vividsolutions.jts.geom.Coordinate;

public class ProjectedCoordinate extends Coordinate {

  private static final long serialVersionUID = 2905131060296578237L;

  final private Coordinate refLatLon;
  final private MathTransform transform;

  public ProjectedCoordinate(MathTransform mathTransform,
    Coordinate to, Coordinate refLatLon) {
    this.transform = mathTransform;
    this.x = to.x;
    this.y = to.y;
    this.refLatLon = refLatLon;
  }

  @JsonSerialize
  public String epsgCode() {
    final String epsgCode =
        "EPSG:" + GeoUtils.getEPSGCodefromUTS(this.refLatLon);
    return epsgCode;
  }

  @JsonSerialize
  public Coordinate getReferenceLatLon() {
    return this.refLatLon;
  }

  @JsonIgnore
  public MathTransform getTransform() {
    return this.transform;
  }

}
