package org.openplans.tools.tracking.impl.util;

import java.awt.geom.Point2D;

import org.codehaus.jackson.annotate.JsonIgnore;

import com.jhlabs.map.proj.Projection;
import com.vividsolutions.jts.geom.Coordinate;

public class ProjectedCoordinate extends Coordinate {
  
  private static final long serialVersionUID = 2905131060296578237L;
  
  final private Projection projection;
  final private int utmZone;

  public ProjectedCoordinate(Projection projection, int zone, Point2D.Double coords) {
    this.projection = projection;
    this.utmZone = zone;
    this.x = coords.x;
    this.y = coords.y;
  }

  @JsonIgnore
  public Projection getProjection() {
    return projection;
  }

  public int getUtmZone() {
    return utmZone;
  }

}
