package api;

import org.codehaus.jackson.map.annotate.JsonSerialize;

import utils.GeoJSONSerializer;

import com.vividsolutions.jts.algorithm.Angle;
import com.vividsolutions.jts.geom.Geometry;

public class OsmSegment {

  private final int id;
  private final Geometry geom;
  private Double angle;
  private final String name;

  public OsmSegment(Integer i, Geometry g, String name) {
    final int length = g.getCoordinates().length;
    if (length > 1)
      angle = Angle.toDegrees(Angle.normalizePositive(Angle.angle(
          g.getCoordinates()[length - 2],
          g.getCoordinates()[length - 1])));
    else
      angle = null;

    id = i;
    geom = g;
    this.name = name;
  }

  @JsonSerialize
  public Double getAngle() {
    return angle;
  }

  @JsonSerialize(using = GeoJSONSerializer.class)
  public Geometry getGeom() {
    return geom;
  }

  @JsonSerialize
  public int getId() {
    return id;
  }

  @JsonSerialize
  public String getName() {
    return name;
  }

}
