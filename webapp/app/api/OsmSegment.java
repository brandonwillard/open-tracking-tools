package api;

import org.codehaus.jackson.map.annotate.JsonSerialize;

import utils.GeoJSONSerializer;

import com.vividsolutions.jts.algorithm.Angle;
import com.vividsolutions.jts.geom.Geometry;

public class OsmSegment {

  private final int id;
  private final Geometry geom;
  private final Double angle;
  private final String name;
  private final Double length;

  public OsmSegment(Integer i, Geometry g, String name) {
    if (g != null) {
      final int points = g.getCoordinates().length;
      this.angle =
          Angle.toDegrees(Angle.normalizePositive(Angle.angle(
              g.getCoordinates()[points - 2],
              g.getCoordinates()[points - 1])));

      this.length = g.getLength();
    } else {
      this.angle = null;
      this.length = null;
    }
    this.id = i;
    this.geom = g;
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
  public Double getLength() {
    return length;
  }

  @JsonSerialize
  public String getName() {
    return name;
  }

}
