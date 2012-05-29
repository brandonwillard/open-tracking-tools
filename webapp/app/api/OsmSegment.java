package api;

import org.codehaus.jackson.annotate.JsonAutoDetect;
import org.codehaus.jackson.map.annotate.JsonSerialize;

import utils.GeoJSONSerializer;

import com.vividsolutions.jts.algorithm.Angle;
import com.vividsolutions.jts.geom.Geometry;

public class OsmSegment {

	private int id;
	private Geometry geom;
	private double angle;
	private String name;

	public OsmSegment(Integer i, Geometry g, String name)
	{
    angle = Angle.angle(g.getCoordinates()[0],
        g.getCoordinates()[1]);
		id = i;
		geom = g;
		this.name = name;
	}
	
	@JsonSerialize
	public int getId() {
		return id;
	}
	
	@JsonSerialize(using=GeoJSONSerializer.class)
	public Geometry getGeom() {
	    return geom;
	}

	@JsonSerialize
  public double getAngle() {
    return angle;
  }

	@JsonSerialize
  public String getName() {
    return name;
  }
	
}
