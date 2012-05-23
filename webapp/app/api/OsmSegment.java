package api;

import org.codehaus.jackson.annotate.JsonAutoDetect;
import org.codehaus.jackson.map.annotate.JsonSerialize;

import utils.GeoJSONSerializer;

import com.vividsolutions.jts.geom.Geometry;

public class OsmSegment {

	private int id;
	private Geometry geom;

	public OsmSegment(Integer i, Geometry g)
	{
		id = i;
		geom = g;
	}
	
	@JsonSerialize
	public int getId() {
		return id;
	}
	
	@JsonSerialize(using=GeoJSONSerializer.class)
	public Geometry getGeom() {
	    return geom;
	}
	
}
