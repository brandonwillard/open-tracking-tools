package org.openplans.tracking.jersey.mapping;

import java.io.IOException;

import org.codehaus.jackson.JsonGenerator;
import org.codehaus.jackson.JsonProcessingException;
import org.codehaus.jackson.map.JsonSerializer;
import org.codehaus.jackson.map.SerializerProvider;
import org.geotools.geojson.geom.GeometryJSON;

import com.vividsolutions.jts.geom.Geometry;

public class GeometrySerializer extends JsonSerializer<Geometry> {
	
	private GeometryJSON geometryJSON;
	
	public GeometrySerializer() {
		geometryJSON = new GeometryJSON();
	}
	
	@Override
	public Class<Geometry> handledType() {
		// TODO Auto-generated method stub
		return Geometry.class;
	}

	@Override
	public void serialize(Geometry value, JsonGenerator jgen, SerializerProvider provider) throws IOException,
			JsonProcessingException {
		jgen.writeRawValue(geometryJSON.toString(value));
	}

}
