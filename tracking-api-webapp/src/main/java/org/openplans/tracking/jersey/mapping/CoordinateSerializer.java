package org.openplans.tracking.jersey.mapping;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

import org.codehaus.jackson.JsonGenerator;
import org.codehaus.jackson.JsonProcessingException;
import org.codehaus.jackson.map.JsonSerializer;
import org.codehaus.jackson.map.SerializerProvider;

import com.vividsolutions.jts.geom.Coordinate;

public class CoordinateSerializer extends JsonSerializer<Coordinate> {

	@Override
	public void serialize(Coordinate value, JsonGenerator jgen, SerializerProvider provider) throws IOException,
			JsonProcessingException {
		Map<String, Double> map = new HashMap<String, Double>();
		map.put("lat", value.x);
		map.put("lon", value.y);
		jgen.writeObject(map);
	}
}
