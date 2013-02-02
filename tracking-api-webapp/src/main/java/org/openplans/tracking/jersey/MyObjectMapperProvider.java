package org.openplans.tracking.jersey;

import gov.sandia.cognition.math.matrix.Vector;

import javax.ws.rs.ext.ContextResolver;

import org.codehaus.jackson.Version;
import org.codehaus.jackson.map.ObjectMapper;
import org.codehaus.jackson.map.SerializationConfig.Feature;
import org.codehaus.jackson.map.module.SimpleModule;
import org.openplans.tracking.jersey.mapping.GeometrySerializer;
import org.opentrackingtools.graph.paths.states.PathState;
import org.opentrackingtools.impl.VehicleState;
import org.opentrackingtools.util.JsonUtils.PathStateSerializer;
import org.opentrackingtools.util.JsonUtils.VectorSerializer;
import org.opentrackingtools.util.JsonUtils.VehicleStateSerializer;

public class MyObjectMapperProvider implements ContextResolver<ObjectMapper> {

	final ObjectMapper defaultObjectMapper;

	public MyObjectMapperProvider() {
		defaultObjectMapper = createDefaultMapper();
	}

	@Override
	public ObjectMapper getContext(Class<?> type) {
		return defaultObjectMapper;
	}

	private static ObjectMapper createDefaultMapper() {

		ObjectMapper mapper = new ObjectMapper();
		mapper.configure(Feature.INDENT_OUTPUT, true);

		SimpleModule module = new SimpleModule("MyModule", new Version(1, 0, 0, null));
		// testModule.addSerializer(new CoordinateSerializer());
		module.addSerializer(new GeometrySerializer());
		module.addSerializer(Vector.class, new VectorSerializer());
		module.addSerializer(VehicleState.class, new VehicleStateSerializer());
		module.addSerializer(PathState.class, new PathStateSerializer());

		mapper.registerModule(module);

		return mapper;
	}
}
