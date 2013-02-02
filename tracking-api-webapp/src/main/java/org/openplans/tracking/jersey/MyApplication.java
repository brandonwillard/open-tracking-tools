package org.openplans.tracking.jersey;

import java.util.HashSet;
import java.util.Set;

import javax.ws.rs.core.Application;

public class MyApplication extends Application {

	@Override
	public Set<Class<?>> getClasses() {

		final Set<Class<?>> classes = new HashSet<Class<?>>();

		// register root resources
		classes.add(HelloWorldResource.class);
		classes.add(TrackingResource.class);

		// register Jackson ObjectMapper resolver
		classes.add(MyObjectMapperProvider.class);

		return classes;
	}

}
