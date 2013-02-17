package org.openplans.tracking.jersey;

import java.util.HashMap;
import java.util.Map;

import javax.ws.rs.GET;
import javax.ws.rs.Path;
import javax.ws.rs.Produces;
import javax.ws.rs.QueryParam;

import com.sun.jersey.api.json.JSONWithPadding;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.Point;

@Path("/helloworld")
public class HelloWorldResource {
	
	GeometryFactory geometryFactory = new GeometryFactory();

	@GET
	@Produces("application/x-javascript")
	public JSONWithPadding getClichedMessage(
			@QueryParam("callback") String callback, 
			@QueryParam("name") String name) {
		
		Point p = geometryFactory.createPoint(new Coordinate(40, -111));
		
		Map<String, Object> map = new HashMap<String, Object>();
		map.put("location", p);
		
		return new JSONWithPadding(map, callback);
	}
	
	public class MyObject {
		private String name;
		private Integer age;
		
		public MyObject(String name, Integer age) {
			this.name = name;
			this.age = age;
		}

		public String getName() {
			return name;
		}

		public void setName(String name) {
			this.name = name;
		}

		public Integer getAge() {
			return age;
		}

		public void setAge(Integer age) {
			this.age = age;
		}
	}
}
