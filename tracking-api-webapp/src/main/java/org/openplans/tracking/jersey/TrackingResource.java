package org.openplans.tracking.jersey;

import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.statistics.DataDistribution;

import java.io.File;
import java.lang.reflect.Constructor;
import java.util.ArrayList;
import java.util.Date;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import javax.ws.rs.GET;
import javax.ws.rs.Path;
import javax.ws.rs.PathParam;
import javax.ws.rs.Produces;
import javax.ws.rs.QueryParam;

import org.codehaus.jackson.Version;
import org.codehaus.jackson.map.ObjectMapper;
import org.codehaus.jackson.map.SerializationConfig;
import org.codehaus.jackson.map.module.SimpleModule;
import org.geotools.geometry.GeometryBuilder;
import org.geotools.geometry.jts.FactoryFinder;
import org.geotools.geometry.jts.JTS;
import org.geotools.geometry.jts.JTSFactoryFinder;
import org.opengis.geometry.MismatchedDimensionException;
import org.opengis.referencing.operation.MathTransform;
import org.opengis.referencing.operation.NoninvertibleTransformException;
import org.opengis.referencing.operation.TransformException;
import org.opentrackingtools.GpsObservation;
import org.opentrackingtools.graph.InferenceGraph;
import org.opentrackingtools.graph.otp.impl.OtpGraph;
import org.opentrackingtools.graph.paths.states.PathState;
import org.opentrackingtools.impl.SimpleObservation;
import org.opentrackingtools.impl.VehicleState;
import org.opentrackingtools.impl.VehicleStateInitialParameters;
import org.opentrackingtools.statistics.filters.vehicles.VehicleTrackingFilter;
import org.opentrackingtools.util.GeoUtils;
import org.opentrackingtools.util.JsonUtils.PathStateSerializer;
import org.opentrackingtools.util.JsonUtils.VectorDeserializer;
import org.opentrackingtools.util.JsonUtils.VectorSerializer;
import org.opentrackingtools.util.JsonUtils.VehicleStateInitialParametersDeserializer;
import org.opentrackingtools.util.JsonUtils.VehicleStateSerializer;
import org.opentrackingtools.util.geom.ProjectedCoordinate;
import org.opentrackingtools.util.tracerunner.TraceRunner.TraceRunnerConfig;

import com.sun.jersey.api.json.JSONWithPadding;
import com.sun.jersey.spi.resource.Singleton;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.CoordinateSequence;
import com.vividsolutions.jts.geom.CoordinateSequenceFactory;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.GeometryCollection;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.LineString;
import com.vividsolutions.jts.geom.Point;
import com.vividsolutions.jts.geom.impl.CoordinateArraySequence;
import com.vividsolutions.jts.geom.impl.CoordinateArraySequenceFactory;

@Singleton
@Path("/vehicles")
public class TrackingResource {

	private InferenceGraph _graph;
	private VehicleStateInitialParameters _ip;
	private Constructor<?> _filterConstructor;
	private Map<String, InferenceInstance> _inferenceInstances = new HashMap<String, InferenceInstance>();

	public TrackingResource() {
		File configFile = new File(
				"/Users/asutula/Documents/OpenPlans/Projects/OpenTrackingTools/Code/open-tracking-tools/open-tracking-tools-api/runner-test-config.json");

		Version version = new Version(1, 0, 0, "SNAPSHOT");
		SimpleModule module = new SimpleModule("MyModuleName", version);
		module = module.addSerializer(Vector.class, new VectorSerializer());
		module = module.addSerializer(VehicleState.class,
				new VehicleStateSerializer());
		module = module.addSerializer(PathState.class,
				new PathStateSerializer());
		module = module.addDeserializer(Vector.class, new VectorDeserializer());
		module = module.addDeserializer(VehicleStateInitialParameters.class,
				new VehicleStateInitialParametersDeserializer());

		// And then configure mapper to use it
		ObjectMapper objectMapper = new ObjectMapper();
		objectMapper.registerModule(module);
		objectMapper.configure(SerializationConfig.Feature.INDENT_OUTPUT, true);

		TraceRunnerConfig config = null;
		try {
			config = objectMapper.readValue(configFile, TraceRunnerConfig.class);
			_ip = objectMapper.readValue(configFile, VehicleStateInitialParameters.class);
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		System.out.println("Loaded config:" + _ip);

		_graph = new OtpGraph(config.getOtpGraphLocation(), null);

		try {
			Class<?> filterType = Class.forName(_ip.getFilterTypeName());
			_filterConstructor = filterType.getConstructor(
					GpsObservation.class, InferenceGraph.class,
					VehicleStateInitialParameters.class, Boolean.class);
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	@GET
	@Path("/{id}/update")
	@Produces({"application/x-javascript", "application/json"})
	public Object trackVehicleAtPoint(@PathParam("id") String id,
			@QueryParam("lon") double lon, @QueryParam("lat") double lat, @QueryParam("callback") String callback) throws Exception {

		System.out.println("Starting... " + id);

		if (!_inferenceInstances.containsKey(id)) {
			InferenceInstance inferenceInstance = new InferenceInstance();
			_inferenceInstances.put(id, inferenceInstance);
		}
		
		InferenceInstance inferenceInstance = _inferenceInstances.get(id);
		
		Date timestamp = new Date();
		double velocity = Double.NaN;
		double heading = Double.NaN;
		double accuracy = Double.NaN;
		Coordinate latLon = new Coordinate(lat, lon);
		final ProjectedCoordinate obsPoint = GeoUtils.convertToEuclidean(latLon);

		GpsObservation obs = new SimpleObservation(id, timestamp, latLon,
				velocity, heading, accuracy, inferenceInstance.recordNumber, inferenceInstance.prevObs, obsPoint);
		
		if (inferenceInstance.filter == null) {
			try {
				inferenceInstance.filter = (VehicleTrackingFilter) _filterConstructor.newInstance(obs, _graph, _ip, true);
				inferenceInstance.filter.getRandom().setSeed(_ip.getSeed());
				inferenceInstance.priorBelief = inferenceInstance.filter.createInitialLearnedObject();
			} catch (Exception e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		} else {
			inferenceInstance.filter.update(inferenceInstance.priorBelief, obs);
		}
		
		inferenceInstance.recordNumber++;
		inferenceInstance.prevObs = obs;

		System.out.println("Ending... " + id);
		
		List<Map<String, Object>> results = new ArrayList<Map<String,Object>>();
		List<Geometry> linesToCombine = new ArrayList<Geometry>();

		VehicleState bestVehicleState = inferenceInstance.priorBelief.getMaxValueKey();
		MathTransform transform = bestVehicleState.getObservation().getObsProjected().getTransform().inverse();
		
		while (bestVehicleState != null) {
			Map<String, Object> map = new HashMap<String, Object>();
			try {
				Coordinate coord = GeoUtils.convertToLatLon(
						bestVehicleState.getMeanLocation(),
						bestVehicleState.getObservation().getObsProjected().getTransform());
				
				Point point = JTSFactoryFinder.getGeometryFactory().createPoint(coord);
				this.switchXY(point);
				map.put("location", point);
				
				// If we're off road and we have a parent (previous) state,
				// create a path from the end of the last path to the
				// current location and make that the path
				if (!bestVehicleState.getBelief().isOnRoad() && bestVehicleState.getParentState() != null) {
					Point lastPoint;
					if (bestVehicleState.getParentState().getBelief().isOnRoad()) {
						Geometry lastPath = bestVehicleState.getParentState().getBelief().getPath().getGeometry();
						lastPoint = ((LineString)lastPath).getEndPoint();
					} else {
						Coordinate lastCoord = new Coordinate(
								bestVehicleState.getParentState().getMeanLocation().getElement(0),
								bestVehicleState.getParentState().getMeanLocation().getElement(1));
						lastPoint = JTSFactoryFinder.getGeometryFactory().createPoint(lastCoord);
					}
					Coordinate currentCoordinate = new Coordinate(
							bestVehicleState.getMeanLocation().getElement(0), 
							bestVehicleState.getMeanLocation().getElement(1));
					Coordinate[] array = {lastPoint.getCoordinate(), currentCoordinate};
					Geometry line = new LineString(
							CoordinateArraySequenceFactory.instance().create(array), 
							JTSFactoryFinder.getGeometryFactory());
					linesToCombine.add(line);
				} else if (!bestVehicleState.getBelief().getPath().isNullPath()) {
					// We don't always get a path back if the vehicle is off road
					linesToCombine.add(bestVehicleState.getBelief().getPath().getGeometry());
//					Geometry path = JTS.transform(
//							bestVehicleState.getBelief().getPath().getGeometry(), 
//							bestVehicleState.getObservation().getObsProjected().getTransform().inverse());
//					map.put("path", path);
				}
			} catch (Exception e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			results.add(map);
			bestVehicleState = bestVehicleState.getParentState();
		}
		
		GeometryFactory factory = FactoryFinder.getGeometryFactory( null );

	     // note the following geometry collection may be invalid (say with overlapping polygons)
		Geometry[] a = {};
		GeometryCollection geometryCollection = new GeometryCollection(linesToCombine.toArray(a), new GeometryFactory());
		Geometry unioned = geometryCollection.union();
		Geometry transformedPath = null;
		try {
			transformedPath = JTS.transform(unioned, transform);
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		System.out.println("count: " + results.size());
		
		this.switchXY(transformedPath);
		
		if (callback != null)
			return new JSONWithPadding(transformedPath, callback);
		
		return transformedPath;
	}

	private class InferenceInstance {
		GpsObservation prevObs = null;
		DataDistribution<VehicleState> priorBelief = null;
		VehicleTrackingFilter<GpsObservation, VehicleState> filter = null;
		int recordNumber = 0;
	}
	
	private void switchXY(Geometry geometry) {
		Coordinate[] coordinates = geometry.getCoordinates();
		for (int i = 0; i < geometry.getNumPoints(); i++) {
			Coordinate coordinate = coordinates[i];
			Coordinate newCoordinate = new Coordinate(coordinate.y, coordinate.x);
			coordinate.setCoordinate(newCoordinate);
		}
		geometry.geometryChanged();
	}

}
