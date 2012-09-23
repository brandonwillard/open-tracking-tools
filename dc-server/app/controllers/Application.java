package controllers;

import static akka.pattern.Patterns.ask;
import akka.actor.*;
import akka.dispatch.Future;
import akka.dispatch.OnSuccess;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;
import play.*;
import play.db.jpa.JPA;
import play.mvc.*;
import java.awt.Color;

import java.io.File;
import java.math.BigInteger;
import java.util.*;

import javax.persistence.Query;

import org.apache.commons.lang.StringUtils;
import org.geotools.geometry.jts.JTS;
import org.opengis.referencing.operation.MathTransform;
import org.openplans.tools.tracking.impl.statistics.filters.VehicleTrackingFilter;
import org.openplans.tools.tracking.impl.VehicleState.VehicleStateInitialParameters;
import org.openplans.tools.tracking.impl.VehicleUpdate;
import org.openplans.tools.tracking.impl.VehicleUpdateResponse;
import org.openplans.tools.tracking.impl.graph.InferredEdge;
import org.openplans.tools.tracking.impl.statistics.filters.VehicleTrackingBootstrapFilter;
import org.openplans.tools.tracking.impl.statistics.filters.VehicleTrackingPLFilter;
import org.openplans.tools.tracking.impl.util.GeoUtils;
import org.openplans.tools.tracking.impl.util.OtpGraph;
import org.opentripplanner.routing.graph.Edge;

import com.google.common.collect.Maps;
import com.google.common.collect.Multimap;
import com.typesafe.config.Config;
import com.typesafe.config.ConfigFactory;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.LineString;

import jobs.ObservationHandler;

import models.*;

public class Application extends Controller {
	
	private final static double OFFSET = 2;
	
	private static GeometryFactory gf = new GeometryFactory();
	
	static Config myConfig = ConfigFactory.empty();
	
	static Date lastTime = new Date();
	
	static ActorSystem system = ActorSystem.create("LookupApplication", ConfigFactory.parseFile(new File("conf/akka.conf")));

	static ActorRef remoteObservationActor = system.actorFor("akka://inferenceSystem@127.0.0.1:2552/user/observationActor");
	
	public static OtpGraph graph = new OtpGraph(
		      Play.configuration.getProperty("application.otpGraphPath"), null);
	
	
	/*public static void updateVehicleStats(VehicleUpdateResponse result)
	{
		if(result.pathList.size() == 0)
			return;
			
		try 
		{ 
			// wrapping everything around a try catch
			if(JPA.local.get() == null)
            {
				JPA.local.set(new JPA());
				JPA.local.get().entityManager = JPA.newEntityManager();
            }
			JPA.local.get().entityManager.getTransaction().begin();

			for(ArrayList<Integer> edges : result.pathList)
			{
				String edgeIds = StringUtils.join(edges, ", ");
				String sql = "UPDATE streetedge SET inpath = inpath + 1 WHERE edgeid IN (" + edgeIds + ") ";
				
				JPA.local.get().entityManager.createNativeQuery(sql).executeUpdate();
			}
			
			JPA.local.get().entityManager.getTransaction().commit();	
		}
        finally 
        {
            JPA.local.get().entityManager.close();
            JPA.local.remove();
        }
	}*/
	
	public static void index() {
		render();
	}
	
	public static void taxi() {
		render();
	}
	
	public static void citom() {
		render();
	}
	
	public static void replay() {
		
		List<LocationUpdate> locations  = LocationUpdate.find("order by timestamp").fetch();
		
		VehicleUpdate updates = new VehicleUpdate(locations.get(0).imei);
		
		for(LocationUpdate location : locations)
		{
			updates.addObservation(location.getObservationData());
		}
		
		Future<Object> future = ask(Application.remoteObservationActor, updates, 60000);
		
		future.onSuccess(new OnSuccess<Object>() {
			public void onSuccess(Object result) {
				
				if(result instanceof VehicleUpdateResponse)
				{
					//Application.updateVehicleStats((VehicleUpdateResponse)result);
					
					if(((VehicleUpdateResponse) result).pathList.size() == 0)
						return;
						
					try 
					{ 
						// wrapping everything around a try catch
						if(JPA.local.get() == null)
			            {
							JPA.local.set(new JPA());
							JPA.local.get().entityManager = JPA.newEntityManager();
			            }
						JPA.local.get().entityManager.getTransaction().begin();

						for(ArrayList<Integer> edges : ((VehicleUpdateResponse) result).pathList)
						{
							String edgeIds = StringUtils.join(edges, ", ");
							String sql = "UPDATE streetedge SET inpath = inpath + 1 WHERE edgeid IN (" + edgeIds + ") ";
							
							JPA.local.get().entityManager.createNativeQuery(sql).executeUpdate();
						}
						
						JPA.local.get().entityManager.getTransaction().commit();	
					}
			        finally 
			        {
			            JPA.local.get().entityManager.close();
			            JPA.local.remove();
			        }
				}
			}
		});
		
		index();
	}
	
	public static void replay2() {
		
		List<LocationUpdate> locations  = LocationUpdate.find("timestamp > ? order by timestamp", lastTime).fetch();
		
		VehicleUpdate updates = new VehicleUpdate(locations.get(0).imei);
		
		for(LocationUpdate location : locations)
		{
			updates.addObservation(location.getObservationData());
			
			lastTime = location.timestamp;
		}
		
		Future<Object> future = ask(Application.remoteObservationActor, updates, 60000);
		
		future.onSuccess(new OnSuccess<Object>() {
			public void onSuccess(Object result) {
				
				if(result instanceof VehicleUpdateResponse)
				{
					//Application.updateVehicleStats((VehicleUpdateResponse)result);
					
					if(((VehicleUpdateResponse) result).pathList.size() == 0)
						return;
						
					try 
					{ 
						// wrapping everything around a try catch
						if(JPA.local.get() == null)
			            {
							JPA.local.set(new JPA());
							JPA.local.get().entityManager = JPA.newEntityManager();
			            }
						JPA.local.get().entityManager.getTransaction().begin();

						for(ArrayList<Integer> edges : ((VehicleUpdateResponse) result).pathList)
						{
							String edgeIds = StringUtils.join(edges, ", ");
							String sql = "UPDATE streetedge SET inpath = inpath + 1 WHERE edgeid IN (" + edgeIds + ") ";
							
							JPA.local.get().entityManager.createNativeQuery(sql).executeUpdate();
						}
						
						JPA.local.get().entityManager.getTransaction().commit();	
					}
			        finally 
			        {
			            JPA.local.get().entityManager.close();
			            JPA.local.remove();
			        }
				}
			}
		});
		
		index();
	}

	public static void edges()
	{
		Multimap<Geometry, Edge> edgeMap = graph.getGeomEdgeMap();
		
		for(Edge edge : edgeMap.values())
		{
			Integer edgeId = graph.getInferredEdge(edge).getEdgeId();
			
			MathTransform transform;
			
		    try {
		    	transform = GeoUtils.getTransform(new Coordinate(38.90911, -77.00932)).inverse();
		    	
		    	if (graph.getEdge(edgeId).getGeometry() != null)
		    	{
			        Coordinate[] oldCoords = graph.getEdge(edgeId).getGeometry().getCoordinates();
			        int nCoords = oldCoords.length;
			        Coordinate[] newCoords = new Coordinate[nCoords];
			        
			        for (int i = 0; i < nCoords - 1; ++i) {
			            Coordinate coord0 = oldCoords[i];
			            Coordinate coord1 = oldCoords[i+1];
			            
			            double dx = coord1.x - coord0.x;
			            double dy = coord1.y - coord0.y;
			            
			            double length = Math.sqrt(dx * dx + dy * dy);
			            
			            Coordinate c0 = new Coordinate(coord0.x - OFFSET * dy / length, coord0.y - OFFSET * dx / length);
			            Coordinate c1 = new Coordinate(coord1.x - OFFSET * dy / length, coord1.y - OFFSET * dx / length);
			            newCoords[i] = c0;
			            newCoords[i+1] = c1; //will get overwritten except at last iteration
			        }
			        
				    final Geometry transformed = JTS.transform( gf.createLineString(newCoords), transform);
				    transformed.setSRID(4326);
					
				    
				    List<String> points = new ArrayList<String>();
		        	
		        	for(Coordinate coord : transformed.getCoordinates())
		        	{
		        		points.add(new Double(coord.x).toString() + " " + new Double(coord.y).toString());
		        		
		        	}
		        	
		        	String linestring = "LINESTRING(" + StringUtils.join(points, ", ") + ")";
				    
				    Query idQuery = StreetEdge.em().createNativeQuery("SELECT NEXTVAL('hibernate_sequence');");
			    	BigInteger nextId = (BigInteger)idQuery.getSingleResult();
			    	
			    	StreetEdge.em().createNativeQuery("INSERT INTO streetedge (id, edgeid, shape)" +
			        	"  VALUES(?, ?, ST_GeomFromText( ?, 4326));")
			          .setParameter(1,  nextId)
			          .setParameter(2,  edgeId)	            
			          .setParameter(3,  linestring)
			          .executeUpdate();
				    
		    	}
		    }
		    catch(Exception e)
		    {
		    	Logger.error("Can't transform geom.");
		    }
		
		}
	}

}