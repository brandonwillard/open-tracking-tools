package controllers;

import gov.sandia.cognition.collection.ScalarMap.Entry;
import gov.sandia.cognition.statistics.DataDistribution;
import inference.InferenceResultRecord;
import inference.InferenceService;

import java.io.IOException;
import java.text.ParseException;
import java.text.SimpleDateFormat;

import models.InferenceInstance;

import org.codehaus.jackson.JsonGenerationException;
import org.codehaus.jackson.map.JsonMappingException;
import org.codehaus.jackson.map.ObjectMapper;
import org.opengis.referencing.operation.TransformException;
import org.openplans.tools.tracking.impl.Observation;
import org.openplans.tools.tracking.impl.TimeOrderException;
import org.openplans.tools.tracking.impl.VehicleState;
import org.openplans.tools.tracking.impl.util.GeoUtils;
import org.openplans.tools.tracking.impl.util.OtpGraph;
import org.opentripplanner.routing.graph.Edge;

import com.google.common.collect.Iterables;
import com.google.common.collect.Lists;
import com.google.common.collect.Maps;
import com.vividsolutions.jts.geom.Coordinate;

import play.*;
import play.mvc.*;

import java.util.*;

import models.*;

import akka.actor.ActorRef;
import akka.actor.ActorSystem;
import akka.actor.Props;
import api.OsmSegment;

public class Api extends Controller {

  public static final SimpleDateFormat sdf = new SimpleDateFormat(
      "yyyy-MM-dd hh:mm:ss");

  public static OtpGraph graph = new OtpGraph(Play.configuration.getProperty("application.otpGraphPath"));

  public static ObjectMapper jsonMapper = new ObjectMapper();

  public static OtpGraph getGraph() {
    return graph;
  }
  
  

  public static void location(String vehicleId, String timestamp,
    String latStr, String lonStr, String velocity, String heading,
    String accuracy) {

    try {

      final Observation location = Observation.createObservation(
          vehicleId, timestamp, latStr, lonStr, velocity, heading, accuracy);

      if (location != null) {
        Application.locationActor.tell(location);
      }

      ok();
    } catch (final Exception e) {
      Logger.error(e.getMessage());
      badRequest();
    }
  }

  public static void convertToLatLon(String x, String y)
      throws JsonGenerationException, JsonMappingException, IOException {
    
    Coordinate rawCoords = new Coordinate(Double.parseDouble(x), 
            Double.parseDouble(y));
    
    Coordinate coords;
//    if (GeoUtils.isInLatLonCoords(rawCoords))
//      coords = rawCoords;
//    else if (GeoUtils.isInProjCoords(rawCoords))
      coords = GeoUtils.convertToLatLon(rawCoords);
//    else
//      coords = null;

    renderJSON(jsonMapper.writeValueAsString(coords));
  }

  public static void segment(Integer segmentId)
      throws JsonGenerationException, JsonMappingException, IOException {
    
    if (segmentId == null)
      badRequest();
    
    final Edge e = graph.getGraph().getEdgeById(segmentId);

    if (e != null) {

      final OsmSegment osmSegment = new OsmSegment(segmentId, e.getGeometry(), e.toString());

      renderJSON(jsonMapper.writeValueAsString(osmSegment));
    } else
      badRequest();
  }
  
  public static void particleDetails(String vehicleId, int recordNumber) throws JsonGenerationException,
      JsonMappingException, IOException { 
    
    final Collection<InferenceResultRecord> results = InferenceService.getTraceResults(vehicleId);
    if (results.isEmpty())
      renderJSON(jsonMapper.writeValueAsString(null));
      
    final InferenceResultRecord tmpResult = Iterables.get(results, recordNumber, null);
    
    if (tmpResult == null)
      error(vehicleId + " result record " + recordNumber + " is out-of-bounds");
    
    List<Map<String, Object>> jsonResults = Lists.newArrayList();
    for (Entry<VehicleState> stateEntry : tmpResult.getFilterDistribution().entrySet()) {
      Map<String, Object> thisMap = Maps.newHashMap();
      thisMap.put("weight", stateEntry.getValue());
      thisMap.put("edgeId", stateEntry.getKey().getInferredEdge().getEdgeId());
      thisMap.put("meanLoc", GeoUtils.getCoordinates(stateEntry.getKey().getMeanLocation()));
      jsonResults.add(thisMap);
    }
    
    renderJSON(jsonMapper.writeValueAsString(jsonResults));
  }
  
  public static void traceParticleRecord(String vehicleId, int recordNumber, Integer particleNumber, Boolean withParent) throws JsonGenerationException,
      JsonMappingException, IOException {
    
    final List<InferenceResultRecord> results = Lists.newArrayList();
    DataDistribution<VehicleState> belief = null;
    if (particleNumber == null) {
      /*
       * Just return the "best" state
       */
      final Collection<InferenceResultRecord> tmpResults = InferenceService.getTraceResults(vehicleId);
      
      if (tmpResults.isEmpty())
        renderJSON(jsonMapper.writeValueAsString(null));
      
      final InferenceResultRecord result = Iterables.get(tmpResults, recordNumber, null);
      
      if (result == null)
        renderJSON(jsonMapper.writeValueAsString(null));
      
      results.add(result);
      
    } else {
      final Collection<InferenceResultRecord> tmpResults = InferenceService.getTraceResults(vehicleId);
      if (tmpResults.isEmpty())
        renderJSON(jsonMapper.writeValueAsString(null));
        
      final InferenceResultRecord tmpResult = Iterables.get(tmpResults, recordNumber, null);
      
      if (tmpResult == null)
        error(vehicleId + " result record " + recordNumber + " is out-of-bounds");
      
      belief = tmpResult.getFilterDistribution();
      
      if (particleNumber < 0) {
        for (VehicleState infState : belief.getDomain()) {
          
          final VehicleState actualState = tmpResult.getActualResults() != null ? tmpResult.getActualResults().getState()
              : null;
          final InferenceResultRecord result = InferenceResultRecord.createInferenceResultRecord(infState.getObservation(), 
              actualState, infState, null);
          results.add(result);
        }
      } else {
        
        final VehicleState infState = Iterables.get(belief.getDomain(), particleNumber, null);
        
        if (infState == null)
          renderJSON(jsonMapper.writeValueAsString(null));
        
        final VehicleState actualState = tmpResult.getActualResults() != null ? tmpResult.getActualResults().getState()
            : null;
        
        final InferenceResultRecord result = InferenceResultRecord.createInferenceResultRecord(infState.getObservation(), 
            actualState, infState, null);
        results.add(result);
      }
    }
    
    List<Map<String, Object>> mapResults = Lists.newArrayList();
    for (InferenceResultRecord result : results) {
      final VehicleState state = result.getInfResults().getState();
      final InferenceResultRecord parent;
      if (withParent == Boolean.TRUE) {
        final VehicleState parentState = state.getParentState();
        if (parentState != null) {
          parent = InferenceResultRecord.createInferenceResultRecord(parentState.getObservation(), 
            null, parentState, null);
        } else {
          parent = null;
        }
      } else {
        parent = null;
      }
      
      Map<String, Object> mapResult = Maps.newHashMap();
      mapResult.put("particle", result);
      
      if (belief != null) {
        final double weight = belief.getFraction(state);
        mapResult.put("weight", weight);
      }
      
      if (parent != null)
        mapResult.put("parent", parent);
      
      mapResults.add(mapResult);
    }
    
    renderJSON(jsonMapper.writeValueAsString(mapResults));
  }

  public static void traces(String vehicleId) throws JsonGenerationException,
      JsonMappingException, IOException {

    renderJSON(jsonMapper.writeValueAsString(InferenceService.getTraceResults(vehicleId)));
  }
  
  public static void vertex() {
    Logger.info("vertices: " + graph.getVertexCount());

    // TODO noop
    
    ok();
  }

}