package controllers;

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
import org.openplans.tools.tracking.impl.util.GeoUtils;
import org.openplans.tools.tracking.impl.util.OtpGraph;
import org.opentripplanner.routing.graph.Edge;

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

  public static void convertEuclidToCoords(String x, String y)
      throws JsonGenerationException, JsonMappingException, IOException {

    Coordinate coords = GeoUtils.convertToLatLon(new Coordinate(Double.parseDouble(x), 
        Double.parseDouble(y)));

    renderJSON(jsonMapper.writeValueAsString(coords));
  }

  public static void segment(Integer segmentId)
      throws JsonGenerationException, JsonMappingException, IOException {
    final Edge e = graph.getGraph().getEdgeById(segmentId);

    if (e != null) {

      final OsmSegment osmSegment = new OsmSegment(segmentId, e.getGeometry());

      renderJSON(jsonMapper.writeValueAsString(osmSegment));
    } else
      badRequest();
  }


  /**
   * Process records from a trace. Note: flags are set that cause the records to
   * be handled differently.
   * 
   * @param csvFileName
   * @param vehicleId
   * @param timestamp
   * @param latStr
   * @param lonStr
   * @param velocity
   * @param heading
   * @param accuracy
   * @return
   * @throws TimeOrderException 
   * @throws TransformException 
   * @throws ParseException 
   * @throws NumberFormatException 
   */
  

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