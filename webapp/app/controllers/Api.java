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

import play.Logger;
import play.data.*;
import play.libs.Akka;
import play.mvc.Controller;
import play.mvc.Result;
import play.mvc.WebSocket;
import akka.actor.ActorRef;
import akka.actor.Props;
import api.OsmSegment;

public class Api extends Controller {

  public static final SimpleDateFormat sdf = new SimpleDateFormat(
      "yyyy-MM-dd hh:mm:ss");

  public static OtpGraph graph = new OtpGraph();

  public static ObjectMapper jsonMapper = new ObjectMapper();

  public static OtpGraph getGraph() {
    return graph;
  }

  public static Result location(String vehicleId, String timestamp,
    String latStr, String lonStr, String velocity, String heading,
    String accuracy) {

    final ActorRef locationActor = Akka.system().actorOf(
        new Props(InferenceService.class));

    try {

      final Observation location = Observation.createObservation(
          vehicleId, timestamp, latStr, lonStr, velocity, heading, accuracy);

      if (location != null) {
        locationActor.tell(location);
      }

      return ok();
    } catch (final Exception e) {
      return badRequest(e.getMessage());
    }
  }

  public static Result convertEuclidToCoords(String x, String y)
      throws JsonGenerationException, JsonMappingException, IOException {

    Coordinate coords = GeoUtils.convertToLatLon(new Coordinate(Double.parseDouble(x), 
        Double.parseDouble(y)));

    return ok(jsonMapper.writeValueAsString(coords)).as("text/json");
  }

  public static Result segment(Integer segmentId)
      throws JsonGenerationException, JsonMappingException, IOException {
    final Edge e = graph.getGraph().getEdgeById(segmentId);

    if (e != null) {

      final OsmSegment osmSegment = new OsmSegment(segmentId, e.getGeometry());

      return ok(jsonMapper.writeValueAsString(osmSegment)).as("text/json");
    } else
      return badRequest();
  }

  public static WebSocket<String> streamTraces() {
    return new WebSocket<String>() {

      @Override
      public void onReady(WebSocket.In<String> in, WebSocket.Out<String> out) {

        out.write("Hello!");
        out.close();

      }
    };
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
  public static void traceLocation(String csvFileName, String vehicleId,
    String timestamp, String latStr, String lonStr, String velocity,
    String heading, String accuracy) throws NumberFormatException, ParseException, 
    TransformException, TimeOrderException {

    final Observation location = Observation.createObservation(
        vehicleId, timestamp, latStr, lonStr, velocity, heading, accuracy);
    
    // TODO set flags for result record handling
    InferenceService.processRecord(location);
    

  }

  public static Result traces(String vehicleId) throws JsonGenerationException,
      JsonMappingException, IOException {

    return ok(
        jsonMapper.writeValueAsString(InferenceService
            .getTraceResults(vehicleId))).as("text/json");
  }
  
  public static Result vertex() {
    Logger.info("vertices: " + graph.getVertexCount());

    return ok();
  }

}