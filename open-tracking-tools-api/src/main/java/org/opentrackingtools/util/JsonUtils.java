package org.opentrackingtools.util;

import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;

import java.io.IOException;
import java.util.Date;
import java.util.Iterator;
import java.util.List;
import java.util.Map;

import org.codehaus.jackson.JsonGenerator;
import org.codehaus.jackson.JsonNode;
import org.codehaus.jackson.JsonParser;
import org.codehaus.jackson.JsonProcessingException;
import org.codehaus.jackson.map.DeserializationContext;
import org.codehaus.jackson.map.JsonDeserializer;
import org.codehaus.jackson.map.JsonSerializer;
import org.codehaus.jackson.map.SerializerProvider;
import org.codehaus.jackson.node.ArrayNode;
import org.opengis.referencing.operation.NoninvertibleTransformException;
import org.opengis.referencing.operation.TransformException;
import org.opentrackingtools.GpsObservation;
import org.opentrackingtools.graph.paths.InferredPath;
import org.opentrackingtools.graph.paths.states.PathState;
import org.opentrackingtools.impl.SimpleObservation;
import org.opentrackingtools.impl.VehicleState;
import org.opentrackingtools.impl.VehicleStateInitialParameters;
import org.opentrackingtools.statistics.distributions.PathStateDistribution;
import org.opentrackingtools.statistics.estimators.vehicles.impl.AbstractRoadTrackingEstimator;

import com.google.common.base.Function;
import com.google.common.collect.Iterators;
import com.google.common.collect.Lists;
import com.google.common.collect.Maps;
import com.google.common.primitives.Doubles;
import com.vividsolutions.jts.geom.Coordinate;

public class JsonUtils {

  private static double[] getDoubleArray(JsonNode node) {
    
    if (!(node instanceof ArrayNode)) 
      return null;
          
    List<Double> resList = Lists.newArrayList();
    Iterator<Double> iterator = Iterators.transform(node.iterator(), 
        new Function<JsonNode, Double>() {
          @Override
          public Double apply(JsonNode input) {
            return input.getDoubleValue();
          }
    });
    Iterators.addAll(resList, iterator);
    
    return Doubles.toArray(resList);
  }
    
  public static class PathStateSerializer extends JsonSerializer<PathState> {
    @Override
    public Class<PathState> handledType() {
      return PathState.class;
    }
    
    @Override
    public void serialize(PathState pathState, JsonGenerator jgen,
      SerializerProvider provider) throws IOException,
        JsonProcessingException {
      
      final Map<String, Object> jsonResult =
          Maps.newHashMap();
      
      jsonResult
          .put("state", pathState.getGlobalState());
      jsonResult
          .put(
              "stateLoc", 
              AbstractRoadTrackingEstimator.getOg()
              .times(pathState.getGroundState()));
      
      InferredPath path = pathState.getTruncatedPathState().getPath();
      if (path.isNullPath())
        jsonResult.put("path", null);
      else
        jsonResult.put("path", 
            Iterators.transform(
                Iterators.forArray(path.getGeometry().getCoordinates()), 
                new Function<Coordinate, Double[]>() {
                  @Override
                  public Double[] apply(Coordinate input) {
                    return new Double[] {input.x, input.y};
                  }
                }));
      
  
      if (pathState instanceof PathStateDistribution) {
        final PathStateDistribution simplePathStateBelief =
            (PathStateDistribution) pathState;
        jsonResult.put("covariance", 
            simplePathStateBelief.getCovariance().toArray());
      }
      
      jgen.writeObject(jsonResult);
    }
  }
  
  public static class MatrixSerializer extends JsonSerializer<Matrix> {

    @Override
    public Class<Matrix> handledType() {
      return Matrix.class;
    }
    
    @Override
    public void serialize(Matrix value, JsonGenerator jgen,
      SerializerProvider provider) throws IOException,
        JsonProcessingException {
      jgen.writeObject(value.toArray());
    }
    
  }
  
  public static class VectorSerializer extends JsonSerializer<Vector> {

    @Override
    public Class<Vector> handledType() {
      return Vector.class;
    }
    
    @Override
    public void serialize(Vector value, JsonGenerator jgen,
      SerializerProvider provider) throws IOException,
        JsonProcessingException {
      jgen.writeObject(value.toArray());
    }
    
  }
  
  public static class VehicleStateSerializer extends JsonSerializer<VehicleState> {

    @Override
    public Class<VehicleState> handledType() {
      return VehicleState.class;
    }
    
    @Override
    public void serialize(VehicleState value, JsonGenerator jgen,
      SerializerProvider provider) throws IOException,
        JsonProcessingException {
      
      Map<String, Object> output = Maps.newHashMap();
      Coordinate gpsMean;
      try {
        gpsMean = GeoUtils.convertToLatLon(
            value.getMeanLocation(), 
            value.getObservation().getObsProjected().getTransform());
        output.put("meanLocation", new double[] {gpsMean.x, gpsMean.y});
        output.put("state", value.getBelief());
      } catch (NoninvertibleTransformException e) {
        e.printStackTrace();
      } catch (TransformException e) {
        e.printStackTrace();
      }
      jgen.writeObject(output);
    }
    
  }
  
  public static class VectorDeserializer extends JsonDeserializer<Vector> {

    @Override
    public Vector deserialize(JsonParser jp,
      DeserializationContext ctxt) throws IOException,
        JsonProcessingException {
      
      Iterator<Double> res = jp.readValuesAs(Double.class);
      List<Double> resList = Lists.newArrayList();
      Iterators.addAll(resList, res);
      
      return VectorFactory.getDefault().copyValues(
        Doubles.toArray(resList));
    }
    
  }
  
  public static class VehicleStateInitialParametersDeserializer 
    extends JsonDeserializer<VehicleStateInitialParameters> {

    @Override
    public VehicleStateInitialParameters deserialize(JsonParser jp,
      DeserializationContext ctxt) throws IOException,
        JsonProcessingException {
      
      JsonNode root = jp.readValueAsTree();
      
      Vector obsCov = VectorFactory.getDefault().copyValues(
            getDoubleArray(root.findValue("obsCov")));
      
      Vector onRoadStateCov = VectorFactory.getDefault().copyValues(
            getDoubleArray(root.findValue("onRoadStateCov")));
      Vector offRoadStateCov = VectorFactory.getDefault().copyValues(
            getDoubleArray(root.findValue("offRoadStateCov")));
      
      Vector offProbs = VectorFactory.getDefault().copyValues(
            getDoubleArray(root.findValue("offTransitionProbs")));
      Vector onProbs = VectorFactory.getDefault().copyValues(
            getDoubleArray(root.findValue("onTransitionProbs")));
      
      VehicleStateInitialParameters result =
      new VehicleStateInitialParameters(
        obsCov, root.findValue("obsCovDof").asInt(),
        onRoadStateCov, root.findValue("onRoadCovDof").asInt(),
        offRoadStateCov, root.findValue("offRoadCovDof").asInt(),
        offProbs,
        onProbs,
        root.findValue("particleFilterTypeName").getTextValue(),
        root.findValue("roadFilterTypeName").getTextValue(),
        root.findValue("numParticles").getIntValue(),
        root.findValue("initialObsFreq").getIntValue(),
        root.findValue("seed").getLongValue()
        );    
      
      return result;
    }
    
  }
  
  public static class ObservationDeserializer 
    extends JsonDeserializer<GpsObservation> {

    @Override
    public GpsObservation deserialize(JsonParser jp,
      DeserializationContext ctxt) throws IOException,
        JsonProcessingException {
      
      JsonNode root = jp.readValueAsTree();
      
      double accuracy = root.findValue("accuracy").getDoubleValue();
      double heading = root.findValue("heading").getDoubleValue();
      Coordinate coords = new Coordinate(
          root.findValue("obsCoordsLatLon").findValue("x").getDoubleValue(),
          root.findValue("obsCoordsLatLon").findValue("y").getDoubleValue()
          );
      
//      root.findValue("obsObsPoint").getTextValue();
//      root.findValue("previousObservation").getTextValue();
      
      int recordNumber = root.findValue("recordNumber").getIntValue();
      JsonNode timestampNode = root.findValue("timestamp");
      
      Date timestamp = new Date(timestampNode.getLongValue());
      
      String sourceId = root.findValue("sourceId").getTextValue();
      double velocity = root.findValue("velocity").getDoubleValue();
      
      return new SimpleObservation(sourceId, timestamp, null, velocity, 
          heading, accuracy, recordNumber, null, null);
    }
    
  }
  
}
