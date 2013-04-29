package org.opentrackingtools.util.tracerunner;

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
import org.opentrackingtools.VehicleStateInitialParameters;
import org.opentrackingtools.estimators.MotionStateEstimatorPredictor;
import org.opentrackingtools.model.GpsObservation;
import org.opentrackingtools.model.VehicleStateDistribution;
import org.opentrackingtools.paths.Path;
import org.opentrackingtools.paths.PathState;
import org.opentrackingtools.util.GeoUtils;

import com.google.common.base.Function;
import com.google.common.collect.Iterators;
import com.google.common.collect.Lists;
import com.google.common.collect.Maps;
import com.google.common.primitives.Doubles;
import com.vividsolutions.jts.geom.Coordinate;

public class JsonUtils {

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

  public static class ObservationDeserializer extends
      JsonDeserializer<GpsObservation> {

    @Override
    public GpsObservation deserialize(JsonParser jp,
      DeserializationContext ctxt) throws IOException,
        JsonProcessingException {

      final JsonNode root = jp.readValueAsTree();

      final double accuracy =
          root.findValue("accuracy").getDoubleValue();
      final double heading =
          root.findValue("heading").getDoubleValue();
      new Coordinate(root.findValue("obsCoordsLatLon").findValue("x")
          .getDoubleValue(), root.findValue("obsCoordsLatLon")
          .findValue("y").getDoubleValue());

      //      root.findValue("obsObsPoint").getTextValue();
      //      root.findValue("previousObservation").getTextValue();

      final int recordNumber =
          root.findValue("recordNumber").getIntValue();
      final JsonNode timestampNode = root.findValue("timestamp");

      final Date timestamp = new Date(timestampNode.getLongValue());

      final String sourceId =
          root.findValue("sourceId").getTextValue();
      final double velocity =
          root.findValue("velocity").getDoubleValue();

      return new GpsObservation(sourceId, timestamp, null, velocity,
          heading, accuracy, recordNumber, null, null);
    }

  }

  public static class PathStateSerializer extends
      JsonSerializer<PathState> {
    @Override
    public Class<PathState> handledType() {
      return PathState.class;
    }

    @Override
    public void serialize(PathState pathState, JsonGenerator jgen,
      SerializerProvider provider) throws IOException,
        JsonProcessingException {

      final Map<String, Object> jsonResult = Maps.newHashMap();

      jsonResult.put("state", pathState.getMotionState());
      jsonResult.put("stateLoc", MotionStateEstimatorPredictor
          .getOg().times(pathState.getGroundState()));

      final Path path = pathState.getTruncatedPathState().getPath();
      if (path.isNullPath()) {
        jsonResult.put("path", null);
      } else {
        jsonResult.put("path", Iterators.transform(
            Iterators.forArray(path.getGeometry().getCoordinates()),
            new Function<Coordinate, Double[]>() {
              @Override
              public Double[] apply(Coordinate input) {
                return new Double[] { input.x, input.y };
              }
            }));
      }

      //      if (pathState instanceof PathStateDistribution) {
      //        final PathStateDistribution simplePathStateDistribution =
      //            (PathStateDistribution) pathState;
      //        jsonResult.put("covariance", 
      //            simplePathStateDistribution.getCovariance().toArray());
      //      }

      jgen.writeObject(jsonResult);
    }
  }

  public static class VectorDeserializer extends
      JsonDeserializer<Vector> {

    @Override
    public Vector deserialize(JsonParser jp,
      DeserializationContext ctxt) throws IOException,
        JsonProcessingException {

      final Iterator<Double> res = jp.readValuesAs(Double.class);
      final List<Double> resList = Lists.newArrayList();
      Iterators.addAll(resList, res);

      return VectorFactory.getDefault().copyValues(
          Doubles.toArray(resList));
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

  public static class VehicleStateInitialParametersDeserializer
      extends JsonDeserializer<VehicleStateInitialParameters> {

    @Override
    public VehicleStateInitialParameters deserialize(JsonParser jp,
      DeserializationContext ctxt) throws IOException,
        JsonProcessingException {

      final JsonNode root = jp.readValueAsTree();

      final Vector obsCov =
          VectorFactory.getDefault().copyValues(
              JsonUtils.getDoubleArray(root.findValue("obsCov")));

      final Vector onRoadStateCov =
          VectorFactory.getDefault().copyValues(
              JsonUtils.getDoubleArray(root
                  .findValue("onRoadStateCov")));
      final Vector offRoadStateCov =
          VectorFactory.getDefault().copyValues(
              JsonUtils.getDoubleArray(root
                  .findValue("offRoadStateCov")));

      final Vector offProbs =
          VectorFactory.getDefault().copyValues(
              JsonUtils.getDoubleArray(root
                  .findValue("offTransitionProbs")));
      final Vector onProbs =
          VectorFactory.getDefault().copyValues(
              JsonUtils.getDoubleArray(root
                  .findValue("onTransitionProbs")));

      final VehicleStateInitialParameters result =
          new VehicleStateInitialParameters(null, obsCov, root
              .findValue("obsCovDof").asInt(), onRoadStateCov, root
              .findValue("onRoadCovDof").asInt(), offRoadStateCov,
              root.findValue("offRoadCovDof").asInt(), offProbs,
              onProbs, root.findValue("numParticles").getIntValue(),
              root.findValue("initialObsFreq").getIntValue(), root
                  .findValue("seed").getLongValue());

      return result;
    }

  }

  public static class VehicleStateSerializer extends
      JsonSerializer<VehicleStateDistribution> {

    @Override
    public Class<VehicleStateDistribution> handledType() {
      return VehicleStateDistribution.class;
    }

    @Override
    public void serialize(VehicleStateDistribution value,
      JsonGenerator jgen, SerializerProvider provider)
        throws IOException, JsonProcessingException {

      final Map<String, Object> output = Maps.newHashMap();
      Coordinate gpsMean;
      try {
        gpsMean =
            GeoUtils.convertToLatLon(value.getMeanLocation(), value
                .getObservation().getObsProjected().getTransform());
        output.put("meanLocation", new double[] { gpsMean.x,
            gpsMean.y });
        output.put("state", value.getPathStateParam().getValue());
      } catch (final NoninvertibleTransformException e) {
        e.printStackTrace();
      } catch (final TransformException e) {
        e.printStackTrace();
      }
      jgen.writeObject(output);
    }

  }

  private static double[] getDoubleArray(JsonNode node) {

    if (!(node instanceof ArrayNode)) {
      return null;
    }

    final List<Double> resList = Lists.newArrayList();
    final Iterator<Double> iterator =
        Iterators.transform(node.iterator(),
            new Function<JsonNode, Double>() {
              @Override
              public Double apply(JsonNode input) {
                return input.getDoubleValue();
              }
            });
    Iterators.addAll(resList, iterator);

    return Doubles.toArray(resList);
  }

}
