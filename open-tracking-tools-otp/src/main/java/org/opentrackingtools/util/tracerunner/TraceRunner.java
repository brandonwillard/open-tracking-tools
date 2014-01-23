package org.opentrackingtools.util.tracerunner;

import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.statistics.DataDistribution;
import gov.sandia.cognition.statistics.bayesian.ParticleFilter;

import java.io.File;
import java.io.FileReader;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.List;
import java.util.Random;

import org.codehaus.jackson.JsonParser.Feature;
import org.codehaus.jackson.Version;
import org.codehaus.jackson.map.ObjectMapper;
import org.codehaus.jackson.map.SerializationConfig;
import org.codehaus.jackson.map.module.SimpleModule;
import org.opentrackingtools.VehicleStateInitialParameters;
import org.opentrackingtools.VehicleStatePLFilter;
import org.opentrackingtools.graph.InferenceGraph;
import org.opentrackingtools.graph.otp.OtpGraph;
import org.opentrackingtools.model.GpsObservation;
import org.opentrackingtools.model.VehicleStateDistribution;
import org.opentrackingtools.model.VehicleStateDistribution.VehicleStateDistributionFactory;
import org.opentrackingtools.paths.PathState;
import org.opentrackingtools.util.GeoUtils;
import org.opentrackingtools.util.tracerunner.JsonUtils.PathStateSerializer;
import org.opentrackingtools.util.tracerunner.JsonUtils.VectorDeserializer;
import org.opentrackingtools.util.tracerunner.JsonUtils.VectorSerializer;
import org.opentrackingtools.util.tracerunner.JsonUtils.VehicleStateInitialParametersDeserializer;
import org.opentrackingtools.util.tracerunner.JsonUtils.VehicleStateSerializer;

import au.com.bytecode.opencsv.CSVReader;

import com.google.common.collect.Iterables;
import com.google.common.collect.Lists;
import com.vividsolutions.jts.geom.Coordinate;

public class TraceRunner {

  public static class TraceRunnerConfig {

    public VehicleStateInitialParameters initialParameters;
    public String otpGraphLocation;
    public String outputFileName;
    public String traceFileName;

    public TraceRunnerConfig() {
    }

    public VehicleStateInitialParameters getInitialParameters() {
      return this.initialParameters;
    }

    public String getOtpGraphLocation() {
      return this.otpGraphLocation;
    }

    public String getOutputFileName() {
      return this.outputFileName;
    }

    public String getTraceFileName() {
      return this.traceFileName;
    }

  }

  private static final SimpleDateFormat sdf = new SimpleDateFormat(
      "yyyy-MM-dd hh:mm:ss");

  public static void main(String[] args) throws Exception {

    /*
     * Read config file and parse observations
     */
    final String configFileName = args[0];
    final File configFile = new File(configFileName);

    final Version version = new Version(1, 0, 0, "SNAPSHOT");
    SimpleModule module = new SimpleModule("MyModuleName", version);
    module =
        module.addSerializer(Vector.class, new VectorSerializer());
    module =
        module.addSerializer(VehicleStateDistribution.class,
            new VehicleStateSerializer());
    module =
        module.addSerializer(PathState.class,
            new PathStateSerializer());
    module =
        module
            .addDeserializer(Vector.class, new VectorDeserializer());
    module =
        module.addDeserializer(VehicleStateInitialParameters.class,
            new VehicleStateInitialParametersDeserializer());
    //    module = module.addDeserializer(GpsObservation.class, 
    //        new ObservationDeserializer());    

    // And then configure mapper to use it
    final ObjectMapper objectMapper = new ObjectMapper();
    objectMapper.registerModule(module);
    objectMapper.configure(SerializationConfig.Feature.INDENT_OUTPUT,
        true);
    objectMapper.configure(Feature.ALLOW_NON_NUMERIC_NUMBERS, true);

    final TraceRunnerConfig config =
        objectMapper.readValue(configFile, TraceRunnerConfig.class);

    final VehicleStateInitialParameters ip;

    ip =
        objectMapper.readValue(configFile,
            VehicleStateInitialParameters.class);

    System.out.println("Loaded config:" + ip);

    final List<GpsObservation> observations = Lists.newArrayList();
    final CSVReader gpsReader =
        new CSVReader(new FileReader(config.getTraceFileName()), ';');

    // TODO take json observations
    //      observations = objectMapper.readValue(
    //          configFile, new TypeReference<List<GpsObservation>>() {});
    //      

    // skip header
    gpsReader.readNext();

    String[] line = gpsReader.readNext();
    int recordNumber = 0;
    GpsObservation prevObs = null;
    do {
      final String sourceId = line[3];
      final Date timestamp = TraceRunner.sdf.parse(line[6]);
      final double velocity = Double.parseDouble(line[7]);
      final double heading = Double.NaN;
      final double accuracy = Double.NaN;
      final Coordinate latLng =
          new Coordinate(Double.parseDouble(line[4]),
              Double.parseDouble(line[5]));
      final org.opentrackingtools.model.ProjectedCoordinate obsPoint =
          GeoUtils.convertToEuclidean(latLng);

      final GpsObservation obs =
          new GpsObservation(sourceId, timestamp, latLng, velocity,
              heading, accuracy, recordNumber, prevObs, obsPoint);

      observations.add(obs);

      ++recordNumber;
      prevObs = obs;

    } while ((line = gpsReader.readNext()) != null);

    gpsReader.close();

    System.out.println("Loaded " + observations.size()
        + " observation(s)");

    /*
     * Create the filter
     */
    final InferenceGraph graph =
        new OtpGraph(config.getOtpGraphLocation());
    final GpsObservation initialObs =
        Iterables.getFirst(observations, null);

    Random rng;
    if (ip.getSeed() != 0) {
      rng = new Random(ip.getSeed());
    } else {
      rng = new Random();
    }

    //    Class<?> filterType = 
    //        Class.forName(ip.getParticleFilterTypeName());
    //      
    //    Constructor<?> ctor = filterType.getConstructor(GpsObservation.class, 
    //          InferenceGraph.class,
    //          VehicleStateInitialParameters.class, 
    //          Boolean.class, Random.class);

    final ParticleFilter<GpsObservation, VehicleStateDistribution<GpsObservation>> filter =
        new VehicleStatePLFilter<GpsObservation, InferenceGraph>(
            initialObs,
            graph,
            new VehicleStateDistributionFactory<GpsObservation, InferenceGraph>(),
            ip, true, rng);
    //        (ParticleFilter) ctor.newInstance(initialObs, graph, ip, true, 
    //            rng);
    filter.setNumParticles(config.getInitialParameters().getNumParticles());

    filter.getRandom().setSeed(ip.getSeed());
    final DataDistribution<VehicleStateDistribution<GpsObservation>> priorBelief =
        filter.createInitialLearnedObject();

    final List<VehicleStateDistribution<GpsObservation>> results =
        Lists.newArrayList();
    /*
     * Filter observations
     */
    for (final GpsObservation obs : Iterables.skip(observations, 1)) {
      // just the "best" particle for now
      results.add(priorBelief.getMaxValueKey());

      filter.update(priorBelief, obs);
    }

    System.out.println("Finished processing observations");

    final File outputFile = new File(config.getOutputFileName());
    objectMapper.writeValue(outputFile, results);

    System.out.println("Output written to "
        + config.getOutputFileName());
  }

}
