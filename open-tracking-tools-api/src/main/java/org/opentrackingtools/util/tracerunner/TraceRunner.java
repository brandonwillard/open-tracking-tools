package org.opentrackingtools.util.tracerunner;

import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.statistics.DataDistribution;

import java.io.File;
import java.io.FileReader;
import java.lang.reflect.Constructor;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.List;

import org.codehaus.jackson.Version;
import org.codehaus.jackson.map.ObjectMapper;
import org.codehaus.jackson.map.module.SimpleModule;
import org.opentrackingtools.GpsObservation;
import org.opentrackingtools.graph.InferenceGraph;
import org.opentrackingtools.graph.otp.impl.OtpGraph;
import org.opentrackingtools.graph.paths.states.PathState;
import org.opentrackingtools.impl.SimpleObservation;
import org.opentrackingtools.impl.VehicleState;
import org.opentrackingtools.impl.VehicleStateInitialParameters;
import org.opentrackingtools.statistics.filters.vehicles.VehicleTrackingFilter;
import org.opentrackingtools.util.GeoUtils;
import org.opentrackingtools.util.JsonUtils.ConfigDeserializer;
import org.opentrackingtools.util.JsonUtils.ObservationDeserializer;
import org.opentrackingtools.util.JsonUtils.PathStateSerializer;
import org.opentrackingtools.util.JsonUtils.VectorDeserializer;
import org.opentrackingtools.util.JsonUtils.VectorSerializer;
import org.opentrackingtools.util.JsonUtils.VehicleStateSerializer;
import org.opentrackingtools.util.geom.ProjectedCoordinate;

import au.com.bytecode.opencsv.CSVReader;

import com.beust.jcommander.internal.Lists;
import com.google.common.collect.Iterables;
import com.vividsolutions.jts.geom.Coordinate;

public class TraceRunner {

  private static final SimpleDateFormat sdf =
      new SimpleDateFormat("yyyy-MM-dd hh:mm:ss");
  
  public static void main(String[] args) throws Exception {

    /*
     * Read config file and parse observations
     */
    String configFileName = args[0];
    String traceFileName = args[1];
    File configFile = new File(configFileName);
    
    Version version = new Version(1, 0, 0, "SNAPSHOT"); 
    SimpleModule module = new SimpleModule("MyModuleName", version);
    module = module.addSerializer(Vector.class, new VectorSerializer());    
    module = module.addSerializer(VehicleState.class, new VehicleStateSerializer());    
    module = module.addSerializer(PathState.class, new PathStateSerializer());    
    module = module.addDeserializer(Vector.class, new VectorDeserializer());    
    module = module.addDeserializer(VehicleStateInitialParameters.class, 
        new ConfigDeserializer());    
//    module = module.addDeserializer(GpsObservation.class, 
//        new ObservationDeserializer());    
    
    // And then configure mapper to use it
    ObjectMapper objectMapper = new ObjectMapper();
    objectMapper.registerModule(module);
    
    final VehicleStateInitialParameters ip;
    
    ip = objectMapper.readValue(
        configFile, VehicleStateInitialParameters.class);
    
    System.out.println("Loaded config:" + ip);
    
    List<GpsObservation> observations = Lists.newArrayList();
    final CSVReader gpsReader =
        new CSVReader(new FileReader(traceFileName), ';');
    
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
      String sourceId = line[3];
      Date timestamp = sdf.parse(line[6]);
      double velocity = Double.parseDouble(line[7]);
      double heading = Double.NaN;
      double accuracy = Double.NaN;
      Coordinate latLng = new Coordinate(Double.parseDouble(line[4]), 
          Double.parseDouble(line[5]));
      final ProjectedCoordinate obsPoint =
          GeoUtils.convertToEuclidean(latLng);
    
      GpsObservation obs = new SimpleObservation(sourceId, 
          timestamp, latLng, velocity, heading, accuracy,
          recordNumber, prevObs, obsPoint);
      
      observations.add(obs);
      
      ++recordNumber;
      prevObs = obs;

    } while ((line = gpsReader.readNext()) != null);
    
    gpsReader.close();
    
    System.out.println("Loaded " + observations.size() + " observation(s)");
    
    /*
     * Create the filter
     */
    VehicleTrackingFilter<GpsObservation, VehicleState> filter;
    
    final InferenceGraph graph = new OtpGraph(
        "/home/bwillard/openplans/OpenTripPlanner/test-graph", null);
    final GpsObservation initialObs = Iterables.getFirst(observations, null);
    
    Class<?> filterType = 
        Class.forName(ip.getFilterTypeName());
      
    Constructor<?> ctor = filterType.getConstructor(GpsObservation.class, 
          InferenceGraph.class,
          VehicleStateInitialParameters.class, 
          Boolean.class);
      
    filter = (VehicleTrackingFilter) ctor.newInstance(initialObs, graph, ip, true);
    
    filter.getRandom().setSeed(ip.getSeed());
    DataDistribution<VehicleState> priorBelief = filter.createInitialLearnedObject();
    
    List<VehicleState> results = Lists.newArrayList();
    /*
     * Filter observations
     */
    for (GpsObservation obs : Iterables.skip(observations, 1)) {
      // just the "best" particle for now
      results.add(priorBelief.getMaxValueKey());
      
      filter.update(priorBelief, obs);
    }
    System.out.println("Finished processing observations");
    
    File outputFile = new File(traceFileName + "-filtered.json");
    objectMapper.writeValue(outputFile, results);
  }

}
