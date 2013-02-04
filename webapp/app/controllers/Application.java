package controllers;

import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;
import gov.sandia.cognition.statistics.distribution.UnivariateGaussian.SufficientStatistic;
import inference.InferenceService;
import inference.SimulationActor;

import java.io.BufferedReader;
import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.lang.reflect.InvocationTargetException;
import java.net.URL;
import java.util.Date;
import java.util.Enumeration;
import java.util.List;
import java.util.Map;
import java.util.ServiceLoader;
import java.util.Set;

import models.InferenceInstance;

import org.opentrackingtools.impl.Simulation;
import org.opentrackingtools.impl.VehicleStateInitialParameters;
import org.opentrackingtools.impl.VehicleStatePerformanceResult;
import org.opentrackingtools.impl.Simulation.SimulationParameters;
import org.opentrackingtools.impl.VehicleStatePerformanceResult.SufficientStatisticRecord;
import org.opentrackingtools.statistics.filters.vehicles.AbstractVehicleTrackingFilter;
import org.opentrackingtools.statistics.filters.vehicles.VehicleTrackingFilter;
import org.opentrackingtools.statistics.filters.vehicles.impl.VehicleTrackingBootstrapFilter;
import org.opentrackingtools.statistics.filters.vehicles.particle_learning.impl.VehicleTrackingPLFilter;
import org.opentrackingtools.statistics.filters.vehicles.road.impl.AbstractRoadTrackingFilter;
import org.opentrackingtools.statistics.filters.vehicles.road.impl.ErrorEstimatingRoadTrackingFilter;
import org.opentrackingtools.statistics.filters.vehicles.road.impl.StandardRoadTrackingFilter;

import play.Logger;
import play.mvc.Controller;
import akka.actor.ActorRef;
import akka.actor.ActorSystem;
import akka.actor.Props;
import async.CsvUploadActor;
import async.CsvUploadActor.TraceParameters;

import com.google.common.collect.Maps;
import com.vividsolutions.jts.geom.Coordinate;

public class Application extends Controller {

  static ActorSystem system = ActorSystem.create("MySystem");
  static ActorRef simActor = system.actorOf(new Props(
      SimulationActor.class), "simActor");
  static ActorRef locationActor = system.actorOf(new Props(
      InferenceService.class), "locationActor");
  static ActorRef csvActor = system.actorOf(new Props(
      CsvUploadActor.class), "csvActor");

  public static void getInferenceInstances() {
    final List<InferenceInstance> instances =
        InferenceService.getInferenceInstances();
    render(instances);
  }

  private static Map<String, Class<? extends VehicleTrackingFilter>> particleFiltersMap = Maps.newHashMap();
  static {
    particleFiltersMap.put(VehicleTrackingPLFilter.class.getName(), VehicleTrackingPLFilter.class);
    particleFiltersMap.put(VehicleTrackingBootstrapFilter.class.getName(), VehicleTrackingBootstrapFilter.class);
  }
  
  private static Map<String, Class<? extends AbstractRoadTrackingFilter>> roadFiltersMap = Maps.newHashMap();
  static {
    roadFiltersMap.put(ErrorEstimatingRoadTrackingFilter.class.getName(), ErrorEstimatingRoadTrackingFilter.class);
    roadFiltersMap.put(StandardRoadTrackingFilter.class.getName(), StandardRoadTrackingFilter.class);
  }
  
  public static void instances() {
    final List<InferenceInstance> instances =
        InferenceService.getInferenceInstances();
      
    final Set<String> filters = particleFiltersMap.keySet();
    final VehicleStateInitialParameters initialParams = InferenceService.getDefaultVehicleStateInitialParams();
    render(instances, filters, initialParams);
  }

  public static void map(String vehicleId) {
    render(vehicleId);
  }
  
  public static void dataPlot() {
    
    final List<InferenceInstance> instances =
        InferenceService.getInferenceInstances();
    
    for (InferenceInstance instance : instances) {
      if (instance.getPerformanceResults() == null) {
        VehicleStatePerformanceResult result = Api.getPerformanceResultsData(instance.getVehicleId());
        instance.setPerformanceResults(result);
      }
    }
    render(instances);
  }

  public static void removeInferenceInstance(String vehicleId) {
    if (vehicleId == null)
      error();
    InferenceService.remove(vehicleId);
    instances();
  }

  public static void setDefaultVehicleStateParams(
    String obs_variance_pair, String obsCovDof_str,
    String road_state_variance, String roadCovDof_str,
    String ground_state_variance_pair, String groundCovDof_str, 
    String off_prob_pair,
    String on_prob_pair, String numParticles_str, 
    String particleFilterTypeName,
    String roadFilterTypeName,
    String initialObsFreq_str) {

    final int obsCovDof = Integer.parseInt(obsCovDof_str);
    final int roadCovDof = Integer.parseInt(roadCovDof_str);
    final int groundCovDof = Integer.parseInt(groundCovDof_str);
    final int initialObsFreq = Integer.parseInt(initialObsFreq_str);
    final int numParticles = Integer.parseInt(numParticles_str);
    final String[] obsPair = obs_variance_pair.split(",");
    final Vector obsVariance =
        VectorFactory.getDefault().createVector2D(
            Double.parseDouble(obsPair[0]),
            Double.parseDouble(obsPair[1]));

    final Vector roadStateVariance =
        VectorFactory.getDefault().createVector1D(
            Double.parseDouble(road_state_variance));

    final String[] groundStatePair =
        ground_state_variance_pair.split(",");
    final Vector groundStateVariance =
        VectorFactory.getDefault().createVector2D(
            Double.parseDouble(groundStatePair[0]),
            Double.parseDouble(groundStatePair[1]));

    final String[] offPair = off_prob_pair.split(",");
    final Vector offProbs =
        VectorFactory.getDefault().createVector2D(
            Double.parseDouble(offPair[0]),
            Double.parseDouble(offPair[1]));

    final String[] onPair = on_prob_pair.split(",");
    final Vector onProbs =
        VectorFactory.getDefault().createVector2D(
            Double.parseDouble(onPair[0]),
            Double.parseDouble(onPair[1]));

    final VehicleStateInitialParameters parameters =
        new VehicleStateInitialParameters(
            obsVariance, obsCovDof,
            roadStateVariance, roadCovDof,
            groundStateVariance, groundCovDof, 
            offProbs, onProbs, particleFilterTypeName, 
            roadFilterTypeName,
            numParticles, 
            initialObsFreq, 0);

    InferenceService.setDefaultVehicleStateInitialParams(parameters);

    instances();
  }

  public static void simulation(
    String obs_variance_pair, String obsCovDof_str,
    String road_state_variance, String roadCovDof_str,
    String ground_state_variance_pair, String groundCovDof_str, 
    String off_prob_pair, String on_prob_pair, String performInference, String useInitialSimParams,
    String start_coordinate_pair, String start_unix_time,
    String duration_str, String frequency_str,
    String numParticles_str, String initialObsFreq_str,
    String particleFilterTypeName, String roadFilterTypeName, 
    String seed_str) throws SecurityException, IllegalArgumentException, ClassNotFoundException, NoSuchMethodException, InstantiationException, IllegalAccessException, InvocationTargetException {
    
    final Coordinate startCoord;
    if (!start_coordinate_pair.isEmpty()) {
      final String[] startCoordPair = start_coordinate_pair.split(",");
      startCoord =
          new Coordinate(Double.parseDouble(startCoordPair[0]),
              Double.parseDouble(startCoordPair[1]));
    } else {
      startCoord = null;
    }
    
    final int obsCovDof = Integer.parseInt(obsCovDof_str);
    final int roadCovDof = Integer.parseInt(roadCovDof_str);
    final int groundCovDof = Integer.parseInt(groundCovDof_str);
    
    final Date startTime = new Date(Long.parseLong(start_unix_time));
    final long seed = Long.parseLong(seed_str);
    final int initialObsFreq = Integer.parseInt(initialObsFreq_str);
    final int numParticles = Integer.parseInt(numParticles_str);
    final long duration = Long.parseLong(duration_str);
    final long frequency = Long.parseLong(frequency_str);
    final boolean inference = Boolean.parseBoolean(performInference);
    final boolean bUseInitialSimParams = Boolean.parseBoolean(useInitialSimParams);

    final String[] obsPair = obs_variance_pair.split(",");
    final Vector obsVariance =
        VectorFactory.getDefault().createVector2D(
            Double.parseDouble(obsPair[0]),
            Double.parseDouble(obsPair[1]));

    final Vector roadStateVariance =
        VectorFactory.getDefault().createVector1D(
            Double.parseDouble(road_state_variance));

    final String[] groundStatePair =
        ground_state_variance_pair.split(",");
    final Vector groundStateVariance =
        VectorFactory.getDefault().createVector2D(
            Double.parseDouble(groundStatePair[0]),
            Double.parseDouble(groundStatePair[1]));

    final String[] offPair = off_prob_pair.split(",");
    final Vector offProbs =
        VectorFactory.getDefault().createVector2D(
            Double.parseDouble(offPair[0]),
            Double.parseDouble(offPair[1]));

    final String[] onPair = on_prob_pair.split(",");
    final Vector onProbs =
        VectorFactory.getDefault().createVector2D(
            Double.parseDouble(onPair[0]),
            Double.parseDouble(onPair[1]));

    final VehicleStateInitialParameters parameters =
        new VehicleStateInitialParameters(
            obsVariance, obsCovDof,
            roadStateVariance, roadCovDof,
            groundStateVariance, groundCovDof, 
            offProbs, onProbs, particleFilterTypeName, 
            roadFilterTypeName,
            numParticles, initialObsFreq, seed);

    final SimulationParameters simParams =
        new SimulationParameters(startCoord, startTime, duration,
            frequency, inference, false, parameters);

    final String simulationName = "sim" + simParams.hashCode();
    if (InferenceService.getInferenceInstance(simulationName) != null) {
      Logger.warn("removing existing inference instance named "
          + simulationName);
      InferenceService.remove(simulationName);
    }
    final Simulation sim =
        new Simulation(simulationName, Api.getGraph(), simParams,
            bUseInitialSimParams ? simParams.getStateParams() : 
              InferenceService.getDefaultVehicleStateInitialParams());
    Logger.info("starting simulation " + sim.getSimulationName());

    Application.simActor.tell(sim);

    instances();
  }

  public static void uploadHandler(File csv,
    String obs_variance_pair, String obsCovDof_str,
    String road_state_variance, String roadCovDof_str,
    String ground_state_variance_pair, String groundCovDof_str, 
    String off_prob_pair, String on_prob_pair, String numParticles_str, String seed_str,
    String particleFilterTypeName, String roadFilterTypeName, String debugEnabled,
    String initialObsFreq_str) {

    if (csv != null) {
      
      final int obsCovDof = Integer.parseInt(obsCovDof_str);
      final int roadCovDof = Integer.parseInt(roadCovDof_str);
      final int groundCovDof = Integer.parseInt(groundCovDof_str);
      
      final String[] obsPair = obs_variance_pair.split(",");
      final Vector obsVariance =
          VectorFactory.getDefault().createVector2D(
              Double.parseDouble(obsPair[0]),
              Double.parseDouble(obsPair[1]));

      final Vector roadStateVariance =
          VectorFactory.getDefault().createVector1D(
              Double.parseDouble(road_state_variance));

      final String[] groundStatePair =
          ground_state_variance_pair.split(",");
      final Vector groundStateVariance =
          VectorFactory.getDefault().createVector2D(
              Double.parseDouble(groundStatePair[0]),
              Double.parseDouble(groundStatePair[1]));

      final String[] offPair = off_prob_pair.split(",");
      final Vector offProbs =
          VectorFactory.getDefault().createVector2D(
              Double.parseDouble(offPair[0]),
              Double.parseDouble(offPair[1]));

      final String[] onPair = on_prob_pair.split(",");
      final Vector onProbs =
          VectorFactory.getDefault().createVector2D(
              Double.parseDouble(onPair[0]),
              Double.parseDouble(onPair[1]));

      final long seed = Long.parseLong(seed_str);
      final int numParticles = Integer.parseInt(numParticles_str);
      final int initialObsFreq = Integer.parseInt(initialObsFreq_str);

      final VehicleStateInitialParameters parameters =
          new VehicleStateInitialParameters(
              obsVariance, obsCovDof,
              roadStateVariance, roadCovDof,
              groundStateVariance, groundCovDof, 
              offProbs, onProbs, particleFilterTypeName, 
              roadFilterTypeName,
              numParticles, initialObsFreq, seed);

      final boolean debug_enabled =
          Boolean.parseBoolean(debugEnabled);

      final File dest = new File("/tmp/upload.csv");
      csv.renameTo(dest);

      final TraceParameters params =
          new TraceParameters(dest, parameters, debug_enabled);
      csvActor.tell(params);
    }

    instances();
  }

  public static Map<String, Class<? extends VehicleTrackingFilter>>
      getFilters() {
    return particleFiltersMap;
  }

  public static void setFilters(
    Map<String, Class<? extends VehicleTrackingFilter>> filters) {
    Application.particleFiltersMap = filters;
  }

}