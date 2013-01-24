package org.openplans.tools.tracking.server;

import gov.sandia.cognition.math.matrix.VectorFactory;

import java.text.SimpleDateFormat;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import javax.servlet.ServletConfig;
import javax.servlet.ServletException;

import org.apache.log4j.Logger;
import org.openplans.tools.tracking.server.shared.InferenceInstance;
import org.openplans.tools.tracking.server.shared.ObservationFactory;
import org.openplans.tools.tracking.client.InferenceService;
import org.opentrackingtools.GpsObservation;
import org.opentrackingtools.graph.InferenceGraph;
import org.opentrackingtools.graph.otp.impl.OtpGraph;
import org.opentrackingtools.impl.VehicleState.VehicleStateInitialParameters;
import org.opentrackingtools.statistics.filters.vehicles.VehicleTrackingFilter;
import org.opentrackingtools.statistics.filters.vehicles.impl.VehicleTrackingBootstrapFilter;
import org.opentrackingtools.statistics.filters.vehicles.particle_learning.impl.VTErrorEstimatingPLFilter;
import org.opentrackingtools.statistics.filters.vehicles.particle_learning.impl.VehicleTrackingPLFilter;

import com.google.common.base.Strings;
import com.google.common.collect.Iterables;
import com.google.common.collect.Lists;
import com.google.common.collect.Maps;
import com.google.common.collect.Multimap;
import com.google.common.collect.TreeMultimap;
import com.google.gwt.user.server.rpc.RemoteServiceServlet;

/**
 * 
 * @author bwillard
 * 
 */
public class InferenceServiceImpl extends RemoteServiceServlet 
	implements InferenceService {
  
  private static final long serialVersionUID = -4813467552086614018L;

  public static final SimpleDateFormat sdf = new SimpleDateFormat(
      "yyyy-MM-dd hh:mm:ss");

  static private String otpGraphPath;
  
  @Override
  public void init(ServletConfig config) throws ServletException {
    super.init(config);
    otpGraphPath = config.getInitParameter("otpGraphPath");
    log.info(otpGraphPath);
    if (!Strings.isNullOrEmpty(otpGraphPath)) {
      try {
        graph = new OtpGraph(otpGraphPath, null);
      } catch (Exception ex) {
        log.error(ex);
      }
    }
  }
  
  private static InferenceGraph graph;
  
  final Logger log = Logger.getLogger(InferenceInstance.class);
  
  private static Map<String, Class<? extends VehicleTrackingFilter>> filtersMap = Maps.newHashMap();
  static {
    filtersMap.put(VTErrorEstimatingPLFilter.class.getName(), VTErrorEstimatingPLFilter.class);
    filtersMap.put(VehicleTrackingPLFilter.class.getName(), VehicleTrackingPLFilter.class);
    filtersMap.put(VehicleTrackingBootstrapFilter.class.getName(), VehicleTrackingBootstrapFilter.class);
  }
  
  @Override
  public Set<String> getFilterTypes() {
    return new HashSet<String>(filtersMap.keySet());
  }
  
  public enum INFO_LEVEL {
    SINGLE_RESULT, ALL_RESULTS, DEBUG
  }

  private static class UpdateRunnable implements java.lang.Runnable {

    final List<GpsObservation> observations;
    final InferenceInstance ie;

    UpdateRunnable(List<GpsObservation> observations, InferenceInstance ie) {
      super();
      this.observations = observations;
      this.ie = ie;
    }

    @Override
    public void run() {
      for (GpsObservation obs : observations) 
        ie.update(obs);
    }

  }

  private static VehicleStateInitialParameters defaultVehicleStateInitialParams =
      new VehicleStateInitialParameters(
          VectorFactory.getDefault().createVector2D(100d, 100d), 20,
          VectorFactory.getDefault().createVector2D(0.000625, 0.000625), 20,
          VectorFactory.getDefault().createVector2D(0.000625, 0.000625), 20,
          VectorFactory.getDefault().createVector2D(5d, 95d),
          VectorFactory.getDefault().createVector2D(95d, 5d), 
          VehicleTrackingPLFilter.class.getName(),
          25, 30, 0l);

  static public final int THREAD_COUNT;

  static {
    final int numProcessors =
        Runtime.getRuntime().availableProcessors();

    if (numProcessors <= 2) {
      THREAD_COUNT = numProcessors;
    } else {
      THREAD_COUNT = numProcessors - 1;
    }
  }

  private static final ExecutorService executor = Executors
      .newFixedThreadPool(THREAD_COUNT);

  private static final Map<String, InferenceInstance> vehicleToInstance =
      Maps.newConcurrentMap();

  public static final String defaultFilterName = 
      Iterables.getFirst(filtersMap.keySet(), null);

  public static INFO_LEVEL defaultInfoLevel = INFO_LEVEL.ALL_RESULTS;

  @Override
  public void onReceive(String obs) throws Exception {
    // TODO XXX
//    synchronized (this) {
//      final GpsObservation observation = createObservation(obs);
//      if (!processRecord(observation)) {
//        InferenceServiceImpl.getOrCreateInferenceInstance(observation.getSourceId(), 
//            defaultVehicleStateInitialParams, null, defaultInfoLevel);
//      }
//
//      log.info("Message received:  "
//          + observation.getTimestamp().toString());
//    }
  }

  public static void clearInferenceData() {
    vehicleToInstance.clear();
  }

  public static INFO_LEVEL getDefaultInfoLevel() {
    return defaultInfoLevel;
  }

  public static VehicleStateInitialParameters
      getDefaultVehicleStateInitialParams() {
    return defaultVehicleStateInitialParams;
  }

  public static ExecutorService getExecutor() {
    return executor;
  }

  public static InferenceInstance getInferenceInstance(
    String vehicleId) {
    final InferenceInstance ie = vehicleToInstance.get(vehicleId);
    return ie;
  }

  public static List<InferenceInstance> getInferenceInstances() {
    return Lists.newArrayList(vehicleToInstance.values());
  }

  public static InferenceInstance getOrCreateInferenceInstance(
    String vehicleId,
    VehicleStateInitialParameters initialParameters,
    VehicleStateInitialParameters simParameters, INFO_LEVEL infoLevel) {

    InferenceInstance ie = vehicleToInstance.get(vehicleId);

    if (ie == null) {
      ie =
          new InferenceInstance(vehicleId, simParameters, infoLevel,
              initialParameters);
      vehicleToInstance.put(vehicleId, ie);
    }

    return ie;
  }

  /**
   * This will process a record for an already existing
   * {@link #InferenceInstance}, or it will create a new one.
   * 
   * @param observationFactory
   * @return boolean true if an instance was found and processed
   */
  public static boolean processRecord(GpsObservation observation) {

    final InferenceInstance ie =
        getInferenceInstance(observation.getSourceId());

    if (ie == null)
      return false;

    executor.execute(new UpdateRunnable(Collections.singletonList(observation), ie));

    return true;
  }

  public static void
      processRecords(List<GpsObservation> observations,
        VehicleStateInitialParameters initialParameters,
        String filterTypeName,
        INFO_LEVEL level) throws InterruptedException {

    final Multimap<InferenceInstance, GpsObservation> instanceToObs = TreeMultimap.create();
    
    for (final GpsObservation obs : observations) {
      final InferenceInstance ie =
          getOrCreateInferenceInstance(obs.getSourceId(),
              initialParameters, null, level);
      instanceToObs.put(ie, obs);
    }

    final List<Callable<Object>> tasks = Lists.newArrayList();
    for (InferenceInstance instance: instanceToObs.keySet()) {
      tasks.add(Executors.callable(new UpdateRunnable(
          Lists.newArrayList(instanceToObs.get(instance)), instance)));
    }
      
    executor.invokeAll(tasks);
  }

  public static void remove(String name) {
    vehicleToInstance.remove(name);
    ObservationFactory.remove(name);
  }

  public static void setDefaultInfoLevel(INFO_LEVEL defaultInfoLevel) {
    InferenceServiceImpl.defaultInfoLevel = defaultInfoLevel;
  }

  public static void setDefaultVehicleStateInitialParams(
    VehicleStateInitialParameters defaultVehicleStateInitialParams) {
    InferenceServiceImpl.defaultVehicleStateInitialParams =
        defaultVehicleStateInitialParams;
  }

  public static Map<String, Class<? extends VehicleTrackingFilter>> getFilters() {
  	return filtersMap;
  }

  public static InferenceGraph getGraph() {
    return graph;
  }

}
