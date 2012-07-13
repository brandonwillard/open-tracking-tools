package org.openplans.tools.tracking.impl.statistics;

import static org.junit.Assert.*;

import java.util.List;

import gov.sandia.cognition.statistics.DataDistribution;

import org.junit.Before;
import org.junit.Test;
import org.openplans.tools.tracking.impl.Observation;
import org.openplans.tools.tracking.impl.Simulation;
import org.openplans.tools.tracking.impl.VehicleTrackingFilter;
import org.openplans.tools.tracking.impl.Simulation.SimulationParameters;
import org.openplans.tools.tracking.impl.VehicleState;
import org.openplans.tools.tracking.impl.util.OtpGraph;

import com.google.common.collect.Lists;

public class SimulationTestHarness {

  private OtpGraph graph;
  private Simulation sim; 
  private VehicleTrackingFilter<Observation, VehicleState> filter;
  private List<DataDistribution<VehicleState>> posteriorDistributions = Lists.newArrayList();
  
  private SimulationTestHarness(String graphName, String simName, SimulationParameters simParams,
    VehicleTrackingFilter<Observation, VehicleState> filter) {
    this.graph = new OtpGraph(graphName);
    this.sim = new Simulation(simName, graph, simParams);
    this.filter = filter; 
  }
      
  @Before
  public void setUp() throws Exception {
  }

  @Test
  public void test() {
    VehicleState vehicleState = this.sim.computeInitialState();
    long time = this.sim.getSimParameters().getStartTime().getTime();
    DataDistribution<VehicleState> currentState = filter.createInitialLearnedObject();
    
    posteriorDistributions.add(currentState.clone());
    
    while (time < this.sim.getSimParameters().getEndTime().getTime()) {
      vehicleState = this.sim.stepSimulation(vehicleState);
      
      time = vehicleState.getObservation().getTimestamp().getTime();
      filter.update(currentState, vehicleState.getObservation());
      
      posteriorDistributions.add(currentState.clone());
    } 
    
    fail("Not yet implemented");
  }

}
